using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Microsoft.Kinect;
using OpenCvSharp;


namespace KinectMotionCapture
{
    public class SkeletonInterpolator
    {
        /// <summary>
        /// カメラ座標系における骨格位置の水平垂直の標準偏差の積を求めます。
        /// </summary>
        /// <param name="joints"></param>
        /// <param name="undist"></param>
        /// <param name="imageSize"></param>
        /// <returns></returns>
        public static double GetSkeletonCamPosVariance(ICollection<CvPoint3D64f> cameraPos)
        {
            //            return CalcEx.GetVariance(absPositions, p => p.X) + CalcEx.GetVariance(absPositions, p => p.Y) + CalcEx.GetVariance(absPositions, p => p.Z);
            return CalcEx.GetStdDev(cameraPos, p => p.X) * CalcEx.GetStdDev(cameraPos, p => p.Y);
        }
        /// <summary>
        /// 指定されたレコードの指定された時刻の指定されたユーザの骨格座標の水平垂直標準偏差積を求めます
        /// </summary>
        /// <param name="record"></param>
        /// <param name="time"></param>
        /// <param name="user"></param>
        /// <returns></returns>
        public double GetVarianceWeight(TrackImageRecordProperty record, DateTime time, int user)
        {
            int prevIndex = ListEx.GetMaxLessEqualIndexFromBinarySearch(record.RecordData.GetIndexBinarySearch(time));
            int nextIndex = ListEx.GetMinGreaterEqualIndexFromBinarySearch(record.RecordData.GetIndexBinarySearch(time));
            if (prevIndex == -1)
                return 0;
            if (nextIndex == record.RecordData.FrameCount)
                return 0;
            TrackFrame prevFrame = record.RecordData.GetTrackFrame(prevIndex);
            TrackFrame nextFrame = record.RecordData.GetTrackFrame(nextIndex);
            Dictionary<JointType, CameraSpacePoint> prevJoints = prevFrame.GetValidJoints(user);
            Dictionary<JointType, CameraSpacePoint> nextJoints = nextFrame.GetValidJoints(user);
            if (prevJoints == null || prevJoints.Count == 0)
                return 0;
            if (nextJoints == null || nextJoints.Count == 0)
                return 0;
            double prevVariance = GetSkeletonCamPosVariance(prevJoints.Values.Select(p => record.UndistortionData.GetRealFromScreenPos(p.ToCvPoint3D(), prevFrame.DepthUserSize)).ToList());
            double nextVariance = GetSkeletonCamPosVariance(prevJoints.Values.Select(p => record.UndistortionData.GetRealFromScreenPos(p.ToCvPoint3D(), nextFrame.DepthUserSize)).ToList());
            return Math.Sqrt(prevVariance * nextVariance);
        }

        /// <summary>
        /// 骨格座標の信頼性の値(分散以外)を求めます
        /// </summary>
        /// <param name="record"></param>
        /// <param name="time"></param>
        /// <param name="user"></param>
        /// <returns></returns>
        public double GetSkeletonReliability(TrackImageRecordProperty record, DateTime time, int user)
        {
            int prevIndex = ListEx.GetMaxLessEqualIndexFromBinarySearch(record.RecordData.GetIndexBinarySearch(time));
            int nextIndex = ListEx.GetMinGreaterEqualIndexFromBinarySearch(record.RecordData.GetIndexBinarySearch(time));
            if (prevIndex == -1)
                return 0;
            if (nextIndex == record.RecordData.FrameCount)
                return 0;
            TrackFrame prevFrame = record.RecordData.GetTrackFrame(prevIndex);
            TrackFrame nextFrame = record.RecordData.GetTrackFrame(nextIndex);
            double periodAfter = (time - prevFrame.Timestamp).TotalSeconds;
            double periodBefore = (nextFrame.Timestamp - time).TotalSeconds;
            double weightPeriod = Math.Exp(-periodAfter / 0.2) + Math.Exp(-periodBefore / 0.2);
            Dictionary<JointType, CameraSpacePoint> prevJoints = prevFrame.GetValidJoints(user);
            Dictionary<JointType, CameraSpacePoint> nextJoints = nextFrame.GetValidJoints(user);
            if (prevJoints == null || prevJoints.Count == 0)
                return 0;
            if (nextJoints == null || nextJoints.Count == 0)
                return 0;
            double prevEdge = 1;
            double nextEdge = 1;
            if (prevJoints.Count > 0)
            {
                prevEdge = prevJoints.Values.Select(p => record.UndistortionData.CameraStruct.GetRatioSqOfPrincipalToFocal(p.X, p.Y)).Select(v => 1.0 / (1.0 + Math.Pow(v, 4))).Average();
            }
            if (nextJoints.Count > 0)
            {
                nextEdge = nextJoints.Values.Select(p => record.UndistortionData.CameraStruct.GetRatioSqOfPrincipalToFocal(p.X, p.Y)).Select(v => 1.0 / (1.0 + Math.Pow(v, 4))).Average();
            }
            return weightPeriod * Math.Sqrt(prevEdge * nextEdge);
        }

        double _weightBase;
        bool _omitWhenDataLack;
        /// <summary>
        /// 規定のコンストラクタ
        /// </summary>
        /// 
        /// <param name="weightBaseOfStddev">各レコードの骨格位置を統合するときの重み係数を計算時に算出値から引く値。
        /// 重みの算出は、そのレコード内の骨格位置の分散の平方根を、全レコード内で最大のその値で割ったものを使う。</param>
        /// <param name="omitWhenDataLack">フレーム間の線形補間時に一方のデータがないときに出力をしないようにする。さもなくば他方のデータを出力する</param>
        public SkeletonInterpolator(double weightBaseOfStddev, bool omitWhenDataLack)
        {
            _weightBase = weightBaseOfStddev;
            _omitWhenDataLack = omitWhenDataLack;
        }

        public Dictionary<JointType, CvPoint3D64f> InterpolateSkeleton(Frame record, DateTime time, int user)
        {
            int prevIndex = ListEx.GetMaxLessEqualIndexFromBinarySearch(record.RecordData.GetIndexBinarySearch(time));
            int nextIndex = ListEx.GetMinGreaterEqualIndexFromBinarySearch(record.RecordData.GetIndexBinarySearch(time));
            if (prevIndex == -1)
                return null;
            if (nextIndex == record.RecordData.FrameCount)
                return null;
            TrackFrame prevFrame = record.RecordData.GetTrackFrame(prevIndex);
            TrackFrame nextFrame = record.RecordData.GetTrackFrame(nextIndex);
            double prevWeight;
            if (prevFrame.Timestamp >= nextFrame.Timestamp)
            {
                prevWeight = 1;
            }
            else
            {
                prevWeight = (time - prevFrame.Timestamp).TotalSeconds / (nextFrame.Timestamp - prevFrame.Timestamp).TotalSeconds;
            }
            double nextWeight = 1.0 - prevWeight;
            Dictionary<JointType, CameraSpacePoint> prevJoints = prevFrame.GetValidJoints(user);
            Dictionary<JointType, CameraSpacePoint> nextJoints = nextFrame.GetValidJoints(user);
            if (prevJoints == null || nextJoints == null)
                return null;
            Dictionary<JointType, CvPoint3D64f> prevData = prevJoints.ToDictionary(p => p.Key, p => record.UndistortionData.GetRealFromScreenPos(p.Value.ToCvPoint3D(), prevFrame.DepthUserSize));
            Dictionary<JointType, CvPoint3D64f> nextData = nextJoints.ToDictionary(p => p.Key, p => record.UndistortionData.GetRealFromScreenPos(p.Value.ToCvPoint3D(), prevFrame.DepthUserSize));
            List<JointType> joints = prevData.Keys.Union(nextData.Keys).ToList();
            Dictionary<JointType, CvPoint3D64f> ret = new Dictionary<JointType, CvPoint3D64f>();
            foreach (JointType joint in joints)
            {
                CvPoint3D64f prevPos, nextPos;
                bool prevFound = prevData.TryGetValue(joint, out prevPos);
                bool nextFound = nextData.TryGetValue(joint, out nextPos);
                if ((prevFound && nextFound))
                {
                    ret[joint] = CvEx.ConvertPoint3D(prevPos * prevWeight + nextPos * nextWeight, record.ToWorldConversion);
                }
                else if (_omitWhenDataLack)
                {
                    if (prevFound)
                    {
                        ret[joint] = CvEx.ConvertPoint3D(prevPos, record.ToWorldConversion);
                    }
                    else if (nextFound)
                    {
                        ret[joint] = CvEx.ConvertPoint3D(nextPos, record.ToWorldConversion);
                    }
                }
            }
            return ret;
        }

        public Dictionary<JointType, CvPoint3D64f> IntegrateSkeleton(List<Frame> records, DateTime time, int user)
        {
            Dictionary<JointType, CvPoint3D64f>[] jointsArr = new Dictionary<JointType, CvPoint3D64f>[records.Count()];

            //records.Select(r => InterpolateSkeleton(r, time, user)).ToArray();
            for (int i = 0; i < jointsArr.Length; i++)
            {
                if (jointsArr[i] != null)
                {
                    jointsArr[i] = jointsArr[i].ToDictionary(p => CalcEx.GetMirroredJoint(p.Key), p => p.Value);
                }
            }
            double[] reliabilityList = records.Select(r => GetSkeletonReliability(r, time, user)).ToArray();
            double[] weightList = records.Select(r => GetVarianceWeight(r, time, user)).ToArray();
            double maxWeight = weightList.Max();
            double[] modifiedReliabilityList = weightList.Select(w => Math.Max(0, (w / maxWeight) - _weightBase)).Zip(reliabilityList, (a, b) => a * b).ToArray();
            var keys = jointsArr.Where(j => j != null).SelectMany(j => j.Keys).Distinct().ToList();
            if (maxWeight == 0)
                return null;

            return CalcEx.LinearMedianSkeletons(jointsArr, modifiedReliabilityList);
        }
    }
}
