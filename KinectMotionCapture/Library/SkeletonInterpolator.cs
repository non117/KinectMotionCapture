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
        public double GetVarianceWeight(MotionData prevFrame, MotionData nextFrame, DateTime time, ulong user)
        {
            Dictionary<JointType, CameraSpacePoint> prevJoints = Utility.GetValidJointPoints(prevFrame.bodies, user);
            Dictionary<JointType, CameraSpacePoint> nextJoints = Utility.GetValidJointPoints(nextFrame.bodies, user);
            if (prevJoints == null || prevJoints.Count == 0)
                return 0;
            if (nextJoints == null || nextJoints.Count == 0)
                return 0;
            double prevVariance = GetSkeletonCamPosVariance(prevJoints.Values.Select(p => (CvPoint3D64f)p.ToCvPoint3D()).ToList());
            double nextVariance = GetSkeletonCamPosVariance(prevJoints.Values.Select(p => (CvPoint3D64f)p.ToCvPoint3D()).ToList());
            return Math.Sqrt(prevVariance * nextVariance);
        }

        /// <summary>
        /// 骨格座標の信頼性の値(分散以外)を求めます
        /// </summary>
        /// <param name="record"></param>
        /// <param name="time"></param>
        /// <param name="user"></param>
        /// <returns></returns>
        public double GetSkeletonReliability(MotionData prevFrame, MotionData nextFrame, DateTime time, ulong user, CameraIntrinsics cameraInfo)
        {
            double periodAfter = (time - prevFrame.TimeStamp).TotalSeconds;
            double periodBefore = (nextFrame.TimeStamp - time).TotalSeconds;
            double weightPeriod = Math.Exp(-periodAfter / 0.2) + Math.Exp(-periodBefore / 0.2);
            Dictionary<JointType, CameraSpacePoint> prevJoints = Utility.GetValidJointPoints(prevFrame.bodies, user);
            Dictionary<JointType, CameraSpacePoint> nextJoints = Utility.GetValidJointPoints(nextFrame.bodies, user);
            if (prevJoints == null || prevJoints.Count == 0)
                return 0;
            if (nextJoints == null || nextJoints.Count == 0)
                return 0;
            double prevEdge = 1;
            double nextEdge = 1;
            if (prevJoints.Count > 0)
            {
                prevEdge = prevJoints.Values.Select(p => Utility.GetRatioSqOfPrincipalToFocal(cameraInfo, p.X, p.Y)).Select(v => 1.0 / (1.0 + Math.Pow(v, 4))).Average();
            }
            if (nextJoints.Count > 0)
            {
                nextEdge = nextJoints.Values.Select(p => Utility.GetRatioSqOfPrincipalToFocal(cameraInfo, p.X, p.Y)).Select(v => 1.0 / (1.0 + Math.Pow(v, 4))).Average();
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

        public Dictionary<JointType, CvPoint3D64f> InterpolateSkeleton(MotionData prevFrame, MotionData nextFrame, DateTime time, ulong user, CvMat ToWorldConversion)
        {
            double prevWeight;
            if (prevFrame.TimeStamp >= nextFrame.TimeStamp)
            {
                prevWeight = 1;
            }
            else
            {
                prevWeight = (time - prevFrame.TimeStamp).TotalSeconds / (nextFrame.TimeStamp - prevFrame.TimeStamp).TotalSeconds;
            }
            double nextWeight = 1.0 - prevWeight;
            Dictionary<JointType, CameraSpacePoint> prevJoints = Utility.GetValidJointPoints(prevFrame.bodies, user);
            Dictionary<JointType, CameraSpacePoint> nextJoints = Utility.GetValidJointPoints(nextFrame.bodies, user);
            if (prevJoints == null || nextJoints == null)
                return null;
            Dictionary<JointType, CvPoint3D64f> prevData = prevJoints.ToDictionary(p => p.Key, p => (CvPoint3D64f)p.Value.ToCvPoint3D());
            Dictionary<JointType, CvPoint3D64f> nextData = nextJoints.ToDictionary(p => p.Key, p => (CvPoint3D64f)p.Value.ToCvPoint3D());
            List<JointType> joints = prevData.Keys.Union(nextData.Keys).ToList();
            Dictionary<JointType, CvPoint3D64f> ret = new Dictionary<JointType, CvPoint3D64f>();
            foreach (JointType joint in joints)
            {
                CvPoint3D64f prevPos, nextPos;
                bool prevFound = prevData.TryGetValue(joint, out prevPos);
                bool nextFound = nextData.TryGetValue(joint, out nextPos);
                if ((prevFound && nextFound))
                {
                    ret[joint] = CvEx.ConvertPoint3D(prevPos * prevWeight + nextPos * nextWeight, ToWorldConversion);
                }
                else if (_omitWhenDataLack)
                {
                    if (prevFound)
                    {
                        ret[joint] = CvEx.ConvertPoint3D(prevPos, ToWorldConversion);
                    }
                    else if (nextFound)
                    {
                        ret[joint] = CvEx.ConvertPoint3D(nextPos, ToWorldConversion);
                    }
                }
            }
            return ret;
        }

        public Dictionary<JointType, CvPoint3D64f> IntegrateSkeleton(Frame frame, ulong user, List<CvMat> ToWorldConversions, CameraIntrinsics cameraInfo)
        {
            Dictionary<JointType, CvPoint3D64f>[] jointsArr = new Dictionary<JointType, CvPoint3D64f>[frame.recordNum];
            double[] reliabilityList = new double[frame.recordNum];
            double[] weightList = new double[frame.recordNum];
            for (int recordNo = 0; recordNo < frame.recordNum - 1; recordNo++)
            {
                MotionData prevData = frame.GetMotionData(recordNo);
                MotionData nextData = frame.GetNextMotionData(recordNo);
                DateTime time = frame.Time;
                jointsArr[recordNo] = this.InterpolateSkeleton(prevData, nextData, time, user, ToWorldConversions[recordNo]);
                reliabilityList[recordNo] = this.GetSkeletonReliability(prevData, nextData, time, user, cameraInfo);
                weightList[recordNo] = this.GetVarianceWeight(prevData, nextData, time, user);
            }
            double maxWeight = weightList.Max();
            double[] modifiedReliabilityList = weightList.Select(w => Math.Max(0, (w / maxWeight) - _weightBase)).Zip(reliabilityList, (a, b) => a * b).ToArray();
            var keys = jointsArr.Where(j => j != null).SelectMany(j => j.Keys).Distinct().ToList();
            if (maxWeight == 0)
                return null;

            return CalcEx.LinearMedianSkeletons(jointsArr, modifiedReliabilityList);
        }

        public void ExportFromProject(FrameSequence frameseq) {
            HashSet<Tuple<ulong, JointType>> uniqueUserJoint = new HashSet<Tuple<ulong, JointType>>();
            foreach(Frame frame in frameseq.Frames){
                for (int i = 0; i < frameseq.recordNum; i++)
                {
                    foreach (SerializableBody body in frame.GetBodyList(i))
                    {
                        foreach (JointType jointType in Utility.GetValidJoints(body.Joints).Keys)
                        {
                            // TrackingIdと勝手に作ったuser idの対応辞書が必要. 以下の処理はとりあえず
                            Tuple<ulong, JointType> userJoint = new Tuple<ulong, JointType>(body.TrackingId, jointType);
                            if (!uniqueUserJoint.Contains(userJoint))
                            {
                                uniqueUserJoint.Add(userJoint);
                            }
                        }
                    }
                }
            }

            List<Tuple<ulong, JointType>> userJointPairs = (
                from pair in uniqueUserJoint
                orderby pair.Item1, pair.Item2
                select pair
                ).ToList();

            SkeletonInterpolator skeletonInterpolator = new SkeletonInterpolator(0.5, true);
            for (int i = 0; i < frameseq.Frames.Count() - 1; i++)
            {
                Dictionary<Tuple<ulong, JointType>, CvPoint3D64f> jointSet = new Dictionary<Tuple<ulong,JointType>,CvPoint3D64f>();
                Frame curr = frameseq.Frames[i];
                foreach (ulong user in userJointPairs.Select(p => p.Item1).Distinct())
                {
                    var joints = skeletonInterpolator.IntegrateSkeleton(curr, user, frameseq.ToWorldConversions, frameseq.CameraInfo);
                        if (joints != null) {
                        foreach (var pair in joints) {
                            Tuple<ulong, JointType> key = new Tuple<ulong, JointType>(user, pair.Key);
                            jointSet[key] = pair.Value;
                        }
                    }
                }
            }
        }
    }
}
