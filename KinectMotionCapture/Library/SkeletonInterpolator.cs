using System;
using System.Collections.Generic;
using System.Diagnostics;
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
        public double GetVarianceWeight(MotionData prevFrame, MotionData nextFrame, Dictionary<JointType, Joint> prevJoints, Dictionary<JointType, Joint> nextJoints, DateTime time)
        {
            if (prevJoints == null || prevJoints.Count == 0)
                return 0;
            if (nextJoints == null || nextJoints.Count == 0)
                return 0;
            double prevVariance = GetSkeletonCamPosVariance(prevJoints.Values.Select(p => (CvPoint3D64f)p.Position.ToCvPoint3D()).ToList());
            double nextVariance = GetSkeletonCamPosVariance(prevJoints.Values.Select(p => (CvPoint3D64f)p.Position.ToCvPoint3D()).ToList());
            return Math.Sqrt(prevVariance * nextVariance);
        }

        /// <summary>
        /// 骨格座標の信頼性の値(分散以外)を求めます
        /// </summary>
        /// <param name="record"></param>
        /// <param name="time"></param>
        /// <param name="user"></param>
        /// <returns></returns>
        public double GetSkeletonReliability(MotionData prevFrame, MotionData nextFrame, Dictionary<JointType, Joint> prevJoints, Dictionary<JointType, Joint> nextJoints, DateTime time, 
            CameraIntrinsics cameraInfo)
        {
            double periodAfter = (time - prevFrame.TimeStamp).TotalSeconds;
            double periodBefore = (nextFrame.TimeStamp - time).TotalSeconds;
            double weightPeriod = Math.Exp(-periodAfter / 0.2) + Math.Exp(-periodBefore / 0.2);
            if (prevJoints == null || prevJoints.Count == 0)
                return 0;
            if (nextJoints == null || nextJoints.Count == 0)
                return 0;
            double prevEdge = 1;
            double nextEdge = 1;
            if (prevJoints.Count > 0)
            {
                prevEdge = prevJoints.Values.Select(p => Utility.GetRatioSqOfPrincipalToFocal(cameraInfo, p.Position.X, p.Position.Y)).Select(v => 1.0 / (1.0 + Math.Pow(v, 4))).Average();
            }
            if (nextJoints.Count > 0)
            {
                nextEdge = nextJoints.Values.Select(p => Utility.GetRatioSqOfPrincipalToFocal(cameraInfo, p.Position.X, p.Position.Y)).Select(v => 1.0 / (1.0 + Math.Pow(v, 4))).Average();
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

        public Dictionary<JointType, CvPoint3D64f> InterpolateSkeleton(MotionData prevFrame, MotionData nextFrame, Dictionary<JointType, Joint> prevJoints, Dictionary<JointType, Joint> nextJoints,
            DateTime time, CvMat ToWorldConversion)
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
            if (prevJoints == null || nextJoints == null)
                return null;
            Dictionary<JointType, CvPoint3D64f> prevData = prevJoints.ToDictionary(p => p.Key, p => (CvPoint3D64f)p.Value.Position.ToCvPoint3D());
            Dictionary<JointType, CvPoint3D64f> nextData = nextJoints.ToDictionary(p => p.Key, p => (CvPoint3D64f)p.Value.Position.ToCvPoint3D());
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

        public Dictionary<JointType, CvPoint3D64f> IntegrateSkeleton(DateTime time, int userInt, FrameSequence frameSeq)
        {
            List<CvMat> ToWorldConversions = frameSeq.ToWorldConversions;
            CameraIntrinsics cameraInfo = frameSeq.CameraInfo;
            List<UserSegmentation> segm = frameSeq.Segmentations;

            Dictionary<JointType, CvPoint3D64f>[] jointsArr = new Dictionary<JointType, CvPoint3D64f>[frameSeq.recordNum];
            double[] reliabilityArr = new double[frameSeq.recordNum];
            double[] weightArr = new double[frameSeq.recordNum];

            for (int recordNo = 0; recordNo < frameSeq.recordNum; recordNo++)
            {
                MotionData prevData = frameSeq.GetPrevData(recordNo, time);
                MotionData nextData = frameSeq.GetNextData(recordNo, time);

                if (prevData == null || nextData == null || prevData.bodies.Length * nextData.bodies.Length == 0 || !prevData.isValid || !nextData.isValid)
                {
                    jointsArr[recordNo] = null;
                    reliabilityArr[recordNo] = 0;
                    weightArr[recordNo] = 0;
                    continue;
                }

                SerializableBody prevBody = prevData.bodies.Where(b => b.integratedId == userInt).FirstOrDefault();
                SerializableBody nextBody = nextData.bodies.Where(b => b.integratedId == userInt).FirstOrDefault();
                if (prevBody == null || nextBody == null || prevBody.Equals(default(SerializableBody)) || nextBody.Equals(default(SerializableBody)))
                {
                    jointsArr[recordNo] = null;
                    reliabilityArr[recordNo] = 0;
                    weightArr[recordNo] = 0;
                    continue;
                }
                // 統計情報によるフィルタリング
                Dictionary<JointType, Joint> prevJoints;
                Dictionary<JointType, Joint> nextJoints;
                if (prevBody.Joints == null || nextBody.Joints == null)
                    continue;
                if (frameSeq.BodyStat != null)
                {
                    prevJoints = frameSeq.BodyStat.FilterBonesByStatistics(prevBody.Joints);
                    nextJoints = frameSeq.BodyStat.FilterBonesByStatistics(nextBody.Joints);
                }
                else
                {
                    prevJoints = Utility.GetValidJoints(prevBody.Joints);
                    nextJoints = Utility.GetValidJoints(nextBody.Joints);
                }

                jointsArr[recordNo] = this.InterpolateSkeleton(prevData, nextData, prevJoints, nextJoints, time, ToWorldConversions[recordNo]);
                reliabilityArr[recordNo] = this.GetSkeletonReliability(prevData, nextData, prevJoints, nextJoints, time, cameraInfo);
                weightArr[recordNo] = this.GetVarianceWeight(prevData, nextData, prevJoints, nextJoints, time);
            }            

            double maxWeight = weightArr.Max();
            double[] modifiedReliabilityList = weightArr.Select(w => Math.Max(0, (w / maxWeight) - _weightBase)).Zip(reliabilityArr, (a, b) => a * b).ToArray();
            var keys = jointsArr.Where(j => j != null).SelectMany(j => j.Keys).Distinct().ToList();
            if (maxWeight == 0)
                return null;

            return CalcEx.LinearMedianSkeletons(jointsArr, modifiedReliabilityList);
        }

        public static Dictionary<int, List<Dictionary<JointType, CvPoint3D64f>>> ExportFromProject(FrameSequence frameseq, int startIndex, int endIndex)
        {
            TimeSpan period = new TimeSpan((long)(10000000 / frameseq.frameRate));
            List<DateTime> timestamps = new List<DateTime>();
            for (DateTime time = frameseq.Frames[startIndex].Time; time < frameseq.Frames[endIndex].Time; time += period)
            {
                timestamps.Add(time);
            }

            HashSet<Tuple<int, JointType>> uniqueUserJoint = new HashSet<Tuple<int, JointType>>();
            List<Frame> frames = frameseq.Slice(startIndex, endIndex);
            foreach(Frame frame in frames){
                for (int i = 0; i < frameseq.recordNum; i++)
                {
                    foreach (SerializableBody body in frame.GetBodyList(i))
                    {
                        foreach (JointType jointType in Utility.GetValidJoints(body.Joints).Keys)
                        {
                            // TrackingIdと勝手に作ったuser idの対応辞書が必要. 以下の処理はとりあえず
                            Tuple<int, JointType> userJoint = new Tuple<int, JointType>(body.integratedId, jointType);
                            if (!uniqueUserJoint.Contains(userJoint))
                            {
                                uniqueUserJoint.Add(userJoint);
                            }
                        }
                    }
                }
            }

            List<Tuple<int, JointType>> userJointPairs = (
                from pair in uniqueUserJoint
                orderby pair.Item1, pair.Item2
                select pair
                ).ToList();

            Dictionary<int, List<Dictionary<JointType, CvPoint3D64f>>> allBodies = new Dictionary<int, List<Dictionary<JointType, CvPoint3D64f>>>();
            SkeletonInterpolator skeletonInterpolator = new SkeletonInterpolator(0.5, true);
            foreach (int user in userJointPairs.Select(p => p.Item1).Distinct())            
            {
                List<Dictionary<JointType, CvPoint3D64f>> jointsSeq = new List<Dictionary<JointType, CvPoint3D64f>>();
                foreach(DateTime time in timestamps)
                {
                    Dictionary<JointType, CvPoint3D64f> joints = skeletonInterpolator.IntegrateSkeleton(time, user, frameseq);
                    // jointsがnullの場合にはダミーデータを突っ込んで長さを稼ぐ
                    if (joints == null)
                    {
                        jointsSeq.Add(new Dictionary<JointType, CvPoint3D64f>() { { JointType.SpineBase, new CvPoint3D64f(float.MaxValue, float.MaxValue, float.MaxValue) } });
                    }
                    else
                    {
                        jointsSeq.Add(joints);
                    }
                }
                // 長さの帳尻合わせ
                jointsSeq.Add(jointsSeq.Last());
                allBodies[user] = jointsSeq;
            }
            return allBodies;
        }
    }
}
