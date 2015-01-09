using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Microsoft.Kinect;
using OpenCvSharp;

namespace KinectMotionCapture
{
    public class JointMirroredCorrection
    {
        class OrderTuple {
            public DateTime Timestamp;
            public int RecordIndex;
            public int FrameIndex;
            public OrderTuple(DateTime timestamp, int recordIndex, int frameIndex) {
                this.Timestamp = timestamp;
                this.RecordIndex = recordIndex;
                this.FrameIndex = frameIndex;
            }
        }

        /// <summary>
        /// 2つのJointsの距離を計算する
        /// </summary>
        /// <param name="jointsOne"></param>
        /// <param name="jointsTwo"></param>
        /// <returns></returns>
        public static Dictionary<JointType, double> CalcBodyDistanceSq(Dictionary<JointType, Joint> jointsOne, Dictionary<JointType, Joint> jointsTwo)
        {
            Dictionary<JointType, double> res = new Dictionary<JointType, double>();
            IEnumerable<JointType> keys = jointsOne.Keys.Intersect(jointsTwo.Keys);
            foreach (JointType key in keys)
            {
                CvPoint3D64f one = jointsOne[key].Position.ToCvPoint3D();
                CvPoint3D64f two = jointsTwo[key].Position.ToCvPoint3D();
                double distanceSq = CvEx.GetDistanceSq(one, two);
                res[key] = distanceSq;
            }
            return res;
        }

        /// <summary>
        /// 修正してくれるやつ
        /// </summary>
        /// <param name="frameSeq"></param>
        public static void Correct(FrameSequence frameSeq) {
            List<OrderTuple> orders = new List<OrderTuple>();
            foreach (var pair in frameSeq.Frames.Select((frame, index) => Tuple.Create(frame, index)))
            {
                for (int recordNo = 0; recordNo < pair.Item1.recordNum; recordNo++)
                {
                    orders.Add(new OrderTuple(pair.Item1.Time, recordNo, pair.Item2));
                }
            }
            // 前フレームのユーザと対応するBodyを保持する
            Dictionary<int, SerializableBody> prevUserBodyMap = new Dictionary<int, SerializableBody>();
            SerializableBody prevUserBody;

            orders = orders.OrderBy(x => x.Timestamp.Ticks).ToList();
            SkeletonInterpolator interp = new SkeletonInterpolator(0.5, true);
            foreach (OrderTuple tuple in orders) {
                Frame currFrame = frameSeq.Frames[tuple.FrameIndex];
                const long maxInterval = (long)(10000000 * 0.1);
                DateTime prev = new DateTime(tuple.Timestamp.Ticks - maxInterval);
                IEnumerable<int> users = currFrame.GetBodyList(tuple.RecordIndex).Select(b => b.integratedId);
                foreach (int user in users)
                {
                    SerializableBody currBody = currFrame.GetSelectedBody(tuple.RecordIndex, integratedId: user);
                    // 前のBodyと比較して同じ場合は処理をスキップする
                    if (prevUserBodyMap.TryGetValue(user, out prevUserBody))
                    {
                        double avgDistanceSq = CalcBodyDistanceSq(prevUserBody.Joints, currBody.Joints).Values.Average();
                        if (avgDistanceSq == 0.0)
                        {
                            // delete body
                            continue;
                        }
                        
                    }
                    prevUserBodyMap[user] = currBody;

                    Dictionary<JointType, CvPoint3D64f> prevJoints = interp.IntegrateSkeleton(prev, user, frameSeq);
                    if (prevJoints != null)
                    {
                        Dictionary<JointType, CvPoint3D64f> currJoints = currBody.Joints.ToDictionary(p => p.Key, p => (CvPoint3D64f)p.Value.Position.ToCvPoint3D());
                        HashSet<JointType> mirroredPrevKeys = new HashSet<JointType>(prevJoints.Keys.Select(j => CalcEx.GetMirroredJoint(j)));
                        if (currJoints != null && prevJoints != null) {
                            var absJoints = currJoints.ToDictionary(p => p.Key, p => CvEx.ConvertPoint3D(p.Value, frameSeq.ToWorldConversions[tuple.RecordIndex]));
                            var absMirroredJoints = absJoints.ToDictionary(p => CalcEx.GetMirroredJoint(p.Key), p => p.Value);
                            var availableKeys = prevJoints.Keys.Where(j => mirroredPrevKeys.Contains(j)).ToList();
                            var keysNormal = availableKeys.Intersect(absJoints.Keys).ToList();
                            var keysMirrored = availableKeys.Intersect(absMirroredJoints.Keys).ToList();

                            if (keysNormal.Count > 0 && keysMirrored.Count > 0) {
                                double avg1 = keysNormal.Select(j => CvEx.GetDistanceSq(prevJoints[j], absJoints[j])).Average();
                                double avg2 = keysMirrored.Select(j => CvEx.GetDistanceSq(prevJoints[j], absMirroredJoints[j])).Average();
                                if (avg2 < avg1) {
                                    currFrame.InverseBody(tuple.RecordIndex, integratedId: user);
                                }
                            }
                        }
                    }
                }
            }
        }

        public static void Correct3(FrameSequence frameSeq)
        {
            CvPoint3D64f[] prevShoulderVectors = new CvPoint3D64f[frameSeq.recordNum];
            CvPoint3D64f[] prevHipVectors = new CvPoint3D64f[frameSeq.recordNum];
            foreach (Frame frame in frameSeq.Frames)
            {
                List<SerializableBody> bodies = frame.GetSelectedBodyList(integratedIds: frameSeq.selecteedIntegretedIdList);
                if (bodies.Count == 0)
                {
                    continue;
                }
                CvPoint3D64f[] shoulderVectors = bodies.Select(b =>
                    (CvPoint3D64f)(b.Joints[JointType.ShoulderLeft].Position.ToCvPoint3D() - b.Joints[JointType.ShoulderRight].Position.ToCvPoint3D())).ToArray();
                CvPoint3D64f[] hipVectors = bodies.Select(b =>
                    (CvPoint3D64f)(b.Joints[JointType.HipLeft].Position.ToCvPoint3D() - b.Joints[JointType.HipRight].Position.ToCvPoint3D())).ToArray();
                // 0回目の処理
                if (prevShoulderVectors == default(CvPoint3D64f[]) || prevHipVectors == default(CvPoint3D64f[]))
                {
                    prevShoulderVectors = shoulderVectors;
                    prevHipVectors = hipVectors;
                    continue;
                }
                double[] shoulderCosArray = shoulderVectors.Zip(prevShoulderVectors, (cur, prev) => CvEx.Cos(cur, prev)).ToArray();
                double[] hipCosArray = hipVectors.Zip(prevHipVectors, (cur, prev) => CvEx.Cos(cur, prev)).ToArray();
                for (int no = 0; no < frame.recordNum; no++)
                {
                    if (shoulderCosArray[no] <= -0.8 || hipCosArray[no] <= -0.8)
                    {
                        frame.InverseBody(no, integratedId: frameSeq.selecteedIntegretedIdList[no]);
                    }
                }
                // Inverseされていた場合には骨の位置関係が変わっているため
                prevShoulderVectors = bodies.Select(b =>
                    (CvPoint3D64f)(b.Joints[JointType.ShoulderLeft].Position.ToCvPoint3D() - b.Joints[JointType.ShoulderRight].Position.ToCvPoint3D())).ToArray();
                prevHipVectors = bodies.Select(b =>
                    (CvPoint3D64f)(b.Joints[JointType.HipLeft].Position.ToCvPoint3D() - b.Joints[JointType.HipRight].Position.ToCvPoint3D())).ToArray();
            }
        }


        public class Range
        {
            public int RecordIndex;
            public int User;
            public int BeginFrameIndex, EndFrameIndex;
            public double Reliability;
            public Range(int recordIndex, int user, int beginFrameIndex, int endFrameIndex)
            {
                this.RecordIndex = recordIndex;
                this.User = user;
                this.BeginFrameIndex = beginFrameIndex;
                this.EndFrameIndex = endFrameIndex;
            }
        }

        public static IList<Range> GetSegmentations(TimeSpan segmentTimeSpan, FrameSequence frameSeq)
        {
            List<Range> ret = new List<Range>();
            for (int no = 0; no < frameSeq.recordNum; no++)
            {
                Dictionary<int, Tuple<int, int, DateTime>> currentRange = new Dictionary<int, Tuple<int, int, DateTime>>();
                int frameIndex = 0;
                foreach(Frame frame in frameSeq.Frames)
                {
                    int[] users = frame.GetMotionData(no).bodies.Select(b => b.integratedId).ToArray();
                    foreach (int user in users)
                    {
                        Tuple<int, int, DateTime> rangeInfo;
                        if (currentRange.TryGetValue(user, out rangeInfo))
                        {
                            // 前回と比べて時間内なら継続
                            if (frame.Time - rangeInfo.Item3 <= segmentTimeSpan)
                            {
                                currentRange[user] = new Tuple<int, int, DateTime>(rangeInfo.Item1, frameIndex, frame.Time);
                                continue;
                            }
                            else
                            {
                                ret.Add(new Range(no, user, rangeInfo.Item1, rangeInfo.Item2 + 1));
                            }
                        }
                        // 新しい範囲
                        currentRange[user] = new Tuple<int, int, DateTime>(frameIndex, frameIndex, frame.Time);
                    }
                    frameIndex++;
                }
                foreach (var pair in currentRange)
                {
                    int user = pair.Key;
                    Tuple<int, int, DateTime> rangeInfo = pair.Value;
                    ret.Add(new Range(no, user, rangeInfo.Item1, rangeInfo.Item2 + 1));
                }
            }
            SkeletonInterpolator interp = new SkeletonInterpolator(0.5, true);
            foreach (Range range in ret)
            {
                List<double> avgDistances = new List<double>();
                for (int frameIndex = range.BeginFrameIndex; frameIndex < range.EndFrameIndex; frameIndex++)
                {
                    Frame frame = frameSeq.Frames[frameIndex];
                    var dict = interp.IntegrateSkeleton(frame.Time, range.User, frameSeq);
                    var body = frame.GetSelectedBody(range.RecordIndex, integratedId: range.User);
                    if (body != null)
                    {
                        var joints = Utility.GetValidJoints(body.Joints);
                        if (joints != null && dict != null)
                        {
                            var absJoints = joints.ToDictionary(p => p.Key, p => CvEx.ConvertPoint3D(p.Value.Position.ToCvPoint3D(), frameSeq.ToWorldConversions[range.RecordIndex]));
                            var keys = dict.Keys.Intersect(absJoints.Keys).ToList();

                            if (keys.Count > 0)
                            {
                                avgDistances.Add(keys.Select(j => CvEx.GetDistanceSq(dict[j], absJoints[j])).Average());
                            }
                        }
                    }
                }
                if (avgDistances.Count > 0)
                {
                    range.Reliability = 1.0 / avgDistances.Average();
                }
                else
                {
                    range.Reliability = 0;
                }
            }
            return ret;
        }

        public static void Correct2(FrameSequence frameSeq)
        {
            IList<Range> ranges = GetSegmentations(new TimeSpan(0, 0, 0, 500), frameSeq);
            ranges = ranges.OrderBy(p => p.Reliability).ToList();
            SkeletonInterpolator interp = new SkeletonInterpolator(0.5, true);
            foreach (Range range in ranges)
            {
                for (int frameIndex = range.BeginFrameIndex; frameIndex < range.EndFrameIndex; frameIndex++)
                {
                    Frame frame = frameSeq.Frames[frameIndex];

                    var dict = interp.IntegrateSkeleton(frame.Time, range.User, frameSeq);
                    var body = frame.GetSelectedBody(range.RecordIndex, integratedId: range.User);
                    if (body != null)
                    {
                        var joints = Utility.GetValidJoints(body.Joints);
                        HashSet<JointType> mirrored = new HashSet<JointType>(joints.Keys.Select(j => CalcEx.GetMirroredJoint(j)));
                        if (joints != null && dict != null)
                        {
                            var absJoints = joints.ToDictionary(p => p.Key, p => CvEx.ConvertPoint3D(p.Value.Position.ToCvPoint3D(), frameSeq.ToWorldConversions[range.RecordIndex]));
                            var absMirroredJoints = absJoints.ToDictionary(p => CalcEx.GetMirroredJoint(p.Key), p => p.Value);
                            var keysJ = joints.Keys.Intersect(dict.Keys).Where(j => mirrored.Contains(j)).ToList();
                            var keys1 = keysJ.Intersect(absJoints.Keys).ToList();
                            var keys2 = keysJ.Intersect(absMirroredJoints.Keys).ToList();

                            if (keys1.Count > 0 && keys2.Count > 0)
                            {
                                double avg1 = keys1.Select(j => CvEx.GetDistanceSq(dict[j], absJoints[j])).Average();
                                double avg2 = keys2.Select(j => CvEx.GetDistanceSq(dict[j], absMirroredJoints[j])).Average();
                                if (avg2 < avg1)
                                {
                                    frame.InverseBody(range.RecordIndex, integratedId: range.User);
                                }
                            }

                        }
                    }
                }
            }

        }
    }
}
