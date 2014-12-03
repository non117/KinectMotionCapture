using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Microsoft.Kinect;
using OpenCvSharp;

namespace KinectMotionCapture
{
    public class JointMirroredCorrection
    {
        class Range {
            public int RecordIndex;
            public int User;
            public int BeginFrameIndex, EndFrameIndex;
            public double Reliability;
            public Range(int recordIndex, int user, int beginFrameIndex, int endFrameIndex) {
                this.RecordIndex = recordIndex;
                this.User = user;
                this.BeginFrameIndex = beginFrameIndex;
                this.EndFrameIndex = endFrameIndex;
            }
        }

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

        //public void Correct(FrameSequence frameSeq) {
        //    List<OrderTuple> orders = new List<OrderTuple>();
        //    foreach (var pair in frameSeq.Frames.Select((frame, index) => Tuple.Create(frame, index)))
        //    {
        //        for (int recordNo = 0; recordNo < pair.Item1.recordNum; recordNo++)
        //        {
        //            orders.Add(new OrderTuple(pair.Item1.Time, recordNo, pair.Item2));
        //        }
        //    }

        //    orders = orders.OrderBy(x => x.Timestamp.Ticks).ToList();
        //    SkeletonInterpolator interp = new SkeletonInterpolator(0.5, true);
        //    foreach (OrderTuple tuple in orders) {
        //        if (tuple.RecordIndex == 0)
        //            continue;
        //        Frame currFrame = frameSeq.Frames[tuple.RecordIndex];
        //        Frame prevFrame = frameSeq.Frames[tuple.RecordIndex - 1];
        //        IEnumerable<int> users = currFrame.GetBodyList(tuple.RecordIndex).Select(b => b.integratedId);
        //        foreach (int user in users)
        //        {
        //            Dictionary<JointType, CvPoint3D64f> prevJoints = interp.IntegrateSkeleton(prevFrame, user, frameSeq);
        //            if (prevJoints != null)
        //            {
        //                Dictionary<JointType, CvPoint3D64f> currJoints = currFrame.GetBodyJoints(tuple.RecordIndex)
        //            }
        //        }

        //        foreach (int user in trackFrame.GetValidUsers()) {
        //            var prevJoints = interp.IntegrateSkeleton(_records, prev, user);
        //            if (prevJoints != null) {
        //                var currentJoints = trackFrame.GetValidJoints(user);
        //                HashSet<SkeletonJoint> mirroredPrevKeys = new HashSet<SkeletonJoint>(prevJoints.Keys.Select(j => CalcEx.GetMirroredJoint(j)));
        //                if (currentJoints != null && prevJoints != null) {
        //                    var absJoints = currentJoints.ToDictionary(p => p.Key, p => CvEx.ConvertPoint3D(record.UndistortionData.GetRealFromScreenPos(p.Value.ToCvPoint3D(), trackFrame.DepthUserSize), record.ToWorldConversion));
        //                    var absMirroredJoints = absJoints.ToDictionary(p => CalcEx.GetMirroredJoint(p.Key), p => p.Value);
        //                    var availableKeys = prevJoints.Keys.Where(j => mirroredPrevKeys.Contains(j)).ToList();
        //                    var keysNormal = availableKeys.Intersect(absJoints.Keys).ToList();
        //                    var keysMirrored = availableKeys.Intersect(absMirroredJoints.Keys).ToList();

        //                    if (keysNormal.Count > 0 && keysMirrored.Count > 0) {
        //                        double avg1 = keysNormal.Select(j => CvEx.GetDistanceSq(prevJoints[j], absJoints[j])).Average();
        //                        double avg2 = keysMirrored.Select(j => CvEx.GetDistanceSq(prevJoints[j], absMirroredJoints[j])).Average();
        //                        if (avg2 < avg1) {
        //                            UserTrackingState state;
        //                            if (trackFrame.UserTrackings.TryGetValue(user, out state)) {
        //                                record.JointMirroredState.InverseMirroredRange(tuple.FrameIndex, tuple.FrameIndex + 1, state.OriginalUserIndex);
        //                            }
        //                        }
        //                    }

        //                }
        //            }
        //        }
        //    }
        //}
    }
}
