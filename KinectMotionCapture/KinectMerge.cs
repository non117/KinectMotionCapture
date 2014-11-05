using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;

using OpenCvSharp;

namespace KinectMotionCapture
{
    /// <summary>
    /// ある瞬間における複数レコードをまとめて管理するクラス
    /// </summary>
    class Frame
    {
        public int recordNum;
        private List<MotionData> motionDataList;
        
        public List<CvMat> ColorMatList
        {
            get { return this.motionDataList.Select((m) => m.imageMat).ToList(); }
        }
        
        public List<CvMat> DepthMatList
        {
            get { return this.motionDataList.Select((m) => m.depthMat).ToList(); }
        }
        
        public List<CvMat> ConvList
        {
            get { return this.motionDataList.Select((m) => m.toWorldConversion).ToList(); }
            set
            {
                for (int i = 0; i < this.recordNum; i++)
                {
                    this.motionDataList[i].toWorldConversion = value[i];
                }
            }
        }

        public List<SerializableBody> SelectedBodyList
        {
            get
            {
                List<SerializableBody> bodies = new List<SerializableBody>();
                foreach (MotionData data in this.motionDataList)
                {
                    SerializableBody body = data.bodies.Where((b) => b.TrackingId == data.SelectedUserId).First();
                    bodies.Add(body);
                }
                return bodies;
            }
        }

        public Frame(List<MotionData> motionDataList)
        {
            this.recordNum = motionDataList.Count();
            this.motionDataList = motionDataList;
        }
    }

    class KinectMerge
    {
        /*
        bool runOperationForEachFrameImage(string message, bool runInSelectedRange, bool isCancelEnabled, Action<DateTime, LockedTrackImageList> actionWithFrameIndexAndImages)
        {
            DateTime beginTime, endTime;
            TimeSpan period = getPeriodFromFrequency();
            if (runInSelectedRange)
            {
                _timePlayer.GetSelectedTime(out beginTime, out endTime);
            }
            else
            {
                beginTime = _timePlayer.BeginTime;
                endTime = _timePlayer.EndTime;
            }
            return ProgressData.DoAction((progress) =>
            {
                long tickCount = (endTime - beginTime).Ticks / period.Ticks;
                progress.InitProgress(message, tickCount);
                DateTime previousTime = _timePlayer.CurrentTime;
                for (DateTime time = beginTime; time <= endTime; time += period)
                {
                    _timePlayer.CurrentTime = time;
                    using (LockedTrackImageList tracks = _project.GetReadLockAllPlayer(_timePlayer.CurrentTime))
                    {
                        actionWithFrameIndexAndImages(time, tracks);
                    }
                    progress.CurrentValue++;

                }
                _timePlayer.CurrentTime = previousTime;
            }, "Running...", isCancelEnabled);
        }*/
        
        /// <summary>
        /// あるフレームにおける座標変換行列を深度情報から計算する
        /// </summary>
        /// <param name="frame"></param>
        public static void AjustFrameFromDepth(Frame frame)
        {
            List<Func<CvPoint3D64f, CvPoint3D64f>> toReal = new List<Func<CvPoint3D64f, CvPoint3D64f>>();
            foreach (CvMat depthMat in frame.DepthMatList)
            {
                toReal.Add((x) => KinectUndistortion.GetOriginalRealFromScreenPos(x, new CvSize(depthMat.Cols, depthMat.Rows)));
            }
            Func<float, double> distance2weight = x => 1.0 / (x * 0 + 400);
            using (ColoredIterativePointMatching sipm = new ColoredIterativePointMatching(frame.DepthMatList, frame.ColorMatList, toReal, frame.ConvList, distance2weight, 200))
            {
                List<CvMat> conversions = sipm.CalculateTransformSequntially(0.2, 3);
                frame.ConvList = conversions;
            }
        }

        /// <summary>
        /// フレーム範囲における座標変換行列を深度情報から計算する
        /// </summary>
        /// <param name="frames"></param>
        public static void AjustFramesFromDepth(List<Frame> frames)
        {
            foreach (Frame frame in frames)
            {
                List<Func<CvPoint3D64f, CvPoint3D64f>> toReal = new List<Func<CvPoint3D64f, CvPoint3D64f>>();
                foreach (CvMat depthMat in frame.DepthMatList)
                {
                    toReal.Add((x) => KinectUndistortion.GetOriginalRealFromScreenPos(x, new CvSize(depthMat.Cols, depthMat.Rows)));
                }
                Func<float, double> distance2weight = x => 1.0 / (x * 0 + 400);
                using (ColoredIterativePointMatching sipm = new ColoredIterativePointMatching(frame.DepthMatList, frame.ColorMatList, toReal, frame.ConvList, distance2weight, 200))
                {
                    List<CvMat> conversions = sipm.CalculateTransformSequntially(0.1, 1);
                    frame.ConvList = conversions;
                }
            }

        }

        /// <summary>
        /// あるフレームにおける座標変換行列を骨格情報から計算する
        /// </summary>
        /// <param name="frame"></param>
        public void AjustFrameFromeBone(Frame frame)
        {
            List<SerializableBody> bodies = frame.SelectedBodyList;
            List<CvMat> convList = frame.ConvList;
            if ( bodies.Count() != frame.recordNum )
            {
                System.Windows.MessageBox.Show("ユーザが選択されていないレコードがあります");
                return;
            }

            for (int j = 1; j < frame.recordNum; j++)
            {
                Dictionary<JointType, Joint> joint1 = bodies[0].Joints;
                Dictionary<JointType, Joint> joint2 = bodies[j].Joints;

                ICoordConversion3D crtc = new CoordRotTransConversion();
                foreach (JointType jointType in Enum.GetValues(typeof(JointType)))
                {
                    if (!joint1.ContainsKey(jointType))
                        continue;
                    if (!joint2.ContainsKey(jointType))
                        continue;
                    CvPoint3D64f from = joint2[jointType].Position.ToCvPoint3D();
                    CvPoint3D64f target = CvEx.ConvertPoint3D(joint1[jointType].Position.ToCvPoint3D(), convList[0]);
                    //bool valid1 = buf1.Data.IsOriginalJointValid(user1, jointType);
                    //bool valid2 = buf2.Data.IsOriginalJointValid(user2, jointType);
                    //if (valid1 && valid2)
                    //{
                    crtc.PutPoint(from, target, 1);
                    //}
                }
                convList[j] = crtc.Solve();
            }
            frame.ConvList = convList;
        }

        // 骨のマージフレーム版
        private void menuCalibUserAll_Click(object sender, RoutedEventArgs e)
        {
            Dictionary<Tuple<int, int>, int> cooccurenceCount = new Dictionary<Tuple<int, int>, int>();
            if (_project.RecordList.Any(r => r.SelectedUser == -1))
            {
                System.Windows.MessageBox.Show("ユーザが選択されていないレコードがあります");
                return;
            }
            if (runOperationForEachFrame("骨格データの解析中...", true, true, (time, imagePairs) =>
            {
                for (int i = 0; i < imagePairs.Count; i++)
                {
                    int user1 = _project.RecordList[i].SelectedUser;
                    TrackFrame trackImage1 = imagePairs[i];
                    if (trackImage1 == null || !trackImage1.UserTrackings.ContainsKey(user1))
                        continue;
                    UserTrackingState state1 = trackImage1.UserTrackings[user1];
                    for (int j = i + 1; j < imagePairs.Count; j++)
                    {
                        int user2 = _project.RecordList[j].SelectedUser;
                        TrackFrame trackImage2 = imagePairs[j];
                        if (!trackImage2.UserTrackings.ContainsKey(user2))
                            continue;

                        Dictionary<SkeletonJoint, Point3D> joints1 = trackImage1.GetValidJoints(user1);
                        Dictionary<SkeletonJoint, Point3D> joints2 = trackImage2.GetValidJoints(user2);

                        foreach (SkeletonJoint joint in joints1.Keys.Intersect(joints2.Keys))
                        {
                            Tuple<int, int> key = new Tuple<int, int>(i, j);
                            int count;
                            if (!cooccurenceCount.TryGetValue(key, out count))
                            {
                                count = 0;
                            }
                            cooccurenceCount[key] = count + 1;
                        }
                    }
                }
            }))
            {
                // 依存関係のツリーを作る
                int baseRecordIndex;
                Dictionary<int, int> dependencies;
                if (!CalcEx.GetDependencyTree(_project.RecordList.Count, cooccurenceCount, list => list.Sum(), out  baseRecordIndex, out dependencies))
                {
                    System.Windows.MessageBox.Show("骨格が他のレコードと同時に映っているフレームがないレコードがあるため計算できませんでした");
                }
                else
                {
                    Dictionary<int, ICoordConversion3D> conversionsPerDependencyKey = new Dictionary<int, ICoordConversion3D>();
                    if (runOperationForEachFrame("座標間の計算中...", true, true, (time, imagePairs) =>
                    {
                        foreach (KeyValuePair<int, int> dependencyPair in CalcEx.EnumerateDependencyPairs(baseRecordIndex, dependencies))
                        {

                            TrackFrame trackImage1 = imagePairs[dependencyPair.Key];
                            TrackFrame trackImage2 = imagePairs[dependencyPair.Value];
                            int user1 = _project.RecordList[dependencyPair.Key].SelectedUser;
                            int user2 = _project.RecordList[dependencyPair.Value].SelectedUser;

                            if (trackImage1 == null || !trackImage1.UserTrackings.ContainsKey(user1))
                                continue;
                            if (trackImage2 == null || !trackImage2.UserTrackings.ContainsKey(user2))
                                continue;

                            CvSize imageSize1 = trackImage1.DepthUserSize;
                            CvSize imageSize2 = trackImage2.DepthUserSize;
                            KinectUndistortion undist1 = _project.RecordList[dependencyPair.Key].UndistortionData;
                            KinectUndistortion undist2 = _project.RecordList[dependencyPair.Value].UndistortionData;

                            // 変換計算用オブジェクトを拾ってくる
                            ICoordConversion3D conv;
                            if (!conversionsPerDependencyKey.TryGetValue(dependencyPair.Key, out conv))
                            {
                                conversionsPerDependencyKey[dependencyPair.Key] = conv = new CoordRotTransConversion();
                            }

                            Dictionary<SkeletonJoint, Point3D> joints1 = trackImage1.GetValidJoints(user1);
                            Dictionary<SkeletonJoint, Point3D> joints2 = trackImage2.GetValidJoints(user2);

                            foreach (SkeletonJoint joint in joints1.Keys.Intersect(joints2.Keys))
                            {
                                CvPoint3D64f camPoint1 = undist1.GetRealFromScreenPos(joints1[joint].ToCvPoint3D(), imageSize1);
                                CvPoint3D64f camPoint2 = undist2.GetRealFromScreenPos(joints2[joint].ToCvPoint3D(), imageSize2);
                                // それぞれのカメラ座標系におけるそれぞれの対応点をセットに入れる
                                conv.PutPoint(camPoint1, camPoint2, 1);
                            }
                        }
                    }))
                    {
                        foreach (KeyValuePair<int, int> dependencyPair in CalcEx.EnumerateDependencyPairs(baseRecordIndex, dependencies))
                        {
                            CvMat relConv = conversionsPerDependencyKey[dependencyPair.Key].Solve();
                            CvMat baseConversion = _project.RecordList[dependencyPair.Value].ToWorldConversion;
                            _project.RecordList[dependencyPair.Key].ToWorldConversion = baseConversion * relConv;
                        }
                        foreach (TrackImageRecordProperty record in _project.RecordList)
                        {
                            record.IsPositionCalibrated = true;
                        }
                    }
                }
            }
        }
    }
}

