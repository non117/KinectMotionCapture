using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using OpenCvSharp;

namespace KinectMotionCapture
{
    class KinectMerge
    {
        // are
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
        }
        
        /// <summary>
        /// あるフレームにおける座標の補正行列を返す
        /// </summary>
        /// <param name="colorMatList"></param>
        /// <param name="depthMatList"></param>
        /// <param name="convList"></param>
        /// <returns></returns>
        public static List<CvMat> AjustDepthPoints(List<CvMat> colorMatList, List<CvMat> depthMatList, List<CvMat> convList)
        {
            List<Func<CvPoint3D64f, CvPoint3D64f>> toReal = new List<Func<CvPoint3D64f, CvPoint3D64f>>();
            foreach (CvMat depthMat in depthMatList)
            {
                toReal.Add((x) => KinectUndistortion.GetOriginalRealFromScreenPos(x, new CvSize(depthMat.Cols, depthMat.Rows)));
            }
            Func<float, double> distance2weight = x => 1.0 / (x * 0 + 400);
            using (ColoredIterativePointMatching sipm = new ColoredIterativePointMatching(depthMatList, colorMatList, toReal, convList, distance2weight, 200))
            {
                List<CvMat> conversions = sipm.CalculateTransformSequntially(0.2, 3);
                return conversions;
            }
        }

        // 深度マップから補正のフレーム版
        private void menuPolygonAll_Click(object sender, RoutedEventArgs e)
        {
            if (runOperationForEachFrameImage("深度マップからカメラ座標を補正しています...", true, true, (time, tracks) =>
            {
                if (tracks.All(x => x != null))
                {
                    if (!tracks.All(x => x.DepthMat != null))
                        return;
                    List<DateTime> times = tracks.Select((x, i) => x.Timestamp).ToList();
                    if (times.Max(x => x.Ticks) - times.Min(x => x.Ticks) > 10000000L / _project.Frequency / 2)
                    {
                        return;
                    }
                    List<CvMat> modelDepthMatList = new List<CvMat>();
                    List<CvMat> modelColorMatList = new List<CvMat>();
                    List<Func<CvPoint3D64f, CvPoint3D64f>> toReal = new List<Func<CvPoint3D64f, CvPoint3D64f>>();
                    List<CvMat> convList = new List<CvMat>();
                    for (int i = 0; i < _displayControls.Count; i++)
                    {
                        CvMat depthMat = tracks[i].DepthMat;
                        CvMat colorMat = tracks[i].ImageMat;
                        modelDepthMatList.Add(depthMat);
                        modelColorMatList.Add(colorMat);
                        toReal.Add((x) => KinectUndistortion.GetOriginalRealFromScreenPos(x, new CvSize(depthMat.Cols, depthMat.Rows)));
                        convList.Add(_project.RecordList[i].ToWorldConversion);
                    }
                    Func<float, double> distance2weight = x => 1.0 / (x * 0 + 400);
                    using (ColoredIterativePointMatching sipm = new ColoredIterativePointMatching(modelDepthMatList, modelColorMatList, toReal, convList, distance2weight, 200))
                    {
                        List<CvMat> conversions = sipm.CalculateTransformSequntially(0.1, 1);
                        for (int i = 0; i < conversions.Count; i++)
                        {
                            _project.RecordList[i].ToWorldConversion = conversions[i];
                            _project.RecordList[i].IsPositionCalibrated = true;
                        }
                    }
                }
            }))
            {
            }
        }

        // 骨のマージ
        private void buttonCalibUser_Click(object sender, RoutedEventArgs e)
        {
            if (_project.RecordList.Any(r => r.SelectedUser == -1))
            {
                System.Windows.MessageBox.Show("ユーザが選択されていないレコードがあります");
                return;
            }

            for (int j = 1; j < _project.RecordList.Count; j++)
            {
                using (LockedBuffer<TrackImageFrame> buf1 = _project.RecordList[0].RecordData.GetLockedImage(_timePlayer.CurrentTime))
                using (LockedBuffer<TrackImageFrame> buf2 = _project.RecordList[j].RecordData.GetLockedImage(_timePlayer.CurrentTime))
                {
                    int user1 = _project.RecordList[0].SelectedUser;
                    int user2 = _project.RecordList[j].SelectedUser;
                    TrackImageFrame trackImage1 = buf1.Data;
                    TrackImageFrame trackImage2 = buf2.Data;
                    if (!trackImage1.UserTrackings.ContainsKey(user1) || !trackImage2.UserTrackings.ContainsKey(user2))
                    {
                        System.Windows.MessageBox.Show("joint not found");
                        return;
                    }
                    UserTrackingState state1 = buf1.Data.UserTrackings[user1];
                    UserTrackingState state2 = buf2.Data.UserTrackings[user2];
                    ICoordConversion3D crtc = new CoordRotTransConversion();
                    foreach (OpenNI.SkeletonJoint joint in Enum.GetValues(typeof(OpenNI.SkeletonJoint)))
                    {
                        if (!state1.OriginalJoints.ContainsKey(joint))
                            continue;
                        if (!state2.OriginalJoints.ContainsKey(joint))
                            continue;
                        CvPoint3D64f from = state2.OriginalJoints[joint].Position.ToCvPoint3D();
                        CvPoint3D64f target = CvEx.ConvertPoint3D(state1.OriginalJoints[joint].Position.ToCvPoint3D(), _project.RecordList[0].ToWorldConversion);
                        bool valid1 = buf1.Data.IsOriginalJointValid(user1, joint);
                        bool valid2 = buf2.Data.IsOriginalJointValid(user2, joint);
                        if (valid1 && valid2)
                        {
                            crtc.PutPoint(from, target, 1);
                        }
                    }
                    _project.RecordList[j].ToWorldConversion = crtc.Solve();
                    _project.RecordList[j].IsPositionCalibrated = true;
                }
            }
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

