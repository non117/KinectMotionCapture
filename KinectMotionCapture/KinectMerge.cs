using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;

using Microsoft.Kinect;
using MsgPack.Serialization;
using OpenCvSharp;

namespace KinectMotionCapture
{
    /// <summary>
    /// フレームの集合
    /// </summary>
    [Serializable]
    class FrameSequence
    {
        private List<CvMat> convList;
        private List<string> dataDirs;
        private TimeSpan timePeriod;

        public int recordNum;
        public double frameRate = 30;
        public DateTime startTime;
        public DateTime endTime;
        public List<ulong> selectedUserIdList;
        // TODO: IEnumerableにしても良さそう。イテレータブロックとか使うらしい。
        public List<List<ulong>> userIdList;
        public List<Frame> Frames { get; set; }
        public LocalCoordinateMapper LocalCoordinateMapper { get; set; }
        
        /// <summary>
        /// 座標系を統合するための変換行列、各レコードに対して
        /// </summary>
        public List<CvMat> ToWorldConversions
        {
            get
            {
                if (this.convList == null)
                {
                    this.convList = new List<CvMat>();
                    for (int i = 0; i < this.recordNum; i++)
                    {
                        this.convList[i] = CvMat.Identity(4, 4, MatrixType.F64C1);
                    }
                }
                return this.convList;
            }
            set
            {
                this.convList = value;
            }
        }

        /// <summary>
        /// 各レコードのキャリブレーションデータ
        /// </summary>
        public List<KinectUndistortion> UndistortionDataList
        {
            get;
            set;
        }        

        public void SetUserID(int recordIndex, ulong bodyId)
        {
            this.selectedUserIdList[recordIndex] = bodyId;
        }

        /// <summary>
        /// 座標変換を適用する
        /// </summary>
        private void ApplyConversion()
        {
            List<CvMat> conversions = this.ToWorldConversions;
            for (int i = 0; i < this.recordNum; i++)
            {
                this.Frames[i].ApplyConversions(conversions[i]);
            }
        }

        /// <summary>
        /// レコードのメタデータをデシリアライズしてくる
        /// </summary>
        /// <param name="filepath"></param>
        /// <returns></returns>
        private List<MotionData> GetMotionDataFromFile(string filepath)
        {
            var serializer = MessagePackSerializer.Get<List<MotionData>>();
            using (FileStream fs = File.Open(filepath, FileMode.Open))
            {
                return serializer.Unpack(fs);
            }
        }

        /// <summary>
        /// レコードを切り出してくるやつ. TODO: 長さ0のを返す場合
        /// </summary>
        /// <param name="records"></param>
        /// <param name="minTime"></param>
        /// <param name="maxTime"></param>
        /// <returns></returns>
        private List<List<MotionData>> SliceFrames(List<List<MotionData>> records, DateTime minTime, DateTime maxTime)
        {
            List<List<MotionData>> newRecords = new List<List<MotionData>>();
            foreach (List<MotionData> record in records)
            {
                List<MotionData> newRecord = record.Where((r) => minTime <= r.TimeStamp && r.TimeStamp <= maxTime).OrderBy(m => m.TimeStamp).ToList();
                newRecords.Add(newRecord);
            }
            return newRecords;
        }

        /// <summary>
        /// 重複した領域だけに区切る
        /// </summary>
        /// <param name="records"></param>
        /// <returns></returns>
        private List<List<MotionData>> SearchDupFrames(List<List<MotionData>> records)
        {
            DateTime minTime = DateTime.MinValue;
            DateTime maxTime = DateTime.MaxValue;
            foreach (List<MotionData> record in records)
            {
                if (record.First().TimeStamp > minTime)
                {
                    minTime = record.First().TimeStamp;
                }
                if (record.Last().TimeStamp < maxTime)
                {
                    maxTime = record.Last().TimeStamp;
                }
            }
            this.startTime = minTime;
            this.endTime = maxTime;
            return this.SliceFrames(records, minTime, maxTime);
        }

        /// <summary>
        /// 時刻を少しずつ進めながらフレームを作っていく
        /// </summary>
        /// <param name="records"></param>
        /// <returns></returns>
        private List<Frame> GenerateFrames(List<List<MotionData>> records)
        {
            List<Frame> frames = new List<Frame>();
            List<SortedList<DateTime, int>> timeInfos = new List<SortedList<DateTime, int>>();

            foreach (List<MotionData> record in records)
            {
                SortedList<DateTime, int> timeInfo = new SortedList<DateTime, int>();
                // 以下2つは時刻被り対策. マシンスペックが遅いとよく発生するので, 線形に補完する.
                DateTime prevTime = DateTime.MinValue;
                List<int> depIndexes = new List<int>();
                
                foreach (var dataSet in record.Select((value, index) => new { value, index }))
                {
                    // キー:時刻がかぶってたらためておく
                    if (timeInfo.Keys.Contains(dataSet.value.TimeStamp))
                    {
                        depIndexes.Add(dataSet.index);
                        continue;
                    }
                    else
                    {
                        timeInfo.Add(dataSet.value.TimeStamp, dataSet.index);
                        // 時刻がちゃんと進んだら貯まってたデータをならして記録する
                        /*
                        if (dataSet.value.TimeStamp != prevTime && depIndexes.Count > 0)
                        {
                            TimeSpan diff = dataSet.value.TimeStamp - prevTime;
                            for (int i = 1; i < depIndexes.Count(); i++)
                            {
                                int msec = diff.Milliseconds * (i + 1) / (depIndexes.Count() + 1);
                                if (msec > 1)
                                {
                                    DateTime time = prevTime + new TimeSpan(0, 0, 0, 0, msec);
                                    // それでもかぶったら諦める
                                    if (!timeInfo.Keys.Contains(time))
                                        continue;
                                        //timeInfo.Add(time, i);
                                }
                            }
                            // ちゃんとたまったデータを消す
                            depIndexes.Clear();

                        }
                         */
                    }
                    prevTime = dataSet.value.TimeStamp;
                }
                timeInfos.Add(timeInfo);
            }

            for (DateTime time = this.startTime; time <= this.endTime; time += this.timePeriod)

            {
                // 同時刻のフレーム集合. Kinectの数だけ入るはず.
                List<MotionData> tempRecords = new List<MotionData>();
                for (int i = 0; i < this.recordNum;i++ )
                {
                    List<MotionData> record = records[i];
                    SortedList<DateTime, int> timeInfo = timeInfos[i];
                    int frameIndex = ListEx.GetMaxLessEqualIndexFromBinarySearch(timeInfo.Keys.BinarySearch(time));
                    if (frameIndex < 0)
                    {
                        frameIndex = 0;
                    }
                    if (frameIndex >= record.Count())
                    {
                        frameIndex = record.Count() - 1;
                    }
                    tempRecords.Add(record[frameIndex]);
                }
                Frame frame = new Frame(tempRecords);
                frame.Time = time;
                frames.Add(frame);
            }
            return frames;
        }

        /// <summary>
        /// 時間を切り出す
        /// </summary>
        /// <param name="start"></param>
        /// <param name="end"></param>
        public List<Frame> ResetTimeSpan(DateTime start, DateTime end)
        {
            this.startTime = start;
            this.endTime = end;
            return this.Frames.Where((r) => start <= r.Time && r.Time <= end).ToList();
        }

        /// <summary>
        /// 時間を切り出す
        /// </summary>
        /// <param name="start"></param>
        /// <param name="end"></param>
        public List<Frame> ResetTimeSpan(int startIndex, int endIndex)
        {
            DateTime startTime = this.Frames[startIndex].Time;
            DateTime endTime = this.Frames[endIndex].Time;
            return this.ResetTimeSpan(startTime, endTime);
        }

        /// <summary>
        /// コンストラクタ
        /// </summary>
        /// <param name="dataDirs"></param>
        public FrameSequence(List<string> dataDirs)
        {
            // TODO 例外処理
            this.recordNum = dataDirs.Count();
            this.selectedUserIdList = new List<ulong>();
            for (int i = 0; i < this.recordNum; i++)
            {
                this.selectedUserIdList.Add(ulong.MaxValue);
            }
            
            this.dataDirs = dataDirs;
            // 外側がKinectの数だけあるレコード、内側がフレーム数分
            List<List<MotionData>> records = new List<List<MotionData>>();
            foreach (string dataDir in dataDirs)
            {
                string metaDataFilePath = Path.Combine(dataDir, "BodyInfo.mpac");
                List<MotionData> mdList = this.GetMotionDataFromFile(metaDataFilePath).OrderBy(md => md.TimeStamp).ToList();
                foreach (MotionData md in mdList)
                {
                    md.ReConstructPaths(dataDir);
                }
                records.Add(mdList);
            }
            records = this.SearchDupFrames(records);

            // いちばん短いレコードに合わせて単位時刻を決定する
            int shortestRecordLength = records.Select((List<MotionData> record) => record.Count()).Min();
            this.timePeriod = new TimeSpan((this.endTime - this.startTime).Ticks / shortestRecordLength);

            this.Frames = this.GenerateFrames(records);

            // レコードごとに含まれるidを列挙する
            this.userIdList = new List<List<ulong>>();
            foreach (List<MotionData> record in records)
            {
                List<ulong> idList = new List<ulong>();
                foreach(MotionData md in record)
                {
                    idList.AddRange(new List<SerializableBody>(md.bodies).Select((SerializableBody body) => body.TrackingId));
                }
                idList = idList.Distinct().ToList();
                this.userIdList.Add(idList);
            }
        }

    }

    /// <summary>
    /// あるフレームにおける複数レコードをまとめて管理するクラス
    /// </summary>
    [Serializable]
    class Frame
    {
        public int recordNum;
        //private List<MotionData> records;
        public List<MotionData> records;
        public DateTime Time { get; set; }

        public List<string> ColorImagePathList
        {
            get { return this.records.Select((m) => m.ImagePath).ToList(); }
        }

        public List<CvMat> ColorMatList
        {
            get { return this.records.Select((m) => m.imageMat).ToList(); }
        }
        
        public List<CvMat> DepthMatList
        {
            get { return this.records.Select((m) => m.depthMat).ToList(); }
        }

        public List<CvSize> ColorSize
        {
            get { return this.records.Select((m) => new CvSize(m.ColorWidth, m.ColorHeight)).ToList(); }
        }

        public List<CvSize> DepthUserSize
        {
            get { return this.records.Select((m) => new CvSize(m.DepthUserWidth, m.DepthUserHeight)).ToList(); }
        }

        public List<string> BodyIdList(int recordIndex)
        {
            return new List<SerializableBody>(this.records[recordIndex].bodies).Select((b) => b.TrackingId.ToString()).ToList();
        }

        public List<Point> BodyCenterPointList(int recordIndex)
        {
            try
            {
                return new List<SerializableBody>(this.records[recordIndex].bodies).Select((b) => b.colorSpacePoints[JointType.SpineBase]).ToList();
            }
            catch (Exception e)
            {
                return new List<Point>();
            }
        }

        public List<Dictionary<JointType, Point>> GetBodyPoints(int recordIndex)
        {
            return new List<SerializableBody>(this.records[recordIndex].bodies).Select((b) => b.colorSpacePoints).ToList();
        }

        /// <summary>
        /// idの一致するBodyを返す
        /// </summary>
        /// <param name="selectedUserIdList"></param>
        /// <returns></returns>
        public List<SerializableBody> SelectedBodyList(List<ulong> selectedUserIdList)
        {
            List<SerializableBody> bodies = new List<SerializableBody>();
            for (int i = 0; i < this.recordNum; i++)
            {
                // あとでなおす
                SerializableBody body = this.records[i].bodies.Where((b) => b.TrackingId == selectedUserIdList[i]).First();
                bodies.Add(body);
            }
            return bodies;
        }

        public Frame(List<MotionData> records)
        {
            this.recordNum = records.Count();
            this.records = records;
        }

        /// <summary>
        /// 座標変換をBodyに適用する
        /// </summary>
        /// <param name="conversion"></param>
        public void ApplyConversions(CvMat conversion)
        {
            foreach (MotionData md in this.records)
            {                
                foreach (SerializableBody body in md.bodies)
                {
                    Dictionary<JointType, Joint> newJoints = new Dictionary<JointType, Joint>();
                    foreach (JointType jointType in body.Joints.Keys)
                    {
                        Joint originalJoint = body.Joints[jointType];
                        CvPoint3D64f fromPoint = originalJoint.Position.ToCvPoint3D();
                        CameraSpacePoint newPoint = CvEx.ConvertPoint3D(fromPoint, conversion).ToCameraSpacePoint();
                        originalJoint.Position = newPoint;
                        //Joint newJoint = originalJoint.CloneDeep();
                        //newJoint.Position = newPoint;
                        //newJoints[jointType] = newJoint;
                    }
                    body.Joints = newJoints;
                }
            }
        }
    }

    class KinectMerge
    {        
        /// <summary>
        /// あるフレームにおける座標変換行列を深度情報から計算する
        /// </summary>
        /// <param name="frame"></param>
        public static List<CvMat> AjustFrameFromDepth(Frame frame, List<CvMat> convList, LocalCoordinateMapper localCoordinateMapper)
        {
            Func<float, double> distance2weight = x => 1.0 / (x * 0 + 400);
            using (ColoredIterativePointMatching sipm = new ColoredIterativePointMatching(frame.DepthMatList, frame.ColorMatList, localCoordinateMapper, convList, distance2weight, 200))
            {
                List<CvMat> conversions = sipm.CalculateTransformSequntially(0.2, 3);
                return conversions;
            }
        }

        /// <summary>
        /// フレーム範囲における座標変換行列を深度情報から計算する
        /// </summary>
        /// <param name="frames"></param>
        public static void AjustFramesFromDepth(FrameSequence frameSeq, int startIndex, int endIndex)
        {
            IEnumerable<Frame> frames = frameSeq.Frames.Skip(startIndex).Take(endIndex);
            foreach (Frame frame in frames)
            {
                Func<float, double> distance2weight = x => 1.0 / (x * 0 + 400);
                using (ColoredIterativePointMatching sipm = new ColoredIterativePointMatching(frame.DepthMatList, frame.ColorMatList, frameSeq.LocalCoordinateMapper, frameSeq.ToWorldConversions, distance2weight, 200))
                {
                    List<CvMat> conversions = sipm.CalculateTransformSequntially(0.1, 1);
                    frameSeq.ToWorldConversions = conversions;
                }
            }

        }

        /// <summary>
        /// あるフレームにおける座標変換行列を骨格情報から計算する
        /// </summary>
        /// <param name="frame"></param>
        public List<CvMat> AjustFrameFromeBone(Frame frame, List<CvMat> convList, List<ulong> selectedUserIdList)
        {
            List<SerializableBody> bodies = frame.SelectedBodyList(selectedUserIdList);
            if ( bodies.Count() != frame.recordNum )
            {
                System.Windows.MessageBox.Show("ユーザが選択されていないレコードがあります");
                return convList;
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
            return convList;
        }

        /// <summary>
        /// フレーム範囲における座標変換行列を骨格情報から計算する
        /// </summary>
        /// <param name="frames"></param>
        public void AjustFramesFromBone(FrameSequence frameSeq, int startIndex, int endIndex)
        {
            Dictionary<Tuple<int, int>, int> cooccurenceCount = new Dictionary<Tuple<int, int>, int>();
            //System.Windows.MessageBox.Show("ユーザが選択されていないレコードがあります");
            //return;
            IEnumerable<Frame> frames = frameSeq.Frames.Skip(startIndex).Take(endIndex);
            foreach (Frame frame in frames)
            {
                List<SerializableBody> bodies = frame.SelectedBodyList(frameSeq.selectedUserIdList);
                for (int i = 0; i < frame.recordNum; i++)
                {;
                    for (int j = i + 1; j < frame.recordNum; j++)
                    {
                        Dictionary<JointType, Joint> joint1 = bodies[i].Joints;
                        Dictionary<JointType, Joint> joint2 = bodies[j].Joints;

                        foreach (JointType jointType in joint1.Keys.Intersect(joint2.Keys))
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
            }//))
            //{
            // 依存関係のツリーを作る
            int baseRecordIndex;
            Dictionary<int, int> dependencies;
            //if (!CalcEx.GetDependencyTree(_project.RecordList.Count, cooccurenceCount, list => list.Sum(), out  baseRecordIndex, out dependencies))
            // とりあえず先頭フレームのレコード数にしてるけど、プロジェクトとかが持つべき値
            if (!CalcEx.GetDependencyTree(frameSeq.recordNum, cooccurenceCount, list => list.Sum(), out  baseRecordIndex, out dependencies))
            {
                System.Windows.MessageBox.Show("骨格が他のレコードと同時に映っているフレームがないレコードがあるため計算できませんでした");
            }
            else
            {
                Dictionary<int, ICoordConversion3D> conversionsPerDependencyKey = new Dictionary<int, ICoordConversion3D>();
                foreach (Frame frame in frameSeq.Frames)
                {
                    List<SerializableBody> bodies = frame.SelectedBodyList(frameSeq.selectedUserIdList);
                    List<KinectUndistortion> undistortions = frameSeq.UndistortionDataList;
                    List<CvSize> depthUsersizeList = frame.DepthUserSize;

                    foreach (KeyValuePair<int, int> dependencyPair in CalcEx.EnumerateDependencyPairs(baseRecordIndex, dependencies))
                    {
                        CvSize imageSize1 = depthUsersizeList[dependencyPair.Key];
                        CvSize imageSize2 = depthUsersizeList[dependencyPair.Value];
                        KinectUndistortion undist1 = undistortions[dependencyPair.Key];
                        KinectUndistortion undist2 = undistortions[dependencyPair.Value];

                        // 変換計算用オブジェクトを拾ってくる
                        ICoordConversion3D conv;
                        if (!conversionsPerDependencyKey.TryGetValue(dependencyPair.Key, out conv))
                        {
                            conversionsPerDependencyKey[dependencyPair.Key] = conv = new CoordRotTransConversion();
                        }

                        Dictionary<JointType, Joint> joint1 = bodies[dependencyPair.Key].Joints;
                        Dictionary<JointType, Joint> joint2 = bodies[dependencyPair.Value].Joints;

                        foreach (JointType jointType in joint1.Keys.Intersect(joint2.Keys))
                        {
                            CvPoint3D64f camPoint1 = undist1.GetRealFromScreenPos(joint1[jointType].Position.ToCvPoint3D(), imageSize1);
                            CvPoint3D64f camPoint2 = undist2.GetRealFromScreenPos(joint2[jointType].Position.ToCvPoint3D(), imageSize2);
                            // それぞれのカメラ座標系におけるそれぞれの対応点をセットに入れる
                            conv.PutPoint(camPoint1, camPoint2, 1);
                        }
                    }
                }
                List<CvMat> convList = frameSeq.ToWorldConversions;
                foreach (KeyValuePair<int, int> dependencyPair in CalcEx.EnumerateDependencyPairs(baseRecordIndex, dependencies))
                {
                    CvMat relConv = conversionsPerDependencyKey[dependencyPair.Key].Solve();
                    CvMat baseConversion = convList[dependencyPair.Value];
                    convList[dependencyPair.Key] = baseConversion * relConv;
                }
                frameSeq.ToWorldConversions = convList;
            }
        }
    }
}

