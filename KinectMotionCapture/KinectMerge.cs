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
        public List<LocalCoordinateMapper> LocalCoordinateMappers { get; set; }
        
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
                        this.convList.Add(CvMat.Identity(4, 4, MatrixType.F64C1));
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
        public void ApplyConversion()
        {
            foreach(Frame frame in Frames)
            {
                frame.ApplyConversions(this.ToWorldConversions);
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
            List<Tuple<List<DateTime>, int[]>> timeInfos = new List<Tuple<List<DateTime>, int[]>>();
            foreach (List<MotionData> record in records)
            {
                DateTime[] dateTimes = record.Select(m => m.TimeStamp).ToArray();
                int[] indexes = record.Select((m, i) => i).ToArray();
                Array.Sort(dateTimes, indexes);
                timeInfos.Add(Tuple.Create(dateTimes.ToList(), indexes));
            }

            for (DateTime time = this.startTime; time <= this.endTime; time += this.timePeriod)

            {
                // 同時刻のフレーム集合. Kinectの数だけ入るはず.
                List<MotionData> tempRecords = new List<MotionData>();
                for (int i = 0; i < this.recordNum;i++ )
                {
                    List<MotionData> record = records[i];
                    List<DateTime> dateTimes = timeInfos[i].Item1;
                    int[] indexes = timeInfos[i].Item2;
                    int frameIndex = ListEx.GetMaxLessEqualIndexFromBinarySearch(dateTimes.BinarySearch(time));
                    if (frameIndex < 0)
                    {
                        frameIndex = 0;
                    }
                    if (frameIndex >= record.Count())
                    {
                        frameIndex = record.Count() - 1;
                    }
                    tempRecords.Add(record[indexes[frameIndex]]);
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
        public List<Frame> Slice(DateTime start, DateTime end)
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
        public List<Frame> Slice(int startIndex, int endIndex)
        {
            DateTime startTime = this.Frames[startIndex].Time;
            DateTime endTime = this.Frames[endIndex].Time;
            return this.Slice(startTime, endTime);
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
                // ここでソートしてる
                List<MotionData> mdList = this.GetMotionDataFromFile(metaDataFilePath).OrderBy(md => md.TimeStamp).ToList();
                // 画像のパスを修正する
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
        public List<MotionData> records;  // for debug
        public DateTime Time { get; set; }

        public List<string> ColorImagePathList
        {
            get { return this.records.Select((m) => m.ImagePath).ToList(); }
        }

        public List<CvMat> ColorMatList
        {
            get { return this.records.Select((m) => CvMat.LoadImageM(m.ImagePath, LoadMode.Unchanged)).ToList(); }
        }
        
        public List<CvMat> DepthMatList
        {
            get { return this.records.Select((m) => CvMat.LoadImageM(m.DepthPath, LoadMode.Unchanged)).ToList(); }
        }

        public List<CvSize> ColorSize
        {
            get { return this.records.Select((m) => new CvSize(m.ColorWidth, m.ColorHeight)).ToList(); }
        }

        public List<CvSize> DepthUserSize
        {
            get { return this.records.Select((m) => new CvSize(m.DepthUserWidth, m.DepthUserHeight)).ToList(); }
        }

        /// <summary>
        /// レコード番号からBody Idをとってくる
        /// </summary>
        /// <param name="recordNo"></param>
        /// <returns></returns>
        public List<string> BodyIdList(int recordNo)
        {
            return new List<SerializableBody>(this.records[recordNo].bodies).Select((b) => b.TrackingId.ToString()).ToList();
        }

        /// <summary>
        /// レコード番号からカラー座標系の関節点群をとってくる
        /// </summary>
        /// <param name="recordNo"></param>
        /// <returns></returns>
        public List<Dictionary<JointType, Point>> GetBodyColorSpacePoints(int recordNo)
        {
            return new List<SerializableBody>(this.records[recordNo].bodies).Select((b) => b.colorSpacePoints).ToList();
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
                    SerializableBody body = this.records[i].bodies.Where((b) => b.TrackingId == selectedUserIdList[i]).FirstOrDefault();
                    
                    /// idが違うときの場合. 本来はセグメンテーションすべき.
                    if ( body == null || body.Equals(default(SerializableBody)))
                        return new List<SerializableBody>();
                    
                    bodies.Add(body);
                }
            return bodies;
        }

        /// <summary>
        /// こんすとらくたん
        /// </summary>
        /// <param name="records"></param>
        public Frame(List<MotionData> records)
        {
            this.recordNum = records.Count();
            this.records = records;
        }

        /// <summary>
        /// 座標変換をBodyに適用する
        /// TODO : return value required. 非破壊に変更
        /// </summary>
        /// <param name="conversion"></param>
        public void ApplyConversions(List<CvMat> conversions)
        {
            for (int i = 0; i < this.recordNum; i++)
            {
                MotionData md = this.records[i];
                CvMat conversion = conversions[i];
                foreach (SerializableBody body in md.bodies)
                {
                    Dictionary<JointType, Joint> newJoints = new Dictionary<JointType, Joint>();
                    foreach (JointType jointType in body.Joints.Keys)
                    {
                        Joint originalJoint = body.Joints[jointType];
                        CvPoint3D64f fromPoint = originalJoint.Position.ToCvPoint3D();
                        // debug

                        CameraSpacePoint newPoint = CvEx.ConvertPoint3D(fromPoint, conversion).ToCameraSpacePoint();
                        originalJoint.Position = newPoint;
                        newJoints[jointType] = originalJoint;
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
        public static List<CvMat> GetConvMatrixFromDepthFrame(Frame frame, List<CvMat> convList, List<LocalCoordinateMapper> localCoordinateMappers)
        {
            Func<float, double> distance2weight = x => 1.0 / (x * 0 + 400);
            using (ColoredIterativePointMatching sipm = new ColoredIterativePointMatching(frame.DepthMatList, frame.ColorMatList, localCoordinateMappers, convList, distance2weight, 200))
            {
                List<CvMat> conversions = sipm.CalculateTransformSequntially(0.2, 3);
                return conversions;
            }
        }

        /// <summary>
        /// フレーム範囲における座標変換行列を深度情報から計算する
        /// </summary>
        /// <param name="frames"></param>
        public static List<CvMat> GetConvMatrixFromDepthFrameSequence(FrameSequence frameSeq, int startIndex, int endIndex)
        {
            List<CvMat> conversions = frameSeq.ToWorldConversions;
            IEnumerable<Frame> frames = frameSeq.Frames.Skip(startIndex).Take(endIndex);
            foreach (Frame frame in frames)
            {
                Func<float, double> distance2weight = x => 1.0 / (x * 0 + 400);
                using (ColoredIterativePointMatching sipm = new ColoredIterativePointMatching(frame.DepthMatList, frame.ColorMatList, frameSeq.LocalCoordinateMappers, conversions, distance2weight, 200))
                {
                    conversions = sipm.CalculateTransformSequntially(0.1, 1);                    
                }
            }
            return conversions;

        }

        /// <summary>
        /// あるフレームにおける座標変換行列を骨格情報から計算する
        /// </summary>
        /// <param name="frame"></param>
        public static List<CvMat> GetConvMatrixFromBoneFrame(Frame frame, List<CvMat> convList, List<ulong> selectedUserIdList)
        {
            List<SerializableBody> bodies = frame.SelectedBodyList(selectedUserIdList);
            if ( bodies.Count() != frame.recordNum )
            {
                System.Windows.MessageBox.Show("ユーザが選択されていないレコードがあります");
                return convList;
            }

            for (int j = 1; j < frame.recordNum; j++)
            {
                Dictionary<JointType, Joint> joint1 = Utility.GetValidJoints(bodies[0].Joints);
                Dictionary<JointType, Joint> joint2 = Utility.GetValidJoints(bodies[j].Joints);

                ICoordConversion3D crtc = new CoordRotTransConversion();
                foreach (JointType jointType in Enum.GetValues(typeof(JointType)))
                {
                    if (!joint1.ContainsKey(jointType))
                        continue;
                    if (!joint2.ContainsKey(jointType))
                        continue;
                    CvPoint3D64f from = joint2[jointType].Position.ToCvPoint3D();
                    CvPoint3D64f target = CvEx.ConvertPoint3D(joint1[jointType].Position.ToCvPoint3D(), convList[0]);
                    // IsOriginlJointValid相当の処理を入れるかどうか
                    crtc.PutPoint(from, target, 1);
                }
                convList[j] = crtc.Solve();
            }
            return convList;
        }

        /// <summary>
        /// フレーム範囲における座標変換行列を骨格情報から計算する
        /// </summary>
        /// <param name="frames"></param>
        public static List<CvMat> GetConvMatrixFromBoneFrameSequence(FrameSequence frameSeq, int startIndex, int endIndex)
        {
            Dictionary<Tuple<int, int>, int> cooccurenceCount = new Dictionary<Tuple<int, int>, int>();
            //System.Windows.MessageBox.Show("ユーザが選択されていないレコードがあります");
            //return;
            IEnumerable<Frame> frames = frameSeq.Frames.Skip(startIndex).Take(endIndex);
            foreach (Frame frame in frames)
            {
                List<SerializableBody> bodies = frame.SelectedBodyList(frameSeq.selectedUserIdList);
                if (bodies.Count() != frame.recordNum)
                    continue;

                for (int i = 0; i < frame.recordNum; i++)
                {
                    for (int j = i + 1; j < frame.recordNum; j++)
                    {
                        Dictionary<JointType, Joint> joint1 = Utility.GetValidJoints(bodies[i].Joints);
                        Dictionary<JointType, Joint> joint2 = Utility.GetValidJoints(bodies[j].Joints);

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
                return frameSeq.ToWorldConversions;
            }
            else
            {
                Dictionary<int, ICoordConversion3D> conversionsPerDependencyKey = new Dictionary<int, ICoordConversion3D>();
                foreach (Frame frame in frameSeq.Frames)
                {
                    List<SerializableBody> bodies = frame.SelectedBodyList(frameSeq.selectedUserIdList);
                    if (bodies.Count() != frame.recordNum)
                        continue;

                    //List<KinectUndistortion> undistortions = frameSeq.UndistortionDataList;
                    List<CvSize> depthUsersizeList = frame.DepthUserSize;

                    foreach (KeyValuePair<int, int> dependencyPair in CalcEx.EnumerateDependencyPairs(baseRecordIndex, dependencies))
                    {
                        //CvSize imageSize1 = depthUsersizeList[dependencyPair.Key];
                        //CvSize imageSize2 = depthUsersizeList[dependencyPair.Value];
                        //KinectUndistortion undist1 = undistortions[dependencyPair.Key];
                        //KinectUndistortion undist2 = undistortions[dependencyPair.Value];

                        // 変換計算用オブジェクトを拾ってくる
                        ICoordConversion3D conv;
                        if (!conversionsPerDependencyKey.TryGetValue(dependencyPair.Key, out conv))
                        {
                            conversionsPerDependencyKey[dependencyPair.Key] = conv = new CoordRotTransConversion();
                        }

                        Dictionary<JointType, Joint> joint1 = Utility.GetValidJoints(bodies[dependencyPair.Key].Joints);
                        Dictionary<JointType, Joint> joint2 = Utility.GetValidJoints(bodies[dependencyPair.Value].Joints);

                        foreach (JointType jointType in joint1.Keys.Intersect(joint2.Keys))
                        {
                            //CvPoint3D64f camPoint1 = undist1.GetRealFromScreenPos(joint1[jointType].Position.ToCvPoint3D(), imageSize1);
                            //CvPoint3D64f camPoint2 = undist2.GetRealFromScreenPos(joint2[jointType].Position.ToCvPoint3D(), imageSize2);
                            CvPoint3D64f camPoint1 = joint1[jointType].Position.ToCvPoint3D();
                            CvPoint3D64f camPoint2 = joint2[jointType].Position.ToCvPoint3D();
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
                return convList;
            }
        }
    }
}

