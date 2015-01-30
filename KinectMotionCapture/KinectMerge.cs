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
    /// 信頼できるデータのメタ情報
    /// </summary>
    public struct TrustData
    {
        public int frameIndex;
        public int recordIndex;
        public int integratedBodyId;
        public TrustData(int fIndex, int rIndex, int bodyId)
        {
            this.frameIndex = fIndex;
            this.recordIndex = rIndex;
            this.integratedBodyId = bodyId;
        }

        /// <summary>
        /// 該当のBodyを返す
        /// </summary>
        /// <param name="frames"></param>
        /// <returns></returns>
        public SerializableBody GetBody(List<Frame> frames)
        {
            return frames[this.frameIndex].GetSelectedBody(this.recordIndex, integratedId: this.integratedBodyId);
        }
    }

    /// <summary>
    /// フレームの集合
    /// </summary>
    [Serializable]
    public class FrameSequence
    {
        private List<CvMat> convList;
        private string[] dataDirs;
        //private string bodyInfoFilename = @"BodyInfo.mpac";
        private string bodyInfoFilename = @"BodyInfo.dump";
        private TimeSpan timePeriod;
        private List<UserSegmentation> segms;
        private List<List<MotionData>> originalRecords;
        private List<Tuple<List<DateTime>, int[]>> timeInfos;

        public int recordNum;
        public double frameRate = 30;
        public DateTime startTime;
        public DateTime endTime;
        public ulong[] selectedOriginalIdList;
        public int[] selecteedIntegretedIdList;
        public List<List<ulong>> userIdList;
        public List<TrustData> trustData;

        public double[] offsets = new double[] { 0, 0, 0, 0, 0};
        
        // TODO: IEnumerableにしても良さそう。イテレータブロックとか使うらしい。        
        public List<Frame> Frames { get; set; }
        public List<LocalCoordinateMapper> LocalCoordinateMappers { get; set; }
        public List<CameraIntrinsics> CameraInfo { get; set; }
        public BodyStatistics BodyStat { get; set; }
        
        public List<UserSegmentation> Segmentations
        {
            get
            {
                return this.segms;
            }
            set
            {
                this.segms = value;
                Dictionary<ulong, int> mapping = this.UserMapping;
                foreach (Frame frame in this.Frames)
                {
                    frame.SetIntegratedId(mapping);
                }
            }
        }

        /// <summary>
        /// オリジナルのTrackingIdとセグメンテーションによって振り直したidの対応
        /// </summary>
        public Dictionary<ulong, int> UserMapping
        {
            get
            {
                Dictionary<ulong, int> mapping = new Dictionary<ulong, int>();
                Dictionary<ulong, int> conversion;
                if (this.segms != null)
                {
                    foreach (UserSegmentation us in this.segms)
                    {
                        conversion = us.Conversions.Last().Value;
                        foreach (ulong longId in conversion.Keys)
                        {
                            mapping[longId] = conversion[longId];
                        }
                    }
                }
                return mapping;
            }
        }

        /// <summary>
        /// 新しいIDを作る
        /// </summary>
        public void CreateNewIds()
        {
            int newId = this.UserMapping.Values.Max() + 1;
            foreach (UserSegmentation usm in this.Segmentations)
            {
                Dictionary<ulong, int> mapping = usm.Conversions.Last().Value;
                Dictionary<ulong, int> newMapping = mapping.CloneDeep();
                foreach (int destId in this.selecteedIntegretedIdList)
                {
                    foreach (var pair in mapping)
                    {
                        if (pair.Value == destId)
                        {
                            ulong destOriginal = pair.Key;
                            newMapping[destOriginal] = newId;
                        }
                    }
                }
                usm.AddNewConversion(newMapping);
            }
        }
        
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
            this.selectedOriginalIdList[recordIndex] = bodyId;
        }

        public void setIntegratedID(int recordIndex, int id)
        {
            this.selecteedIntegretedIdList[recordIndex] = id;
        }

        /// <summary>
        /// レコードのメタデータをデシリアライズしてくる
        /// </summary>
        /// <param name="filepath"></param>
        /// <returns></returns>
        private List<MotionData> GetMotionDataFromFile(string filepath)
        {
            string ext = Path.GetExtension(filepath);
            if (ext == ".mpac")
            {
                var serializer = MessagePackSerializer.Get<List<MotionData>>();
                using (FileStream fs = File.Open(filepath, FileMode.Open))
                {
                    return serializer.Unpack(fs);
                }
            }
            else if (ext == ".dump")
            {
                return Utility.LoadFromSequentialBinary(filepath).Select(o => (MotionData)o).ToList();
            }
            return new List<MotionData>();
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
        /// オフセットを適用する
        /// </summary>
        /// <param name="records"></param>
        /// <returns></returns>
        private List<List<MotionData>> ApplyOffset(List<List<MotionData>> records)
        {
            for (int recordNo = 0; recordNo < recordNum; recordNo++)
            {
                foreach (MotionData md in records[recordNo])
                {
                    TimeSpan offset = new TimeSpan(0, 0, 0, (int)Math.Truncate(offsets[recordNo]), (int)(offsets[recordNo] % 1.0));
                    md.TimeStamp += offset;
                }
            }
            return records;
        }

        /// <summary>
        /// 時刻を少しずつ進めながらフレームを作っていく
        /// </summary>
        /// <param name="records"></param>
        /// <returns></returns>
        private List<Frame> GenerateFrames()
        {
            List<Frame> frames = new List<Frame>();
            List<Tuple<List<DateTime>, int[]>> timeInfos = new List<Tuple<List<DateTime>, int[]>>();
            foreach (List<MotionData> record in this.originalRecords)
            {
                DateTime[] dateTimes = record.Select(m => m.TimeStamp).ToArray();
                int[] indexes = record.Select((m, i) => i).ToArray();
                Array.Sort(dateTimes, indexes);
                timeInfos.Add(Tuple.Create(dateTimes.ToList(), indexes));
            }
            this.timeInfos = timeInfos;

            for (DateTime time = this.startTime; time <= this.endTime; time += this.timePeriod)

            {
                // 同時刻のフレーム集合. Kinectの数だけ入るはず.
                List<MotionData> currentRecords = new List<MotionData>();
                List<MotionData> nextRecords = new List<MotionData>();
                for (int i = 0; i < this.recordNum; i++)
                {
                    List<MotionData> record = this.originalRecords[i];
                    List<DateTime> dateTimes = timeInfos[i].Item1;
                    int[] indexes = timeInfos[i].Item2;
                    int frameIndex = ListEx.GetMaxLessEqualIndexFromBinarySearch(dateTimes.BinarySearch(time));
                    int nextIndex = ListEx.GetMinGreaterEqualIndexFromBinarySearch(dateTimes.BinarySearch(time));
                    if (frameIndex < 0)
                        frameIndex = 0;
                    if (frameIndex >= record.Count())
                        frameIndex = record.Count() - 1;
                    if (nextIndex < 0)
                        nextIndex = 0;
                    if (nextIndex >= record.Count())
                        nextIndex = record.Count() - 1;
                    currentRecords.Add(record[indexes[frameIndex]]);
                    nextRecords.Add(record[indexes[nextIndex]]);
                }
                Frame frame = new Frame(currentRecords, nextRecords);
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
        /// あるレコードのMotionDataを取ってくる. 重複有りの時系列
        /// </summary>
        /// <param name="recordNo"></param>
        /// <returns></returns>
        public IEnumerable<MotionData> GetMotionDataSequence(int recordNo)
        {
            return this.Frames.Select(f => f.GetMotionData(recordNo));
        }

        /// <summary>
        /// ある時刻の近傍で前のMotionDataを取ってくる
        /// </summary>
        /// <param name="recordNo"></param>
        /// <param name="time"></param>
        /// <returns></returns>
        public MotionData GetPrevData(int recordNo, DateTime time)
        {
            List<MotionData> record = this.originalRecords[recordNo];
            List<DateTime> dateTimes = timeInfos[recordNo].Item1;
            int[] indexes = timeInfos[recordNo].Item2;
            int prevIndex = ListEx.GetMaxLessEqualIndexFromBinarySearch(dateTimes.BinarySearch(time));
            if (prevIndex == -1)
                return null;
            return record[indexes[prevIndex]];
        }

        /// <summary>
        /// ある時刻の近傍で次のMotionDataを取ってくる
        /// </summary>
        /// <param name="recordNo"></param>
        /// <param name="time"></param>
        /// <returns></returns>
        public MotionData GetNextData(int recordNo, DateTime time)
        {
            List<MotionData> record = this.originalRecords[recordNo];
            List<DateTime> dateTimes = timeInfos[recordNo].Item1;
            int[] indexes = timeInfos[recordNo].Item2;
            int nextIndex = ListEx.GetMinGreaterEqualIndexFromBinarySearch(dateTimes.BinarySearch(time));
            if (nextIndex == indexes.Count())
                return null;
            return record[indexes[nextIndex]];
        }

        /// <summary>
        /// コンストラクタ
        /// </summary>
        /// <param name="dataDirs"></param>
        public FrameSequence(string[] dataDirs)
        {
            // TODO 例外処理
            this.recordNum = dataDirs.Count();
            this.selectedOriginalIdList = new ulong[this.recordNum];
            this.selecteedIntegretedIdList = new int[this.recordNum];
            this.trustData = new List<TrustData>();
            
            this.dataDirs = dataDirs;
            // 外側がKinectの数だけあるレコード、内側がフレーム数分
            List<List<MotionData>> records = new List<List<MotionData>>();
            foreach (string dataDir in dataDirs)
            {
                string metaDataFilePath = Path.Combine(dataDir, this.bodyInfoFilename);
                // ここでソートしてる
                List<MotionData> mdList = this.GetMotionDataFromFile(metaDataFilePath).OrderBy(md => md.TimeStamp).ToList();
                // 画像のパスを修正する
                foreach (MotionData md in mdList)
                {
                    md.ReConstructPaths(dataDir);
                }
                records.Add(mdList);
            }
            records = this.ApplyOffset(records);
            records = this.SearchDupFrames(records);
            this.originalRecords = records;

            // いちばん短いレコードに合わせて単位時刻を決定する
            int shortestRecordLength = records.Select((List<MotionData> record) => record.Count()).Min();
            this.timePeriod = new TimeSpan((this.endTime - this.startTime).Ticks / shortestRecordLength);

            this.Frames = this.GenerateFrames();

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
    public class Frame
    {
        public int recordNum;
        private List<MotionData> records;
        private List<MotionData> nextRecords;
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

        public List<CvMat> UserMatList
        {
            get { return this.records.Select((m) => CvMat.LoadImageM(m.UserPath, LoadMode.Unchanged)).ToList(); }
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
        /// レコード番号からMotionDataをとってくる
        /// </summary>
        /// <param name="recordNo"></param>
        /// <returns></returns>
        public MotionData GetMotionData(int recordNo)
        {
            return this.records[recordNo];
        }

        /// <summary>
        /// レコード番号から次のフレームのMotionDataをとってくる
        /// </summary>
        /// <param name="recordNo"></param>
        /// <returns></returns>
        public MotionData GetNextMotionData(int recordNo)
        {
            return this.nextRecords[recordNo];
        }

        /// <summary>
        /// レコード番号からBody Idをとってくる
        /// </summary>
        /// <param name="recordNo"></param>
        /// <returns></returns>
        public List<string> GetBodyIdList(int recordNo)
        {
            return new List<SerializableBody>(this.records[recordNo].bodies).Select((b) => b.TrackingId.ToString()).ToList();
        }

        /// <summary>
        /// レコード番号からBodyをとってくる
        /// </summary>
        /// <param name="recordNo"></param>
        /// <returns></returns>
        public List<SerializableBody> GetBodyList(int recordNo)
        {
            return this.records[recordNo].bodies.ToList();
        }

        /// <summary>
        /// レコード番号からカラー座標系の関節点群をとってくる
        /// </summary>
        /// <param name="recordNo"></param>
        /// <returns></returns>
        public List<Dictionary<JointType, Point>> GetBodyColorSpaceJoints(int recordNo)
        {
            SerializableBody[] bodies = this.records[recordNo].bodies;
            return bodies.Select(b => b.colorSpacePoints).ToList();
        }

        /// <summary>
        /// レコード番号からJointsをとってくる
        /// </summary>
        /// <param name="recordNo"></param>
        /// <returns></returns>
        public List<Dictionary<JointType, Joint>> GetBodyJoints(int recordNo)
        {
            return this.records[recordNo].bodies.ToList().Select((b) => b.Joints).ToList();
        }

        /// <summary>
        /// Bodyのidと代表点の座標を返す
        /// </summary>
        /// <param name="recordNo"></param>
        /// <returns></returns>
        public List<Tuple<ulong, Point>> GetIdAndPosition(int recordNo)
        {
            MotionData md = this.records[recordNo];
            List<Tuple<ulong, Point>> ret = new List<Tuple<ulong, Point>>();
            foreach (SerializableBody body in md.bodies)
            {
                if (body.colorSpacePoints != null && body.colorSpacePoints.Count != 0)
                {
                    ret.Add(Tuple.Create(body.TrackingId, Utility.GetAveragePoint(body.colorSpacePoints.Values)));
                }
                else
                {
                    ret.Add(null);
                }
            }
            return ret;
        }

        /// <summary>
        /// Bodyデータに新しい統合IDをセットする
        /// </summary>
        /// <param name="mapping"></param>
        public void SetIntegratedId(Dictionary<ulong, int> mapping)
        {
            foreach (MotionData md in this.records)
            {
                foreach (SerializableBody body in md.bodies)
                {
                    int integratedId = mapping[body.TrackingId];
                    body.integratedId = integratedId;
                }
            }
            foreach (MotionData md in this.nextRecords)
            {
                foreach (SerializableBody body in md.bodies)
                {
                    // 何故かキーが落ちた
                    try
                    {
                        int integratedId = mapping[body.TrackingId];
                        body.integratedId = integratedId;
                    }
                    catch (Exception e)
                    {
                        Debug.WriteLine("why???");
                    }
                }
            }
        }

        /// <summary>
        /// idの一致するBodyを返す
        /// </summary>
        /// <param name="recordNo"></param>
        /// <param name="integratedId"></param>
        /// <param name="originalId"></param>
        /// <returns></returns>
        public SerializableBody GetSelectedBody(int recordNo, int integratedId = -1, ulong originalId = 0)
        {
            SerializableBody body = null;
            if (integratedId != -1)
            {
                body = this.records[recordNo].bodies.Where((b) => b.integratedId == integratedId).FirstOrDefault();
            }
            else if (originalId != 0)
            {
                body = this.records[recordNo].bodies.Where((b) => b.TrackingId == originalId).FirstOrDefault();
            }           
            /// idが違うときの場合. 本来はセグメンテーションすべき.
            if (body == null || body.Equals(default(SerializableBody)))
                return null;
            return body;
        }

        /// <summary>
        /// idの一致するBody[]を返す
        /// </summary>
        /// <param name="selectedUserIdList"></param>
        /// <returns></returns>
        public List<SerializableBody> GetSelectedBodyList(int[] integratedIds = null, ulong[] originalIds = null)
        {
            List<SerializableBody> bodies = new List<SerializableBody>();
                for (int recordNo = 0; recordNo < this.recordNum; recordNo++)
                {
                    SerializableBody body = null;
                    if (integratedIds != null && integratedIds.Count() == this.recordNum)
                    {
                        body = this.GetSelectedBody(recordNo, integratedId:integratedIds[recordNo]);
                    }
                    else if (originalIds != null && originalIds.Count() == this.recordNum)
                    {
                        body = this.GetSelectedBody(recordNo, originalId: originalIds[recordNo]);
                    }                                        
                    if ( body == null)
                        return new List<SerializableBody>();                    
                    bodies.Add(body);
                }
            return bodies;
        }

        /// <summary>
        /// Bodyが0個以上各レコードに詰まっているかどうか
        /// </summary>
        /// <returns></returns>
        public bool IsAllBodyAvailable()
        {
            return this.records.All(md => md.bodies.Length != 0);
        }

        /// <summary>
        /// validフラグをまとめて返すやつ
        /// </summary>
        /// <returns></returns>
        public bool[] GetValidFlags()
        {
            return this.records.Select(md => md.isValid).ToArray();
        }

        /// <summary>
        /// Bodyの左右を反転する
        /// </summary>
        /// <param name="recordNo"></param>
        /// <param name="UserIds"></param>
        public void InverseBody(int recordNo, int integratedId = -1, ulong originalId = 0)
        {
            MotionData md = this.records[recordNo];
            foreach (SerializableBody body in md.bodies)
            {
                if (integratedId != -1 && body.integratedId == integratedId)
                {
                    body.InverseJoints();
                    body.mirrored = !body.mirrored;
                }
                else if (originalId != 0 && body.TrackingId == originalId)
                {
                    body.InverseJoints();
                    body.mirrored = !body.mirrored;
                }
            }
        }

        public void DeleteBody(int recordNo, int integratedId = -1, ulong originalId = 0)
        {
            MotionData md = this.records[recordNo];
            for (int i = 0; i < md.bodies.Length; i++)
            {
                SerializableBody body = md.bodies[i];
                if (integratedId != -1 && body.integratedId == integratedId)
                {
                    Array.Clear(md.bodies, i, 1);
                }
                else if (originalId != 0 && body.TrackingId == originalId)
                {
                    Array.Clear(md.bodies, i, 1);
                }
            }
            md.bodies = md.bodies.Where(b => b != null).ToArray();
        }

        /// <summary>
        /// 全てのBodyの反転状態を初期化する
        /// </summary>
        public void ResetInversedBody()
        {
            foreach (MotionData md in this.records)
            {
                foreach (SerializableBody body in md.bodies)
                {
                    if (body.mirrored)
                    {
                        body.InverseJoints();
                    }
                }
            }
        }

        /// <summary>
        /// recordNoのデータを使わないように設定する
        /// </summary>
        /// <param name="recordNo"></param>
        public void SetDataNotValid(int recordNo)
        {
            MotionData md = this.records[recordNo];
            md.isValid = false;
        }

        /// <summary>
        /// validフラグをリセットする
        /// </summary>
        public void ResetAllValidFlags()
        {
            foreach (MotionData md in this.records)
            {
                md.isValid = true;
            }
        }

        /// <summary>
        /// こんすとらくたん
        /// </summary>
        /// <param name="records"></param>
        public Frame(List<MotionData> records, List<MotionData> nextrecords)
        {
            this.recordNum = records.Count();
            this.records = records;
            this.nextRecords = nextrecords;
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
            Func<float, double> distance2weight = x => 1.0 / (x * 0 + 400 / 1000f);
            using (ColoredIterativePointMatching sipm = new ColoredIterativePointMatching(frame, localCoordinateMappers, convList, distance2weight, 200))
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
            IEnumerable<Frame> frames = frameSeq.Slice(startIndex, endIndex);
            foreach (Frame frame in frames)
            {
                Func<float, double> distance2weight = x => 1.0 / (x * 0 + 400 / 1000f);
                using (ColoredIterativePointMatching sipm = new ColoredIterativePointMatching(frame, frameSeq.LocalCoordinateMappers, conversions, distance2weight, 200))
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
        /// <param name="convList"></param>
        /// <param name="selectedUserIdList"></param>
        /// <returns></returns>
        public static List<CvMat> GetConvMatrixFromBoneFrame(Frame frame, List<CvMat> convList, ulong[] originalIds)
        {
            List<SerializableBody> bodies = frame.GetSelectedBodyList(originalIds:originalIds);
            return GetConvMatrixFromBoneFrame(frame, convList, bodies);
        }

        /// <summary>
        /// あるフレームにおける座標変換行列を骨格情報から計算する
        /// </summary>
        /// <param name="frame"></param>
        /// <param name="convList"></param>
        /// <param name="selectedUserIdList"></param>
        /// <returns></returns>
        public static List<CvMat> GetConvMatrixFromBoneFrame(Frame frame, List<CvMat> convList, int[] integratedIs)
        {
            List<SerializableBody> bodies = frame.GetSelectedBodyList(integratedIds:integratedIs);
            return GetConvMatrixFromBoneFrame(frame, convList, bodies);
        }

        /// <summary>
        /// あるフレームにおける座標変換行列を骨格情報から計算する
        /// </summary>
        /// <param name="frame"></param>
        /// <param name="convList"></param>
        /// <param name="bodies"></param>
        /// <returns></returns>
        public static List<CvMat> GetConvMatrixFromBoneFrame(Frame frame, List<CvMat> convList, List<SerializableBody> bodies)
        {
            if ( bodies.Count() != frame.recordNum )
            {
                System.Windows.MessageBox.Show("ユーザが選択されていないレコードがあります");
                return convList;
            }

            bool[] validFlags = frame.GetValidFlags();

            for (int j = 1; j < frame.recordNum; j++)
            {
                Dictionary<JointType, Joint> joint1 = Utility.GetValidJoints(bodies[0].Joints);
                Dictionary<JointType, Joint> joint2 = Utility.GetValidJoints(bodies[j].Joints);
                if (validFlags[0] && validFlags[j] == false)
                {
                    continue;
                }
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
            IEnumerable<Frame> frames = frameSeq.Frames.Skip(startIndex).Take(endIndex);
            List<SerializableBody> bodies;
            foreach (Frame frame in frames)
            {
                if (frameSeq.Segmentations == null)
                {
                    bodies = frame.GetSelectedBodyList(originalIds:frameSeq.selectedOriginalIdList);
                }
                else
                {
                    bodies = frame.GetSelectedBodyList(integratedIds:frameSeq.selecteedIntegretedIdList);
                }
                if (bodies.Count() != frame.recordNum)
                {
                    continue;
                }

                bool[] validFlags = frame.GetValidFlags();

                for (int i = 0; i < frame.recordNum; i++)
                {
                    for (int j = i + 1; j < frame.recordNum; j++)
                    {
                        if (validFlags[i] && validFlags[j] == false)
                        {
                            continue;
                        }
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
            }

            // 依存関係のツリーを作る
            int baseRecordIndex;
            Dictionary<int, int> dependencies;
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
                    if (frameSeq.Segmentations == null)
                    {
                        bodies = frame.GetSelectedBodyList(originalIds:frameSeq.selectedOriginalIdList);
                    }
                    else
                    {
                        bodies = frame.GetSelectedBodyList(integratedIds:frameSeq.selecteedIntegretedIdList);
                    }
                    if (bodies.Count() != frame.recordNum)
                        continue;

                    List<CvSize> depthUsersizeList = frame.DepthUserSize;
                    bool[] validFlags = frame.GetValidFlags();

                    foreach (KeyValuePair<int, int> dependencyPair in CalcEx.EnumerateDependencyPairs(baseRecordIndex, dependencies))
                    {
                        if (validFlags[dependencyPair.Key] && validFlags[dependencyPair.Value] == false)
                        {
                            continue;
                        }

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

