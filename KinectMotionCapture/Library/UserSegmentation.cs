using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Xml.Serialization;

using Microsoft.Kinect;
using OpenCvSharp;

namespace KinectMotionCapture
{
    public struct TupleStruct<T, U>
    {
        public T Item1;
        public U Item2;
        public TupleStruct(T item1, U item2)
        {
            this.Item1 = item1;
            this.Item2 = item2;
        }
        public TupleStruct(KeyValuePair<T, U> pair)
            : this(pair.Key, pair.Value)
        {
        }
    }

    public class UserSegmentation
    {
        int _numUsers = 0;
        SortedList<int, Dictionary<ulong, int>> _conversions = new SortedList<int, Dictionary<ulong, int>>();
        [XmlIgnore]
        public SortedList<int, Dictionary<ulong, int>> Conversions
        {
            get { return _conversions; }
            set { _conversions = value; }
        }

        //public TupleStruct<int, TupleStruct<int, int>[]>[] SerializableConversions
        //{
        //    get
        //    {
        //        return _conversions.Select(p => new TupleStruct<int, TupleStruct<int, int>[]>(p.Key, p.Value.Select(q => new TupleStruct<int, int>(q.Key, q.Value)).ToArray())).ToArray();
        //    }
        //    set
        //    {
        //        _conversions = new SortedList<int, Dictionary<int, int>>();
        //        foreach (var pair in value)
        //        {
        //            Dictionary<int, int> conv = new Dictionary<int, int>();
        //            foreach (var innerPair in pair.Item2)
        //            {
        //                conv.Add(innerPair.Item1, innerPair.Item2);
        //                if (_numUsers <= innerPair.Item2)
        //                {
        //                    _numUsers = innerPair.Item2 + 1;
        //                }
        //            }
        //            _conversions.Add(pair.Item1, conv);
        //        }
        //    }
        //}

        public void MergeSegment(int fromUser, int toUser)
        {
            if (_conversions == null)
                return;
            foreach (var dict in _conversions.Values)
            {
                if (dict.ContainsValue(fromUser))
                {
                    ulong key = dict.First(p => p.Value == fromUser).Key;
                    dict[key] = toUser;
                }
            }
        }

        public UserSegmentation()
        {
        }

        public static UserSegmentation Segment(IEnumerable<MotionData> recordData, TimeSpan segmentTimeSpan)
        {
            int numUser = 0;
            UserSegmentation segm = new UserSegmentation();
            Dictionary<ulong, Tuple<int, DateTime>> currentConversions = new Dictionary<ulong, Tuple<int, DateTime>>();
            int frameIndex = 0;
            foreach (MotionData md in recordData)
            {
                bool changed = false;
                foreach (ulong user in md.bodies.ToList().Select(b => b.TrackingId))
                {
                    Tuple<int, DateTime> converted;
                    if (currentConversions.TryGetValue(user, out converted))
                    {
                        // 前回と比べて時間内なら継続
                        if (md.TimeStamp - converted.Item2 <= segmentTimeSpan)
                        {
                            currentConversions[user] = new Tuple<int, DateTime>(converted.Item1, md.TimeStamp);
                            continue;
                        }
                    }
                    // 新しい番号を与える
                    currentConversions[user] = new Tuple<int, DateTime>(numUser, md.TimeStamp);
                    numUser++;
                    changed = true;
                }
                if (changed)
                {
                    segm.Conversions[frameIndex] = currentConversions.ToDictionary(p => p.Key, p => p.Value.Item1);
                }
                frameIndex++;
            }
            segm._numUsers = numUser;
            return segm;
        }

        public void SetNewID(int beginFrameIndex, int endFrameIndex, ulong originalUser)
        {
            SetID(beginFrameIndex, endFrameIndex, originalUser, _numUsers);
        }

        public int GetNewID()
        {
            return _numUsers;
        }

        void fixNumUsers()
        {
            foreach (var dict in _conversions.Values)
            {
                foreach (int toUser in dict.Values)
                {
                    if (_numUsers <= toUser)
                        _numUsers = toUser + 1;
                }
            }
        }

        public void SetID(int beginFrameIndex, int endFrameIndex, ulong originalUser, int segID)
        {
            int idAtEnd = GetSegmentedUser(endFrameIndex, originalUser);
            var list = getSegmentedRange(beginFrameIndex, endFrameIndex);
            foreach (var pair in list)
            {
                setIDAt(pair.Key, originalUser, segID);
            }
            setIDAt(endFrameIndex, originalUser, idAtEnd);
            _numUsers = Math.Max(_numUsers, segID + 1);
        }

        void setIDAt(int frameIndex, ulong user, int segID)
        {
            Dictionary<ulong, int> dict = new Dictionary<ulong, int>(getSegmentedUsers(frameIndex));
            dict[user] = segID;
            _conversions[frameIndex] = dict;
        }
        //public int GetSegmentedUser(TrackImageRecordData recordData, DateTime timestamp, int originalUser)
        //{
        //    int frameIndex = ListEx.GetMaxLessEqualIndexFromBinarySearch(recordData.GetIndexBinarySearch(timestamp));
        //    return this.GetSegmentedUser(frameIndex, originalUser);
        //}

        public int GetSegmentedUser(int frameIndex, ulong originalUser)
        {
            var dict = getSegmentedUsers(frameIndex);
            int ret;
            if (!dict.TryGetValue(originalUser, out ret))
                return -1;
            return ret;
        }

        public Dictionary<ulong, int> GetSegmentedUsers(int frameIndex)
        {
            return getSegmentedUsers(frameIndex).ToDictionary(p => p.Key, p => p.Value);
        }

        Dictionary<ulong, int> getSegmentedUsers(int frameIndex)
        {
            int index = ListEx.GetMaxLessEqualIndexFromBinarySearch(_conversions.Keys.BinarySearch(frameIndex));
            if (index < 0 || index >= _conversions.Count)
                return new Dictionary<ulong, int>();
            var dict = _conversions.Values[index];
            return dict;
        }

        SortedList<int, Dictionary<ulong, int>> getSegmentedRange(int beginFrameIndex, int endFrameIndex)
        {
            int beginKey = ListEx.GetMaxLessEqualIndexFromBinarySearch(_conversions.Keys.BinarySearch(beginFrameIndex));
            int endKey = ListEx.GetMinGreaterEqualIndexFromBinarySearch(_conversions.Keys.BinarySearch(endFrameIndex));
            SortedList<int, Dictionary<ulong, int>> ret = new SortedList<int, Dictionary<ulong, int>>();
            for (int key = beginKey; key < endKey; key++)
            {
                if (key >= 0)
                {
                    ret[_conversions.Keys[key]] = _conversions.Values[key];
                }
            }
            return ret;
        }

        /// <summary>
        /// Jointから絶対座標を引くDictionary二つの間の距離を求めます
        /// </summary>
        /// <param name="jointWithAbsPosition1">骨格ごとの絶対座標1</param>
        /// <param name="jointWithAbsPosition2">骨格ごとの絶対座標2</param>
        /// <param name="mirrored">一方の骨格を左右反転した結果で距離を求めるかどうか</param>
        /// <returns></returns>
        static double getDistance(IDictionary<JointType, CvPoint3D64f> jointWithAbsPosition1, IDictionary<JointType, CvPoint3D64f> jointWithAbsPosition2, bool mirrored)
        {
            IEnumerable<JointType> joints2Alt = jointWithAbsPosition2.Keys.Select(j => mirrored ? CalcEx.GetMirroredJoint(j) : j);
            List<JointType> intersect = jointWithAbsPosition1.Keys.Intersect(joints2Alt).ToList();
            List<double> distanceSqList = new List<double>();
            foreach (JointType joint in intersect)
            {
                JointType jointAlt = mirrored ? CalcEx.GetMirroredJoint(joint) : joint;
                distanceSqList.Add(CvEx.GetDistanceSq(jointWithAbsPosition1[joint], jointWithAbsPosition2[jointAlt]));
            }
            // 中央値の平方根
            return Math.Sqrt(CalcEx.GetMedian(distanceSqList));
        }

        struct RecordAndUser
        {
            public int RecordIndex, UserIndex;
            public RecordAndUser(int recordIndex, int userIndex)
            {
                this.RecordIndex = recordIndex;
                this.UserIndex = userIndex;
            }
            public override string ToString()
            {
                return string.Format("{0}: {1}", this.RecordIndex, this.UserIndex);
            }
        }

        public static UserSegmentation[] Identification(double frequency, IList<TrackImageRecordProperty> records, double maxDistance, ProgressData progress)
        {
            if (records.Any(r => r.UserSegmentationData == null))
                throw new InvalidOperationException("ユーザトラッキングデータがセグメンテーションされていません");
            progress.InitProgress("Initializing... ", 0);

            HashSet<Tuple<RecordAndUser, RecordAndUser>> contemporaryList = new HashSet<Tuple<RecordAndUser, RecordAndUser>>();


            for (int i = 0; i < records.Count; i++)
            {
                TrackImageRecordProperty record = records[i];
                int frameIndex = 0;
                foreach (TrackFrame trackFrame in record.RecordData.EnumerateTrackFrame())
                {
                    IList<int> users = trackFrame.GetValidUsers();
                    foreach (var tuple in users.SelectMany(u => users.Select(v => new Tuple<RecordAndUser, RecordAndUser>(new RecordAndUser(i, u), new RecordAndUser(i, v)))))
                    {
                        contemporaryList.Add(tuple);
                    }
                    frameIndex++;
                }
            }
            DateTime beginTime = new DateTime(records.Min(r => r.RecordData.BeginTime.Ticks));
            DateTime endTime = new DateTime(records.Max(r => r.RecordData.EndTime.Ticks));
            TimeSpan increment = new TimeSpan((long)(10000000 / frequency));
            long totalCount = (endTime.Ticks - beginTime.Ticks) / increment.Ticks;
            long totalIndex = 0;
            progress.InitProgress("Calculate Distances... ", totalCount + 100);
            Dictionary<Tuple<RecordAndUser, RecordAndUser>, List<double>> distanceListMatrix = new Dictionary<Tuple<RecordAndUser, RecordAndUser>, List<double>>();
            for (DateTime time = beginTime; time < endTime; time += increment)
            {
                progress.SetProgress(totalIndex);
                // 現在の時刻での各レコードの各ユーザの各骨格の絶対座標を求める
                Dictionary<int, Dictionary<JointType, CvPoint3D64f>>[] absPositions = new Dictionary<int, Dictionary<JointType, CvPoint3D64f>>[records.Count];
                for (int i = 0; i < records.Count; i++)
                {
                    TrackImageRecordProperty record = records[i];
                    int frameIndex = ListEx.GetMaxLessEqualIndexFromBinarySearch(record.RecordData.GetIndexBinarySearch(time));
                    // 最後のフレームを使わないようにして、最後のフレームをいつまでも引きずらないようにする
                    int nextIndex = ListEx.GetMinGreaterEqualIndexFromBinarySearch(record.RecordData.GetIndexBinarySearch(time));
                    if (frameIndex == -1 || nextIndex >= record.RecordData.FrameCount)
                        continue;
                    Dictionary<int, Dictionary<JointType, CvPoint3D64f>> recordUserPositions = new Dictionary<int, Dictionary<JointType, CvPoint3D64f>>();
                    TrackFrame trackFrame = record.RecordData.GetTrackFrame(frameIndex);
                    IList<int> users = trackFrame.GetValidUsers();
                    foreach (int user in users)
                    {
                        Dictionary<JointType, CvPoint3D64f> userPositions = new Dictionary<JointType, CvPoint3D64f>();
                        foreach (var jointPair in trackFrame.UserTrackings[user].OriginalJoints)
                        {
                            CvPoint3D64f posInCamera = record.UndistortionData.GetRealFromScreenPos(jointPair.Value.Position.ToCvPoint3D(), trackFrame.DepthUserSize);
                            CvPoint3D64f posInWorld = CvEx.ConvertPoint3D(posInCamera, record.ToWorldConversion);
                            userPositions[jointPair.Key] = posInWorld;
                        }
                        recordUserPositions[user] = userPositions;
                    }
                    absPositions[i] = recordUserPositions;
                }
                // 現在の時刻で各レコード間のユーザ間の距離を求める
                for (int i = 0; i < records.Count; i++)
                {
                    if (absPositions[i] == null)
                        continue;
                    for (int j = i + 1; j < records.Count; j++)
                    {
                        if (absPositions[j] == null)
                            continue;
                        foreach (var userJoint1 in absPositions[i])
                        {
                            RecordAndUser recordUser1 = new RecordAndUser(i, userJoint1.Key);
                            foreach (var userJoint2 in absPositions[j])
                            {
                                RecordAndUser recordUser2 = new RecordAndUser(j, userJoint2.Key);
                                double distanceNormal = getDistance(userJoint1.Value, userJoint2.Value, false);
                                double distanceMirrored = getDistance(userJoint1.Value, userJoint2.Value, true);
                                double distance = Math.Min(distanceNormal, distanceMirrored);
                                Tuple<RecordAndUser, RecordAndUser> key = new Tuple<RecordAndUser, RecordAndUser>(recordUser1, recordUser2);
                                List<double> distanceList;
                                if (!distanceListMatrix.TryGetValue(key, out distanceList))
                                {
                                    distanceListMatrix[key] = distanceList = new List<double>();
                                }
                                distanceList.Add(distance);
                            }
                        }
                    }
                }
                totalIndex++;
            }
            progress.SetProgress("Calculate Identification", totalCount + 50);
            // 中央値で集計して小さい順に並べる
            Dictionary<Tuple<RecordAndUser, RecordAndUser>, double> distanceMatrix = distanceListMatrix.ToDictionary(p => p.Key, p => CalcEx.GetMedian(p.Value));
            List<Tuple<RecordAndUser, RecordAndUser, double>> neighborList = (
                from x in distanceMatrix
                orderby x.Value
                select new Tuple<RecordAndUser, RecordAndUser, double>(x.Key.Item1, x.Key.Item2, x.Value)
                ).ToList();
            IdentificationSet<RecordAndUser> identificationSet = new IdentificationSet<RecordAndUser>();
            // 同一判定をする
            foreach (var neighbor in neighborList)
            {
                if (neighbor.Item3 > maxDistance)
                {
                    identificationSet.Add(neighbor.Item1);
                    identificationSet.Add(neighbor.Item2);
                    continue;
                }
                IList<RecordAndUser> recordUsers1 = identificationSet.GetEquivalentElements(neighbor.Item1);
                IList<RecordAndUser> recordUsers2 = identificationSet.GetEquivalentElements(neighbor.Item2);
                // 同フレーム内にいるか判定
                bool contemporary = (
                from ru1 in recordUsers1
                from ru2 in recordUsers2
                select new Tuple<RecordAndUser, RecordAndUser>(ru1, ru2)).Any(pair => contemporaryList.Contains(pair));
                if (!contemporary)
                {
                    // 同フレーム内にいなければ同一視
                    identificationSet.MakeEquivalent(neighbor.Item1, neighbor.Item2);
                }
            }
            // 番号を圧縮
            identificationSet.CompactIdentificationNumber();
            // 新しいセグメンテーション番号を与える
            UserSegmentation[] ret = Enumerable.Range(0, records.Count).Select(i => new UserSegmentation()).ToArray();
            for (int i = 0; i < records.Count; i++)
            {
                TrackImageRecordProperty record = records[i];
                foreach (var pair in record.UserSegmentationData.Conversions)
                {
                    int frameIndex = pair.Key;
                    Dictionary<int, int> newConversions = new Dictionary<int, int>();
                    foreach (var conv in pair.Value)
                    {
                        int ident = identificationSet.ConvertToIdentificationNumber(new RecordAndUser(i, conv.Value));
                        newConversions[conv.Key] = ident;
                    }
                    ret[i].Conversions[frameIndex] = newConversions;
                }
                ret[i].fixNumUsers();
            }
            return ret;
        }
    }
}

