﻿using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;

using MsgPack.Serialization;
using OpenCvSharp;

namespace KinectMotionCapture
{
    class FrameSequence
    {
        private List<CvMat> convList = null;
        private List<KinectUndistortion> undistortionDataList;
        
        public int recordNum;
        // TODO: IEnumerableにしても良さそう。イテレータブロックとか使うらしい。
        public List<Frame> frames;
        
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

        public List<KinectUndistortion> UndistortionDataList
        {
            get { return this.undistortionDataList; }
        }

        private List<MotionData> GetMotionDataFromFile(string filepath)
        {
            var serializer = MessagePackSerializer.Get<List<MotionData>>();
            using (FileStream fs = File.Open(filepath, FileMode.Open))
            {
                return serializer.Unpack(fs);
            }
        }

        private List<List<MotionData>> SliceFrames(List<List<MotionData>> records, DateTime minTime, DateTime maxTime)
        {
            List<List<MotionData>> newRecords = new List<List<MotionData>>();
            foreach (List<MotionData> record in records)
            {
                List<MotionData> newRecord = record.Where((r) => minTime <= r.TimeStamp && r.TimeStamp <= maxTime).ToList();
                newRecords.Add(newRecord);
            }
            return newRecords;
        }

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
            return this.SliceFrames(records, minTime, maxTime);
        }

        public FrameSequence(List<string> dataDirs)
        {
            List<List<MotionData>> records = new List<List<MotionData>>();
            foreach (string dataDir in dataDirs)
            {
                string metaDataFilePath = Path.Combine(dataDir, "BodyInfo.mpac");
                records.Add(this.GetMotionDataFromFile(dataDir));
            }
            records = this.SearchDupFrames(records);
            // 近いレコード取ってくるやつを実装する
            // ListEx.GetMaxLessEqualIndexFromBinarySearch(record.RecordData.GetIndexBinarySearch(time));
            // public int GetIndexBinarySearch(DateTime timestamp) {
            //     timeInfoのインデックスを返す
            //     return _timeInfo.Keys.BinarySearch(timestamp - this.TimeOffset);
        }
        }

    }

    /// <summary>
    /// あるフレームにおける複数レコードをまとめて管理するクラス
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

        public List<CvSize> DepthUserSize
        {
            get { return this.motionDataList.Select((m) => m.DepthUserSize).ToList(); }
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
        /// <summary>
        /// あるフレームにおける座標変換行列を深度情報から計算する
        /// </summary>
        /// <param name="frame"></param>
        public static List<CvMat> AjustFrameFromDepth(Frame frame, List<CvMat> convList)
        {
            List<Func<CvPoint3D64f, CvPoint3D64f>> toReal = new List<Func<CvPoint3D64f, CvPoint3D64f>>();
            foreach (CvMat depthMat in frame.DepthMatList)
            {
                toReal.Add((x) => KinectUndistortion.GetOriginalRealFromScreenPos(x, new CvSize(depthMat.Cols, depthMat.Rows)));
            }
            Func<float, double> distance2weight = x => 1.0 / (x * 0 + 400);
            using (ColoredIterativePointMatching sipm = new ColoredIterativePointMatching(frame.DepthMatList, frame.ColorMatList, toReal, convList, distance2weight, 200))
            {
                List<CvMat> conversions = sipm.CalculateTransformSequntially(0.2, 3);
                return conversions;
            }
        }

        /// <summary>
        /// フレーム範囲における座標変換行列を深度情報から計算する
        /// </summary>
        /// <param name="frames"></param>
        public static void AjustFramesFromDepth(FrameSequence frameSeq)
        {
            foreach (Frame frame in frameSeq.frames)
            {
                List<Func<CvPoint3D64f, CvPoint3D64f>> toReal = new List<Func<CvPoint3D64f, CvPoint3D64f>>();
                foreach (CvMat depthMat in frame.DepthMatList)
                {
                    toReal.Add((x) => KinectUndistortion.GetOriginalRealFromScreenPos(x, new CvSize(depthMat.Cols, depthMat.Rows)));
                }
                Func<float, double> distance2weight = x => 1.0 / (x * 0 + 400);
                using (ColoredIterativePointMatching sipm = new ColoredIterativePointMatching(frame.DepthMatList, frame.ColorMatList, toReal, frameSeq.ToWorldConversions, distance2weight, 200))
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
        public List<CvMat> AjustFrameFromeBone(Frame frame, List<CvMat> convList)
        {
            List<SerializableBody> bodies = frame.SelectedBodyList;
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
        public void AjustFramesFromBone(FrameSequence frameSeq)
        {
            Dictionary<Tuple<int, int>, int> cooccurenceCount = new Dictionary<Tuple<int, int>, int>();
            //System.Windows.MessageBox.Show("ユーザが選択されていないレコードがあります");
            //return;
            foreach (Frame frame in frameSeq.frames)
            {
                List<SerializableBody> bodies = frame.SelectedBodyList;
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
                foreach (Frame frame in frameSeq.frames)
                {
                    List<SerializableBody> bodies = frame.SelectedBodyList;
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

