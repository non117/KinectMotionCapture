﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using OpenCvSharp;
using Microsoft.Kinect;

namespace KinectMotionCapture
{
    using Bone = Tuple<JointType, JointType>;

    /// <summary>
    /// あるJointの座標時系列
    /// </summary>
    public class PointSequence
    {
        public List<CvPoint3D64f?> points;
        public List<DateTime> times;
        public JointType jointType;
        
        /// <summary>
        /// こんすとらくたん
        /// </summary>
        /// <param name="points"></param>
        /// <param name="times"></param>
        /// <param name="jointType"></param>
        public PointSequence(List<CvPoint3D64f?> points, List<DateTime> times, JointType jointType)
        {
            this.points = points;
            this.times = times;
            this.jointType = jointType;
        }
        /// <summary>
        /// 平滑化
        /// </summary>
        public void Smoothing(int n = 10)
        {
            this.points = Utility.MovingMedianAverage(this.points, n);
        }
        /// <summary>
        /// csvに吐く
        /// </summary>
        public void Dump()
        {
            List<List<string>> outputs = new List<List<string>>();
            DateTime start = this.times[0];
            foreach(var pair in times.Zip(points, (time, point) => new {time, point}))
            {
                List<string> line = new List<string>();
                line.Add((pair.time - start).TotalSeconds.ToString());
                if (pair.point == null)
                {
                    line.Add("");
                    line.Add("");
                    line.Add("");
                }
                else
                {
                    line.Add(pair.point.Value.X.ToString());
                    line.Add(pair.point.Value.Y.ToString());
                    line.Add(pair.point.Value.Z.ToString());
                }
                outputs.Add(line);
            }
            Utility.SaveToCsv(jointType.ToString() + ".csv", outputs);
        }
    }

    /// <summary>
    /// ある瞬間の姿勢
    /// </summary>
    public struct Pose
    {
        public Dictionary<JointType, CvPoint3D64f> joints;
        public DateTime timeStamp;
        
        /// <summary>
        /// こんすとらくたん
        /// </summary>
        /// <param name="joints"></param>
        /// <param name="time"></param>
        public Pose(Dictionary<JointType, CvPoint3D64f> joints, DateTime time)
        {
            this.joints = joints;
            this.timeStamp = time;
        }
        /// <summary>
        /// あるjointTypeの点を返す。null許容。
        /// </summary>
        /// <param name="jointType"></param>
        /// <returns></returns>
        public CvPoint3D64f? GetPoint(JointType jointType)
        {
            if (this.joints.ContainsKey(jointType))
            {
                return this.joints[jointType];
            }
            return null;
        }
    }

    /// <summary>
    /// Jointの接続関係をアレする
    /// </summary>
    public struct JointNode
    {
        JointNode[] nextNodes;
        JointType parent;
        JointType chid;
        public JointNode(JointType parent, JointType child, JointNode[] next)
        {
            this.parent = parent;
            this.chid = child;
            this.nextNodes = next;
        }
    }

    /// <summary>
    /// 統括するメタデータ
    /// </summary>
    public struct MotionMetaData
    {
        public List<Pose> motionLog;
        public Dictionary<JointType, PointSequence> pointSeqs;
        public List<Dictionary<Bone, BoneStatistics>> stats;
        public Dictionary<Bone, double> boneLengthes;
        public JointNode rootNode;

        /// <summary>
        /// 骨の長さの代表値を決定する
        /// </summary>
        private void CalcBoneLength()
        {
            foreach (Bone bone in Utility.GetBones())
            {
                double length = Utility.CalcMedianLength(stats.Select(d => d[bone]).ToList());
                this.boneLengthes[bone] = length;
            }
        }

        /// <summary>
        /// SequenceをPoseに戻す
        /// </summary>
        private void SequenceToPose()
        {
            int n = motionLog.Count();
            motionLog.Clear();
            for (int i = 0; i < n; i++)
            {
                Dictionary<JointType, CvPoint3D64f> joints = new Dictionary<JointType, CvPoint3D64f>();
                DateTime time = new DateTime();
                foreach (JointType jointType in Enum.GetValues(typeof(JointType)))
                {
                    joints[jointType] = (CvPoint3D64f)pointSeqs[jointType].points[i];
                    time = pointSeqs[jointType].times[i];
                }
                motionLog.Add(new Pose(joints, time));
            }
        }

        private JointNode InitializeNodes()
        {
            JointNode neckHead = new JointNode(JointType.Neck, JointType.Head, new JointNode[] { });
            JointNode spineShoulderNeck = new JointNode(JointType.SpineShoulder, JointType.Neck, new JointNode[] { neckHead });

            JointNode elbowHandLeft = new JointNode(JointType.ElbowLeft, JointType.HandLeft, new JointNode[] { });
            JointNode shoulderElbowLeft = new JointNode(JointType.ShoulderLeft, JointType.ElbowLeft, new JointNode[] { elbowHandLeft });
            JointNode spineShoulderLeft = new JointNode(JointType.SpineShoulder, JointType.ShoulderLeft, new JointNode[] { shoulderElbowLeft });

            JointNode elbowHandRight = new JointNode(JointType.ElbowRight, JointType.HandRight, new JointNode[] { });
            JointNode shoulderElbowRight = new JointNode(JointType.ShoulderRight, JointType.ElbowRight, new JointNode[] { elbowHandRight });
            JointNode spineShoulderRight = new JointNode(JointType.SpineShoulder, JointType.ShoulderRight, new JointNode[] { shoulderElbowRight });

            JointNode ankleFootLeft = new JointNode(JointType.AnkleLeft, JointType.FootLeft, new JointNode[] { });
            JointNode kneeAnkleLeft = new JointNode(JointType.KneeLeft, JointType.AnkleLeft, new JointNode[] { ankleFootLeft });
            JointNode hipKneeLeft = new JointNode(JointType.HipLeft, JointType.KneeLeft, new JointNode[] { kneeAnkleLeft });
            JointNode spineHipLeft = new JointNode(JointType.SpineBase, JointType.HipLeft, new JointNode[] { hipKneeLeft });

            JointNode ankleFootRight = new JointNode(JointType.AnkleRight, JointType.FootRight, new JointNode[] { });
            JointNode kneeAnkleRight = new JointNode(JointType.KneeRight, JointType.AnkleRight, new JointNode[] { ankleFootRight });
            JointNode hipKneeRight = new JointNode(JointType.HipRight, JointType.KneeRight, new JointNode[] { kneeAnkleRight });
            JointNode spineHipRight = new JointNode(JointType.SpineBase, JointType.HipRight, new JointNode[] { hipKneeRight });

            JointNode spineMidShoulder = new JointNode(JointType.SpineMid, JointType.SpineShoulder, new JointNode[] { spineShoulderNeck, spineShoulderRight, spineShoulderLeft });
            JointNode spineBaseMid = new JointNode(JointType.SpineBase, JointType.SpineMid, new JointNode[] { spineMidShoulder });
            JointNode root = new JointNode(default(JointType), default(JointType), new JointNode[] { spineBaseMid, spineHipRight, spineHipLeft });

            return root;
        }

        /// <summary>
        /// こんすとらくたん
        /// </summary>
        /// <param name="jointsSeq"></param>
        /// <param name="timeSeq"></param>
        /// <param name="stats"></param>
        public MotionMetaData(List<Dictionary<JointType, CvPoint3D64f>> jointsSeq, List<DateTime> timeSeq, List<Dictionary<Bone, BoneStatistics>> stats)
        {
            this.rootNodes = new JointNode();
            // poseのデータを生成
            this.motionLog = new List<Pose>();
            foreach (var pair in jointsSeq.Zip(timeSeq, (joints, time) => new { joints, time }))
            {
                Pose pose = new Pose(pair.joints, pair.time);
                this.motionLog.Add(pose);
            }
            // seqに詰め込む
            this.pointSeqs = new Dictionary<JointType, PointSequence>();
            foreach (JointType jointType in Enum.GetValues(typeof(JointType)))
            {
                List<CvPoint3D64f?> points = this.motionLog.Select(m => m.GetPoint(jointType)).ToList();
                pointSeqs[jointType] = new PointSequence(points, timeSeq, jointType);
            }

            // すむーじんぐ            
            foreach (PointSequence pointSeq in this.pointSeqs.Values)
            {
                pointSeq.Smoothing();
            }

            this.stats = stats;
            this.boneLengthes = new Dictionary<Bone, double>();
            this.CalcBoneLength();

            // nodeの接続関係をイニシャライズ            
            this.rootNode = this.InitializeNodes();
        }
        // TODO, 分節化, Normalizeの呼び出し, 出力への整形
    }
}
