using System;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.Serialization.Formatters.Binary;

using Microsoft.Kinect;
using OpenCvSharp;

namespace KinectDataAnalyzer
{
    using Bone = Tuple<JointType, JointType>;

    /// <summary>
    /// 骨の統計情報
    /// </summary>
    public struct BoneStatistics
    {
        public double minLengthSq;
        public double maxLengthSq;
        public double medianLengthSq;
        public double averageLengthSq;
        public double stdLengthSq;
        public BoneStatistics(double minLength, double maxLength, double medianLength, double averageLength, double stdLength)
        {
            this.minLengthSq = minLength;
            this.maxLengthSq = maxLength;
            this.medianLengthSq = medianLength;
            this.averageLengthSq = averageLength;
            this.stdLengthSq = stdLength;
        }
        /// <summary>
        /// 統計情報の許容範囲かどうか
        /// </summary>
        /// <param name="lengthSq"></param>
        /// <returns></returns>
        public bool IsValidBone(double lengthSq)
        {
            if (lengthSq >= minLengthSq && lengthSq <= maxLengthSq)
            {
                return true;
            }
            return false;
        }
    }

    /// <summary>
    /// あるJointの座標時系列
    /// </summary>
    public class PointSequence
    {
        public List<CvPoint3D64f?> points;
        public List<DateTime> times;
        public JointType jointType;
        public PointSequence(List<CvPoint3D64f?> points, List<DateTime> times, JointType jointType)
        {
            this.points = points;
            this.times = times;
            this.jointType = jointType;
        }
        /// <summary>
        /// 変な点をはじく処理
        /// </summary>
        public void DropIrregularPoint()
        {
            // ここに変な点をはじく処理
        }
        /// <summary>
        /// 無い点を補完する
        /// </summary>
        public void Interpolate()
        {
            // 補完処理
        }
        /// <summary>
        /// csvに吐く
        /// </summary>
        public void Dump()
        {
            List<List<string>> outputs = new List<List<string>>();
            foreach(var pair in times.Zip(points, (time, point) => new {time, point}))
            {
                List<string> line = new List<string>();
                line.Add(pair.time.ToString("mm:ss:fff"));
                if (points == null)
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
    /// 統括するメタデータ
    /// </summary>
    public struct MotionMetaData
    {
        public List<Pose> motionLog;
        public Dictionary<JointType, PointSequence> pointSeqs;
        public BoneStatistics stat;
        public MotionMetaData(List<Dictionary<JointType, CvPoint3D64f>> jointsSeq, List<DateTime> timeSeq, BoneStatistics stat)
        {
            this.motionLog = new List<Pose>();
            foreach (var pair in jointsSeq.Zip(timeSeq, (joints, time) => new { joints, time }))
            {
                Pose pose = new Pose(pair.joints, pair.time);
                this.motionLog.Add(pose);
            }
            this.pointSeqs = new Dictionary<JointType, PointSequence>();
            foreach (JointType jointType in Enum.GetValues(typeof(JointType)))
            {
                List<CvPoint3D64f?> points = this.motionLog.Select(m => m.GetPoint(jointType)).ToList();
                pointSeqs[jointType] = new PointSequence(points, timeSeq, jointType);
            }
            this.stat = stat;
        }
        // TODO, filter, interpolate呼び出し, Poseへの書き戻し, Normalizeの呼び出し, 出力への整形
    }


}
