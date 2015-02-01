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

    public struct Pose
    {
        public Dictionary<JointType, CvPoint3D64f> joints;
        public DateTime timeStamp;
        public Pose(Dictionary<JointType, CvPoint3D64f> joints, DateTime time)
        {
            this.joints = joints;
            this.timeStamp = time;
        }
    }

    public struct MotionMetaData
    {
        public List<Pose> motionLog;
        public BoneStatistics stat;
        public MotionMetaData(List<Dictionary<JointType, CvPoint3D64f>> jointsSeq, List<DateTime> timeSeq, BoneStatistics stat)
        {
            this.motionLog = new List<Pose>();
            foreach (var pair in jointsSeq.Zip(timeSeq, (joints, time) => new { joints, time }))
            {
                Pose pose = new Pose(pair.joints, pair.time);
                this.motionLog.Add(pose);
            }
            this.stat = stat;
        }
    }


}
