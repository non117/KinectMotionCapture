using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Microsoft.Kinect;
using OpenCvSharp;

namespace KinectMotionCapture
{
    using Bone = Tuple<JointType, JointType>;

    /// <summary>
    /// 骨の統計情報を格納するための構造体
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
}
