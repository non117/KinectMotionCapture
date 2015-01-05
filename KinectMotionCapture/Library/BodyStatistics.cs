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
    public class BodyStatistics
    {
        /// <summary>
        /// 骨の統計情報を格納するための構造体
        /// </summary>
        public struct BoneStatistics
        {
            double minLength;
            double maxLength;
            double medianLength;
            public BoneStatistics(double minLength, double maxLength, double medianLength)
            {
                this.minLength = minLength;
                this.maxLength = maxLength;
                this.medianLength = medianLength;
            }
        }

        // 骨一覧
        private List<Bone> bones;
        private Dictionary<Bone, List<double>> boneLengthSqLog;
        private Dictionary<Bone, BoneStatistics> boneLengthStatistics;

        /// <summary>
        /// 骨のデータを蓄積する
        /// </summary>
        /// <param name="joints"></param>
        public void StoreBoneLength(Dictionary<JointType, Joint> joints)
        {
            Dictionary<JointType, Joint> validJoints = Utility.GetValidJoints(joints);
            Joint firstJoint, secondJoint;
            Bone boneKey;
            List<double> lengthVal;
            foreach (var bone in bones)
            {
                if (validJoints.TryGetValue(bone.Item1, out firstJoint) && validJoints.TryGetValue(bone.Item2, out secondJoint))
                {
                    double lengthSq = CvEx.GetDistanceSq(firstJoint.Position.ToCvPoint3D(), secondJoint.Position.ToCvPoint3D());
                    boneKey = Tuple.Create(firstJoint.JointType, secondJoint.JointType);

                    if (boneLengthSqLog.TryGetValue(boneKey, out lengthVal))
                    {
                        lengthVal.Add(lengthSq);
                    }
                    else
                    {
                        boneLengthSqLog[boneKey] = new List<double>() { lengthSq };
                    }
                    
                }
            }
            
        }

        /// <summary>
        /// 統計情報を計算し格納する
        /// </summary>
        /// <param name="ratio"></param>
        public void CalcMedianBoneRange(double ratio = 0.75)
        {
            foreach (Bone bone in this.bones)
            {
                List<double> data = this.boneLengthSqLog[bone].OrderBy(num => num).ToList();
                double median = CalcEx.GetMedian(data);
                IEnumerable<Tuple<double, int>> cleanedData = data.Select((num, index) => Tuple.Create(Math.Abs(num - median), index)).OrderBy(tup => tup.Item1);
                int cutNum = (int)(data.Count() * ratio);
                List<int> indexes = new List<int>();
                foreach(Tuple<double, int> pair in cleanedData)
                {
                    if (indexes.Count > cutNum)
                    {
                        break;
                    }
                    indexes.Add(pair.Item2);
                }
                double minLength = data[indexes.Min()];
                double maxLength = data[indexes.Max()];
                BoneStatistics bs = new BoneStatistics(maxLength, minLength, median);
                this.boneLengthStatistics.Add(bone, bs);
            }
        }

        /// <summary>
        /// こんすとらくたん
        /// </summary>
        public BodyStatistics()
        {
            this.bones = Utility.GetBones();
            this.boneLengthSqLog = new Dictionary<Bone, List<double>>();
            this.boneLengthStatistics = new Dictionary<Bone, BoneStatistics>();
        }
    }
}
