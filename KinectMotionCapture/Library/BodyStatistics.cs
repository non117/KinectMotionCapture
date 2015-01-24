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

    public class BodyStatistics
    {
        // 骨一覧
        private List<Bone> bones;
        private JointType[] arms;
        private List<CvPoint3D64f> rightBodyCrossVectors;
        private List<CvPoint3D64f> leftBodyCrossVectors;
        private Dictionary<Bone, List<double>> boneLengthSqLog;
        public Dictionary<Bone, BoneStatistics> boneLengthSqStatistics;

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
            foreach (Bone bone in bones)
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
            // 右半身の外積ベクトル
            if (validJoints.ContainsKey(JointType.ShoulderRight) && validJoints.ContainsKey(JointType.SpineShoulder) && validJoints.ContainsKey(JointType.SpineBase))
            {
                CvPoint3D64f spineBaseToSpineShoulder = validJoints[JointType.SpineBase].Position.ToCvPoint3D() - validJoints[JointType.SpineShoulder].Position.ToCvPoint3D();
                CvPoint3D64f spineBaseToRightShoulder = validJoints[JointType.SpineBase].Position.ToCvPoint3D() - validJoints[JointType.ShoulderRight].Position.ToCvPoint3D();
                this.rightBodyCrossVectors.Add(CvEx.Cross(spineBaseToSpineShoulder, spineBaseToRightShoulder));
            }
            // 左半身の外積ベクトル
            if (validJoints.ContainsKey(JointType.ShoulderLeft) && validJoints.ContainsKey(JointType.SpineShoulder) && validJoints.ContainsKey(JointType.SpineBase))
            {
                CvPoint3D64f spineBaseToSpineShoulder = validJoints[JointType.SpineBase].Position.ToCvPoint3D() - validJoints[JointType.SpineShoulder].Position.ToCvPoint3D();
                CvPoint3D64f spineBaseToLeftShoulder = validJoints[JointType.SpineBase].Position.ToCvPoint3D() - validJoints[JointType.ShoulderLeft].Position.ToCvPoint3D();
                this.leftBodyCrossVectors.Add(CvEx.Cross(spineBaseToSpineShoulder, spineBaseToLeftShoulder));
            }
        }

        /// <summary>
        /// 統計情報を計算し格納する
        /// z = 0.904をデフォルトとする、このとき中央値から65%を網羅できる
        /// </summary>
        /// <param name="z">標準正規分布表のZ</param>
        public void CalcMedianBoneRange(double z = 0.904)
        {
            foreach (Bone bone in this.bones)
            {
                List<double> data = this.boneLengthSqLog[bone];
                double median = CalcEx.GetMedian(data);
                double average = data.Average();
                double threshold = Math.Max(median, average);
                // 中央値あるいは平均値の大きい方の2倍未満の区間に限定する
                data = data.Where(d => d <= average * 2.1).ToList();
                // 計算し直す
                median = CalcEx.GetMedian(data);
                average = data.Average();
                double std = Math.Abs(Math.Sqrt(data.Select(d => Math.Pow(d - average, 2)).Sum() / (data.Count() - 1)));
                double minLength = median - std * z;
                double maxLength = median + std * z;
                BoneStatistics bs = new BoneStatistics(minLength, maxLength, median, average, std);
                this.boneLengthSqStatistics.Add(bone, bs);
            }
        }

        /// <summary>
        /// 両腕の位置がかぶっているかどうか
        /// </summary>
        /// <param name="joints"></param>
        /// <returns></returns>
        public bool CheckArmDuplication(Dictionary<JointType, Joint> joints)
        {
            List<float> zs = new List<float>();
            Joint joint;
            foreach (JointType jointType in this.arms)
            {
                if (joints.TryGetValue(jointType, out joint))
                {
                    zs.Add(joint.Position.Z);
                }
            }
            if (zs.Count == 0)
            {
                return false;
            }
            float avg = zs.Average();
            double variance = zs.Select(f => Math.Pow(f - avg, 2)).Sum() / (zs.Count() - 1);
            if (variance < 0.2 * 0.2 && zs.Count > 5)
            {
                return true;
            }
            return false;
        }

        /// <summary>
        /// 統計値から有用な範囲の骨のみを残す
        /// </summary>
        /// <param name="joints"></param>
        /// <returns></returns>
        public Dictionary<JointType, Joint> FilterBonesByStatistics(Dictionary<JointType, Joint> joints)
        {
            // TODO crossから腕とか削除するやつ
            Dictionary<JointType, Joint> validJoints = Utility.GetValidJoints(joints);
            Dictionary<JointType, Joint> result = validJoints.CloneDeep();
            Dictionary<JointType, bool> adaptJoints = new Dictionary<JointType, bool>();
            Joint firstJoint, secondJoint;
            Bone boneKey;
            foreach (Bone bone in bones)
            {
                if (validJoints.TryGetValue(bone.Item1, out firstJoint) && validJoints.TryGetValue(bone.Item2, out secondJoint))
                {
                    boneKey = Tuple.Create(firstJoint.JointType, secondJoint.JointType);
                    double lengthSq = CvEx.GetDistanceSq(firstJoint.Position.ToCvPoint3D(), secondJoint.Position.ToCvPoint3D());
                    if (boneLengthSqStatistics[bone].IsValidBone(lengthSq))
                    {
                        adaptJoints[firstJoint.JointType] = true;
                        adaptJoints[secondJoint.JointType] = true;
                    }
                }
            }
            bool armDup = this.CheckArmDuplication(validJoints);
            if (armDup)
            {
                foreach (JointType jointType in this.arms)
                {
                    adaptJoints[jointType] = false;
                }
            }

            foreach (JointType jointType in adaptJoints.Keys)
            {
                if (adaptJoints[jointType] == false)
                {
                    result.Remove(jointType);
                }
            }
            return result;
        }

        /// <summary>
        /// こんすとらくたん
        /// </summary>
        public BodyStatistics()
        {
            this.bones = Utility.GetBones();
            this.arms = new JointType[]{JointType.ElbowLeft, JointType.ElbowRight,
                JointType.HandLeft, JointType.HandRight, JointType.HandTipLeft, JointType.HandTipRight,
                JointType.ThumbLeft, JointType.ThumbRight, JointType.WristLeft, JointType.WristRight
            };
            this.boneLengthSqLog = new Dictionary<Bone, List<double>>();
            this.boneLengthSqStatistics = new Dictionary<Bone, BoneStatistics>();
        }
    }
}
