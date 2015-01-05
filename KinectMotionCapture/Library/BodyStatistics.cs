using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Microsoft.Kinect;
using OpenCvSharp;

namespace KinectMotionCapture
{
    public class BodyStatistics
    {
        // 骨一覧
        private List<Tuple<JointType, JointType>> bones;
        private Dictionary<Tuple<JointType, JointType>, List<double>> boneLengthSqLog;

        /// <summary>
        /// 骨のデータを蓄積する
        /// </summary>
        /// <param name="joints"></param>
        public void StoreBoneLength(Dictionary<JointType, Joint> joints)
        {
            Dictionary<JointType, Joint> validJoints = Utility.GetValidJoints(joints);
            Joint firstJoint, secondJoint;
            Tuple<JointType, JointType> boneKey;
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

        public void GetMedianBoneLengths()
        {
            // TODO
            // 引数に範囲とか
            //List<double> data = this.boneLengthSqLog[this.bones[4]];
            //List<List<double>> hoge = data.Select(d => new List<double>() { d }).ToList();
            //Utility.SaveToCsv(@"stat.csv", new List<string>() { bones[4].ToString() + "length" }, hoge);
        }

        public BodyStatistics()
        {
            this.bones = Utility.GetBones();
            this.boneLengthSqLog = new Dictionary<Tuple<JointType, JointType>, List<double>>();
        }
    }
}
