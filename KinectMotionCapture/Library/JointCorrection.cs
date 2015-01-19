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

    class JointCorrection
    {
        /// <summary>
        /// 胴体の外積ベクトル
        /// </summary>
        /// <param name="body"></param>
        /// <returns></returns>
        private CvPoint3D64f BodyCrossVector(SerializableBody body)
        {
            CvPoint3D64f torsoToRightShoulder = body.Joints[JointType.SpineMid].Position.ToCvPoint3D()
                - body.Joints[JointType.ShoulderRight].Position.ToCvPoint3D();
            CvPoint3D64f torsoToLeftShoulder = body.Joints[JointType.SpineMid].Position.ToCvPoint3D()
                - body.Joints[JointType.ShoulderLeft].Position.ToCvPoint3D();
            CvPoint3D64f bodyCross = CvEx.Cross(torsoToRightShoulder, torsoToLeftShoulder);
            return bodyCross;
        }

        /// <summary>
        /// z軸正方向に対する胴体ベクトルの向き
        /// </summary>
        /// <param name="bodyCrossVector"></param>
        /// <returns></returns>
        private double BodyAngle(CvPoint3D64f bodyCrossVector)
        {
            CvPoint3D64f zVector = new CvPoint3D64f(0, 0, 1);
            double angle = CvEx.Cos(bodyCrossVector, zVector);
            return angle;
        }

        /// <summary>
        /// 足の長さを統計情報をもとに正規化する
        /// </summary>
        /// <param name="body"></param>
        private void NormalizeLegJoints(Dictionary<JointType, Joint> joints, Dictionary<Bone, BoneStatistics> boneStatistics)
        {
            List<Tuple<JointType, JointType>> legBones = Utility.GetLegBones();
            foreach (Bone bone in legBones)
            {
                Joint joint1 = joints[bone.Item1];
                Joint joint2 = joints[bone.Item2];
                // 1の骨は動かさない. SpineBaseから順番に修正されることは保証されている.
                double medianLength = Math.Sqrt(boneStatistics[bone].medianLengthSq);
                CvPoint3D64f normalizedVector = CvEx.Normalize(joint1.Position.ToCvPoint3D() - joint2.Position.ToCvPoint3D());
                CvPoint3D32f expandedVector = (CvPoint3D32f)(normalizedVector * medianLength);
                joint2.Position = (joint1.Position.ToCvPoint3D() + expandedVector).ToCameraSpacePoint();
                joints[bone.Item2] = joint2;
            }
        }

        private void RemoveHalfBody(Dictionary<JointType, Joint> joints, List<JointType> removeJoints)
        {

        }
    }
}
