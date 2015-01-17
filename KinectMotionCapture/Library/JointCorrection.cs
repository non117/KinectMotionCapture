using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Microsoft.Kinect;
using OpenCvSharp;

namespace KinectMotionCapture
{
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

        private void NormalizeLegJoints(SerializableBody body)
        {
            List<Tuple<JointType, JointType>> legBones = Utility.GetLegBones();
        }
    }
}
