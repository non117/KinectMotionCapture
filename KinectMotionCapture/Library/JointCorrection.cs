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
        private CvPoint3D64f BodyCrossVector(Dictionary<JointType, CvPoint3D64f> joints)
        {
            if (joints.ContainsKey(JointType.SpineBase) && joints.ContainsKey(JointType.ShoulderRight) && joints.ContainsKey(JointType.ShoulderLeft))
            {
                CvPoint3D64f torsoToRightShoulder = joints[JointType.SpineMid] - joints[JointType.ShoulderRight];
                CvPoint3D64f torsoToLeftShoulder = joints[JointType.SpineMid] - joints[JointType.ShoulderLeft];
                CvPoint3D64f bodyCross = CvEx.Cross(torsoToRightShoulder, torsoToLeftShoulder);
                return bodyCross;
            }
            return default(CvPoint3D64f);
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
        private Dictionary<JointType, Joint> NormalizeLegJoints(Dictionary<JointType, Joint> joints, Dictionary<Bone, BoneStatistics> boneStatistics)
        {
            List<Tuple<JointType, JointType>> legBones = Utility.GetLegBones();
            foreach (Bone bone in legBones)
            {
                if (joints.ContainsKey(bone.Item1) && joints.ContainsKey(bone.Item2))
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
            return joints;
        }

        /// <summary>
        /// 骨を取り除く
        /// </summary>
        /// <param name="joints"></param>
        /// <param name="removeJoints"></param>
        private Dictionary<JointType, Joint> RemoveJoints(Dictionary<JointType, Joint> joints, List<JointType> removeJoints)
        {
            Dictionary<JointType, Joint> newJoints = joints.CloneDeep();
            foreach (JointType jointType in removeJoints)
            {
                if (newJoints.ContainsKey(jointType))
                {
                    newJoints.Remove(jointType);
                }
            }
            return newJoints;
        }

        /// <summary>
        /// 身体の骨を反転する
        /// </summary>
        /// <param name="joints"></param>
        private Dictionary<JointType, Joint> ReverseBody(Dictionary<JointType, Joint> joints)
        {
            return joints.ToDictionary(p => CalcEx.GetMirroredJoint(p.Key), p => p.Value);
        }

        /// <summary>
        /// 遮蔽された身体を削除する
        /// </summary>
        /// <param name="body"></param>
        /// <returns></returns>
        private SerializableBody CleanOcculusions(SerializableBody body)
        {
            SerializableBody newBody = body.CloneDeep();
            // 閾値はてきとう
            if (Math.Abs(body.bodyAngle) < 0.309)
            {
                if (body.bodyCrossVector[0] > 0)
                {
                    List<JointType> removeJoints = Utility.RightBody.ToList().Concat(Utility.Spines.ToList()).ToList();
                    newBody.Joints = this.RemoveJoints(body.Joints, removeJoints);
                }
                else
                {
                    List<JointType> removeJoints = Utility.LeftBody.ToList().Concat(Utility.Spines.ToList()).ToList();
                    newBody.Joints = this.RemoveJoints(body.Joints, removeJoints);
                }
            }
            return newBody;
        }

        /// <summary>
        /// ミラー状態を補正するための基準骨格フレーム複数から、比較すべきフレームの範囲を決定する
        /// </summary>
        /// <param name="frameLength"></param>
        /// <param name="trustDataList"></param>
        /// <returns></returns>
        private List<Tuple<TrustData, int>> GenerateIterationRanges(int frameLength, List<TrustData> trustDataList)
        {
            List<Tuple<TrustData, int>> iterationRanges = new List<Tuple<TrustData, int>>();
            iterationRanges.Add(Tuple.Create(trustDataList.First(), 0));
            // 基準フレームが複数の場合
            if (trustDataList.Count >= 2)
            {
                for (int i = 0; i < trustDataList.Count; i++)
                {
                    TrustData curr = trustDataList[i];
                    TrustData next = trustDataList[i + 1];
                    int halfIndex = (curr.frameIndex + next.frameIndex) / 2;
                    iterationRanges.Add(Tuple.Create(curr, halfIndex));
                    iterationRanges.Add(Tuple.Create(next, halfIndex + 1));
                }
            }
            iterationRanges.Add(Tuple.Create(trustDataList.Last(), frameLength - 1));
            return iterationRanges;
        }

        /// <summary>
        /// 昇順かどうかを判定して適切なRangeを生成する
        /// </summary>
        /// <param name="first"></param>
        /// <param name="second"></param>
        /// <returns></returns>
        private IEnumerable<int> GenerateContinuousRange(int first, int second)
        {
            if (first < second)
            {
                return Enumerable.Range(first, second);
            }
            else
            {
                return Enumerable.Reverse(Enumerable.Range(second, first));
            }
        }

        public void Correct(FrameSequence frameSeq)
        {
            // generate iteration
            List<Tuple<TrustData, int>> iterations = this.GenerateIterationRanges(frameSeq.Frames.Count(), frameSeq.trustData);
            foreach (Tuple<TrustData, int> iterationRange in iterations)
            {
                // set and calcurate pivot
                TrustData trustData = iterationRange.Item1;
                SerializableBody pivotBody = trustData.GetBody(frameSeq.Frames);
                // translate to world coordinate
                Dictionary<JointType, CvPoint3D64f> pivotJoints = pivotBody.Joints.ToDictionary(p => p.Key, 
                    p => CvEx.ConvertPoint3D(p.Value.Position.ToCvPoint3D(), frameSeq.ToWorldConversions[trustData.recordIndex]));
                CvPoint3D64f pivotBodyCrossVector = this.BodyCrossVector(pivotJoints);
                pivotBody.bodyCrossVector = pivotBodyCrossVector.ToArrayPoints();
                double bodyAngle = this.BodyAngle(pivotBodyCrossVector);
                pivotBody.bodyAngle = bodyAngle;
                // 繰り返し範囲の連続indexを生成して回す
                foreach (int frameIndex in this.GenerateContinuousRange(trustData.frameIndex, iterationRange.Item2))
                {
                    for (int recordNo = 0; recordNo < frameSeq.recordNum; recordNo++)
                    {
                        // pivotと一致した場合
                        if (trustData.recordIndex == recordNo && trustData.frameIndex == frameIndex)
                        {
                            continue;
                        }

                    }
                }
                
            }
            // 3. vector, angle
            // 4. reverse
            // 5. occulusion
            // 6. normalize
        }
    }
}
