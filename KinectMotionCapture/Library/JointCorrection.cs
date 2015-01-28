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
        /// 右胸の外積ベクトル
        /// </summary>
        /// <param name="body"></param>
        /// <returns></returns>
        private CvPoint3D64f CalcRightChestCrossVector(Dictionary<JointType, CvPoint3D64f> joints)
        {
            if (joints.ContainsKey(JointType.SpineBase) && joints.ContainsKey(JointType.ShoulderRight))
            {
                CvPoint3D64f torsoToRightShoulder = joints[JointType.ShoulderRight] - joints[JointType.SpineMid];
                CvPoint3D64f spine = joints[JointType.SpineShoulder] - joints[JointType.SpineMid];
                CvPoint3D64f bodyCross = CvEx.Cross(spine, torsoToRightShoulder);
                return CvEx.Normalize(bodyCross);
            }
            return default(CvPoint3D64f);
        }

        /// <summary>
        /// 左胸の外積ベクトル
        /// </summary>
        /// <param name="body"></param>
        /// <returns></returns>
        private CvPoint3D64f CalcLeftChestCrossVector(Dictionary<JointType, CvPoint3D64f> joints)
        {
            if (joints.ContainsKey(JointType.SpineBase) && joints.ContainsKey(JointType.ShoulderLeft))
            {
                CvPoint3D64f torsoToLeftShoulder = joints[JointType.ShoulderLeft] - joints[JointType.SpineMid];
                CvPoint3D64f spine = joints[JointType.SpineShoulder] - joints[JointType.SpineMid];
                CvPoint3D64f bodyCross = CvEx.Cross(torsoToLeftShoulder, spine);
                return CvEx.Normalize(bodyCross);
            }
            return default(CvPoint3D64f);
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
        /// ミラー状態を補正するための基準骨格フレーム複数から、比較すべきフレームの範囲を決定する
        /// </summary>
        /// <param name="frameLength"></param>
        /// <param name="trustDataList"></param>
        /// <returns></returns>
        private List<Tuple<TrustData, int>> GenerateIterationRanges(int frameLength, List<TrustData> trustDataList)
        {
            trustDataList.Sort((a, b) => a.frameIndex - b.frameIndex);
            List<Tuple<TrustData, int>> iterationRanges = new List<Tuple<TrustData, int>>();
            iterationRanges.Add(Tuple.Create(trustDataList.First(), 0));
            // 基準フレームが複数の場合
            if (trustDataList.Count >= 2)
            {
                for (int i = 0; i < trustDataList.Count() - 1; i++)
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
                return Enumerable.Range(first, second - first + 1);
            }
            else
            {
                return Enumerable.Reverse(Enumerable.Range(second, first - second + 1));
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
                CvPoint3D64f pivotBodyRightVector = this.CalcRightChestCrossVector(pivotJoints);
                CvPoint3D64f pivotBodyLeftVector = this.CalcLeftChestCrossVector(pivotJoints);
                CvPoint3D64f pivotBodyCrossVector = CvEx.Normalize(pivotBodyRightVector + pivotBodyLeftVector);
                // z軸との角度 +だったらあっち向いてる
                double pivotCrossZCos = CvEx.Cos(pivotBodyCrossVector, new CvPoint3D64f(0, 0, 1));
                // ので、反転してたら修正する
                if (pivotCrossZCos > 0)
                {
                    pivotBody.InverseJoints();
                    pivotJoints = pivotBody.Joints.ToDictionary(p => p.Key,
                    p => CvEx.ConvertPoint3D(p.Value.Position.ToCvPoint3D(), frameSeq.ToWorldConversions[trustData.recordIndex]));
                    pivotBodyRightVector = this.CalcRightChestCrossVector(pivotJoints);
                    pivotBodyLeftVector = this.CalcLeftChestCrossVector(pivotJoints);
                    pivotBodyCrossVector = CvEx.Normalize(pivotBodyRightVector + pivotBodyLeftVector);
                }

                // 繰り返し範囲の連続indexを生成して回す
                IEnumerable<int> continuousRange = this.GenerateContinuousRange(trustData.frameIndex, iterationRange.Item2);
                foreach (int frameIndex in continuousRange)
                {
                    // 前のpivotとのベクトルの差が小さいやつを選んでいく投票空間
                    double[] bodyCos = new double[frameSeq.recordNum];
                    CvPoint3D64f[] bodyCrosses = new CvPoint3D64f[frameSeq.recordNum];
                    for (int recordNo = 0; recordNo < frameSeq.recordNum; recordNo++)
                    {
                        // pivotと一致した場合
                        if (trustData.recordIndex == recordNo && trustData.frameIndex == frameIndex)
                        {
                            bodyCos[recordNo] = 1;
                            bodyCrosses[recordNo] = pivotBodyCrossVector;
                            continue;
                        }
                        SerializableBody body = frameSeq.Frames[frameIndex].GetSelectedBody(recordNo, integratedId: trustData.integratedBodyId);
                        if (body == null || body == default(SerializableBody) || body.Joints.Count == 0)
                        {
                            bodyCos[recordNo] = -1;
                            continue;
                        }
                        Dictionary<JointType, CvPoint3D64f> joints = body.Joints.ToDictionary(p => p.Key,
                            p => CvEx.ConvertPoint3D(p.Value.Position.ToCvPoint3D(), frameSeq.ToWorldConversions[recordNo]));
                        // 右胸、左胸の外積ベクトル（正規化済み）
                        CvPoint3D64f rightVector = this.CalcRightChestCrossVector(joints);
                        CvPoint3D64f leftVector = this.CalcLeftChestCrossVector(joints);
                        // 前フレームの基準ベクトルとの角度(cos)
                        double bothCrossAngle = CvEx.Cos(rightVector, leftVector);
                        double rightPivotAngle = CvEx.Cos(rightVector, pivotBodyCrossVector);
                        double leftPivotAngle = CvEx.Cos(leftVector, pivotBodyCrossVector);
                        bool removedFlag = false;
                        // そもそも骨がなかった場合
                        if (rightVector == default(CvPoint3D64f))
                        {
                            body.RemoveJoints(Utility.RightBody.ToList());
                            removedFlag = true;
                        }
                        if (leftVector == default(CvPoint3D64f))
                        {
                            body.RemoveJoints(Utility.LeftBody.ToList());
                            removedFlag = true;
                        }
                        // 右と左のベクトルが離れすぎてる場合
                        if (bothCrossAngle <= 0)
                        {
                            body.RemoveJoints(Utility.Body.ToList());
                            removedFlag = true;
                        }
                        if (removedFlag)
                        {
                            bodyCos[recordNo] = -1;
                            continue;
                        }

                        CvPoint3D64f bodyCrossVector = CvEx.Normalize(rightVector + leftVector);
                        double bodyCrossdiff = CvEx.Cos(bodyCrossVector, pivotBodyCrossVector);
                        // reverse check
                        if (bodyCrossdiff <= -0.8)
                        {
                            // reverse and update
                            body.InverseJoints();
                            joints = body.Joints.ToDictionary(p => p.Key,
                                p => CvEx.ConvertPoint3D(p.Value.Position.ToCvPoint3D(), frameSeq.ToWorldConversions[recordNo]));
                            rightVector = this.CalcRightChestCrossVector(joints);
                            leftVector = this.CalcLeftChestCrossVector(joints);
                            bodyCrossVector = CvEx.Normalize(rightVector + leftVector);
                            bodyCrossdiff = CvEx.Cos(bodyCrossVector, pivotBodyCrossVector);
                        }
                        // update body angle
                        bodyCos[recordNo] = bodyCrossdiff;
                        bodyCrosses[recordNo] = bodyCrossVector;
                    }
                    // 光軸と胸の正面方向が逆、-1に近いほどよい
                    int pivotRecordNo = bodyCos.ToList().IndexOf(bodyCos.Min());
                    // update pivot body angle, vector TODO
                    //pivotBody = frameSeq.Frames[frameIndex].GetSelectedBody(pivotRecordNo, integratedId: trustData.integratedBodyId);
                }
            }
        }


        public void CleanAndNormalize(FrameSequence frameSeq)
        {
            // TODO : 
            //body.Joints = this.NormalizeLegJoints(body.Joints, frameSeq.BodyStat.boneLengthSqStatistics);
        }
    }
}
