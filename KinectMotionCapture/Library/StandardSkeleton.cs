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
    /// Jointの接続関係をアレする
    /// </summary>
    [Serializable]
    public struct JointNode
    {
        public JointNode[] nextNodes;
        public JointType from;
        public JointType to;
        public JointNode(JointType from, JointType to, JointNode[] next)
        {
            this.from = from;
            this.to = to;
            this.nextNodes = next;
        }
        public bool isRoot()
        {
            return this.from == default(JointType) && this.to == default(JointType);
        }
    }

    [Serializable]
    class StandardSkeleton
    {
        Dictionary<Bone, double> boneLengthes;
        JointNode rootNode;
        // TODO : SpineBaseの長さから比率を計算する標準骨格生成も必要かもしれない
        public StandardSkeleton(Dictionary<Bone, BoneStatistics> boneStats)
        {
            this.boneLengthes = boneStats.ToDictionary(p => p.Key, p => p.Value.medianAverageLength);
            // nodeの接続関係をイニシャライズ            
            this.rootNode = this.InitializeNodes();
        }

        /// <summary>
        /// 骨の長さを探してくる
        /// </summary>
        /// <param name="from"></param>
        /// <param name="to"></param>
        /// <returns></returns>
        private double SearchBoneLength(JointType from, JointType to)
        {
            Bone fromTo = Tuple.Create(from, to);
            Bone toFrom = Tuple.Create(to, from);
            double length;
            if (this.boneLengthes.TryGetValue(fromTo, out length))
            {
                return length;
            }
            else
            {
                return this.boneLengthes[toFrom];
            }
        }


        /// <summary>
        /// 長さの調整された関節点をつくる
        /// </summary>
        /// <param name="origFrom"></param>
        /// <param name="origTo"></param>
        /// <param name="normedFrom"></param>
        /// <param name="length"></param>
        /// <returns></returns>
        private CvPoint3D64f GetFixedJoint(CvPoint3D64f origFrom, CvPoint3D64f origTo, CvPoint3D64f normedFrom, double length)
        {
            return CvEx.Normalize(origTo - origFrom) * length + normedFrom;
        }

        /// <summary>
        /// 骨の長さをなおす。ぜんぶ。
        /// </summary>
        public Dictionary<JointType, CvPoint3D64f> FixBoneLength(Dictionary<JointType, CvPoint3D64f> originalJoints)
        {
            Stack<JointNode> stack = new Stack<JointNode>();
            stack.Clear();
            stack.Push(this.rootNode);
            Dictionary<JointType, CvPoint3D64f> newJoints = new Dictionary<JointType, CvPoint3D64f>();
            newJoints[JointType.SpineBase] = originalJoints[JointType.SpineBase];
            while (stack.Count() != 0)
            {
                JointNode currentNode = stack.Pop();
                // rootじゃなかったら今のノードに対して骨の修正
                if (!currentNode.isRoot())
                {
                    JointType from = currentNode.from;
                    JointType to = currentNode.to;
                    double length = this.SearchBoneLength(from, to);
                    CvPoint3D64f newToPoint = this.GetFixedJoint(originalJoints[from], originalJoints[to], newJoints[from], length);
                    newJoints[to] = newToPoint;
                }
                foreach (JointNode childNode in currentNode.nextNodes)
                {
                    stack.Push(childNode);
                }
            }
            return newJoints;
        }

        /// <summary>
        /// いにしゃらいず
        /// </summary>
        /// <returns></returns>
        private JointNode InitializeNodes()
        {
            JointNode neckHead = new JointNode(JointType.Neck, JointType.Head, new JointNode[] { });
            JointNode spineShoulderNeck = new JointNode(JointType.SpineShoulder, JointType.Neck, new JointNode[] { neckHead });

            JointNode elbowWristLeft = new JointNode(JointType.ElbowLeft, JointType.WristLeft, new JointNode[] { });
            JointNode shoulderElbowLeft = new JointNode(JointType.ShoulderLeft, JointType.ElbowLeft, new JointNode[] { elbowWristLeft });
            JointNode spineShoulderLeft = new JointNode(JointType.SpineShoulder, JointType.ShoulderLeft, new JointNode[] { shoulderElbowLeft });

            JointNode elbowWristRight = new JointNode(JointType.ElbowRight, JointType.WristRight, new JointNode[] { });
            JointNode shoulderElbowRight = new JointNode(JointType.ShoulderRight, JointType.ElbowRight, new JointNode[] { elbowWristRight });
            JointNode spineShoulderRight = new JointNode(JointType.SpineShoulder, JointType.ShoulderRight, new JointNode[] { shoulderElbowRight });

            JointNode ankleFootLeft = new JointNode(JointType.AnkleLeft, JointType.FootLeft, new JointNode[] { });
            JointNode kneeAnkleLeft = new JointNode(JointType.KneeLeft, JointType.AnkleLeft, new JointNode[] { ankleFootLeft });
            JointNode hipKneeLeft = new JointNode(JointType.HipLeft, JointType.KneeLeft, new JointNode[] { kneeAnkleLeft });
            JointNode spineHipLeft = new JointNode(JointType.SpineBase, JointType.HipLeft, new JointNode[] { hipKneeLeft });

            JointNode ankleFootRight = new JointNode(JointType.AnkleRight, JointType.FootRight, new JointNode[] { });
            JointNode kneeAnkleRight = new JointNode(JointType.KneeRight, JointType.AnkleRight, new JointNode[] { ankleFootRight });
            JointNode hipKneeRight = new JointNode(JointType.HipRight, JointType.KneeRight, new JointNode[] { kneeAnkleRight });
            JointNode spineHipRight = new JointNode(JointType.SpineBase, JointType.HipRight, new JointNode[] { hipKneeRight });

            JointNode spineMidShoulder = new JointNode(JointType.SpineMid, JointType.SpineShoulder, new JointNode[] { spineShoulderNeck, spineShoulderRight, spineShoulderLeft });
            JointNode spineBaseMid = new JointNode(JointType.SpineBase, JointType.SpineMid, new JointNode[] { spineMidShoulder });
            JointNode root = new JointNode(default(JointType), default(JointType), new JointNode[] { spineBaseMid, spineHipRight, spineHipLeft });

            return root;
        }
    }
}
