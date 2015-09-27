using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Microsoft.Kinect;
using OpenCvSharp;

namespace KinectMotionCapture
{
    struct PointAndColor
    {
        public CvPoint3D64f point;
        public CvColor color;
        public JointType nearestJoint;
        public double distance;
        public PointAndColor(CvPoint3D64f point, CvColor color, JointType nearest, double distance)
        {
            this.point = point;
            this.color = color;
            this.nearestJoint = nearest;
            this.distance = distance;
        }

        /// <summary>
        /// dump用
        /// </summary>
        /// <returns></returns>
        public float[] ToFloatArr()
        {
            return new float[] { (float)point.X, (float)point.Y, (float)point.Z, color.R, color.G, color.B };
        }

        public Tuple<CvPoint3D64f, CvColor> ToPair()
        {
            return Tuple.Create(point, color);
        }
    }

    class VotingSpace
    {
        public float unit;           // 最小単位
        public float min, max;       // min, max 正の値である
        public int side;             // 立方体gridの一辺の長さ
        public int maxNum;           // 最大投票個数
        public CvPoint3D64f offset;  // 座標変換のオフセット
        public int[] grids;
        public VotingSpace(float unit, float min, float max, CvPoint3D64f offset)
        {
            this.unit = unit;
            this.min = min; this.max = max;
            this.side = (int)((max - min) / unit);
            this.grids = new int[side * side * side];
            this.maxNum = -1;
            this.offset = offset;
        }
        public void vote(CvPoint3D64f point)
        {
            point += offset;
            if ( point.X < min || point.Y < min || point.Z < min ||
                 point.X > max || point.Y > max || point.Z > max )
            {
                return;
            }
            int x = (int)(point.X / unit);
            int y = (int)(point.Y / unit);
            int z = (int)(point.Z / unit);
            int count = ++grids[x + y * side + z * side * side];
            if (this.maxNum < count)
                this.maxNum = count;
        }
        public void voteByList(List<CvPoint3D64f> points)
        {
            points.ForEach(point => this.vote(point));
        }
        public CvPoint3D64f fromIndex(int index)
        {
            int z = index / (side * side);
            index = index % (side * side);
            int y = index / side;
            int x = index % side;
            return new CvPoint3D64f(x * unit, y * unit, z * unit);
        }
        public List<CvPoint3D64f> OutputByThreshold(int threshold)
        {
            List<CvPoint3D64f> res = new List<CvPoint3D64f>();
            foreach(var pair in this.grids.Select((count, index) => new { count, index }))
            {
                if(pair.count >= threshold)
                {
                    CvPoint3D64f p = this.fromIndex(pair.index) - offset;
                    res.Add(p);
                }
            }
            return res;
        }
    }

    class PointRefiner
    {
        Dictionary<JointType, CvPoint3D64f> standardJoints;
        Dictionary<JointType, List<PointAndColor>> pointColorStore;

        public Dictionary<JointType, List<PointAndColor>> PointColors
        {
            get
            {
                return pointColorStore;
            }
        }

        StandardSkeleton standardSkeleton;
        public VotingSpace votingSpace;
        // 骨と表面情報を受け取って、標準骨格のまわりにマッピングしていく。
        // とりあえず胴体近傍を出力することが目標
        /// <summary>
        /// こんすとらくたん
        /// 標準骨格を与えられた骨格と骨の長さ情報をもとに作っておく
        /// </summary>
        /// <param name="skltn"></param>
        /// <param name="standardPose"></param>
        public PointRefiner(StandardSkeleton skltn, Dictionary<JointType, CvPoint3D64f> standardPose)
        {
            this.standardSkeleton = skltn;
            this.standardJoints = this.standardSkeleton.FixBoneLength(standardPose).ConvertJointsByJointType(JointType.SpineBase);
            this.pointColorStore = new Dictionary<JointType, List<PointAndColor>>();
            this.votingSpace = new VotingSpace(0.005f, 0f, 2f, new CvPoint3D64f(1f, 1f, 1f));
        }

        /// <summary>
        /// 最近傍の関節を見つけてデータを貯める
        /// </summary>
        /// <param name="rawData"></param>
        public void AddData(List<Tuple<CvPoint3D64f, CvColor>> rawData)
        {
            this.votingSpace.voteByList(rawData.Select(tuple => tuple.Item1).ToList());
            //var pacList = rawData.AsParallel().Select(pair => this.ConvertToPointAndColor(pair));
            //foreach (PointAndColor pac in pacList)
            //{
            //    if (!this.pointColorStore.ContainsKey(pac.nearestJoint))
            //    {
            //        this.pointColorStore[pac.nearestJoint] = new List<PointAndColor>() { pac };
            //    }
            //    else
            //    {
            //        this.pointColorStore[pac.nearestJoint].Add(pac);
            //    }
            //}
        }

        /// <summary>
        /// 変換するやつ
        /// </summary>
        /// <param name="pointColorPair"></param>
        /// <returns></returns>
        public PointAndColor ConvertToPointAndColor(Tuple<CvPoint3D64f, CvColor> pointColorPair)
        {
            Dictionary<JointType, double> distanceSqMap = new Dictionary<JointType, double>();
            foreach (JointType joint in this.standardJoints.Keys)
            {
                distanceSqMap[joint] = CvEx.GetDistanceSq(pointColorPair.Item1, this.standardJoints[joint]);
            }
            var pair = distanceSqMap.OrderBy(p => p.Value).First();
            return new PointAndColor(pointColorPair.Item1, pointColorPair.Item2, pair.Key, pair.Value);
        }
    }
}
