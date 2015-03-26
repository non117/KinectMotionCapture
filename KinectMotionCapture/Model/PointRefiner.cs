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
    }

    class PointRefiner
    {
        Dictionary<JointType, CvPoint3D64f> standardJoints;
        Dictionary<JointType, List<PointAndColor>> pointColorStore;
        StandardSkeleton standardSkeleton;
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
            this.standardJoints = this.standardSkeleton.FixBoneLength(standardPose);
            this.pointColorStore = new Dictionary<JointType, List<PointAndColor>>();
            foreach (JointType key in this.pointColorStore.Keys)
            {
                this.pointColorStore[key] = new List<PointAndColor>();
            }
        }

        /// <summary>
        /// 最近傍の関節を見つけてデータを貯める
        /// </summary>
        /// <param name="rawData"></param>
        public void AddData(List<Tuple<CvPoint3D64f, CvColor>> rawData)
        {
            foreach (var pointColorPair in rawData)
            {
                PointAndColor pac = this.ConvertToPointAndColor(pointColorPair);
                this.pointColorStore[pac.nearestJoint].Add(pac);
            }
        }

        public PointAndColor ConvertToPointAndColor(Tuple<CvPoint3D64f, CvColor> pointColorPair)
        {
            
            
        }
    }
}
