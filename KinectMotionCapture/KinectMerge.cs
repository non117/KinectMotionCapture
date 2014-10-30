using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using OpenCvSharp;

namespace KinectMotionCapture
{
    class KinectMerge
    {
        /// <summary>
        /// あるフレームにおける座標の補正行列を返す
        /// </summary>
        /// <param name="colorMatList"></param>
        /// <param name="depthMatList"></param>
        /// <param name="convList"></param>
        /// <returns></returns>
        public static List<CvMat> AjustDepthPoints(List<CvMat> colorMatList, List<CvMat> depthMatList, List<CvMat> convList)
        {
            List<Func<CvPoint3D64f, CvPoint3D64f>> toReal = new List<Func<CvPoint3D64f, CvPoint3D64f>>();
            foreach (CvMat depthMat in depthMatList)
            {
                toReal.Add((x) => KinectUndistortion.GetOriginalRealFromScreenPos(x, new CvSize(depthMat.Cols, depthMat.Rows)));
            }
            Func<float, double> distance2weight = x => 1.0 / (x * 0 + 400);
            using (ColoredIterativePointMatching sipm = new ColoredIterativePointMatching(depthMatList, colorMatList, toReal, convList, distance2weight, 200))
            {
                List<CvMat> conversions = sipm.CalculateTransformSequntially(0.2, 3);
                return conversions;
            }
        }
    }
}

