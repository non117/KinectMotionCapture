using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Microsoft.Kinect;

namespace KinectMotionCapture
{
    [Serializable]
    class LocalCoordinateMapper
    {
        private int depthMax = 8000;
        private PointF[] depthFrameToCameraSpaceTable = null;
        private float[] depthToColorYTable = null;
        private float[][] depthToColorXTable = null;
        private int depthWidth = 0;
        private int depthHeight = 0;

        /// <summary>
        /// 深度画像座標からワールド空間座標へ変換するやつ
        /// </summary>
        /// <param name="depthSpacePoint"></param>
        /// <param name="depth"></param>
        /// <returns></returns>
        public CameraSpacePoint MapDepthPointToCameraSpace(DepthSpacePoint depthSpacePoint, ushort depth)
        {
            float distance = depth / 1000;
            int index = (int)(this.depthWidth * depthSpacePoint.Y + depthSpacePoint.X);
            PointF cameraPoint = this.depthFrameToCameraSpaceTable[index];
            return new CameraSpacePoint() { X = cameraPoint.X * distance, Y = cameraPoint.Y * distance, Z = distance };
        }

        /// <summary>
        /// 深度画像座標からカラー画像座標へ変換するやつ
        /// </summary>
        /// <param name="depthSpacePoint"></param>
        /// <param name="depth"></param>
        /// <returns></returns>
        public ColorSpacePoint MapDepthPointToColorSpace(DepthSpacePoint depthSpacePoint, ushort depth)
        {
            int index = (int)(this.depthWidth * depthSpacePoint.Y + depthSpacePoint.X);
            float y = this.depthToColorYTable[index];
            float x = this.depthToColorXTable[(int)depth][index];
            return new ColorSpacePoint() { X = x, Y = y };
        }

        public LocalCoordinateMapper(CoordinateMapper originalCoordinateMapper, int depthWidth, int depthHeight)
        {
            this.depthWidth = depthWidth;
            this.depthHeight = depthHeight;
            this.depthFrameToCameraSpaceTable = originalCoordinateMapper.GetDepthFrameToCameraSpaceTable();

            ushort[] depthBuffer = new ushort[this.depthHeight * this.depthWidth];
            ColorSpacePoint[] colorSpacePoints = new ColorSpacePoint[this.depthHeight * this.depthWidth];

            this.depthToColorYTable = new float[this.depthHeight * this.depthWidth];
            this.depthToColorXTable = new float[this.depthMax][];

            using (System.IO.StreamWriter sw = new System.IO.StreamWriter("hoge.csv", false, System.Text.Encoding.UTF8))
            {
                for (int d = 0; d < this.depthMax; d += 10)
                {
                    for (int i = 0; i < depthBuffer.Count(); i++)
                    {
                        depthBuffer[i] = (ushort)d;
                    }
                    originalCoordinateMapper.MapDepthFrameToColorSpace(depthBuffer, colorSpacePoints);
                    // d == 0のときにはinfinityがかえってきていることに注意
                    if (d == 1)
                    {
                        this.depthToColorYTable = colorSpacePoints.Select((ColorSpacePoint csp) => csp.Y).ToArray();
                    }
                    sw.Write(d.ToString() + "," + string.Join(",", colorSpacePoints.Take(100).Select((ColorSpacePoint csp) => csp.X).ToList()) + "\n");
                    //this.depthToColorXTable[d] = colorSpacePoints.Select((ColorSpacePoint csp) => csp.X).ToArray();
                }
            }
        }
    }
}
