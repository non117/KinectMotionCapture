using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Microsoft.Kinect;
using OpenCvSharp;

namespace KinectMotionCapture
{
    [Serializable]
    public class LocalCoordinateMapper
    {
        // Kinect2の内部パラメータを推定したやつ
        private double D = 56231.3471302;
        private PointF[] depthFrameToCameraSpaceTable = null;
        private PointF[] depthFrameToColorSpacfeTable = null;
        private int originalColorWidth = 1920;
        private int originalColorHeight = 1080;
        private int depthWidth = 0;
        private int depthHeight = 0;

        /// <summary>
        /// 変換テーブルをLUTから生成する。パラメータはKinect2から推定
        /// </summary>
        /// <param name="xtable"></param>
        /// <param name="ytable"></param>
        private void GenerateDepthToColorTable()
        {
            this.depthFrameToColorSpacfeTable = new PointF[this.depthHeight * this.depthWidth];            
            for (int i = 0; i < this.depthFrameToCameraSpaceTable.Count(); i++)
            {
                PointF lut = this.depthFrameToCameraSpaceTable[i];
                PointF tempP = new PointF();
                tempP.X = (float)(lut.X * 1064.01339491 + 961.22925594);
                tempP.Y = (float)(lut.Y * -1066.00639947 + 544.7353984);
                this.depthFrameToColorSpacfeTable[i] = tempP;
            }
        }

        /// <summary>
        /// 深度画像座標からワールド空間座標へ変換するやつ
        /// </summary>
        /// <param name="depthSpacePoint"></param>
        /// <param name="depth"></param>
        /// <returns></returns>
        public CameraSpacePoint MapDepthPointToCameraSpace(DepthSpacePoint depthSpacePoint, ushort depth)
        {
            float distance = (float)depth / 1000;
            int index = (int)(this.depthWidth * depthSpacePoint.Y + depthSpacePoint.X);
            PointF cameraPoint = this.depthFrameToCameraSpaceTable[index];
            return new CameraSpacePoint() { X = cameraPoint.X * distance, Y = cameraPoint.Y * distance, Z = distance };
        }

        /// <summary>
        /// 深度画像座標からワールド空間座標へ
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="depth"></param>
        /// <returns></returns>
        public CameraSpacePoint MapDepthPointToCameraSpace(int x, int y, ushort depth)
        {
            return this.MapDepthPointToCameraSpace(new DepthSpacePoint() { X = x, Y = y }, depth);
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
            PointF lut = this.depthFrameToColorSpacfeTable[index];
            ColorSpacePoint tempP = new ColorSpacePoint();
            tempP.Y = lut.Y;
            tempP.X = lut.X + (float)(this.D / depth);
            return tempP;
        }

        /// <summary>
        /// 深度画像座標からカラー画像座標へ
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="depth"></param>
        /// <returns></returns>
        public ColorSpacePoint MapDepthPointToColorSpace(int x, int y, ushort depth)
        {
            return this.MapDepthPointToColorSpace(new DepthSpacePoint() { X = x, Y = y }, depth);
        }

        /// <summary>
        /// 縮小されてた場合に対応する深度TOカラー変換
        /// </summary>
        /// <param name="depthSpacePoint"></param>
        /// <param name="depth"></param>
        /// <param name="colorWidth"></param>
        /// <param name="colorHeight"></param>
        /// <returns></returns>
        public ColorSpacePoint MapDepthPointToColorSpace(DepthSpacePoint depthSpacePoint, ushort depth, int colorWidth, int colorHeight)
        {
            ColorSpacePoint csp = this.MapDepthPointToColorSpace(depthSpacePoint, depth);
            csp.X = csp.X * colorWidth / this.originalColorWidth;
            csp.Y = csp.Y * colorHeight / this.originalColorHeight;
            return csp;
        }

        /// <summary>
        /// 縮小対策入り深度TOカラー
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="depth"></param>
        /// <param name="colorWidth"></param>
        /// <param name="colorHeight"></param>
        /// <returns></returns>
        public ColorSpacePoint MapDepthPointToColorSpace(int x, int y, ushort depth, int colorWidth, int colorHeight)
        {
            return this.MapDepthPointToColorSpace(new DepthSpacePoint() { X = x, Y = y }, depth, colorWidth, colorHeight);
        }

        public List<Tuple<CvPoint3D64f, CvColor>> GetUserColorPoints(CvMat colorMat, CvMat depthMat, CvMat userMat)
        {
            Dictionary<int, List<Tuple<CvPoint3D64f, CvColor>>> eachUserColorPoints = this.GetEachUserColorPoints(colorMat, depthMat, userMat);
            return eachUserColorPoints.Values.SelectMany(l => l).ToList();
        }

        /// <summary>
        /// ユーザのボクセルを返す
        /// </summary>
        /// <param name="depthData"></param>
        /// <param name="colorFrameData"></param>
        /// <param name="bodyIndexFrameData"></param>
        /// <param name="colorSize"></param>
        /// <returns></returns>
        public Dictionary<int, List<Tuple<CvPoint3D64f, CvColor>>> GetEachUserColorPoints(CvMat colorMat, CvMat depthMat, CvMat userMat)
        {
            Dictionary<int, List<Tuple<CvPoint3D64f, CvColor>>> res = new Dictionary<int, List<Tuple<CvPoint3D64f, CvColor>>>();
            List<Tuple<CvPoint3D64f, CvColor>> lis;
            int bytesPerPixel = colorMat.ElemChannels;
            CvSize colorSize = colorMat.GetSize();
            unsafe
            {
                byte* colorArr = colorMat.DataByte;
                short* depthArr = depthMat.DataInt16;                
                short* userArr = userMat.DataInt16;

                for (int y = 0; y < depthHeight; ++y)
                {
                    for (int x = 0; x < depthWidth; ++x)
                    {                        
                        int depthIndex = (y * depthWidth) + x;
                        short player = userArr[depthIndex];
                        // TODO : check
                        if (player != 255)
                        {
                            ushort depthVal = (ushort)depthArr[depthIndex];
                            ColorSpacePoint colorPoint = this.MapDepthPointToColorSpace(x, y, depthVal, colorSize.Width, colorSize.Height);
                            CameraSpacePoint cameraPoint = this.MapDepthPointToCameraSpace(x, y, depthVal);
                            // make sure the depth pixel maps to a valid point in color space
                            int colorX = (int)Math.Floor(colorPoint.X + 0.5);
                            int colorY = (int)Math.Floor(colorPoint.Y + 0.5);
                            if ((colorX >= 0) && (colorX < colorSize.Width) && (colorY >= 0) && (colorY < colorSize.Height))
                            {
                                // calculate index into color array
                                int colorIndex = ((colorY * colorSize.Width) + colorX) * bytesPerPixel;
                                CvColor color = new CvColor(colorArr[colorIndex + 2], colorArr[colorIndex + 1], colorArr[colorIndex + 0]);
                                if (res.TryGetValue((int)player, out lis))
                                {
                                    res[(int)player].Add(Tuple.Create((CvPoint3D64f)cameraPoint.ToCvPoint3D(), color));
                                }
                                else
                                {
                                    res[(int)player] = new List<Tuple<CvPoint3D64f, CvColor>>();
                                    res[(int)player].Add(Tuple.Create((CvPoint3D64f)cameraPoint.ToCvPoint3D(), color));
                                }
                            }
                        }
                    }
                }
            }
            return res;
        }

        /// <summary>
        /// 深度データからリアルワールドの色を見つける
        /// </summary>
        /// <param name="depthData"></param>
        /// <param name="colorFrameData"></param>
        /// <param name="colorSize"></param>
        /// <returns></returns>
        public List<Tuple<CvPoint3D64f, CvColor>> DepthColorMatToRealPoints(CvMat colorMat, CvMat depthMat)
        {
            List<Tuple<CvPoint3D64f, CvColor>> res = new List<Tuple<CvPoint3D64f, CvColor>>();
            int bytesPerPixel = colorMat.ElemChannels;
            CvSize colorSize = colorMat.GetSize();
            unsafe
            {
                short* depthArr = depthMat.DataInt16;
                byte* colorArr = colorMat.DataByte;                
                
                for (int y = 0; y < depthHeight; ++y)
                {
                    for (int x = 0; x < depthWidth; ++x)
                    {
                        int depthIndex = (y * depthWidth) + x;
                        ushort depthVal = (ushort)depthArr[depthIndex];
                        ColorSpacePoint colorPoint = this.MapDepthPointToColorSpace(x, y, depthVal, colorSize.Width, colorSize.Height);
                        CameraSpacePoint cameraPoint = this.MapDepthPointToCameraSpace(x, y, depthVal);
                        // make sure the depth pixel maps to a valid point in color space
                        int colorX = (int)Math.Floor(colorPoint.X + 0.5);
                        int colorY = (int)Math.Floor(colorPoint.Y + 0.5);
                        if ((colorX >= 0) && (colorX < colorSize.Width) && (colorY >= 0) && (colorY < colorSize.Height))
                        {
                            // calculate index into color array
                            int colorIndex = ((colorY * colorSize.Width) + colorX) * bytesPerPixel;
                            CvColor color = new CvColor(colorArr[colorIndex + 2], colorArr[colorIndex + 1], colorArr[colorIndex + 0]);
                            res.Add(Tuple.Create((CvPoint3D64f)cameraPoint.ToCvPoint3D(), color));
                        }
                    }
                }
            }
            return res;
        }

        /// <summary>
        /// オフラインでLUTだけ与えるコンストラクタ
        /// </summary>
        /// <param name="depthFrameToCameraSpaceTable"></param>
        /// <param name="depthWidth"></param>
        /// <param name="depthHeight"></param>
        public LocalCoordinateMapper(PointF[] depthFrameToCameraSpaceTable, int depthWidth, int depthHeight)
        {
            this.depthWidth = depthWidth;
            this.depthHeight = depthHeight;
            this.depthFrameToCameraSpaceTable = depthFrameToCameraSpaceTable;
            this.GenerateDepthToColorTable();
        }

        /// <summary>
        /// Kinectにつないでる時だけ使えるコンストラクタ
        /// </summary>
        /// <param name="originalCoordinateMapper"></param>
        /// <param name="depthWidth"></param>
        /// <param name="depthHeight"></param>
        public LocalCoordinateMapper(CoordinateMapper originalCoordinateMapper, int depthWidth, int depthHeight)
        {
            this.depthWidth = depthWidth;
            this.depthHeight = depthHeight;
            this.depthFrameToCameraSpaceTable = originalCoordinateMapper.GetDepthFrameToCameraSpaceTable();

            int length = depthWidth * depthHeight;
            ushort[] depthBuffer = new ushort[length];
            ColorSpacePoint[] colorSpacePoints = new ColorSpacePoint[length];            
            this.depthFrameToColorSpacfeTable = new PointF[length];

            int depth = 1500; // なんでもいい
            for (int i = 0; i < length; i++)
            {
                depthBuffer[i] = (ushort)depth;
            }
            originalCoordinateMapper.MapDepthFrameToColorSpace(depthBuffer, colorSpacePoints);
            for (int i = 0; i < length; i++)
            {
                PointF tempP = new PointF();
                tempP.X = (float)(colorSpacePoints[i].X - D / depth);
                tempP.Y = colorSpacePoints[i].Y;
                this.depthFrameToColorSpacfeTable[i] = tempP;
            }
            Debug.WriteLine(1);
        }

        public void dump(string path = "")
        {
            if (path == "")
            {
                path = "coordmapper.dump";
            }
            Utility.SaveToBinary(this, path);
        }
    }
}
