﻿using System;
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

        /// <summary>
        /// 深度とカラーMatから点群の座標と色を返す
        /// </summary>
        /// <param name="depthMat"></param>
        /// <param name="colorMat"></param>
        /// <returns></returns>
        public List<Tuple<CvPoint3D64f, CvColor>> DepthColorMatToRealPoints(CvMat depthMat, CvMat colorMat)
        {
            List<Tuple<CvPoint3D64f, CvColor>> list = new List<Tuple<CvPoint3D64f, CvColor>>();
            unsafe
            {
                short* depthArr = depthMat.DataInt16;
                byte* colorArr = colorMat.DataByte;
                for (int y = 0; y < depthMat.Rows; y++)
                {
                    int offset = y * depthMat.Cols;                    
                    for (int x = 0; x < depthMat.Cols; x++)
                    {
                        ushort depth = (ushort)depthArr[offset + x];
                        if (depth == 0)
                            continue;
                        ColorSpacePoint csp = this.MapDepthPointToColorSpace(x, y, depth, depthMat.Cols, depthMat.Rows);
                        if (csp.Y < 0)
                            csp.Y = 0;
                        CvPoint3D64f point = this.MapDepthPointToCameraSpace(x, y, depth).ToCvPoint3D();
                        int offsetXColor = (int)Math.Round(csp.X) * colorMat.ElemChannels;
                        int offsetColor = (int)Math.Round(csp.Y) * colorMat.Cols * colorMat.ElemChannels;
                        CvColor col = new CvColor(colorArr[offsetColor + offsetXColor + 0], colorArr[offsetColor + offsetXColor + 1], colorArr[offsetColor + offsetXColor + 2]);
                        list.Add(new Tuple<CvPoint3D64f, CvColor>(point, col));
                    }
                }
            }
            return list;
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