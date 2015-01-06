using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Collections.Concurrent;
using System.IO;
using OpenCvSharp;

using Microsoft.Kinect;

using System.IO.Compression;

namespace KinectMotionCapture
{
    public static class CalcEx
    {

        /// <summary>
        /// 深度画像に対してバイラテラルフィルタをかけます
        /// </summary>
        /// <param name="dest">出力先のCvMat。srcとフォーマットが異なる場合はsrcと同じフォーマットのマトリックスが代入される</param>
        /// <param name="src">深度画像</param>
        /// <param name="sigmaValue">深度値の差に対する標準偏差</param>
        /// <param name="sigmaDistance">画素の距離に対する標準偏差</param>
        /// <param name="windowSize">計算対象とする周辺ピクセルをあらわすウィンドウの一辺のサイズ。奇数のみ受け付ける</param>
        public static void BilateralFilterDepthMat(ref CvMat dest, CvMat src, double sigmaValue, double sigmaDistance, int windowSize)
        {
            const short invalidValue = 0;
            int windowOffset = windowSize / 2;
            if (windowOffset * 2 + 1 != windowSize)
            {
                throw new ArgumentException("windowSize must be odd");
            }
            CvEx.InitCvMat(ref dest, src);
            int width = src.Cols;
            int height = src.Rows;
            double sigmaValueSq = -sigmaValue * sigmaValue * 2;
            double sigmaDistanceSq = -sigmaDistance * sigmaDistance * 2;
            unsafe
            {
                short* srcArr = src.DataInt16;
                short* destArr = dest.DataInt16;
                Parallel.For(0, height, y =>
                {
                    for (int x = 0; x < width; x++)
                    {
                        double valueSum = 0;
                        double weightSum = 0;
                        int offsetCenter = y * width + x;
                        // 対象の画素の深度値
                        short center = srcArr[offsetCenter];
                        if (center == invalidValue)
                        {
                            destArr[offsetCenter] = invalidValue;
                            continue;
                        }
                        // 周辺画素について 値×重みの和 と 重みの和 を計算
                        int dxBegin = Math.Max(0, x - windowOffset) - x;
                        int dxEnd = Math.Min(width - 1, x + windowOffset) - x;
                        int dyBegin = Math.Max(0, y - windowOffset) - y;
                        int dyEnd = Math.Min(height - 1, y + windowOffset) - y;
                        for (int dy = dyBegin; dy <= dyEnd; dy++)
                        {
                            int yOffset = dy * width;
                            for (int dx = dxBegin; dx <= dxEnd; dx++)
                            {
                                // 周辺画素の深度値
                                short value = srcArr[offsetCenter + yOffset + dx];
                                if (value == invalidValue)
                                    continue;
                                double valueDiff = value - center;
                                double distanceSq = dx * dx + dy * dy;
                                double valueWeight = Math.Exp(valueDiff * valueDiff / sigmaValueSq);
                                double distanceWeight = Math.Exp(distanceSq / sigmaDistanceSq);
                                double weight = valueWeight * distanceWeight;
                                valueSum += value * weight;
                                weightSum += weight;
                            }
                        }
                        if (weightSum == 0)
                        {
                            destArr[offsetCenter] = invalidValue;
                            continue;
                        }
                        destArr[offsetCenter] = (short)(valueSum / weightSum);
                    }
                });
            }
        }
        /// <summary>
        /// 深度画像の一点に対してバイラテラルフィルタをかけた結果を返します．
        /// </summary>
        /// <param name="point"></param>
        /// <param name="src"></param>
        /// <param name="sigmaValue"></param>
        /// <param name="sigmaDistance"></param>
        /// <param name="windowSize"></param>
        /// <returns></returns>
        public static short BilateralFilterDepthMatSinglePixel(CvPoint point, CvMat src, double sigmaValue, double sigmaDistance, int windowSize)
        {
            int windowOffset = windowSize / 2;
            if (windowOffset * 2 + 1 != windowSize)
            {
                throw new ArgumentException("windowSize must be odd");
            }
            int width = src.Cols;
            int height = src.Rows;
            double sigmaValueSq = -sigmaValue * sigmaValue * 2;
            double sigmaDistanceSq = -sigmaDistance * sigmaDistance * 2;
            unsafe
            {
                short* srcArr = src.DataInt16;
                int y = point.Y;
                int x = point.X;
                double valueSum = 0;
                double weightSum = 0;
                int offsetCenter = y * width + x;
                // 対象の画素の深度値
                short center = srcArr[offsetCenter];
                if (center == InvalidDepth)
                {
                    return InvalidDepth;
                }
                // 周辺画素について 値×重みの和 と 重みの和 を計算
                int dxBegin = Math.Max(0, x - windowOffset) - x;
                int dxEnd = Math.Min(width - 1, x + windowOffset) - x;
                int dyBegin = Math.Max(0, y - windowOffset) - y;
                int dyEnd = Math.Min(height - 1, y + windowOffset) - y;
                for (int dy = dyBegin; dy <= dyEnd; dy++)
                {
                    int yOffset = dy * width;
                    for (int dx = dxBegin; dx <= dxEnd; dx++)
                    {
                        // 周辺画素の深度値
                        short value = srcArr[offsetCenter + yOffset + dx];
                        if (value == InvalidDepth)
                            continue;
                        double valueDiff = value - center;
                        double distanceSq = dx * dx + dy * dy;
                        double valueWeight = Math.Exp(valueDiff * valueDiff / sigmaValueSq);
                        double distanceWeight = Math.Exp(distanceSq / sigmaDistanceSq);
                        double weight = valueWeight * distanceWeight;
                        valueSum += value * weight;
                        weightSum += weight;
                    }
                }
                if (weightSum == 0)
                {
                    return InvalidDepth;
                }
                return (short)(valueSum / weightSum);
            }
        }
        /// <summary>
        /// 深度画像の一点に対してバイラテラルフィルタをかけた結果を返します．
        /// </summary>
        /// <param name="point"></param>
        /// <param name="src"></param>
        /// <param name="sigmaValue"></param>
        /// <param name="sigmaDistance"></param>
        /// <param name="windowSize"></param>
        /// <returns></returns>
        public static double? BilateralFilterDepthMatSinglePixel(CvPoint2D32f point, CvMat src, double sigmaValue, double sigmaDistance, int windowSize)
        {
            return BilateralFilterDepthMatSinglePixel((CvPoint2D64f)point, src, sigmaValue, sigmaDistance, windowSize);
        }
        /// <summary>
        /// 深度画像の一点に対してバイラテラルフィルタをかけた結果を返します．
        /// </summary>
        /// <param name="point"></param>
        /// <param name="src"></param>
        /// <param name="sigmaValue"></param>
        /// <param name="sigmaDistance"></param>
        /// <param name="windowSize"></param>
        /// <returns></returns>
        public static double? BilateralFilterDepthMatSinglePixel(CvPoint2D64f point, CvMat src, double sigmaValue, double sigmaDistance, int windowSize)
        {
            const short invalidValue = 0;
            int prevX = (int)Math.Floor(point.X);
            int prevY = (int)Math.Floor(point.Y);
            int nextX = prevX == point.X ? prevX : prevX + 1;
            int nextY = prevY == point.Y ? prevY : prevY + 1;
            double fracX = point.X - prevX;
            double fracY = point.Y - prevY;
            if (nextX >= src.Cols || nextY >= src.Rows)
            {
                return null;
            }
            double depth00 = BilateralFilterDepthMatSinglePixel(new CvPoint(prevX, prevY), src, sigmaValue, sigmaDistance, windowSize);
            if (prevX == nextX && prevY == nextY)
            {
                return depth00;
            }
            double depth01 = BilateralFilterDepthMatSinglePixel(new CvPoint(nextX, prevY), src, sigmaValue, sigmaDistance, windowSize);
            double depth10 = BilateralFilterDepthMatSinglePixel(new CvPoint(prevX, nextY), src, sigmaValue, sigmaDistance, windowSize);
            double depth11 = BilateralFilterDepthMatSinglePixel(new CvPoint(nextX, nextY), src, sigmaValue, sigmaDistance, windowSize);
            if (depth00 == invalidValue || depth01 == invalidValue || depth10 == invalidValue || depth11 == invalidValue)
            {
                return null;
            }
            return CalcEx.Lerp(CalcEx.Lerp(depth00, depth01, fracX), CalcEx.Lerp(depth10, depth11, fracX), fracY);
        }

        const short InvalidDepth = 0;
        /// <summary>
        /// デプスのデータ欠損を埋める
        /// </summary>
        /// <param name="dest">出力先のCvMat</param>
        /// <param name="src">入力CvMat</param>
        /// <param name="windowSize">周辺ピクセル矩形の一辺</param>
        /// <param name="thresholdCount">補間を行う条件として、周辺ピクセルの有効数の下限</param>
        /// <param name="thresholdEdgeCount"></param>
        /// <param name="thresholdVariance">補間を行う条件として、周辺ピクセルの有効な値の最大と最小の差の上限</param>
        public static void FillHoleDepthMat(ref CvMat dest, CvMat src, int windowSize, double countRatio, double edgeRatio, double thresholdMinMax)
        {
            int windowOffset = windowSize / 2;
            if (windowOffset * 2 + 1 != windowSize)
            {
                throw new ArgumentException("windowSize must be odd");
            }
            CvEx.InitCvMat(ref dest, src);
            int width = src.Cols;
            int height = src.Rows;
            int[] countEdgeThre = new int[windowOffset];
            int[] countThre = new int[windowOffset];

            double[] minMaxThre = new double[windowOffset];
            for (int i = 0; i < windowOffset; i++)
            {
                int countEdge = (i + 2) * 4;
                int count = (i * 2 + 3) * (i * 2 + 3);
                countEdgeThre[i] = (int)Math.Ceiling(countEdge * edgeRatio);
                countThre[i] = (int)Math.Ceiling(count * countRatio);
                minMaxThre[i] = thresholdMinMax * (i + 1) / windowOffset;
            }
            unsafe
            {
                short* srcArr = src.DataInt16;
                short* destArr = dest.DataInt16;
                Parallel.For(0, height, y =>
                {
                    for (int x = 0; x < width; x++)
                    {
                        int offsetCenter = y * width + x;
                        // 対象の画素の深度値
                        short center = srcArr[offsetCenter];
                        if (center != InvalidDepth)
                        {
                            destArr[offsetCenter] = center;
                            continue;
                        }
                        int count = 0;
                        // 周辺画素について 
                        int dxBegin = Math.Max(0, x - windowOffset) - x;
                        int dxEnd = Math.Min(width - 1, x + windowOffset) - x;
                        int dyBegin = Math.Max(0, y - windowOffset) - y;
                        int dyEnd = Math.Min(height - 1, y + windowOffset) - y;
                        short[] maxArr = new short[windowOffset];
                        short[] minArr = new short[windowOffset];
                        int[] countArr = new int[windowOffset];
                        List<short>[] valueArr = new List<short>[windowOffset];
                        for (int i = 0; i < windowOffset; i++)
                        {
                            maxArr[i] = short.MinValue;
                            minArr[i] = short.MaxValue;
                            valueArr[i] = new List<short>();
                        }
                        for (int dy = dyBegin; dy <= dyEnd; dy++)
                        {
                            int yOffset = dy * width;
                            int absDy = dy > 0 ? dy : -dy;
                            for (int dx = dxBegin; dx <= dxEnd; dx++)
                            {
                                // 周辺画素の深度値
                                short value = srcArr[offsetCenter + yOffset + dx];
                                if (value == InvalidDepth)
                                    continue;
                                int absDx = dx > 0 ? dx : -dx;
                                count++;
                                int distCenter = (absDx > absDy ? absDx : absDy) - 1;
                                if (value > maxArr[distCenter])
                                {
                                    maxArr[distCenter] = value;
                                }
                                if (value < minArr[distCenter])
                                {
                                    minArr[distCenter] = value;
                                }
                                countArr[distCenter]++;
                                valueArr[distCenter].Add(value);
                            }
                        }
                        if (count == 0)
                        {
                            destArr[offsetCenter] = InvalidDepth;
                            continue;
                        }

                        bool found = false;
                        int countSum = 0;
                        short max = short.MinValue;
                        short min = short.MaxValue;
                        List<double> valueList = new List<double>();

                        for (int i = 0; i < windowOffset; i++)
                        {
                            countSum += countArr[i];
                            valueList.AddRange(valueArr[i].Select(v => (double)v));
                            if (maxArr[i] > max)
                                max = maxArr[i];
                            if (minArr[i] < min)
                                min = minArr[i];

                            if (countSum < countThre[i])
                                continue;
                            if (countArr[i] < countEdgeThre[i])
                                continue;
                            if (max - min >= minMaxThre[i])
                                continue;
                            found = true;
                            break;
                        }
                        if (!found)
                        {
                            destArr[offsetCenter] = InvalidDepth;
                            continue;
                        }

                        double median = CalcEx.GetMedian(valueList);

                        destArr[offsetCenter] = (short)Math.Round(median);
                    }
                });
            }
        }
        /// <summary>
        /// デプスのデータ欠損を埋める
        /// </summary>
        /// <param name="dest">出力先のCvMat</param>
        /// <param name="src">入力CvMat</param>
        /// <param name="windowSize">周辺ピクセル矩形の一辺</param>
        /// <param name="thresholdCount">補間を行う条件として、周辺ピクセルの有効数の下限</param>
        /// <param name="thresholdEdgeCount">補間を行う条件として、周辺ピクセルのうち縁の上のあるピクセルの有効数の下限</param>
        /// <param name="thresholdVariance">補間を行う条件として、周辺ピクセルの有効な値の最大と最小の差の上限</param>
        public static void FillHoleDepthMat(ref CvMat dest, CvMat src, int windowSize, int thresholdCount)
        {
            int windowOffset = windowSize / 2;
            if (windowOffset * 2 + 1 != windowSize)
            {
                throw new ArgumentException("windowSize must be odd");
            }
            CvEx.InitCvMat(ref dest, src);
            int width = src.Cols;
            int height = src.Rows;
            unsafe
            {
                short* srcArr = src.DataInt16;
                short* destArr = dest.DataInt16;
                Parallel.For(0, height, y =>
                {
                    List<double> valueList = new List<double>();
                    for (int x = 0; x < width; x++)
                    {
                        int offsetCenter = y * width + x;
                        // 対象の画素の深度値
                        short center = srcArr[offsetCenter];
                        if (center != InvalidDepth)
                        {
                            destArr[offsetCenter] = center;
                            continue;
                        }
                        valueList.Clear();
                        // 周辺画素について 
                        int dxBegin = Math.Max(0, x - windowOffset) - x;
                        int dxEnd = Math.Min(width - 1, x + windowOffset) - x;
                        int dyBegin = Math.Max(0, y - windowOffset) - y;
                        int dyEnd = Math.Min(height - 1, y + windowOffset) - y;
                        for (int dy = dyBegin; dy <= dyEnd; dy++)
                        {
                            int yOffset = dy * width;
                            for (int dx = dxBegin; dx <= dxEnd; dx++)
                            {
                                // 周辺画素の深度値
                                short value = srcArr[offsetCenter + yOffset + dx];
                                if (value == InvalidDepth)
                                    continue;
                                valueList.Add((double)value);
                            }
                        }
                        if (valueList.Count < thresholdCount)
                        {
                            destArr[offsetCenter] = InvalidDepth;
                            continue;
                        }
                        double median = CalcEx.GetMedian(valueList);
                        destArr[offsetCenter] = (short)Math.Round(median);
                    }
                });
            }
        }

        /// <summary>
        /// 深度映像から周辺ピクセルを見て面の法線を計算する。計算できない場合には0,0,0を返す
        /// </summary>
        /// <param name="depthMat"></param>
        /// <param name="point"></param>
        /// <param name="undist"></param>
        /// <param name="windowSize"></param>
        /// <returns></returns>
        public static CvPoint3D64f GetNormalOfDepthMat(CvMat depthMat, CvPoint2D32f point, KinectUndistortion undist, int windowSize)
        {
            CvSize depthImageSize = new CvSize(depthMat.Cols, depthMat.Rows);
            int windowOffset = windowSize / 2;
            if (windowOffset * 2 + 1 != windowSize)
            {
                throw new ArgumentException("windowSize must be odd");
            }
            double? depth = CvEx.Get2DSubPixel(depthMat, point, 0);

            if (!depth.HasValue)
            {
                return new CvPoint3D64f();
            }
            CvPoint3D64f pos1 = undist.GetRealFromScreenPos(point.X, point.Y, depth.Value, depthImageSize);
            // 周辺ピクセルをみて平面の傾きを求める。平面は対象の点(pos1)を通るようにする = -pos1だけ平行移動してpos1が原点になるようにして計算
            // ax+by=d
            List<double[]> left = new List<double[]>();
            List<double> right = new List<double>();
            for (int dx = -windowOffset; dx <= windowOffset; dx++)
            {
                for (int dy = -windowOffset; dy <= windowOffset; dy++)
                {
                    CvPoint2D32f point2 = point + new CvPoint2D32f(dx, dy);
                    double? depth2 = CvEx.Get2DSubPixel(depthMat, point2, 0);
                    if (depth2.HasValue)
                    {
                        CvPoint3D64f pos2 = undist.GetRealFromScreenPos(point2.X, point2.Y, depth2.Value, depthImageSize);
                        left.Add(new double[] { pos2.X - pos1.X, pos2.Y - pos1.Y });
                        right.Add(pos2.Z - pos1.Z);
                    }
                }
            }
            if (left.Count < 3)
                return new CvPoint3D64f();
            double[] slope = CvEx.Solve(left, right, InvertMethod.Svd);
            // 傾きから法線を出す
            CvPoint3D64f pos10 = new CvPoint3D64f(1, 0, slope[0]);
            CvPoint3D64f pos01 = new CvPoint3D64f(0, 1, slope[1]);
            CvPoint3D64f posNormal = CvEx.Cross(pos10, pos01);
            return CvEx.Normalize(posNormal);
        }

        /// <summary>
        /// OpenNIの深度値の間隔の係数。深度dに対して間隔は (d/581)^2
        /// </summary>
        public const double IntervalDivAtOpenNI = 581;

        public static CvPoint2D64f? GetSlopeOfDepthMat(CvMat depthMat, CameraMatrix cameraMatrix)
        {
            return GetSlopeOfDepthMat(depthMat, cameraMatrix, null);
        }
        /// <summary>
        /// 深度映像の全体の傾き係数A,Bを求めます (Z=A(x-cx)+B(y-cy)+cz), Z:結果の深度値, c(x,y,z): 主点のスクリーン座標と深度値
        /// </summary>
        /// <param name="depthMat">深度マップ</param>
        /// <param name="cameraMatrix">カメラ情報</param>
        /// <param name="undist">ゆがみ補正係数</param>
        /// <returns></returns>
        public static CvPoint2D64f? GetSlopeOfDepthMat(CvMat depthMat, CameraMatrix cameraMatrix, KinectUndistortion undist)
        {
            CvSize size = new CvSize(depthMat.Cols, depthMat.Rows);

            unsafe
            {
                // 平面化
                List<double[]> left = new List<double[]>(size.Width * size.Height);
                List<double> right = new List<double>(size.Width * size.Height);

                short* depthArr = depthMat.DataInt16;
                double pX = cameraMatrix.PrincipalX;
                double pY = cameraMatrix.PrincipalY;
                double fX = cameraMatrix.FocalX;
                double fY = cameraMatrix.FocalY;
                double? pDepth = CvEx.Get2DSubPixel(depthMat, new CvPoint2D32f(pX, pY), 0);

                if (pDepth.HasValue)
                {
                    if (undist != null)
                    {
                        pDepth = undist.UndistortDepth(new CvPoint3D64f(pX, pY, pDepth.Value), size);
                    }
                    for (int y = 0; y < size.Height; y++)
                    {
                        int iOffset = y * size.Width;
                        double dy = (y + 0.5 - pY) / (fY / 2);
                        for (int x = 0; x < size.Width; x++)
                        {
                            double dx = (x + 0.5 - pX) / (fX / 2);
                            if (dx * dx + dy * dy >= 1)
                                continue;
                            double depth = depthArr[iOffset + x];
                            if (depth != 0)
                            {
                                if (undist != null)
                                {
                                    depth = undist.UndistortDepth(new CvPoint3D64f(x, y, depth), size);
                                }
                                left.Add(new double[] { x - pX, y - pY });
                                right.Add(depth - pDepth.Value);
                            }
                        }
                    }
                    double[] ans = CvEx.Solve(left, right, InvertMethod.Svd);
                    return new CvPoint2D64f(ans[0], ans[1]);
                }
            }
            return null;
        }

        /// <summary>
        /// センサから深度が飛び飛びの値で得られるので隣接した値の間を滑らかにする
        /// </summary>
        /// <param name="dest"></param>
        /// <param name="src"></param>
        /// <param name="kinectSdk"></param>
        public static void SmoothDepthStep(ref CvMat dest, CvMat src, int windowSize)
        {
            int windowOffset = windowSize / 2;
            if (windowOffset * 2 + 1 != windowSize)
            {
                throw new ArgumentException("windowSize must be odd");
            }


            CvEx.InitCvMat(ref dest, src);
            int width = src.Cols;
            int height = src.Rows;
            double sigma = windowOffset * Math.Sqrt(0.5);
            double sigmaSq = sigma * sigma;
            double[] distanceMat = new double[windowSize * windowSize];
            for (int dy = -windowOffset; dy <= windowOffset; dy++)
            {
                int dOffset = (dy + windowOffset) * windowSize;
                for (int dx = -windowOffset; dx <= windowOffset; dx++)
                {
                    distanceMat[dOffset + (dx + windowOffset)] = Math.Exp(-(dx * dx + dy * dy) / sigmaSq);
                }
            }
            unsafe
            {
                short* srcArr = src.DataInt16;
                short* destArr = dest.DataInt16;
                Parallel.For(0, height, y =>
                {
                    for (int x = 0; x < width; x++)
                    {
                        int offsetCenter = y * width + x;
                        // 対象の画素の深度値
                        short center = srcArr[offsetCenter];
                        if (center == InvalidDepth)
                        {
                            destArr[offsetCenter] = center;
                            continue;
                        }

                        int dxBegin = Math.Max(0, x - windowOffset) - x;
                        int dxEnd = Math.Min(width - 1, x + windowOffset) - x;
                        int dyBegin = Math.Max(0, y - windowOffset) - y;
                        int dyEnd = Math.Min(height - 1, y + windowOffset) - y;

                        double valueSum = 0;
                        double weightSum = 0;

                        double div1 = (double)center / IntervalDivAtOpenNI;
                        double interval = div1 * div1;
                        // 大体3段階分
                        short prevValue = (short)Math.Floor(center - interval * 2.97 - 65);
                        short nextValue = (short)Math.Ceiling(center + interval * 3.11 + 65);

                        for (int dy = dyBegin; dy <= dyEnd; dy++)
                        {
                            int yOffset = dy * width;
                            int dOffset = (dy + windowOffset) * windowSize;

                            for (int dx = dxBegin; dx <= dxEnd; dx++)
                            {
                                // 周辺画素の深度値
                                short value = srcArr[offsetCenter + yOffset + dx];
                                if (value == InvalidDepth)
                                    continue;
                                if (prevValue <= value && value <= nextValue)
                                {
                                    double weight = distanceMat[dOffset + (dx + windowOffset)];
                                    valueSum += value * weight;
                                    weightSum += weight;
                                }
                            }
                        }
                        destArr[offsetCenter] = (short)Math.Round(valueSum / weightSum);
                    }
                });
            }
        }
        /// <summary>
        /// 深度値に中央値フィルタをかけます(0値以外)
        /// </summary>
        /// <param name="dest">出力先のCvMat。フォーマットが異なる場合は新しいインスタンスが作成されます</param>
        /// <param name="src">入力CvMat</param>
        /// <param name="windowSize">周辺のサイズ</param>
        public static void MedianDepthMat(ref CvMat dest, CvMat src, int windowSize)
        {
            int windowOffset = windowSize / 2;
            if (windowOffset * 2 + 1 != windowSize)
            {
                throw new ArgumentException("windowSize must be odd");
            }


            CvEx.InitCvMat(ref dest, src);
            int width = src.Cols;
            int height = src.Rows;
            unsafe
            {
                short* srcArr = src.DataInt16;
                short* destArr = dest.DataInt16;
                Parallel.For(0, height, y =>
                {
                    for (int x = 0; x < width; x++)
                    {
                        int offsetCenter = y * width + x;
                        // 対象の画素の深度値
                        short center = srcArr[offsetCenter];
                        if (center == InvalidDepth)
                        {
                            destArr[offsetCenter] = center;
                            continue;
                        }

                        int dxBegin = Math.Max(0, x - windowOffset) - x;
                        int dxEnd = Math.Min(width - 1, x + windowOffset) - x;
                        int dyBegin = Math.Max(0, y - windowOffset) - y;
                        int dyEnd = Math.Min(height - 1, y + windowOffset) - y;
                        List<double> data = new List<double>();
                        for (int dy = dyBegin; dy <= dyEnd; dy++)
                        {
                            int yOffset = dy * width;

                            for (int dx = dxBegin; dx <= dxEnd; dx++)
                            {
                                // 周辺画素の深度値
                                short value = srcArr[offsetCenter + yOffset + dx];
                                if (value == InvalidDepth)
                                    continue;
                                data.Add(value);
                            }
                        }
                        destArr[offsetCenter] = (short)Math.Round(CalcEx.GetMedian(data));
                    }
                });
            }
        }


        public static double Lerp(double v0, double v1, double interpolator)
        {
            return v0 * (1.0 - interpolator) + v1 * interpolator;
        }

        /// <summary>
        /// OpenNIから得られるRGB画像と深度画像のサイズが異なる場合の対応点の相互変換を行います
        /// </summary>
        /// <param name="point">変換もとの点の座標</param>
        /// <param name="fromSize">変換もとの画像サイズ</param>
        /// <param name="toSize">変換先の画像サイズ</param>
        /// <returns></returns>
        public static CvPoint2D32f ScalePointOnOpenNIImage(CvPoint2D32f point, CvSize fromSize, CvSize toSize)
        {
            float scale = (float)toSize.Width / fromSize.Width;
            return new CvPoint2D32f(point.X * scale, point.Y * scale);
        }
        /// <summary>
        /// OpenNIから得られるRGB画像と深度画像のサイズが異なる場合の対応点の相互変換を行います
        /// </summary>
        /// <param name="point">変換もとの点の座標</param>
        /// <param name="fromMat">変換元の画像</param>
        /// <param name="toMat">変換先の座標</param>
        /// <returns></returns>
        public static CvPoint2D32f ScalePointOnOpenNIImage(CvPoint2D32f point, CvMat fromMat, CvMat toMat)
        {
            return ScalePointOnOpenNIImage(point, new CvSize(fromMat.Cols, fromMat.Rows), new CvSize(toMat.Cols, toMat.Rows));
        }

        /// <summary>
        /// OpenNIから得られるRGB画像と深度画像のサイズが異なる場合の対応点のX座標の相互変換を行います
        /// </summary>
        /// <param name="x">変換元の点のX座標</param>
        /// <param name="fromSize">変換元の画像サイズ</param>
        /// <param name="toSize">変換先の画像サイズ</param>
        /// <returns></returns>
        public static float ScaleXOnOpenNIImage(float x, CvSize fromSize, CvSize toSize)
        {
            float scale = (float)toSize.Width / fromSize.Width;
            return x * scale;
        }
        public static float ScaleXOnOpenNIImage(float x, CvMat fromMat, CvMat toMat)
        {
            return ScaleXOnOpenNIImage(x, new CvSize(fromMat.Cols, fromMat.Rows), new CvSize(toMat.Cols, toMat.Rows));
        }
        /// <summary>
        /// OpenNIから得られるRGB画像と深度画像のサイズが異なる場合の対応点のY座標の相互変換を行います
        /// </summary>
        /// <param name="y">変換元の点のY座標</param>
        /// <param name="fromSize">変換元の画像サイズ</param>
        /// <param name="toSize">変換先の画像サイズ</param>
        /// <returns></returns>
        public static float ScaleYOnOpenNIImage(float y, CvSize fromSize, CvSize toSize)
        {
            float scale = (float)toSize.Width / fromSize.Width;
            return y * scale;
        }
        public static float ScaleYOnOpenNIImage(float y, CvMat fromMat, CvMat toMat)
        {
            return ScaleYOnOpenNIImage(y, new CvSize(fromMat.Cols, fromMat.Rows), new CvSize(toMat.Cols, toMat.Rows));
        }

        /// <summary>
        /// 指定された変換を用いてスクリーン座標+深度の点列からカメラ座標系の点列を求めます
        /// </summary>
        /// <param name="list">出力先のリスト</param>
        /// <param name="mat">深度画像</param>
        /// <param name="project">座標変換メソッド</param>
        public static void DepthMatToRealPoints(List<CvPoint3D64f> list, CvMat mat, Func<CvPoint3D64f, CvPoint3D64f> project)
        {
            unsafe
            {
                short* depthArr = mat.DataInt16;
                for (int y = 0; y < mat.Rows; y++)
                {
                    int offset = y * mat.Cols;
                    for (int x = 0; x < mat.Cols; x++)
                    {
                        short depth = depthArr[offset + x];
                        if (depth == 0)
                            continue;
                        CvPoint3D64f fromPoint = new CvPoint3D64f(x, y, depth);
                        CvPoint3D64f toPoint = project(fromPoint);
                        list.Add(toPoint);
                    }
                }
            }
        }
        /// <summary>
        /// 指定された変換を用いてスクリーン座標+深度の点列からカメラ座標系の点列を求めます
        /// </summary>
        /// <param name="mat">深度画像</param>
        /// <param name="project">座標変換メソッド</param>
        /// <returns></returns>
        public static List<CvPoint3D64f> DepthMatToRealPoints(CvMat mat, Func<CvPoint3D64f, CvPoint3D64f> project)
        {
            List<CvPoint3D64f> ret = new List<CvPoint3D64f>();
            DepthMatToRealPoints(ret, mat, project);
            return ret;
        }

        /// <summary>
        /// 指定された変換と平行移動・回転行列を用いてスクリーン座標+深度の点列から絶対座標系の点列を求めます
        /// </summary>
        /// <param name="list">出力先のリスト</param>
        /// <param name="mat">深度画像</param>
        /// <param name="project">座標変換メソッド</param>
        /// <param name="transform">縦ベクトル座標変換用4x4行列</param>
        public static void DepthMatToRealPoints(List<CvPoint3D64f> list, CvMat mat, Func<CvPoint3D64f, CvPoint3D64f> project, CvMat transform)
        {
            unsafe
            {
                short* depthArr = mat.DataInt16;
                for (int y = 0; y < mat.Rows; y++)
                {
                    int offset = y * mat.Cols;
                    for (int x = 0; x < mat.Cols; x++)
                    {
                        short depth = depthArr[offset + x];
                        if (depth == 0)
                            continue;
                        CvPoint3D64f fromPoint = new CvPoint3D64f(x, y, depth);
                        CvPoint3D64f toPoint = CvEx.ConvertPoint3D(project(fromPoint), transform);
                        list.Add(toPoint);
                    }
                }
            }
        }

        /// <summary>
        /// 指定された変換と平行移動・回転行列を用いてスクリーン座標+深度の点列から絶対座標系の点列を求めます
        /// </summary>
        /// <param name="mat">深度画像</param>
        /// <param name="project">座標変換メソッド</param>
        /// <param name="transform">縦ベクトル座標変換用4x4行列</param>
        public static List<CvPoint3D64f> DepthMatToRealPoints(CvMat mat, Func<CvPoint3D64f, CvPoint3D64f> project, CvMat transform)
        {
            List<CvPoint3D64f> ret = new List<CvPoint3D64f>();
            DepthMatToRealPoints(ret, mat, project, transform);
            return ret;
        }

        /// <summary>
        /// 数値の列挙から分散を求めます
        /// </summary>
        /// <param name="source"></param>
        /// <returns></returns>
        public static double GetStdDev(IEnumerable<double> source)
        {
            double[] arr = source.ToArray();
            return Math.Sqrt(arr.Select(x => x * x).Average() - Math.Pow(arr.Average(), 2)) * arr.Length / (arr.Length - 1);
        }
        /// <summary>
        /// 列挙の各要素を数値に変換して分散を求めます
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="source"></param>
        /// <param name="conversion"></param>
        /// <returns></returns>
        public static double GetStdDev<T>(IEnumerable<T> source, Func<T, double> conversion)
        {
            double[] arr = source.Select(x => conversion(x)).ToArray();
            return Math.Sqrt(arr.Select(x => x * x).Average() - Math.Pow(arr.Average(), 2)) * arr.Length / (arr.Length - 1);
        }

        /// <summary>
        /// Tupleのデフォルト比較器
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <typeparam name="U"></typeparam>
        class tupleValueComparer<T, U> : IComparer<Tuple<T, U>>
        {
            public int Compare(Tuple<T, U> x, Tuple<T, U> y)
            {
                return Comparer<U>.Default.Compare(x.Item2, y.Item2);
            }
        }

        /// <summary>
        /// 安定結婚問題を単純に解きます
        /// </summary>
        /// <param name="fromLength">一つ目の群の要素の数</param>
        /// <param name="toLength">二つ目の群の要素の数</param>
        /// <param name="distances">一つ目の要素のインデックスと二つ目の要素のインデックスによる要素間の距離</param>
        /// <returns></returns>
        public static Dictionary<int, KeyValuePair<int, double>> SolveStableMarriage(int fromLength, int toLength, Dictionary<int, Dictionary<int, double>> distances)
        {
            Dictionary<int, KeyValuePair<int, double>> matches = new Dictionary<int, KeyValuePair<int, double>>();
            Queue<int> fromList = new Queue<int>(Enumerable.Range(0, fromLength));
            while (fromList.Count > 0 && matches.Count < toLength)
            {
                int from = fromList.Dequeue();
                double minDist = double.NaN;
                int minTo = -1;
                for (int to = 0; to < toLength; to++)
                {
                    Dictionary<int, double> tempDist;
                    if (!distances.TryGetValue(from, out tempDist))
                    {
                        continue;
                    }
                    double dist;
                    if (!tempDist.TryGetValue(to, out dist))
                    {
                        continue;
                    }
                    if (double.IsNaN(dist))
                        continue;
                    if (double.IsNaN(minDist) || dist < minDist)
                    {
                        minDist = dist;
                        minTo = to;
                    }
                }
                KeyValuePair<int, double> prevMatching;
                if (matches.TryGetValue(minTo, out prevMatching))
                {
                    if (minDist < prevMatching.Value)
                    {
                        fromList.Enqueue(prevMatching.Key);
                        matches[minTo] = new KeyValuePair<int, double>(from, minDist);
                    }
                    else
                    {
                        distances[from][minTo] = double.NaN;
                        fromList.Enqueue(from);
                    }
                }
                else
                {
                    matches[minTo] = new KeyValuePair<int, double>(from, minDist);
                }
            }
            return matches.ToDictionary(x => x.Value.Key, x => new KeyValuePair<int, double>(x.Key, x.Value.Value));
        }

        /// <summary>
        /// 安定結婚問題を解きます
        /// </summary>
        /// <param name="fromLength">一つ目の群の要素の数</param>
        /// <param name="toLength">二つ目の群の要素の数</param>
        /// <param name="distances">一つ目の要素のインデックスと二つ目の要素のインデックスによる要素間の距離</param>
        /// <returns></returns>
        public static Dictionary<int, Tuple<int, double>> SolveStableMarriage2(int fromLength, int toLength, Func<int, int, double> funcDistance)
        {
            Dictionary<int, KeyValuePair<int, double>> matches = new Dictionary<int, KeyValuePair<int, double>>();
            Queue<int> fromList = new Queue<int>(Enumerable.Range(0, fromLength));
            ConcurrentDictionary<int, List<Tuple<int, double>>> distanceListSet = new ConcurrentDictionary<int, List<Tuple<int, double>>>(8, fromLength);
            tupleValueComparer<int, double> cmp = new tupleValueComparer<int, double>();
            Parallel.For(0, fromLength, i =>
            {
                List<Tuple<int, double>> distanceList = new List<Tuple<int, double>>();
                for (int j = 0; j < toLength; j++)
                {
                    distanceList.Add(new Tuple<int, double>(j, funcDistance(i, j)));
                }
                distanceList.Sort(cmp);
                distanceListSet[i] = distanceList;
            });
            double supMatchedDistance = 0;
            while (fromList.Count > 0)
            {
                int from = fromList.Dequeue();
                List<Tuple<int, double>> distanceList = distanceListSet[from];
                if (distanceList.Count == 0)
                    continue;
                int minTo = distanceList[0].Item1;
                double minDist = distanceList[0].Item2;
                KeyValuePair<int, double> prevMatching;
                if (matches.Count == toLength)
                {
                    if (minDist > supMatchedDistance)
                    {
                        // ペア数が最大のときに、このfromの距離の最小が全ペアの距離の最大より大きければこのfromがマッチすることはない
                        continue;
                    }
                }
                if (matches.TryGetValue(minTo, out prevMatching))
                {
                    if (minDist < prevMatching.Value)
                    {
                        // ほかのfromに打ち勝ったらそのfromを追い出す
                        fromList.Enqueue(prevMatching.Key);
                        matches[minTo] = new KeyValuePair<int, double>(from, minDist);
                        supMatchedDistance = Math.Max(supMatchedDistance, minDist);
                    }
                    else
                    {
                        distanceList.RemoveAt(0);
                        fromList.Enqueue(from);
                    }
                }
                else
                {
                    matches[minTo] = new KeyValuePair<int, double>(from, minDist);
                    supMatchedDistance = Math.Max(supMatchedDistance, minDist);
                }
            }
            return matches.ToDictionary(x => x.Value.Key, x => new Tuple<int, double>(x.Key, x.Value.Value));
        }

        /// <summary>
        /// 三段ハッシュテーブルによる最近傍点の近似解を求めます
        /// </summary>
        /// <param name="nodes"></param>
        /// <param name="vicinity"></param>
        /// <returns></returns>
        public static Dictionary<int, Tuple<int, double>> FindNearestNode(IList<float[]> nodes, double vicinity)
        {
            if (nodes.Count == 0)
                return new Dictionary<int, Tuple<int, double>>();
            int descLength = nodes[0].Length;
            Dictionary<ThreeTuple<short>, List<int>> mappedNodes = new Dictionary<ThreeTuple<short>, List<int>>();
            Func<float[], ThreeTuple<short>> mapDiscrete = (point) =>
            {
                double var1 = 0, var2 = 0, var3 = 0;
                for (int i = 0; i < descLength * 1 / 3; i++) { var1 += point[i]; }
                for (int i = descLength * 1 / 3; i < descLength * 2 / 3; i++) { var2 += point[i]; }
                for (int i = descLength * 2 / 3; i < descLength; i++) { var3 += point[i]; }
                return new ThreeTuple<short>((short)(var1 / vicinity), (short)(var2 / vicinity), (short)(var3 / vicinity));
            };

            List<ThreeTuple<short>> cachedMap = nodes.AsParallel().AsOrdered().Select(n => mapDiscrete(n)).ToList();

            Parallel.For(0, nodes.Count, (index) =>
            {
                //            for (int index = 0; index < second.Count; index++) {
                float[] point = nodes[index];
                List<int> mappedList;
                ThreeTuple<short> key = cachedMap[index];
                lock (mappedNodes)
                {
                    if (!mappedNodes.TryGetValue(key, out mappedList))
                    {
                        mappedNodes[key] = mappedList = new List<int>();
                    }
                }
                lock (mappedList)
                {
                    mappedList.Add(index);
                }
            }
            );
            Dictionary<int, Tuple<int, double>> distanceListSet = new Dictionary<int, Tuple<int, double>>();

            Parallel.For(0, nodes.Count, i =>
            {
                //foreach (int i in Enumerable.Range(0, first.Count)) {
                float[] firstPoint = nodes[i];
                List<Tuple<int, double>> distanceList = new List<Tuple<int, double>>();
                ThreeTuple<short> center = cachedMap[i];
                for (int i1 = -1; i1 <= 1; i1++)
                {
                    for (int i2 = -1; i2 <= 1; i2++)
                    {
                        for (int i3 = -1; i3 <= 1; i3++)
                        {
                            ThreeTuple<short> neighbor = new ThreeTuple<short>((short)(center.Item1 + i1), (short)(center.Item2 + i2), (short)(center.Item3 + i3));
                            List<int> other;
                            if (mappedNodes.TryGetValue(neighbor, out other))
                            {
                                foreach (int index in other)
                                {
                                    if (index == i)
                                        continue;
                                    float[] otherPoint = nodes[index];
                                    double distanceSq = 0;
                                    for (int j = 0; j < descLength; j++)
                                    {
                                        double d = (firstPoint[j] - otherPoint[j]);
                                        distanceSq += d * d;

                                    }
                                    distanceList.Add(new Tuple<int, double>(index, distanceSq));
                                }
                            }
                        }
                    }
                }
                if (distanceList.Count > 0)
                {
                    lock (distanceListSet)
                    {
                        double minDistance = distanceList.Select(x => x.Item2).Min();
                        distanceListSet[i] = distanceList.First(x => x.Item2 == minDistance);
                    }
                }
                //}
            });
            return distanceListSet;
        }

        static Random _rand = new Random();
        static int getRandomIndex(int count)
        {
            lock (_rand)
            {
                return _rand.Next(count);
            }
        }
        /// <summary>
        /// O(1)でサンプリングによる疑似中央値を求めます
        /// </summary>
        /// <param name="data"></param>
        /// <param name="sampleCount"></param>
        /// <returns></returns>
        public static double GetSampledMedian(IList<double> data, int sampleCount)
        {
            if (data.Count < sampleCount)
                return GetMedian(data);
            List<double> sample = new List<double>();
            for (int i = 0; i < sampleCount; i++)
            {
                sample.Add(data[i]);
            }
            return GetMedian(sample);
        }

        /// <summary>
        /// O(n)で中央値を求めます．要素が偶数個の場合は中央2要素の平均を返します
        /// </summary>
        /// <param name="data"></param>
        /// <returns></returns>
        public static double GetMedian(IList<double> data)
        {
            if (data.Count == 0)
            {
                throw new ArgumentException("data contains no element");
            }
            if (data.Count % 2 == 0)
            {
                return (GetNth(data, data.Count / 2) + GetNth(data, data.Count / 2 - 1)) / 2;
            }
            else
            {
                return GetNth(data, data.Count / 2);
            }
        }
        /// <summary>
        /// O(n)でリストの中でn番目に小さい要素を求めます
        /// </summary>
        /// <param name="data"></param>
        /// <param name="index"></param>
        /// <returns></returns>
        public static double GetNth(IList<double> data, int index)
        {
            if (index < 0 || index >= data.Count)
            {
                throw new ArgumentOutOfRangeException("index must be 0 to data.Count");
            }
            if (data is List<double>)
            {
                return getNthAux((List<double>)data, 0, index);
            }
            else
            {
                return getNthAux(data.ToList(), 0, index);
            }
        }


        static double getNthAux(List<double> data, int offset, int targetIndex)
        {
            if (data.Count == 1)
            {
                // すぐに終了
                return data[0];
            }
            if (data.Count == 2)
            {
                // すぐに終了
                if (offset == targetIndex)
                    return data[0] < data[1] ? data[0] : data[1];
                else
                    return data[0] > data[1] ? data[0] : data[1];
            }
            // 適当に軸を取る
            int pivotIndex = getRandomIndex(data.Count);
            double pivot = data[pivotIndex];
            // 軸未満の要素と軸以上の要素
            List<double> left = new List<double>();
            List<double> right = new List<double>();
            int sameCount = 0;
            for (int i = 0; i < data.Count; i++)
            {
                if (data[i] < pivot)
                {
                    left.Add(data[i]);
                }
                else if (data[i] == pivot)
                {
                    sameCount++;
                }
                else
                {
                    right.Add(data[i]);
                }
            }
            if (targetIndex < offset + left.Count)
            {
                // 左にnthがある
                return getNthAux(left, offset, targetIndex);
            }
            if (targetIndex < offset + left.Count + sameCount)
            {
                // 軸がちょうどnth
                return pivot;
            }
            // 右にnthがある
            return getNthAux(right, offset + left.Count + sameCount, targetIndex);
        }

        /// <summary>
        /// 重み付き中央値(値と重みのペアを値で昇順に並べたときに、それより下にあるペアの重みの和と上にあるペアの重みの和ができるだけ同じになるようなペアの値)を返します。
        /// </summary>
        /// <param name="valueAndWeightList"></param>
        /// <returns></returns>
        public static double GetWeightedMedian(IList<Tuple<double, double>> valueAndWeightList)
        {
            double target = valueAndWeightList.Select(x => x.Item2).Sum() / 2;
            List<Tuple<double, double>> data = valueAndWeightList as List<Tuple<double, double>>;
            if (data == null)
            {
                data = valueAndWeightList.ToList();
            }
            if (data.Count == 0)
                return double.NaN;
            return getWeightedNth(data, 0, target);
        }
        static double getWeightedNth(List<Tuple<double, double>> data, double offset, double targetIndex)
        {
            if (data.Count == 1)
            {
                return data[0].Item1;
            }
            // 適当に軸を選ぶ
            int pivotIndex = getRandomIndex(data.Count);
            double pivot = data[pivotIndex].Item1;
            List<Tuple<double, double>> left = new List<Tuple<double, double>>();
            List<Tuple<double, double>> right = new List<Tuple<double, double>>();

            double leftWeight = 0;
            double rightWeight = 0;
            double sameWeight = 0;
            for (int i = 0; i < data.Count; i++)
            {
                if (data[i].Item1 < pivot)
                {
                    left.Add(data[i]);
                    leftWeight += data[i].Item2;
                }
                else if (data[i].Item1 == pivot)
                {
                    sameWeight += data[i].Item2;
                }
                else
                {
                    right.Add(data[i]);
                    rightWeight += data[i].Item2;
                }
            }
            // 境界上の場合はすぐ右にあるのが選ばれる

            // 左
            if (targetIndex < offset + leftWeight)
            {
                return getWeightedNth(left, offset, targetIndex);
            }
            // ちょうど
            if (targetIndex < offset + leftWeight + sameWeight)
            {
                return pivot;
            }
            // 右
            return getWeightedNth(right, offset + leftWeight + sameWeight, targetIndex);
        }

        public static double GetWeightedLinearMedian(IList<Tuple<double, double>> valueAndWeightList)
        {
            double target = valueAndWeightList.Select(x => x.Item2).Sum() / 2;
            List<Tuple<double, double>> data = valueAndWeightList as List<Tuple<double, double>>;
            if (data == null)
            {
                data = valueAndWeightList.ToList();
            }
            if (data.Count == 0)
                return double.NaN;
            double just = getWeightedNth(data, 0, target);
            double prev = double.MinValue;
            double next = double.MaxValue;
            double justWeight = 0;
            double lessWeight = 0;
            foreach (var tuple in valueAndWeightList)
            {
                if (tuple.Item1 < just)
                {
                    lessWeight += tuple.Item2;
                    prev = Math.Max(prev, tuple.Item1);
                }
                else if (tuple.Item1 == just)
                {
                    justWeight += tuple.Item2;
                }
                else
                {
                    next = Math.Min(next, tuple.Item1);
                }
            }
            if (prev == double.MinValue)
                prev = just;
            if (next == double.MaxValue)
                next = just;
            double interpolator = (target - lessWeight) / justWeight;
            return (prev + just) / 2 * (1.0 - interpolator) + (next + just) / 2 * interpolator;
        }
        /// <summary>
        /// HSV色空間をRGB色空間に変換します
        /// </summary>
        /// <param name="HDegree">360度分割の色相値</param>
        /// <param name="S">彩度</param>
        /// <param name="V">明度</param>
        /// <returns></returns>
        public static CvColor HSVtoRGB(double HDegree, double S, double V)
        {
            HDegree = ((HDegree % 360) + 360) % 360;
            S = Math.Min(Math.Max(0, S), 1);
            V = Math.Min(Math.Max(0, V), 1);
            int Hi = (int)Math.Floor(HDegree / 60);
            double f = HDegree / 60 - Hi;
            double p = V * (1f - S);
            double q = V * (1f - f * S);
            double t = V * (1f - (1f - f) * S);
            double R = 0, G = 0, B = 0;
            switch (Hi)
            {
                case 0:
                    R = V;
                    G = t;
                    B = p;
                    break;
                case 1:
                    R = q;
                    G = V;
                    B = p;
                    break;
                case 2:
                    R = p;
                    G = V;
                    B = t;
                    break;
                case 3:
                    R = p;
                    G = q;
                    B = V;
                    break;
                case 4:
                    R = t;
                    G = p;
                    B = V;
                    break;
                case 5:
                    R = V;
                    G = p;
                    B = q;
                    break;
            }
            return new CvColor((int)(R * 255), (int)(G * 255), (int)(B * 255));
        }

        /// <summary>
        /// 各ノードペアのスコアからできるだけスコアの最小値が高くなるようにツリーを作ります(おそらく最適解とは限らない)。
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="nodeCount">ノードの数</param>
        /// <param name="scoreParPair">各ノード間のスコア。キーは必ず一要素目の方が二要素目より小さくなければならない(=無向グラフのみ)</param>
        /// <param name="scoreAggregation">複数スコアを集計するときの計算方法</param>
        /// <param name="rootIndex">出力されるツリーのルートインデックス</param>
        /// <param name="dependencies">出力されるツリーの、子から親を求めるディクショナリ</param>
        /// <returns></returns>
        public static bool GetDependencyTree<T>(int nodeCount, IDictionary<Tuple<int, int>, T> scoreParPair, Func<IList<T>, double> scoreAggregation, out int rootIndex, out Dictionary<int, int> dependencies)
        {
            // 依存関係のツリーを作る
            Dictionary<int, double> scorePerIndex = new Dictionary<int, double>();
            for (int i = 0; i < nodeCount; i++)
            {
                List<T> scores = new List<T>();
                for (int j = 0; j < nodeCount; j++)
                {
                    if (i == j)
                        continue;
                    Tuple<int, int> key = new Tuple<int, int>(Math.Min(i, j), Math.Max(i, j));
                    T subScore = default(T);
                    if (scoreParPair.TryGetValue(key, out subScore))
                    {
                        scores.Add(subScore);
                    }
                }
                scorePerIndex[i] = scoreAggregation(scores);
            }
            double maxOccurence = scorePerIndex.Values.Max();
            rootIndex = scorePerIndex.Where(p => p.Value == maxOccurence).First().Key;
            dependencies = new Dictionary<int, int>();
            while (dependencies.Count + 1 < nodeCount)
            {
                double maxCount = double.MinValue;
                int childIndexAtMaxCount = 0;
                int parentIndexAtMaxCount = 0;
                for (int childIndex = 0; childIndex < nodeCount; childIndex++)
                {
                    if (childIndex == rootIndex || dependencies.ContainsKey(childIndex))
                        continue;
                    foreach (int parentCandidate in dependencies.Keys.Concat(new int[] { rootIndex }))
                    {
                        Tuple<int, int> key = new Tuple<int, int>(Math.Min(childIndex, parentCandidate), Math.Max(childIndex, parentCandidate));
                        T subScore;
                        if (scoreParPair.TryGetValue(key, out subScore))
                        {
                            double score = scoreAggregation(new T[] { subScore });
                            if (score > maxCount)
                            {
                                maxCount = score;
                                childIndexAtMaxCount = childIndex;
                                parentIndexAtMaxCount = parentCandidate;
                            }
                        }
                    }
                }
                if (maxCount == -1 || maxCount == double.MinValue)
                {
                    return false;
                }
                dependencies[childIndexAtMaxCount] = parentIndexAtMaxCount;
            }
            return true;
        }
        /// <summary>
        /// ツリーのルートと、子から親を求めるディクショナリから、ルートからノードを水平に順次辿って行ったときのノードとその親ノードのインデックスを列挙します。ルートノードは列挙されません。
        /// </summary>
        /// <param name="rootIndex">ルートノードのインデックス</param>
        /// <param name="dependencies">ツリーの子から親を求めるディクショナリ</param>
        /// <returns></returns>
        public static IEnumerable<KeyValuePair<int, int>> EnumerateDependencyPairs(int rootIndex, IDictionary<int, int> dependencies)
        {
            HashSet<int> completedIndex = new HashSet<int>();
            completedIndex.Add(rootIndex);
            while (completedIndex.Count < dependencies.Count + 1)
            {
                // 親が終わっていて子が終わっていないようなエッジの子ノードを求める
                List<int> targetIndices = dependencies.Where(x => !completedIndex.Contains(x.Key) && completedIndex.Contains(x.Value)).Select(x => x.Key).ToList();
                foreach (int nodeIndex in targetIndices)
                {
                    int parentIndex = dependencies[nodeIndex];
                    yield return new KeyValuePair<int, int>(nodeIndex, parentIndex);
                    completedIndex.Add(nodeIndex);
                }
            }
        }
        /// <summary>
        /// LMedSによる最小二乗近似を求めます
        /// </summary>
        /// <typeparam name="TData">入力となる要素列の型</typeparam>
        /// <typeparam name="TOut">出力される要素列の型</typeparam>
        /// <param name="data">入力要素列のリスト</param>
        /// <param name="minSquare">入力要素列のリストから通常の最小二乗法により出力要素列を求めるメソッド</param>
        /// <param name="getErrorSq">入力要素列と出力要素列から誤差を求めるメソッド</param>
        /// <param name="samplingCounts">誤差の評価時にサンプリングする要素列数，のリスト</param>
        /// <param name="trialPerSamplingCount">サンプル要素列数ごとの試行回数</param>
        /// <param name="medianSampleCount">誤差の中央値の計算時の高速化のためのサンプリング数</param>
        /// <returns></returns>
        public static TOut LMedS<TData, TOut>(IList<TData> data, Func<IList<TData>, TOut> minSquare, Func<TData, TOut, double> getErrorSq, IEnumerable<int> samplingCounts, int trialPerSamplingCount, int medianSampleCount)
        {
            object lockObj = new object();
            List<int> samplingCountList = samplingCounts.ToList();
            TOut ret = default(TOut);
            double minError = double.MaxValue;
            foreach (int sampleCount in samplingCountList)
            {
                if (sampleCount > 0 && sampleCount <= data.Count)
                {
                    Parallel.For(0, trialPerSamplingCount, j =>
                    {
                        Random rand = new Random();
                        int[] sampleIndices = Enumerable.Range(0, data.Count).ToArray();
                        for (int k = 0; k < sampleCount; k++)
                        {
                            // シャッフル
                            int exchange = rand.Next(data.Count - k);
                            int temp = sampleIndices[exchange];
                            sampleIndices[exchange] = sampleIndices[k];
                            sampleIndices[k] = temp;
                        }
                        List<TData> samples = new List<TData>();
                        for (int k = 0; k < sampleCount; k++)
                        {
                            TData sample = data[sampleIndices[k]];
                            samples.Add(sample);
                        }
                        TOut convParam = minSquare(samples);
                        List<double> errors = new List<double>();
                        foreach (TData datum in data)
                        {
                            errors.Add(getErrorSq(datum, convParam));
                        }
                        double medError = CalcEx.GetSampledMedian(errors, medianSampleCount);
                        lock (lockObj)
                        {
                            if (medError < minError)
                            {
                                minError = medError;
                                ret = convParam;
                            }
                        }
                    });
                }
            }
            return ret;
        }

        public static TOut WeightedLMedS<TData, TOut>(IList<TData> data, Func<TData, double> getWeight, Func<IList<TData>, TOut> minSquare, Func<TData, TOut, double> getErrorSq, IEnumerable<int> samplingCounts, int trialPerSamplingCount)
        {
            object lockObj = new object();
            List<int> samplingCountList = samplingCounts.ToList();
            TOut ret = default(TOut);
            double minError = double.MaxValue;
            // 重みのhistgram作成
            List<double> histgram = new List<double>();
            List<double> weightList = new List<double>();
            double weightSum = 0;
            for (int i = 0; i < data.Count; i++)
            {
                TData datum = data[i];
                double weight = getWeight(datum);
                weightList.Add(weight);
                weightSum += weight;
                histgram.Add(weightSum);
            }
            foreach (int sampleCount in samplingCountList)
            {
                if (sampleCount > 0)
                {
                    Parallel.For(0, trialPerSamplingCount, j =>
                    {
                        int[] sampleIndices = new int[sampleCount];
                        Random rand = new Random();
                        for (int k = 0; k < sampleCount; k++)
                        {
                            // インデックスがかぶっても気にしない
                            double position = rand.NextDouble() * weightSum;
                            int index = ListEx.GetMinGreaterEqualIndexFromBinarySearch(ListEx.BinarySearch(histgram, position));
                            sampleIndices[k] = index;
                        }
                        List<TData> samples = new List<TData>();
                        for (int k = 0; k < sampleCount; k++)
                        {
                            TData sample = data[sampleIndices[k]];
                            samples.Add(sample);
                        }
                        TOut convParam = minSquare(samples);
                        List<Tuple<double, double>> errors = new List<Tuple<double, double>>();
                        for (int i = 0; i < data.Count; i++)
                        {
                            TData datum = data[i];
                            errors.Add(new Tuple<double, double>(getErrorSq(datum, convParam), weightList[i]));
                        }
                        double medError = CalcEx.GetWeightedMedian(errors);
                        if (medError < minError)
                        {
                            minError = medError;
                            ret = convParam;
                        }
                    });
                }
            }
            return ret;
        }
        public static TOut DividedWeightedLMedS<TData, TOut>(IList<List<TData>> dataList, Func<TData, double> getWeight, Func<IList<TData>, TOut> minSquare, Func<TData, TOut, double> getErrorSq, IEnumerable<int> samplingCounts, int trialPerSamplingCount)
        {
            return DividedWeightedLMedS(dataList.Select(p => (IList<TData>)p).ToList(), getWeight, minSquare, getErrorSq, samplingCounts, trialPerSamplingCount);
        }
        public static TOut DividedWeightedLMedS<TData, TOut>(IList<IList<TData>> dataList, Func<TData, double> getWeight, Func<IList<TData>, TOut> minSquare, Func<TData, TOut, double> getErrorSq, IEnumerable<int> samplingCounts, int trialPerSamplingCount)
        {
            object lockObj = new object();
            List<int> samplingCountList = samplingCounts.ToList();
            TOut ret = default(TOut);
            double minError = double.MaxValue;
            // 重みのhistgram作成
            List<List<double>> histgram = new List<List<double>>();
            List<List<double>> weightList = new List<List<double>>();

            int elemAll = 0;
            List<double> weightSum = new List<double>();
            for (int j = 0; j < dataList.Count; j++)
            {
                List<double> histgramTemp = new List<double>();
                List<double> weightListTemp = new List<double>();

                double weightSumTemp = 0;
                for (int i = 0; i < dataList[j].Count; i++)
                {
                    TData datum = dataList[j][i];
                    double weight = getWeight(datum);
                    weightListTemp.Add(weight);
                    weightSumTemp += weight;
                    histgramTemp.Add(weightSumTemp);
                }
                histgram.Add(histgramTemp);
                weightList.Add(weightListTemp);
                weightSum.Add(weightSumTemp);
                elemAll++;
            }

            foreach (int sampleCount in samplingCountList)
            {
                if (sampleCount > 0)
                {
                    Parallel.For(0, trialPerSamplingCount, j =>
                    {
                        int[] sampleIndices = new int[sampleCount];
                        Random rand = new Random();
                        List<TData> samples = new List<TData>();
                        for (int l = 0; l < dataList.Count; l++)
                        {
                            for (int k = 0; k < sampleCount; k++)
                            {
                                // インデックスがかぶっても気にしない
                                double position = rand.NextDouble() * weightSum[l];
                                int index = ListEx.GetMinGreaterEqualIndexFromBinarySearch(ListEx.BinarySearch(histgram[l], position));
                                sampleIndices[k] = index;
                            }
                            for (int k = 0; k < sampleCount; k++)
                            {
                                TData sample = dataList[l][sampleIndices[k]];
                                samples.Add(sample);
                            }
                        }
                        TOut convParam = minSquare(samples);
                        double medError = 0;
                        for (int l = 0; l < dataList.Count; l++)
                        {
                            List<Tuple<double, double>> errors = new List<Tuple<double, double>>();
                            for (int i = 0; i < dataList[l].Count; i++)
                            {
                                TData datum = dataList[l][i];
                                errors.Add(new Tuple<double, double>(getErrorSq(datum, convParam), weightList[l][i]));
                            }
                            double medErrorTemp = CalcEx.GetWeightedMedian(errors);
                            medError += medErrorTemp * dataList[l].Count / elemAll;
                        }
                        if (medError < minError)
                        {
                            minError = medError;
                            ret = convParam;
                        }
                    });
                }
            }
            return ret;
        }


        /// <summary>
        /// 配列のリストから配列インデックス毎の値の分散を求めます
        /// </summary>
        /// <param name="descSet"></param>
        /// <param name="dimension"></param>
        /// <returns></returns>
        public static double[] GetDimensionVariances(IEnumerable<float[]> descSet, int dimension)
        {
            int count = 0;
            double[] sum = new double[dimension];
            double[] sqSum = new double[dimension];

            foreach (float[] desc in descSet)
            {
                for (int i = 0; i < dimension; i++)
                {
                    float v = desc[i];
                    sum[i] += v;
                    sqSum[i] += v * v;
                }
                count++;
            }

            if (count == 0)
                return new double[dimension];
            double[] variances = new double[dimension];
            for (int i = 0; i < dimension; i++)
            {
                double avg = sum[i] / count;
                variances[i] = sqSum[i] / count - avg * avg;
            }
            return variances;
        }

        /// <summary>
        /// 左右を入れ替えた骨格要素を返します
        /// </summary>
        /// <param name="joint"></param>
        /// <returns></returns>
        public static JointType GetMirroredJoint(JointType joint)
        {
            switch (joint)
            {
                case JointType.AnkleLeft:
                    return JointType.AnkleRight;
                case JointType.ElbowLeft:
                    return JointType.ElbowRight;
                case JointType.FootLeft:
                    return JointType.FootRight;
                case JointType.HandLeft:
                    return JointType.HandRight;
                case JointType.HandTipLeft:
                    return JointType.HandTipRight;
                case JointType.HipLeft:
                    return JointType.HipRight;
                case JointType.KneeLeft:
                    return JointType.KneeRight;
                case JointType.ShoulderLeft:
                    return JointType.ShoulderRight;
                case JointType.ThumbLeft:
                    return JointType.ThumbRight;
                case JointType.WristLeft:
                    return JointType.WristRight;

                case JointType.AnkleRight:
                    return JointType.AnkleLeft;
                case JointType.ElbowRight:
                    return JointType.ElbowLeft;
                case JointType.FootRight:
                    return JointType.FootLeft;
                case JointType.HandRight:
                    return JointType.HandLeft;
                case JointType.HandTipRight:
                    return JointType.HandTipLeft;
                case JointType.HipRight:
                    return JointType.HipLeft;
                case JointType.KneeRight:
                    return JointType.KneeLeft;
                case JointType.ShoulderRight:
                    return JointType.ShoulderLeft;
                case JointType.ThumbRight:
                    return JointType.ThumbLeft;
                case JointType.WristRight:
                    return JointType.WristLeft;

            }
            return joint;
        }

        public static void TrimEdgeDepthMat(ref CvMat dest, CvMat src, ref CvMat[] tempMat)
        {
            if (tempMat == null || tempMat.Length != 7)
            {
                tempMat = new CvMat[7];
            }
            CvEx.InitCvMat(ref tempMat[0], src, MatrixType.U8C1);
            CvEx.InitCvMat(ref tempMat[1], src, MatrixType.U8C1);
            CvEx.InitCvMat(ref tempMat[2], src, MatrixType.S16C1);
            CvEx.InitCvMat(ref tempMat[3], src);
            CvEx.InitCvMat(ref tempMat[4], src);
            CvEx.InitCvMat(ref tempMat[5], src, MatrixType.U8C1);
            CvEx.InitCvMat(ref tempMat[6], src);
            CvEx.InitCvMat(ref dest, src);

            CvMat u8 = tempMat[0], u8bin = tempMat[1], lap = tempMat[2], s16lap = tempMat[3], s16bin = tempMat[4], u8erode = tempMat[5], s16lapFill = tempMat[6];

            src.ConvertScaleAbs(u8, 1.0 / 40);
            u8.Threshold(u8bin, 1, 1, ThresholdType.Binary);
            u8bin.Erode(u8erode, null, 1);
            u8erode.Convert(s16bin);
            u8.Laplace(lap);
            lap.ConvertScaleAbs(u8);
            u8.Threshold(u8, 20, 1, ThresholdType.BinaryInv);
            u8.Convert(s16lap);
            FillHoleDepthMat(ref s16lapFill, s16lap, 3, 0.4, 0, 100);
            s16lapFill.Erode(s16lap, null, 1);
            src.Mul(s16lap, dest);
            dest.Mul(s16bin, dest);
        }

        public static List<CvPoint3D64f> GetRealPointsFromDepthMat(CvMat depthMat, KinectUndistortion undist, int pixelSkip, CvMat conversion)
        {
            List<CvPoint3D64f> ret = new List<CvPoint3D64f>();
            unsafe
            {
                short* depthArr = depthMat.DataInt16;
                CvSize size = new CvSize(depthMat.Cols, depthMat.Rows);
                Parallel.For(0, size.Height / pixelSkip, (yTemp) =>
                {
                    int y = yTemp * pixelSkip;
                    int offset = y * depthMat.Cols;
                    for (int x = 0; x < depthMat.Cols; x += pixelSkip)
                    {
                        short depth = depthArr[offset + x];
                        if (depth != 0)
                        {
                            CvPoint3D64f pos = undist.GetRealFromScreenPos(x, y, depth, size);
                            if (conversion != null)
                            {
                                pos = CvEx.ConvertPoint3D(pos, conversion);
                            }
                            lock (ret)
                            {
                                ret.Add(pos);
                            }
                        }
                    }
                });
            }
            return ret;
        }
        public static List<CvPoint3D64f> GetRealPointsFromDepthMat(CvMat depthMat, KinectUndistortion undist, int pixelSkip)
        {
            return GetRealPointsFromDepthMat(depthMat, undist, pixelSkip, null);
        }

        public static List<CvPoint3D64f> GetScreenPoints(CvMat depthMat, int pixelSkip)
        {
            List<CvPoint3D64f> ret = new List<CvPoint3D64f>();
            unsafe
            {
                short* depthArr = depthMat.DataInt16;
                CvSize size = new CvSize(depthMat.Cols, depthMat.Rows);
                for (int y = 0; y < depthMat.Rows; y += pixelSkip)
                {
                    int offset = y * depthMat.Cols;
                    for (int x = 0; x < depthMat.Cols; x += pixelSkip)
                    {
                        short depth = depthArr[offset + x];
                        if (depth != 0)
                        {
                            lock (ret)
                            {
                                ret.Add(new CvPoint3D64f(x, y, depth));
                            }
                        }
                    }
                }
            }
            return ret;
        }

        public static ThreeTuple<short> Add(this ThreeTuple<short> tupleShort, ThreeTuple<short> other)
        {
            return new ThreeTuple<short>((short)(tupleShort.Item1 + other.Item1), (short)(tupleShort.Item2 + other.Item2), (short)(tupleShort.Item3 + other.Item3));
        }
        public static ThreeTuple<short> Add(this ThreeTuple<short> tupleShort, int i1, int i2, int i3)
        {
            return new ThreeTuple<short>((short)(tupleShort.Item1 + i1), (short)(tupleShort.Item2 + i2), (short)(tupleShort.Item3 + i3));
        }

        /*
        public static CvMat AdjustLocalizationWithVoxel(CvMat depthMat, VoxelSet voxelSet, KinectUndistortion undist, int pixelSkip, CvMat convLocal2world, int repetition)
        {
            CvSize size = new CvSize(depthMat.Cols, depthMat.Rows);
            List<CvPoint3D64f> points = (from p in GetScreenPoints(depthMat, pixelSkip).AsParallel()
                                         where undist.CameraStruct.GetRatioSqOfPrincipalToFocal(p.X, p.Y) <= 0.85 && p.Z <= 6000
                                         select undist.GetRealFromScreenPos(p, size)).ToList();
            for (int i = 0; i < repetition; i++)
            {
                CoordRotTransConversion crtc = new CoordRotTransConversion();
                int okCount = 0;
                bool errorFound = false;
                Parallel.ForEach(points, point =>
                {
                    CvPoint3D64f pos = CvEx.ConvertPoint3D(point, convLocal2world);
                    ThreeTuple<short> center = voxelSet.GetKeyFromPoint(pos);
                    double weight = 1.0 / (Math.Abs(point.Z - 1500) + 5000);
                    VoxelValue vvc;
                    if (voxelSet.TryGetValue(center, out vvc) && vvc.Reliability >= 2)
                    {
                        crtc.PutPoint(point, point, weight);
                        System.Threading.Interlocked.Increment(ref okCount);
                    }
                    else
                    {
                        CvPoint3D64f minNeighborPos = new CvPoint3D64f();
                        double minDistanceSq = double.MaxValue;
                        int count = 0;
                        for (int i1 = -1; i1 <= 1; i1++)
                        {
                            for (int i2 = -1; i2 <= 1; i2++)
                            {
                                for (int i3 = -1; i3 <= 1; i3++)
                                {
                                    if (i1 == 0 && i2 == 0 && i3 == 0)
                                        continue;
                                    ThreeTuple<short> neighbor = center.Add(i1, i2, i3);
                                    VoxelValue vv;
                                    if (voxelSet.TryGetValue(neighbor, out vv) && vv.Reliability >= 2)
                                    {
                                        count++;
                                        CvPoint3D64f neighborPos = voxelSet.GetPointFromKey(neighbor, 0.5);
                                        double distanceSq = CvEx.GetDistanceSq(pos, neighborPos);
                                        if (distanceSq < minDistanceSq)
                                        {
                                            minDistanceSq = distanceSq;
                                            minNeighborPos = neighborPos;
                                        }
                                    }
                                }
                            }
                        }
                        if (count >= 8)
                        {
                            crtc.PutPoint(point, minNeighborPos, weight / 10);
                            errorFound = true;
                        }
                    }
                });
                if (!errorFound)
                {
                    break;
                }
                if (okCount > crtc.PointCount * 0.75)
                {
                    convLocal2world = crtc.Solve() * convLocal2world;
                }
                else
                {
                    break;
                }
            }
            return convLocal2world.Clone();
        }
         */

        public static Dictionary<JointType, CvPoint3D64f> LinearMedianSkeletons(Dictionary<JointType, CvPoint3D64f>[] skeletonList, double[] weightList)
        {
            if (skeletonList.Length != weightList.Length)
            {
                throw new ArgumentException("Array length mismatch");
            }
            HashSet<JointType> joints = new HashSet<JointType>();
            foreach (var skeleton in skeletonList)
            {
                if (skeleton != null)
                {
                    foreach (JointType joint in skeleton.Keys)
                    {
                        joints.Add(joint);
                    }
                }
            }
            if (joints.Count == 0)
                return null;
            Dictionary<JointType, CvPoint3D64f> ret = new Dictionary<JointType, CvPoint3D64f>();
            foreach (JointType joint in joints)
            {
                List<Tuple<CvPoint3D64f, double>> positionWeight = new List<Tuple<CvPoint3D64f, double>>();
                for (int i = 0; i < skeletonList.Length; i++)
                {
                    var skeleton = skeletonList[i];
                    if (skeleton != null)
                    {
                        double weight = weightList[i];
                        CvPoint3D64f position;
                        if (skeleton.TryGetValue(joint, out position))
                        {
                            positionWeight.Add(new Tuple<CvPoint3D64f, double>(position, weight));
                        }
                    }
                }
                if (positionWeight.Select(w => w.Item2).Sum() > 0)
                {
                    double medianX = GetWeightedLinearMedian(positionWeight.Select(q => new Tuple<double, double>(q.Item1.X, q.Item2)).ToList());
                    double medianY = GetWeightedLinearMedian(positionWeight.Select(q => new Tuple<double, double>(q.Item1.Y, q.Item2)).ToList());
                    double medianZ = GetWeightedLinearMedian(positionWeight.Select(q => new Tuple<double, double>(q.Item1.Z, q.Item2)).ToList());
                    ret[joint] = new CvPoint3D64f(medianX, medianY, medianZ);
                }
            }
            return ret;
        }

        public static Dictionary<JointType, CvPoint3D64f> LinearAverageSkeletons(Dictionary<JointType, CvPoint3D64f>[] skeletonList, double[] weightList)
        {
            if (skeletonList.Length != weightList.Length)
            {
                throw new ArgumentException("Array length mismatch");
            }
            HashSet<JointType> joints = new HashSet<JointType>();
            foreach (var skeleton in skeletonList)
            {
                if (skeleton != null)
                {
                    foreach (JointType joint in skeleton.Keys)
                    {
                        joints.Add(joint);
                    }
                }
            }
            if (joints.Count == 0)
                return null;
            Dictionary<JointType, CvPoint3D64f> ret = new Dictionary<JointType, CvPoint3D64f>();
            foreach (JointType joint in joints)
            {
                List<Tuple<CvPoint3D64f, double>> positionWeight = new List<Tuple<CvPoint3D64f, double>>();
                for (int i = 0; i < skeletonList.Length; i++)
                {
                    var skeleton = skeletonList[i];
                    if (skeleton != null)
                    {
                        double weight = weightList[i];
                        CvPoint3D64f position;
                        if (skeleton.TryGetValue(joint, out position))
                        {
                            positionWeight.Add(new Tuple<CvPoint3D64f, double>(position, weight));
                        }
                    }
                }
                if (positionWeight.Select(w => w.Item2).Sum() > 0)
                {
                    double averageX = System.Linq.Enumerable.Average(positionWeight.Select(q => q.Item1.X).ToList());
                    double averageY = System.Linq.Enumerable.Average(positionWeight.Select(q => q.Item1.Y).ToList());
                    double averageZ = System.Linq.Enumerable.Average(positionWeight.Select(q => q.Item1.Z).ToList());
                    ret[joint] = new CvPoint3D64f(averageX, averageY, averageZ);
                }
            }
            return ret;
        }
    }
    

    /// <summary>
    /// 同じ型が3つ並んだ組
    /// </summary>
    /// <typeparam name="T"></typeparam>
    public struct ThreeTuple<T>
    {
        public T Item1, Item2, Item3;
        public ThreeTuple(T item1, T item2, T item3)
        {
            this.Item1 = item1;
            this.Item2 = item2;
            this.Item3 = item3;
        }
        /// <summary>
        /// 通常のTupleからの変換
        /// </summary>
        /// <param name="item"></param>
        /// <returns></returns>
        public static implicit operator ThreeTuple<T>(Tuple<T, T, T> item)
        {
            return new ThreeTuple<T>(item.Item1, item.Item2, item.Item3);
        }

        public override bool Equals(object obj)
        {
            if (obj is ThreeTuple<T>)
            {
                ThreeTuple<T> that = (ThreeTuple<T>)obj;
                return this.Item1.Equals(that.Item1) && this.Item2.Equals(that.Item2) && this.Item3.Equals(that.Item3);
            }
            return base.Equals(obj);
        }
        public override int GetHashCode()
        {
            int hash1 = this.Item1.GetHashCode();
            int hash2 = this.Item2.GetHashCode();
            int hash3 = this.Item3.GetHashCode();
            return (hash1 << 20) ^ (hash1 >> 12) ^ (hash2 << 10) ^ (hash2 >> 22) ^ hash3;
        }
        public override string ToString()
        {
            return string.Format("{0}, {1}, {2}", this.Item1, this.Item2, this.Item3);
        }

        /// <summary>
        /// エディントンのイプシロン(三要素が偶順列なら1，奇順列なら-1，それ以外は0)
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <param name="c"></param>
        /// <returns></returns>
        public double LeviCivita(int a, int b, int c)
        {
            a = a % 3;
            b = b % 3;
            c = c % 3;
            if (a == 0 && b == 1 && c == 2)
                return 1;
            if (a == 1 && b == 2 && c == 0)
                return 1;
            if (a == 2 && b == 0 && c == 2)
                return 1;
            if (a == 0 && b == 2 && c == 1)
                return -1;
            if (a == 1 && b == 0 && c == 2)
                return -1;
            if (a == 2 && b == 1 && c == 0)
                return -1;
            return 0;
        }
    }
}
