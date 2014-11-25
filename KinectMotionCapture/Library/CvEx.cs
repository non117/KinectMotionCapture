using System.Windows;
using System.Windows.Media.Imaging;
using OpenCvSharp;
using OpenCvSharp.Extensions;
using System;
using System.Collections.Generic;
using Media3D = System.Windows.Media.Media3D;
using System.Runtime.InteropServices;
using System.Linq;
using Microsoft.Kinect;


namespace KinectMotionCapture
{
    /// <summary>
    /// OpenCV用支援メソッドクラス
    /// </summary>
    public static class CvEx
    {
        
        /// <summary>
        /// CameraSpacePointをCvPoint3D32fに変換します
        /// </summary>
        /// <param name="point"></param>
        /// <returns></returns>
        public static CvPoint3D32f ToCvPoint3D(this CameraSpacePoint point)
        {
            return new CvPoint3D32f(point.X, point.Y, point.Z);
        }
        /*
        public static Microsoft.Xna.Framework.Vector3 ToVector3(this CvPoint3D64f point)
        {
            return new Microsoft.Xna.Framework.Vector3((float)point.X, (float)point.Y, (float)point.Z);
        }
        public static Microsoft.Xna.Framework.Vector3 ToVector3(this CvPoint3D32f point)
        {
            return new Microsoft.Xna.Framework.Vector3(point.X, point.Y, point.Z);
        }
         */
        /// <summary>
        /// CvPoint3D32fをCameraSpacePointに変換します
        /// </summary>
        /// <param name="point"></param>
        /// <returns></returns>
        public static CameraSpacePoint ToCameraSpacePoint(this CvPoint3D32f point)
        {
            return new CameraSpacePoint(){X = point.X, Y = point.Y, Z = point.Z};
        }

        /// <summary>
        /// CvPoint3D64fをOpenNI.Point3Dに変換します
        /// </summary>
        /// <param name="point"></param>
        /// <returns></returns>
        public static CameraSpacePoint ToCameraSpacePoint(this CvPoint3D64f point)
        {
            return new CameraSpacePoint() { X = (float)point.X, Y = (float)point.Y, Z = (float)point.Z };
        }

        /*
        public static Microsoft.Xna.Framework.Color ToXnaColor(this CvColor color)
        {
            return new Microsoft.Xna.Framework.Color(color.R, color.G, color.B);
        }
        */
        /// <summary>
        /// 最小二乗法を解く．doubleのリストをCvMatに変換してCv.Solveを呼び出し，結果をdoubleの配列で返します．
        /// </summary>
        /// <param name="left">方程式左辺の線形結合の係数の配列の列挙．配列長が一定でなければ例外を投げます</param>
        /// <param name="right">方程式の右辺の列挙．左辺の列挙数と異なる場合は例外を投げます</param>
        /// <param name="method">逆行列を解くときの手段</param>
        /// <exception cref="ArgumentException">入力サイズに不一致がある</exception>
        /// <returns></returns>
        public static double[] Solve(IEnumerable<double[]> left, IEnumerable<double> right, InvertMethod method)
        {
            double[][] mat = left.ToArray();
            if (mat.Length == 0)
                return null;
            int len = mat[0].Length;
            CvMat lMat = new CvMat(mat.Length, len, MatrixType.F64C1);

            int i = 0;
            foreach (double[] arr in mat)
            {
                if (len != arr.Length)
                {
                    throw new ArgumentException("left must be matrix");
                }
                for (int k = 0; k < len; k++)
                {
                    lMat[i, k] = arr[k];
                }
                i++;
            }

            int j = 0;
            CvMat rMat = new CvMat(mat.Length, 1, MatrixType.F64C1);
            foreach (double r in right)
            {
                if (j >= mat.Length)
                {
                    throw new ArgumentException("right length mismatches");
                }
                rMat[j, 0] = r;
                j++;
            }
            if (j != mat.Length)
            {
                throw new ArgumentException("right length mismatches");
            }
            CvMat ans = new CvMat(len, 1, MatrixType.F64C1);
            if (!Cv.Solve(lMat, rMat, ans, method))
                return null;
            return ans.Select(x => x.Val0).ToArray();
        }
        /// <summary>
        /// ベクトルのユークリッド距離における長さを返します
        /// </summary>
        /// <param name="vector"></param>
        /// <returns></returns>
        public static double GetLength(CvPoint3D64f vector)
        {
            return Math.Sqrt(vector.X * vector.X + vector.Y * vector.Y + vector.Z * vector.Z);
        }
        /// <summary>
        /// ベクトルのユークリッド距離における長さを返します
        /// </summary>
        /// <param name="vector"></param>
        /// <returns></returns>
        public static double GetLengthSq(CvPoint3D64f vector)
        {
            return vector.X * vector.X + vector.Y * vector.Y + vector.Z * vector.Z;
        }
        /// <summary>
        /// ベクトルのユークリッド距離における長さを返します
        /// </summary>
        /// <param name="vector"></param>
        /// <returns></returns>        
        public static double GetLength(CvPoint2D64f vector)
        {
            return Math.Sqrt(vector.X * vector.X + vector.Y * vector.Y);
        }
        /// <summary>
        /// ベクトルの二乗和を返します
        /// </summary>
        /// <param name="vector"></param>
        /// <returns></returns>
        public static double GetLengthSq(CvPoint2D64f vector)
        {
            return vector.X * vector.X + vector.Y * vector.Y;
        }
        /// <summary>
        /// 二つのベクトルの二乗距離を返します
        /// </summary>
        /// <param name="first"></param>
        /// <param name="second"></param>
        /// <returns></returns>
        public static double GetDistanceSq(CvPoint3D64f first, CvPoint3D64f second)
        {
            return GetLengthSq(first - second);
        }
        /// <summary>
        /// 二つのベクトルの二乗距離を返します
        /// </summary>
        /// <param name="first"></param>
        /// <param name="second"></param>
        /// <returns></returns>
        public static double GetDistanceSq(CvPoint2D64f first, CvPoint2D64f second)
        {
            return GetLengthSq(first - second);
        }

        /// <summary>
        /// 領域が未確保またはフォーマットが異なる場合は新しく領域を確保します．
        /// </summary>
        /// <param name="dest"></param>
        /// <param name="rows"></param>
        /// <param name="cols"></param>
        /// <param name="type"></param>
        public static void InitCvMat(ref CvMat dest, int rows, int cols, MatrixType type)
        {
            if (dest == null || dest.Cols != cols || dest.Rows != rows || dest.ElemType != type)
            {
                if (dest != null)
                {
                    dest.Dispose();
                }
                dest = new CvMat(rows, cols, type);
            }
        }
        /// <summary>
        /// 指定されたフォーマットのCvMatを返します．
        /// </summary>
        /// <param name="rows"></param>
        /// <param name="cols"></param>
        /// <param name="type"></param>
        /// <returns></returns>
        public static CvMat InitCvMat(int rows, int cols, MatrixType type)
        {
            CvMat ret = null;
            InitCvMat(ref ret, rows, cols, type);
            return ret;
        }

        /// <summary>
        /// 領域が未確保またはフォーマットが異なる場合は新しく領域を確保します．
        /// </summary>
        /// <param name="dest"></param>
        /// <param name="sizeSrc"></param>
        /// <param name="type"></param>
        public static void InitCvMat(ref CvMat dest, CvMat sizeSrc, MatrixType type)
        {
            InitCvMat(ref dest, sizeSrc.Rows, sizeSrc.Cols, type);
        }
        /// <summary>
        /// 指定されたフォーマットのCvMatを返します．
        /// </summary>
        /// <param name="sizeSrc"></param>
        /// <param name="type"></param>
        /// <returns></returns>
        public static CvMat InitCvMat(CvMat sizeSrc, MatrixType type)
        {
            CvMat dest = null;
            InitCvMat(ref dest, sizeSrc, type);
            return dest;
        }
        /// <summary>
        /// 領域が未確保またはフォーマットが異なる場合は新しく領域を確保します．
        /// </summary>
        /// <param name="dest"></param>
        /// <param name="formatSrc"></param>
        public static void InitCvMat(ref CvMat dest, CvMat formatSrc)
        {
            InitCvMat(ref dest, formatSrc, formatSrc.ElemType);
        }
        /// <summary>
        /// 指定されたCvMatと同じフォーマットのCvMatを返します．
        /// </summary>
        /// <param name="formatSrc"></param>
        /// <returns></returns>
        public static CvMat InitCvMat(CvMat formatSrc)
        {
            CvMat dest = null;
            InitCvMat(ref dest, formatSrc, formatSrc.ElemType);
            return dest;
        }

        /// <summary>
        /// 領域が未確保またはフォーマットが異なる場合は新しく領域を確保し，データをコピーします
        /// </summary>
        /// <param name="dest"></param>
        /// <param name="src"></param>
        public static void CloneCvMat(ref CvMat dest, CvMat src)
        {
            InitCvMat(ref dest, src);
            src.Copy(dest);
        }
        /// <summary>
        /// 新しいCvMatを作成し，データをコピーして返します．
        /// </summary>
        /// <param name="src"></param>
        /// <returns></returns>
        public static CvMat CloneCvMat(CvMat src)
        {
            CvMat dest = null;
            InitCvMat(ref dest, src);
            src.Copy(dest);
            return dest;
        }

        /// <summary>
        /// オブジェクトが確保されている場合にDisposeします
        /// </summary>
        /// <param name="obj"></param>
        public static void DisposeToNull(ref CvMat obj)
        {
            if (obj != null)
            {
                obj.Dispose();
                obj = null;
            }
        }

        /// <summary>
        /// 領域が未確保またはフォーマットが異なる場合に新しく領域を確保し，データをコピーします
        /// </summary>
        /// <param name="bmp"></param>
        /// <param name="mat"></param>
        /// <param name="format"></param>
        public static void GetBmpFromMat(ref WriteableBitmap bmp, CvMat mat, System.Windows.Media.PixelFormat format)
        {            
            if (bmp == null || bmp.PixelWidth != mat.Height || bmp.PixelHeight != mat.Width || bmp.Format != format)
            {
                bmp = new WriteableBitmap(mat.Width, mat.Height, 96, 96, format, null);
            }
            bmp.WritePixels(new Int32Rect(0, 0, bmp.PixelWidth, bmp.PixelHeight), mat.Data, bmp.BackBufferStride * bmp.PixelHeight, bmp.BackBufferStride, 0, 0);
             
        }

        [Obsolete("Cv.Rodrigues2", true)]
        public static void Rodrigues(double rx, double ry, double rz, CvMat dst)
        {
            double[] I = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
            double theta = Math.Sqrt(rx * rx + ry * ry + rz * rz);
            double c = Math.Cos(theta);
            double s = Math.Sin(theta);
            double c1 = 1.0 - c;
            double itheta = theta != 0 ? 1.0 / theta : 0.0;

            rx *= itheta;
            ry *= itheta;
            rz *= itheta;

            double[] rrt = { rx * rx, rx * ry, rx * rz, rx * ry, ry * ry, ry * rz, rx * rz, ry * rz, rz * rz };
            double[] _r_x_ = { 0, -rz, ry, rz, 0, -rx, -ry, rx, 0 };
            double[] R = new double[9];

            // R = cos(theta)*I + (1 - cos(theta))*r*rT + sin(theta)*[r_x]
            // where [r_x] is [0 -rz ry; rz 0 -rx; -ry rx 0]
            for (int k = 0; k < 9; k++)
                R[k] = c * I[k] + c1 * rrt[k] + s * _r_x_[k];

            for (int k = 0; k < 9; k++)
                dst[k] = R[k];
        }

        /// <summary>
        /// 縦ベクトル用の4x4平行移動行列を作成します
        /// </summary>
        /// <param name="translation"></param>
        /// <returns></returns>
        public static CvMat GetTranslation(CvPoint3D64f translation)
        {
            CvMat ret = CvMat.Identity(4, 4, MatrixType.F64C1);
            ret[0, 3] = translation.X;
            ret[1, 3] = translation.Y;
            ret[2, 3] = translation.Z;
            return ret;
        }
        /// <summary>
        /// CvMatを指定されたデータで埋めます
        /// </summary>
        /// <param name="mat"></param>
        /// <param name="arr"></param>
        public static void FillCvMat(CvMat mat, params CvScalar[] arr)
        {
            FillCvMat(mat, (IList<CvScalar>)arr);
        }
        /// <summary>
        /// CvMatを指定されたデータで埋めます
        /// </summary>
        /// <param name="mat"></param>
        /// <param name="arr"></param>
        public static void FillCvMat(CvMat mat, IList<CvScalar> arr)
        {
            int count = 0;
            for (int i = 0; i < mat.Rows; i++)
            {
                for (int j = 0; j < mat.Cols; j++)
                {
                    mat.Set2D(i, j, arr[count++]);
                }
            }
        }

        /// <summary>
        /// 1チャンネルのCvMatを配列で埋めます。
        /// </summary>
        /// <param name="mat"></param>
        /// <param name="arr"></param>
        public static void FillCvMat(CvMat mat, IList<double> arr)
        {
            int count = 0;
            for (int i = 0; i < mat.Rows; i++)
            {
                for (int j = 0; j < mat.Cols; j++)
                {
                    mat[i, j] = arr[count++];
                }
            }
        }
        /// <summary>
        /// 1チャンネルのCvMatを配列で埋めます。
        /// </summary>
        /// <param name="mat"></param>
        /// <param name="arr"></param>
        public static void FillCvMat(CvMat mat, IList<float> arr)
        {
            int count = 0;
            for (int i = 0; i < mat.Rows; i++)
            {
                for (int j = 0; j < mat.Cols; j++)
                {
                    mat[i, j] = arr[count++];
                }
            }
        }
        /// <summary>
        /// 1チャンネルのCvMatを配列で埋めます。
        /// </summary>
        /// <param name="mat"></param>
        /// <param name="arr"></param>
        public static void FillCvMat(CvMat mat, IList<int> arr)
        {
            int count = 0;
            for (int i = 0; i < mat.Rows; i++)
            {
                for (int j = 0; j < mat.Cols; j++)
                {
                    mat[i, j] = arr[count++];
                }
            }
        }

        /*
        /// <summary>
        /// Cv.ExtractSURFを呼び出すためのヘルパーメソッド
        /// </summary>
        /// <param name="imageMat">1チャンネル8bit画像</param>
        /// <param name="param">SURFに渡すパラメータ</param>
        /// <param name="surfPoints">出力される特徴点の配列</param>
        /// <param name="descriptorList">出力される特徴量の配列</param>
        public static void SURF(CvMat imageMat, CvSURFParams param, out CvSURFPoint[] surfPoints, out List<float[]> descriptorList)
        {
            using (CvMemStorage mems = new CvMemStorage())
            {
                descriptorList = new List<float[]>();
                CvSeq<CvSURFPoint> keyPoints;
                CvSeq<float> descSeq;
                Cv.ExtractSURF(imageMat, null, out keyPoints, out descSeq, mems, param);
                using (keyPoints)
                using (descSeq)
                {
                    surfPoints = keyPoints.ToArray();
                    // descriptor読むために回りくどい処理をしている。普通に読むと落ちる
                    using (CvSeqReader descReader = new CvSeqReader())
                    {
                        Cv.StartReadSeq(descSeq, descReader);
                        int len = (int)(descSeq.ElemSize / sizeof(float));
                        for (int i = 0; i < surfPoints.Length; i++)
                        {
                            float[] arr = new float[len];
                            Marshal.Copy(descReader.Ptr, arr, 0, len);
                            descriptorList.Add(arr);
                            Cv.NEXT_SEQ_ELEM(descSeq.ElemSize, descReader);
                        }
                    }
                }
            }
        }
         */
        /// <summary>
        /// 1チャンネル画像のサブピクセル値を返します．座標の小数点以下が0の場合にその座標の値と等しくなります
        /// </summary>
        /// <param name="mat">1チャンネル画像</param>
        /// <param name="point">座標</param>
        /// <param name="invalidValue">無効とみなされる値</param>
        /// <returns></returns>
        public static double? Get2DSubPixel(CvMat mat, CvPoint2D32f point, double invalidValue)
        {
            return Get2DSubPixel(mat, new CvPoint2D64f(point.X, point.Y), invalidValue);
        }
        /// <summary>
        /// 1チャンネル画像のサブピクセル値を返します．座標の小数点以下が0の場合にその座標の値と等しくなります
        /// </summary>
        /// <param name="mat">1チャンネル画像</param>
        /// <param name="point">座標</param>
        /// <param name="invalidValue">無効とみなされる値</param>
        /// <returns></returns>
        public static double? Get2DSubPixel(CvMat mat, CvPoint2D64f point, double invalidValue)
        {
            int preX = (int)Math.Floor(point.X);
            int preY = (int)Math.Floor(point.Y);
            int postX = preX == point.X ? preX : preX + 1;
            int postY = preY == point.Y ? preY : preY + 1;
            double fracX = point.X - preX;
            double fracY = point.Y - preY;
            if (postX >= mat.Cols || postY >= mat.Rows)
            {
                return null;
            }
            double depth00 = mat[preY, preX];
            double depth01 = mat[preY, postX];
            double depth10 = mat[postY, preX];
            double depth11 = mat[postY, postX];
            if (depth00 == invalidValue || depth01 == invalidValue || depth10 == invalidValue || depth11 == invalidValue)
            {
                return null;
            }
            return CalcEx.Lerp(CalcEx.Lerp(depth00, depth01, fracX), CalcEx.Lerp(depth10, depth11, fracX), fracY);
        }
        /// <summary>
        /// チェスボードコーナーの過度の四点を結ぶ四角形を描画します
        /// </summary>
        /// <param name="mat"></param>
        /// <param name="pattern_size"></param>
        /// <param name="corner"></param>
        /// <param name="color"></param>
        public static void DrawChessboardCornerFrame(CvMat mat, CvSize pattern_size, CvPoint2D32f[] corner, CvScalar color)
        {
            CvPoint2D32f[] points = new CvPoint2D32f[] { corner[0], corner[pattern_size.Width - 1], corner[pattern_size.Width * pattern_size.Height - 1], corner[(pattern_size.Height - 1) * pattern_size.Width] };
            mat.DrawPolyLine(new CvPoint[][] { points.Select(p => new CvPoint((int)Math.Round(p.X), (int)Math.Round(p.Y))).ToArray() }, true, color, 1, LineType.AntiAlias);
        }
        /// <summary>
        /// 3次元点(縦ベクトル)を4x4行列で(左掛けで)変換します。
        /// </summary>
        /// <param name="from">変換前の三次元点の座標</param>
        /// <param name="conversion">64bit浮動小数点1チャンネル4x4行列</param>
        /// <returns></returns>
        public static CvPoint3D64f ConvertPoint3D(CvPoint3D64f from, CvMat conversion)
        {
            double x = from.X * conversion[0, 0] + from.Y * conversion[0, 1] + from.Z * conversion[0, 2] + conversion[0, 3];
            double y = from.X * conversion[1, 0] + from.Y * conversion[1, 1] + from.Z * conversion[1, 2] + conversion[1, 3];
            double z = from.X * conversion[2, 0] + from.Y * conversion[2, 1] + from.Z * conversion[2, 2] + conversion[2, 3];
            return new CvPoint3D64f(x, y, z);
            using (CvMat fromMat = new CvMat(4, 1, MatrixType.F64C1))
            {
                fromMat[0, 0] = from.X;
                fromMat[1, 0] = from.Y;
                fromMat[2, 0] = from.Z;
                fromMat[3, 0] = 1;
                using (CvMat worldPoint = conversion * fromMat)
                {
                    return new CvPoint3D64f(worldPoint[0, 0], worldPoint[1, 0], worldPoint[2, 0]);
                }
            }
        }
        /// <summary>
        /// 3次元点(縦ベクトル)を3x3行列で(左掛けで)回転変換します。
        /// </summary>
        /// <param name="from"></param>
        /// <param name="conversion"></param>
        /// <returns></returns>
        public static CvPoint3D64f RotatePoint3D(CvPoint3D64f from, CvMat conversion)
        {
            using (CvMat fromMat = new CvMat(3, 1, MatrixType.F64C1))
            {
                fromMat[0, 0] = from.X;
                fromMat[1, 0] = from.Y;
                fromMat[2, 0] = from.Z;
                using (CvMat worldPoint = conversion * fromMat)
                {
                    return new CvPoint3D64f(worldPoint[0, 0], worldPoint[1, 0], worldPoint[2, 0]);
                }
            }
        }

        /// <summary>
        /// ベクトルを正規化します
        /// </summary>
        /// <param name="vector"></param>
        /// <returns></returns>
        public static CvPoint3D64f Normalize(CvPoint3D64f vector)
        {
            double sqrt = Math.Sqrt(GetLengthSq(vector));
            if (sqrt == 0)
                return new CvPoint3D64f(0, 0, 0);
            return vector * (1.0 / sqrt);
        }
        /// <summary>
        /// 2ベクトルの内積を求めます
        /// </summary>
        /// <param name="vector1"></param>
        /// <param name="vector2"></param>
        /// <returns></returns>
        public static double Dot(CvPoint3D64f vector1, CvPoint3D64f vector2)
        {
            return vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z;
        }
        /// <summary>
        /// 2ベクトルの外積を求めます
        /// </summary>
        /// <param name="v1"></param>
        /// <param name="v2"></param>
        /// <returns></returns>
        public static CvPoint3D64f Cross(CvPoint3D64f v1, CvPoint3D64f v2)
        {
            return new CvPoint3D64f(v1.Y * v2.Z - v1.Z * v2.Y, v1.Z * v2.X - v1.X * v2.Z, v1.X * v2.Y - v1.Y * v2.X);
        }
        /// <summary>
        /// クォータニオンの値から3x3回転行列を求めます．
        /// </summary>
        /// <param name="w"></param>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="z"></param>
        /// <returns></returns>
        public static CvMat QuaternionToMat3D(double w, double x, double y, double z)
        {
            CvMat ret = new CvMat(3, 3, MatrixType.F64C1);
            double x2 = x * x;
            double y2 = y * y;
            double z2 = z * z;
            ret[0, 0] = 1.0 - 2.0 * y2 - 2.0 * z2;
            ret[0, 1] = 2.0 * x * y - 2.0 * w * z;
            ret[0, 2] = 2.0 * x * z + 2.0 * w * y;

            ret[1, 0] = 2.0 * x * y + 2.0 * w * z;
            ret[1, 1] = 1.0 - 2.0 * x2 - 2.0 * z2;
            ret[1, 2] = 2.0 * y * z - 2.0 * w * x;

            ret[2, 0] = 2.0 * x * z - 2.0 * w * y;
            ret[2, 1] = 2.0 * y * z + 2.0 * w * x;
            ret[2, 2] = 1.0 - 2.0 * x2 - 2.0 * y2;

            return ret;
        }
        /// <summary>
        /// クォータニオンの値から3x3回転行列を求めます．
        /// </summary>
        /// <param name="w"></param>
        /// <param name="V"></param>
        /// <returns></returns>
        public static CvMat QuaternionToMat3D(double w, CvPoint3D64f V)
        {
            return QuaternionToMat3D(w, V.X, V.Y, V.Z);
        }

        /// <summary>
        /// 指定された軸に対して指定された角度だけ回転させる3x3行列を返します。縦ベクトルで右手系ならば軸の向きを奥の向きとした場合に時計回りに回転させます。
        /// </summary>
        /// <param name="axis"></param>
        /// <param name="theta"></param>
        /// <returns></returns>
        public static CvMat GetRotation3D(CvPoint3D64f axis, double theta)
        {
            // sin theta / 2 ってどっちだ
            axis = Normalize(axis);
            double cos = Math.Cos(theta / 2.0);
            double sin = Math.Sin(theta / 2.0);
            return QuaternionToMat3D(cos, axis * (sin));
        }

        /// <summary>
        /// マトリックスを拡大します。新しく追加された要素は、対角成分ならば1、さもなくば0になります
        /// </summary>
        /// <param name="src">元のマトリックス</param>
        /// <param name="rows">新しいマトリックスの行数</param>
        /// <param name="cols">新しいマトリックスの列数</param>
        /// <returns></returns>
        public static CvMat GetExtendedTransformMat(CvMat src, int rows, int cols)
        {
            CvMat ret = CvMat.Identity(rows, cols, MatrixType.F64C1);
            for (int i = 0; i < src.Cols && i < cols; i++)
            {
                for (int j = 0; j < src.Rows && j < rows; j++)
                {
                    ret[i, j] = src[i, j];
                }
            }
            return ret;
        }

        /*
        public static CvMat GetCorrectedMatF32FromMatrix(Microsoft.Xna.Framework.Matrix m)
        {
            CvMat mat = new CvMat(4, 4, MatrixType.F32C1);
            CvEx.FillCvMat(mat, new float[] { (float)m.M11, (float)m.M21, (float)m.M31, (float)m.M41, (float)m.M12, (float)m.M22, (float)m.M32, (float)m.M42, (float)m.M13, (float)m.M23, (float)m.M33, (float)m.M43, (float)m.M14, (float)m.M24, (float)m.M34, (float)m.M44 });
            return mat;
        }

        public static CvMat GetCorrectedMatF64FromMatrix(Microsoft.Xna.Framework.Matrix m)
        {
            CvMat mat = new CvMat(4, 4, MatrixType.F64C1);
            CvEx.FillCvMat(mat, new double[] { (float)m.M11, (float)m.M21, (float)m.M31, (float)m.M41, (float)m.M12, (float)m.M22, (float)m.M32, (float)m.M42, (float)m.M13, (float)m.M23, (float)m.M33, (float)m.M43, (float)m.M14, (float)m.M24, (float)m.M34, (float)m.M44 });
            return mat;
        }
         * */

    }
}
