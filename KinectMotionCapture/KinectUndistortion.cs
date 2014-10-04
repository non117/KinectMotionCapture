using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using OpenCvSharp;
using System.Collections.Concurrent;
using System.Threading.Tasks;
using System.Xml.Serialization;
using Microsoft.Kinect;

namespace KinectMotionCapture
{
    /// <summary>
    /// カメラの主点と焦点距離のデータ
    /// </summary>
    public struct CameraMatrix : IEquatable<CameraMatrix>
    {
        public double FocalX, FocalY;
        public double PrincipalX, PrincipalY;
        public static CameraMatrix CreateFrom(CvMat mat)
        {
            CameraMatrix ret = new CameraMatrix();
            ret.FocalX = mat[0, 0];
            ret.FocalY = mat[1, 1];
            ret.PrincipalX = mat[0, 2];
            ret.PrincipalY = mat[1, 2];
            return ret;
        }

        public bool Equals(CameraMatrix other)
        {
            return this.FocalX == other.FocalX && this.FocalY == other.FocalY && this.PrincipalX == other.PrincipalX && this.PrincipalY == other.PrincipalY;
        }
        public void CopyTo(ref CvMat mat)
        {
            CvEx.InitCvMat(ref mat, 3, 3, MatrixType.F64C1);
            mat.Zero();
            mat[0, 0] = this.FocalX;
            mat[1, 1] = this.FocalY;
            mat[0, 2] = this.PrincipalX;
            mat[1, 2] = this.PrincipalY;
            mat[2, 2] = 1;
        }

        public CvMat CreateCvMat()
        {
            CvMat ret = null;
            this.CopyTo(ref ret);
            return ret;
        }

        /// <summary>
        /// 指定された点から主点までの長さ を 焦点距離/2 で割った値の二乗を返します。
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <returns></returns>
        public double GetRatioSqOfPrincipalToFocal(double x, double y)
        {
            double dx = (x + 0.5 - this.PrincipalX) / (this.FocalX / 2);
            double dy = (y + 0.5 - this.PrincipalY) / (this.FocalY / 2);
            return dx * dx + dy * dy;
        }
        public bool IsInFocalLength(double x, double y)
        {
            return GetRatioSqOfPrincipalToFocal(x, y) <= 1;
        }
        public bool IsInFocalLength(CvPoint2D64f point)
        {
            return IsInFocalLength(point.X, point.Y);
        }
    }
    /// <summary>
    /// カメラの半径方向のゆがみと円周方向のゆがみのデータ
    /// </summary>
    public struct DistortionMatrix : IEquatable<DistortionMatrix>
    {
        public double K1, K2, P1, P2, K3, K4, K5, K6;
        public static DistortionMatrix CreateFrom(CvMat mat)
        {
            DistortionMatrix ret = new DistortionMatrix();
            int size = mat.Cols * mat.Rows;
            if (size >= 8)
            {
                ret.K6 = mat[7].Val0;
                ret.K5 = mat[6].Val0;
                ret.K4 = mat[5].Val0;
            }
            else
            {
                ret.K6 = 0;
                ret.K5 = 0;
                ret.K4 = 0;
            }
            if (size >= 5)
            {
                ret.K3 = mat[4].Val0;
            }
            else
            {
                ret.K3 = 0;
            }
            ret.K1 = mat[0].Val0;
            ret.K2 = mat[1].Val0;
            ret.P1 = mat[2].Val0;
            ret.P2 = mat[3].Val0;
            return ret;
        }

        public bool Equals(DistortionMatrix other)
        {
            return this.K1 == other.K1 && this.K2 == other.K2 && this.P1 == other.P1 && this.P2 == other.P2 && this.K3 == other.K3 && this.K4 == other.K4 && this.K5 == other.K5 && this.K6 == other.K6;
        }

        public void CopyTo(ref CvMat mat, bool columnVector)
        {
            if (columnVector)
            {
                CvEx.InitCvMat(ref mat, 8, 1, MatrixType.F64C1);
            }
            else
            {
                CvEx.InitCvMat(ref mat, 1, 8, MatrixType.F64C1);
            }
            CvEx.FillCvMat(mat, new double[] { this.K1, this.K2, this.P1, this.P2, this.K3, this.K4, this.K5, this.K6 });
        }
        public CvMat CreateCvMat(bool columnVector)
        {
            CvMat ret = null;
            this.CopyTo(ref ret, columnVector);
            return ret;
        }

    }

    struct cornerEdgePair
    {
        public bool Horizontal;
        public CvPoint3D64f Point1, Point2;
        public cornerEdgePair(CvPoint3D64f point1, CvPoint3D64f point2, bool horizontal)
        {
            this.Point1 = point1;
            this.Point2 = point2;
            this.Horizontal = horizontal;
        }
    }


    /// <summary>
    /// 奥行きのゆがみ補正器
    /// </summary>
    public class DepthUndistortionLinearCalibrator
    {
        CameraMatrix _camera;
        int _dimension;
        public DepthUndistortionLinearCalibrator(CameraMatrix camera, int dimension)
        {
            _camera = camera;
            if (dimension < 1 || dimension > 3)
                throw new ArgumentOutOfRangeException("dimension must be 1, 2 or 3");
            _dimension = dimension;
        }
        //ConcurrentDictionary<Tuple<int, int>, Tuple<int, double>> _depthErrorSum = new ConcurrentDictionary<Tuple<int, int>, Tuple<int, double>>();

        MemorySavingLeastSquare[] _depthErrorSum;
        CvSize _retSize = CvSize.Empty;

        /// <summary>
        /// 平面奥行きデータを渡して奥行き歪み補正のサンプルデータとします
        /// </summary>
        /// <param name="depthMat"></param>
        public void PutDepthImage(ref CvMat resultMat, CvMat depthMat, KinectUndistortion undist)
        {
            // aZ+b=E

            //       
            // Z^2 Z * a = EZ
            // Z   1   b   E

            CvEx.InitCvMat(ref resultMat, depthMat, MatrixType.F32C1);
            resultMat.Set(1.0);
            // ax+by+c=z
            CvSize size = new CvSize(depthMat.Cols, depthMat.Rows);
            if (_retSize == CvSize.Empty)
            {
                _retSize = size;
                _depthErrorSum = Enumerable.Range(0, size.Width * size.Height).Select(x => new MemorySavingLeastSquare(_dimension + 1)).ToArray();
                using (System.IO.StreamWriter sw = new System.IO.StreamWriter("slope3.csv", false))
                {
                }
            }
            else if (_retSize != size)
                throw new ArgumentException("Inconsistent image sizes");

            List<int> poss = Enumerable.Range(0, 12).Select(x => x * 50).ToList();
            using (System.IO.StreamWriter sw = new System.IO.StreamWriter("slope3.csv", true))
            {
                unsafe
                {
                    CvPoint2D64f? slopeCoef = CalcEx.GetSlopeOfDepthMat(depthMat, _camera, undist);

                    short* depthArr = depthMat.DataInt16;
                    double pX = _camera.PrincipalX;
                    double pY = _camera.PrincipalY;
                    double? pDepth = CvEx.Get2DSubPixel(depthMat, new CvPoint2D32f(pX, pY), 0);

                    if (slopeCoef.HasValue && pDepth.HasValue)
                    {
                        double xCoef = slopeCoef.Value.X;
                        double yCoef = slopeCoef.Value.Y;
                        unsafe
                        {
                            float* resultArr = resultMat.DataSingle;
                            Parallel.For(0, size.Height, y =>
                            {
                                int iOffset = y * size.Width;
                                for (int x = 0; x < size.Width; x++)
                                {
                                    double depth = depthArr[iOffset + x];
                                    if (depth != 0)
                                    {
                                        double exp = (x - pX) * xCoef + (y - pY) * yCoef + pDepth.Value;
                                        switch (_dimension)
                                        {
                                            case 1:
                                                _depthErrorSum[iOffset + x].PutPoint(new double[] { depth / exp, 1.0 / exp }, exp / exp);
                                                break;
                                            case 2:
                                                _depthErrorSum[iOffset + x].PutPoint(new double[] { depth * depth / exp, depth / exp, 1.0 / exp }, exp / exp);
                                                break;
                                            case 3:
                                                _depthErrorSum[iOffset + x].PutPoint(new double[] { depth * depth * depth / exp, depth * depth / exp, depth / exp, 1.0 / exp }, exp / exp);
                                                break;
                                        }
                                        resultArr[iOffset + x] = (float)(depth / exp);
                                    }
                                }
                            });
                            foreach (int x in poss)
                            {
                                int y = 60;
                                int iOffset = y * size.Width;
                                double depth = depthArr[iOffset + x];
                                double exp = (x - pX) * xCoef + (y - pY) * yCoef + pDepth.Value;
                                if (depth != 0)
                                {
                                    sw.Write("{0},{1},", exp, depth);
                                }
                                else
                                {
                                    sw.Write("{0},{1},", "", "");
                                }
                            }
                            sw.WriteLine();
                        }
                    }
                }
            }
        }

        /// <summary>
        /// 平面奥行きデータを渡して奥行き歪み補正のサンプルデータとします
        /// </summary>
        /// <param name="depthMat"></param>
        public static double EvaluateUndistortion(ref CvMat resultMat, CvMat depthMat, KinectUndistortion undist)
        {
            // aZ+b=E

            //       
            // Z^2 Z * a = EZ
            // Z   1   b   E

            CvEx.InitCvMat(ref resultMat, depthMat, MatrixType.F32C1);
            resultMat.Set(1.0);
            // ax+by+c=z
            CvSize size = new CvSize(depthMat.Cols, depthMat.Rows);

            unsafe
            {
                CvPoint2D64f? slopeCoef = CalcEx.GetSlopeOfDepthMat(depthMat, undist.CameraStruct, undist);

                short* depthArr = depthMat.DataInt16;
                double pX = undist.CameraStruct.PrincipalX;
                double pY = undist.CameraStruct.PrincipalY;
                double? pDepth = CvEx.Get2DSubPixel(depthMat, new CvPoint2D32f(pX, pY), 0);
                ConcurrentBag<double> errors = new ConcurrentBag<double>();

                if (slopeCoef.HasValue && pDepth.HasValue)
                {
                    pDepth = undist.UndistortDepth(new CvPoint3D64f(pX, pY, pDepth.Value), size);
                    double xCoef = slopeCoef.Value.X;
                    double yCoef = slopeCoef.Value.Y;
                    unsafe
                    {
                        float* resultArr = resultMat.DataSingle;
                        Parallel.For(0, size.Height, y =>
                        {
                            int iOffset = y * size.Width;
                            for (int x = 0; x < size.Width; x++)
                            {
                                double depth = depthArr[iOffset + x];
                                if (depth != 0)
                                {
                                    depth = undist.UndistortDepth(new CvPoint3D64f(x, y, depth), size);
                                    double exp = (x - pX) * xCoef + (y - pY) * yCoef + pDepth.Value;

                                    resultArr[iOffset + x] = (float)(depth / exp);
                                    if (undist.CameraStruct.IsInFocalLength(x, y))
                                    {
                                        errors.Add((depth - exp) / exp);
                                    }
                                }
                            }
                        });
                    }
                    return CalcEx.GetStdDev(errors);
                }
                return double.NaN;
            }
        }

        /// <summary>
        /// 各画素の奥行き補正の係数マトリックスを返します
        /// </summary>
        /// <returns></returns>
        public CvMat GetUndistortCoefMat()
        {

            CvMat mat;
            switch (_dimension)
            {
                case 1:
                    mat = CvEx.InitCvMat(_retSize.Height, _retSize.Width, MatrixType.F32C2);
                    mat.Set(new CvScalar(1.0, 0));
                    break;
                case 2:
                    mat = CvEx.InitCvMat(_retSize.Height, _retSize.Width, MatrixType.F32C3);
                    mat.Set(new CvScalar(0, 1.0, 0));
                    break;
                case 3:
                    mat = CvEx.InitCvMat(_retSize.Height, _retSize.Width, MatrixType.F32C4);
                    mat.Set(new CvScalar(0, 0, 1.0, 0));
                    break;
                default:
                    throw new InvalidOperationException();
            }
            for (int y = 0; y < _retSize.Height; y++)
            {
                int iOffset = y * _retSize.Width;
                for (int x = 0; x < _retSize.Width; x++)
                {
                    ;
                    if (_depthErrorSum[iOffset + x].Count > 16)
                    {
                        double[] ans = _depthErrorSum[iOffset + x].Solve();
                        switch (_dimension)
                        {
                            case 1:
                                mat[iOffset + x] = new CvScalar(ans[0], ans[1]);
                                break;
                            case 2:
                                mat[iOffset + x] = new CvScalar(ans[0], ans[1], ans[2]);
                                break;
                            case 3:
                                mat[iOffset + x] = new CvScalar(ans[0], ans[1], ans[2], ans[3]);
                                break;
                        }
                    }
                }
            }
            return mat;
        }
    }

    /// <summary>
    /// 奥行きのゆがみ補正器
    /// </summary>
    public class DepthUndistortionScaleCalibrator
    {
        CameraMatrix _camera;
        public DepthUndistortionScaleCalibrator(CameraMatrix matrix)
        {
            _camera = matrix;
        }
        //ConcurrentDictionary<Tuple<int, int>, Tuple<int, double>> _depthErrorSum = new ConcurrentDictionary<Tuple<int, int>, Tuple<int, double>>();
        int[] _depthErrorCount;
        double[] _depthErrorSum;
        CvSize _retSize = CvSize.Empty;
        /// <summary>
        /// 平面奥行きデータを渡して奥行き歪み補正のサンプルデータとします
        /// </summary>
        /// <param name="depthMat"></param>
        public void PutDepthImage(ref CvMat resultMat, CvMat depthMat)
        {
            CvEx.InitCvMat(ref resultMat, depthMat, MatrixType.F32C1);
            resultMat.Set(1.0);
            // ax+by+c=z
            CvSize size = new CvSize(depthMat.Cols, depthMat.Rows);
            if (_retSize == CvSize.Empty)
            {
                _retSize = size;
                _depthErrorCount = new int[size.Width * size.Height];
                _depthErrorSum = new double[size.Width * size.Height];
            }
            else if (_retSize != size)
                throw new ArgumentException("Inconsistent image sizes");

            unsafe
            {
                CvPoint2D64f? slopeCoef = CalcEx.GetSlopeOfDepthMat(depthMat, _camera);

                short* depthArr = depthMat.DataInt16;
                double pX = _camera.PrincipalX;
                double pY = _camera.PrincipalY;
                double? pDepth = CvEx.Get2DSubPixel(depthMat, new CvPoint2D32f(pX, pY), 0);

                if (slopeCoef.HasValue && pDepth.HasValue)
                {
                    double xCoef = slopeCoef.Value.X;
                    double yCoef = slopeCoef.Value.Y;
                    unsafe
                    {
                        float* resultArr = resultMat.DataSingle;
                        Parallel.For(0, size.Height, y =>
                        {
                            int iOffset = y * size.Width;
                            for (int x = 0; x < size.Width; x++)
                            {
                                double depth = depthArr[iOffset + x];
                                if (depth != 0)
                                {
                                    double exp = (x - pX) * xCoef + (y - pY) * yCoef + pDepth.Value;
                                    double ratio = exp / depth;
                                    _depthErrorCount[iOffset + x]++;
                                    _depthErrorSum[iOffset + x] += ratio;
                                    resultArr[iOffset + x] = (float)ratio;
                                }
                            }
                        });
                    }
                }
            }
        }
        /// <summary>
        /// 各画素の奥行き補正の係数マトリックスを返します
        /// </summary>
        /// <returns></returns>
        public CvMat GetUndistortCoefMat()
        {
            CvMat mat = CvEx.InitCvMat(_retSize.Height, _retSize.Width, MatrixType.F32C1);
            mat.Set(1.0);
            for (int y = 0; y < _retSize.Height; y++)
            {
                int iOffset = y * _retSize.Width;
                for (int x = 0; x < _retSize.Width; x++)
                {
                    int count = _depthErrorCount[iOffset + x];
                    double sum = _depthErrorSum[iOffset + x];
                    if (count > 0)
                    {
                        mat[iOffset + x] = sum / count;
                    }
                }
            }
            return mat;
        }
    }

    /// <summary>
    /// Kinectのゆがみ補正のデータ
    /// </summary>
    public class KinectUndistortion
    {
        /// <summary>
        /// 深度画像の水平視野角。OpenNIより取得
        /// </summary>
        public const double DepthFovHorizontalAngle = 1.23220245190799668130;//1.0144686707507438;
        /// <summary>
        /// 深度画像の片側視野角。OpenNIより取得
        /// </summary>
        public const double DepthFovVerticalAngle = 1.047197551196597746154;//0.78980943449644714;
        //public const double DepthScaleXPerZ = 1.1114666461944582;
        /// <summary>
        /// 片側水平視野角のtangentの二倍の値。右手系にするために負の値にしている
        /// </summary>
        public const double DepthScaleXPerZ = -1.4160789342560455782435;//-1.1114666461944582;
        /// <summary>
        /// 片側垂直視野角のtangentの二倍の値
        /// </summary>
        public const double DepthScaleYPerZ = -1.1547005383792515290180;//-0.83359998464584351;
        /// <summary>
        /// 奥行きゆがみ補正に用いたレコードのパス
        /// </summary>
        public string UndistortionDepthSourcePath;
        /// <summary>
        /// カメラキャリブレーションに用いたレコードのパス
        /// </summary>
        public string CalibrationSourcePath;
        CvPoint[] _imageCorrectionMap = null;
        CvPoint[] _depthCorrectionMap = null;
        /// <summary>
        /// キャリブレーション時の画像サイズ
        /// </summary>
        CvSize _imageMatSize;



        void clearCorrectionMap()
        {
            _imageCorrectionMap = null;
            _depthCorrectionMap = null;
        }

        double _imageXScale = 1;
        public double ImageXScale
        {
            get { return _imageXScale; }
            set
            {
                clearCorrectionMap();
                clearUndistorionImageXYCache();
                _imageXScale = value;
            }
        }
        double _imageYScale = 1;

        public double ImageYScale
        {
            get { return _imageYScale; }
            set
            {
                clearCorrectionMap();
                clearUndistorionImageXYCache();
                _imageYScale = value;
            }
        }

        double _xScale = 1;

        public double XScale
        {
            get { return _xScale; }
            set
            {
                clearCorrectionMap();
                _xScale = value;
            }
        }
        double _yScale = 1;

        public double YScale
        {
            get { return _yScale; }
            set
            {
                clearCorrectionMap();
                _yScale = value;
            }
        }
        double _zOffset = 0;

        public double ZOffset
        {
            get { return _zOffset; }
            set
            {
                clearCorrectionMap();
                _zOffset = value;
            }
        }


        public CvSize ImageMatSize
        {
            get { return _imageMatSize; }
            set
            {
                clearCorrectionMap();
                clearUndistorionImageXYCache();
                _imageMatSize = value;
            }
        }
        CvSize _depthMatSize;

        public static double GetRealXFromPerspective(double x, double cols, double z)
        {
            return (x / cols - 0.5) * DepthScaleXPerZ * z;
        }
        public static double GetRealYFromPerspective(double y, double rows, double z)
        {
            return (y / rows - 0.5) * DepthScaleYPerZ * z;
        }
        public static double GetPerspectiveXFromReal(double x, double cols, double z)
        {
            return (x / z / DepthScaleXPerZ + 0.5) * cols;
        }
        public static double GetPerspectiveYFromReal(double y, double rows, double z)
        {
            return (y / z / DepthScaleYPerZ + 0.5) * rows;
        }

        CameraMatrix _cameraStruct;

        public CameraMatrix CameraStruct
        {
            get { return _cameraStruct; }
            set
            {
                clearCorrectionMap();
                clearUndistorionImageXYCache();
                _cameraStruct = value;
                _cameraStruct.CopyTo(ref _cameraMat);
            }
        }
        DistortionMatrix _distortStruct;

        public DistortionMatrix DistortStruct
        {
            get { return _distortStruct; }
            set
            {
                clearCorrectionMap();
                clearUndistorionImageXYCache();
                _distortStruct = value;
                _distortStruct.CopyTo(ref _distortMat, true);
            }
        }
        CvMat _cameraMat;
        CvMat _distortMat;

        CvMat _undistortionDepthMat;

        [XmlIgnore]
        public CvMat UndistortionDepthMat
        {
            get { return _undistortionDepthMat; }
            set
            {
                _undistortionDepthMat = value;
                if (value != null)
                {
                    _depthMatSize = new CvSize(value.Cols, value.Rows);
                }
                else
                {
                    _depthMatSize = CvSize.Empty;
                }
            }
        }

        public SerializableBinaryMat SerializableUndistortionDepthMat
        {
            get { return SerializableBinaryMat.CreateOrNull(this.UndistortionDepthMat); }
            set { this.UndistortionDepthMat = SerializableBinaryMat.CreateOrNull(value); }
        }


        public KinectUndistortion()
        {
            this.CameraStruct = new CameraMatrix { FocalX = 505.112548908415, PrincipalX = 316.440613825143, FocalY = 506.315144495061, PrincipalY = 255.766317036433 };
            this.DistortStruct = new DistortionMatrix();
        }

        public KinectUndistortion(KinectUndistortion obj)
        {
            this.CameraStruct = obj.CameraStruct;
            this.DistortStruct = obj.DistortStruct;
            this.ImageXScale = obj.ImageXScale;
            this.ImageYScale = obj.ImageYScale;
            this.XScale = obj.XScale;
            this.YScale = obj.YScale;
            this.ZOffset = obj.ZOffset;
            this.ImageMatSize = obj.ImageMatSize;
            this.CalibrationSourcePath = obj.CalibrationSourcePath;
            this.UndistortionDepthSourcePath = obj.UndistortionDepthSourcePath;

            if (obj.UndistortionDepthMat == null)
            {
                this.UndistortionDepthMat = null;
            }
            else
            {
                this.UndistortionDepthMat = CvEx.CloneCvMat(obj.UndistortionDepthMat);
            }
        }



        public void SetUndistortionDepthMat(CvMat mat, string sourcePath)
        {
            this.UndistortionDepthMat = mat;
            this.UndistortionDepthSourcePath = sourcePath;
        }


        public static OpenCvSharp.CvPoint3D64f GetOriginalRealFromScreenPos(CvPoint3D64f pos, OpenCvSharp.CvSize imageSize)
        {
            return GetOriginalRealFromScreenPos(pos.X, pos.Y, pos.Z, imageSize);
        }

        public static OpenCvSharp.CvPoint3D64f GetOriginalRealFromScreenPos(double x, double y, double z, OpenCvSharp.CvSize imageSize)
        {
            double x2 = KinectUndistortion.GetRealXFromPerspective(x, imageSize.Width, z);
            double y2 = KinectUndistortion.GetRealYFromPerspective(y, imageSize.Height, z);
            double z2 = z;
            return new OpenCvSharp.CvPoint3D64f(x2, y2, z);
        }

        public static OpenCvSharp.CvPoint3D64f GetOriginalScreenPosFromReal(double x, double y, double z, OpenCvSharp.CvSize imageSize)
        {
            double x2 = KinectUndistortion.GetPerspectiveXFromReal(x, imageSize.Width, z);
            double y2 = KinectUndistortion.GetPerspectiveYFromReal(y, imageSize.Height, z);
            double z2 = z;
            return new OpenCvSharp.CvPoint3D64f(x2, y2, z);
        }

        public static OpenCvSharp.CvPoint3D64f GetOriginalScreenPosFromReal(CameraSpacePoint point, OpenCvSharp.CvSize imageSize)
        {
            return GetOriginalScreenPosFromReal(point.X, point.Y, point.Z, imageSize);
        }

        public static OpenCvSharp.CvPoint3D64f GetOriginalScreenPosFromReal(CvPoint3D64f point, OpenCvSharp.CvSize imageSize)
        {
            return GetOriginalScreenPosFromReal(point.X, point.Y, point.Z, imageSize);
        }

        public double UndistortDepth(CvPoint3D64f point, CvSize imageSize)
        {
            return this.UndistortDepth(point.X, point.Y, point.Z, imageSize);
        }

        public double UndistortDepth(double x, double y, double z, CvSize imageSize)
        {
            CvMat mat = this.UndistortionDepthMat;
            //double shear = 1.0 + (x / imageSize.Width - 0.5) * 0.006 + (y / imageSize.Height - 0.5) * -0.0045;
            //double shear = 1.0 + ((x - CameraStruct.PrincipalX / 2) / CameraStruct.FocalX) * ((y - CameraStruct.PrincipalY / 2) / CameraStruct.FocalY) * -0.061;
            const double shear = 1.0;
            if (mat == null)
            {
                return z * shear;
            }
            else
            {
                int newX, newY;
                if (mat.Cols == imageSize.Width && mat.Rows == imageSize.Height)
                {
                    newX = (int)Math.Floor(x);
                    newY = (int)Math.Floor(y);
                }
                else
                {
                    double scale = (double)mat.Cols / imageSize.Width;
                    newX = (int)Math.Floor(x * scale);
                    newY = (int)Math.Floor(y * scale);
                }
                if (newX >= 0 && newX < mat.Cols && newY >= 0 && newY < mat.Rows)
                {
                    switch (mat.ElemChannels)
                    {
                        case 1:
                            return z * shear * mat[newY, newX];
                        case 2:
                            {
                                CvScalar coef = mat.Get2D(newY, newX);
                                return z * shear * coef.Val0 + coef.Val1;
                            }
                        case 3:
                            {
                                CvScalar coef = mat.Get2D(newY, newX);
                                double z2 = z * shear;
                                return z2 * z2 * coef.Val0 + z2 * coef.Val1 + coef.Val2;
                            }
                        default:
                            return z;
                    }
                }
                else
                {
                    return z * shear;
                }
            }
        }

        public void ClearUndistortionCache()
        {
            clearUndistorionImageXYCache();
        }

        ConcurrentDictionary<CvPoint2D64f, CvPoint2D64f> _undistortionImageXYCache = new ConcurrentDictionary<CvPoint2D64f, CvPoint2D64f>();

        void clearUndistorionImageXYCache()
        {
            _undistortionImageXYCache = new ConcurrentDictionary<CvPoint2D64f, CvPoint2D64f>();
        }

        /// <summary>
        /// 計算されたゆがみ補正の値を使用して、指定されたイメージ上の座標に対してゆがみ補正後の座標を返します
        /// </summary>
        /// <param name="point"></param>
        /// <param name="imageSize"></param>
        /// <returns></returns>
        public CvPoint2D64f UndistortImageXY(CvPoint2D64f point, CvSize imageSize)
        {
            if (_imageMatSize == CvSize.Empty)
            {
                return point;
            }
            // イメージのサイズがCalibrateCamera2に与えたイメージのサイズと異なる場合、
            // 幅が同じサイズになるように、点の座標をスケーリングする
            double preScale = (double)_imageMatSize.Width / imageSize.Width;
            CvPoint2D64f preScaledPoint = point * preScale;
            // キャッシュがあれば使う(スケーリング済みの点をキーにする)
            ConcurrentDictionary<CvPoint2D64f, CvPoint2D64f> cache = _undistortionImageXYCache;
            CvPoint2D64f temp;
            if (cache.TryGetValue(preScaledPoint, out temp))
            {
                return temp;
            }

            using (CvMat input = new CvMat(1, 1, MatrixType.F64C2))
            using (CvMat output = new CvMat(1, 1, MatrixType.F64C2))
            {
                // レンズゆがみ補正
                CvEx.FillCvMat(input, new CvScalar(preScaledPoint.X, preScaledPoint.Y));
                Cv.UndistortPoints(input, output, _cameraMat, _distortMat, null, _cameraMat);
                CvScalar outScalar = output.Get2D(0, 0);
                CvPoint2D64f undistort = new CvPoint2D64f(outScalar.Val0, outScalar.Val1);
                // ピクセルサイズが変わらないようなスケーリングをする
                double scaledX = (undistort.X - this.CameraStruct.PrincipalX) * this.ImageXScale + this.CameraStruct.PrincipalX;
                double scaledY = (undistort.Y - this.CameraStruct.PrincipalY) * this.ImageYScale + this.CameraStruct.PrincipalY;

                //scaledY += scaledX * -0.05;
                // イメージサイズによるスケーリングを元に戻す
                CvPoint2D64f ret = new CvPoint2D64f(scaledX, scaledY) * (1.0 / preScale);



                // キャッシュに入れる(スケーリング済みの点をキーにする)
                if (cache.Count < 131072)
                {
                    cache.TryAdd(preScaledPoint, ret);
                }
                return ret;
            }
        }

        public CvPoint3D64f GetRealFromScreenPos(CvPoint3D64f point, OpenCvSharp.CvSize imageSize)
        {
            return GetRealFromScreenPos(point.X, point.Y, point.Z, imageSize);
        }

        /// <summary>
        /// ゆがみ補正前のイメージ上の点の座標と、対応する深度値から、カメラ座標系に投影された点の座標を求めます
        /// </summary>
        /// <param name="x">ゆがみ補正前のイメージ上のX座標</param>
        /// <param name="y">ゆがみ補正前のイメージ上のY座標</param>
        /// <param name="z">イメージ上の点に対応する深度マップの点の値</param>
        /// <param name="imageSize">イメージのサイズ</param>
        /// <returns></returns>
        public CvPoint3D64f GetRealFromScreenPos(double x, double y, double z, OpenCvSharp.CvSize imageSize)
        {
            if (this.UndistortionDepthMat == null && _imageMatSize == CvSize.Empty)
            {
                return GetOriginalRealFromScreenPos(x, y, z, imageSize);
            }
            CvPoint2D64f newPoint = UndistortImageXY(new CvPoint2D64f(x, y), imageSize);


            double rz = UndistortDepth(x, y, z, imageSize) + ZOffset;
            double rx = KinectUndistortion.GetRealXFromPerspective(newPoint.X, imageSize.Width, rz) * XScale;
            double ry = KinectUndistortion.GetRealYFromPerspective(newPoint.Y, imageSize.Height, rz) * YScale;
            return new OpenCvSharp.CvPoint3D64f(rx, ry, rz);
        }


        public static void InitChessBoardMat(ref CvMat mat, int cols, int rows, double blockLength, int count)
        {
            int numPoint = cols * rows;
            if (mat == null || mat.Rows != numPoint * count || mat.Cols != 1)
            {
                mat = new CvMat(numPoint * count, 1, MatrixType.F64C3);
            }
            for (int y = 0; y < rows; y++)
            {
                for (int x = 0; x < cols; x++)
                {
                    for (int i = 0; i < count; i++)
                    {
                        mat.Set2D(i * numPoint + y * cols + x, 0, new CvScalar(x * blockLength, y * blockLength, 0));
                    }
                }
            }
        }


        /// <summary>
        /// RGBカメラのゆがみ補正、および補正前とピクセル間のサイズが変わらないようにするためのスケーリングを計算します、
        /// </summary>
        /// <param name="cornerPoints">チェスボードのコーナー点列の集合</param>
        /// <param name="cols">チェスボードのコーナー点の行数</param>
        /// <param name="rows">チェスボードのコーナー点の列数</param>
        /// <param name="blockLength">チェスボードのコーナー点のブロック間のサイズ(mm)。正確でなくてもよいらしい</param>
        /// <param name="imageSize">入力するRGBカメラのイメージサイズ</param>
        /// <param name="maximumSelectionCount">Cv.CalibrateCamera2に対して使用するフレームの最大数</param>
        /// <param name="sourcePath">イメージソースのパス</param>
        public void CalibrateCamera(IList<CvPoint2D32f[]> cornerPoints, int cols, int rows, double blockLength, CvSize imageSize, int maximumSelectionCount, string sourcePath)
        {
            CvMat objectPoints = null;
            int numPoints = cols * rows;

            int[] selection = Enumerable.Range(0, cornerPoints.Count).ToArray();
            Random rand = new Random();
            for (int i = 0; i < maximumSelectionCount && i < selection.Length; i++)
            {
                int index = rand.Next(i, selection.Length);
                int temp = selection[index];
                selection[index] = selection[i];
                selection[i] = temp;
            }
            cornerPoints = selection.Take(maximumSelectionCount).Select(x => cornerPoints[x]).ToList();

            InitChessBoardMat(ref objectPoints, cols, rows, blockLength, cornerPoints.Count);

            CvMat cornerMat2 = new CvMat(numPoints * cornerPoints.Count, 1, MatrixType.F32C2);
            CvMat countMat2 = new CvMat(1, cornerPoints.Count, MatrixType.S32C1);
            CvEx.FillCvMat(cornerMat2, cornerPoints.SelectMany(x => x).Select(x => new CvScalar(x.X, x.Y)).ToArray());
            CvEx.FillCvMat(countMat2, cornerPoints.Select(x => x.Length).ToArray());
            _cameraMat = new CvMat(3, 3, MatrixType.F64C1);
            _distortMat = new CvMat(8, 1, MatrixType.F64C1);

            Cv.CalibrateCamera2(objectPoints, cornerMat2, countMat2, imageSize, _cameraMat, _distortMat, null, null, CalibrationFlag.RationalModel);
            this.CameraStruct = CameraMatrix.CreateFrom(_cameraMat);
            this.DistortStruct = DistortionMatrix.CreateFrom(_distortMat);
            this.ImageMatSize = imageSize;
            correctXYScale(imageSize);

            this.CalibrationSourcePath = sourcePath;
        }

        /// <summary>
        /// 焦点距離内のピクセル間隔がゆがみ補正前と同様になるように、ゆがみ補正後のイメージに対するスケーリングを求めます
        /// </summary>
        /// <param name="imageSize">イメージサイズ</param>
        void correctXYScale(CvSize imageSize)
        {
            double count = 0;
            this.ImageXScale = 1.0;
            this.ImageYScale = 1.0;
            double xDiffSum = 0, yDiffSum = 0;
            for (int y = 0; y < imageSize.Height; y++)
            {
                for (int x = 0; x < imageSize.Width; x++)
                {
                    double focalRatioSq = this.CameraStruct.GetRatioSqOfPrincipalToFocal(x, y);
                    if (focalRatioSq >= 1)
                        continue;
                    CvPoint2D64f leftUp = UndistortImageXY(new CvPoint2D64f(x, y), imageSize);
                    CvPoint2D64f rightUp = UndistortImageXY(new CvPoint2D64f(x + 1, y), imageSize);
                    CvPoint2D64f leftDown = UndistortImageXY(new CvPoint2D64f(x, y + 1), imageSize);
                    double weight = 1.0 - focalRatioSq;
                    count += weight;
                    xDiffSum += (rightUp.X - leftUp.X) * weight;
                    yDiffSum += (leftDown.Y - leftUp.Y) * weight;
                }
            }
            this.ImageXScale = count / xDiffSum;
            this.ImageYScale = count / yDiffSum;
        }

        class chessBoardEdgeInfo
        {
            public CvPoint3D32f Screen1, Screen2;
            public CvPoint3D64f Real1, Real2;
            public CvPoint2D64f PerZ1, PerZ2;
            public double ExpectedLength;
            public chessBoardEdgeInfo(CvPoint3D32f undistScreen1, CvPoint3D32f undistScreen2, CvPoint3D64f undistReal1, CvPoint3D64f undistReal2, CvPoint2D64f perZ1, CvPoint2D64f perZ2, double expectedLength)
            {
                this.Screen1 = undistScreen1;
                this.Screen2 = undistScreen2;
                this.Real1 = undistReal1;
                this.Real2 = undistReal2;
                this.PerZ1 = perZ1;
                this.PerZ2 = perZ2;
                this.ExpectedLength = expectedLength;
            }
        }

        /// <summary>
        /// 深度カメラの奥行きの値の補正と、カメラ座標系に投影された空間における水平・垂直方向のスケーリングを求めます。
        /// </summary>
        /// <param name="cornerImagePointsAndDepths">チェスボードコーナーの点列のリスト。X, Yの値はイメージ上の点の補正前の座標、Zは深度マップの対応する点の補正前の値(mm)。深度データがない点はnull</param>
        /// <param name="cols">チェスボードのコーナーの行数</param>
        /// <param name="rows">チェスボードのコーナーの列数</param>
        /// <param name="horizLength">チェスボードの列間の長さ(mm)</param>
        /// <param name="vertLength">チェスボードの行間の長さ(mm)</param>
        /// <param name="imageSize">イメージのサイズ。cornerImagePointsAndDepths を求めるために使用したイメージと同じサイズになるようにします</param>
        public void CalibrateRealScaleAndOffset(IList<CvPoint3D32f?[]> cornerImagePointsAndDepths, int cols, int rows, double horizLength, double vertLength, CvSize imageSize)
        {
            this.ZOffset = 0;

            List<Tuple<double, double>> depthList = new List<Tuple<double, double>>();
            List<chessBoardEdgeInfo> infoList = new List<chessBoardEdgeInfo>();
            foreach (CvPoint3D32f?[] corner in cornerImagePointsAndDepths)
            {
                for (int x = 0; x < cols; x++)
                {
                    for (int y = 0; y < rows; y++)
                    {
                        if (!corner[x + y * cols].HasValue)
                            continue;
                        CvPoint3D32f point1 = corner[x + y * cols].Value;
                        CvPoint2D64f undistortPoint1 = UndistortImageXY(new CvPoint2D64f(point1.X, point1.Y), imageSize);
                        double z1 = UndistortDepth(point1, imageSize);
                        CvPoint3D64f undistScreen1 = new CvPoint3D64f(undistortPoint1.X, undistortPoint1.Y, z1);

                        double realX1 = GetRealXFromPerspective(undistortPoint1.X, imageSize.Width, z1);
                        double realY1 = GetRealYFromPerspective(undistortPoint1.Y, imageSize.Height, z1);
                        CvPoint3D64f real1 = GetOriginalRealFromScreenPos(undistScreen1, imageSize);
                        CvPoint3D64f pre = KinectUndistortion.GetOriginalRealFromScreenPos(point1, imageSize);
                        foreach (var offset in new[] { new { X = 1, Y = 0, D = horizLength }, new { X = 0, Y = 1, D = vertLength } })
                        {
                            int dx = x + offset.X;
                            int dy = y + offset.Y;
                            if (dx >= cols || dy >= rows)
                                continue;
                            if (!corner[dx + dy * cols].HasValue)
                                continue;

                            CvPoint3D32f point2 = corner[dx + dy * cols].Value;
                            CvPoint2D64f undistortPoint2 = UndistortImageXY(new CvPoint2D64f(point2.X, point2.Y), imageSize);
                            double z2 = UndistortDepth(point2, imageSize);
                            CvPoint3D64f undistScreen2 = new CvPoint3D64f(undistortPoint2.X, undistortPoint2.Y, z2);
                            CvPoint3D64f real2 = GetOriginalRealFromScreenPos(undistScreen2, imageSize);
                            double realX2 = GetRealXFromPerspective(undistortPoint2.X, imageSize.Width, z2);
                            double realY2 = GetRealYFromPerspective(undistortPoint2.Y, imageSize.Height, z2);

                            double distanceSq = Math.Pow(realX1 - realX2, 2) + Math.Pow(realY1 - realY2, 2);
                            double distance = Math.Sqrt(distanceSq);

                            double distance3D = Math.Sqrt(CvEx.GetDistanceSq(real1, real2));

                            // zが1増えたときのカメラ座標系のx,yの変化
                            double xPerZ1 = GetRealXFromPerspective(undistortPoint1.X, imageSize.Width, 1);
                            double yPerZ1 = GetRealYFromPerspective(undistortPoint1.Y, imageSize.Height, 1);
                            double xPerZ2 = GetRealXFromPerspective(undistortPoint2.X, imageSize.Width, 1);
                            double yPerZ2 = GetRealYFromPerspective(undistortPoint2.Y, imageSize.Height, 1);
                            CvPoint2D64f perZ1 = new CvPoint2D64f(xPerZ1, yPerZ1);
                            CvPoint2D64f perZ2 = new CvPoint2D64f(xPerZ2, yPerZ2);

                            CvPoint3D64f post = KinectUndistortion.GetOriginalRealFromScreenPos(point2, imageSize);
                            depthList.Add(new Tuple<double, double>((z1 + z2) / 2, distance));

                            infoList.Add(new chessBoardEdgeInfo(undistScreen1, undistScreen2, real1, real2, perZ1, perZ2, offset.D));
                        }
                    }
                }
            }
            double zOffset = 0;
            // Len*A+LenPerZ*(A*B)=Distancer
            // 疑似解
            List<double[]> left = new List<double[]>();
            List<double> right = new List<double>();
            foreach (var info in infoList)
            {
                double[] l = new double[] { Math.Sqrt(CvEx.GetDistanceSq(info.Real1, info.Real2)), CvEx.GetLength(info.PerZ1 - info.PerZ2) };
                left.Add(l);
                right.Add(info.ExpectedLength);
            }
            double[] ans = CvEx.Solve(left, right, InvertMethod.Svd);
            this.XScale = ans[0];
            this.YScale = ans[0];
            zOffset = ans[1] / ans[0];
            this.ZOffset = zOffset;

            // (Dx1 * Z1 -Dx2*Z2)^2 * alpha + (Dy1*Z1-Dy2*Z2) * beta = DistanceSq
            double[] coefRet = CvEx.Solve(infoList.Select(c => new double[] {
                Math.Pow(c.PerZ1.X * (c.Real1.Z + this.ZOffset) - c.PerZ2.X * (c.Real2.Z + this.ZOffset), 2),
                Math.Pow(c.PerZ1.Y * (c.Real1.Z + this.ZOffset) - c.PerZ2.Y * (c.Real2.Z + this.ZOffset), 2)
            }), infoList.Select(c => c.ExpectedLength * c.ExpectedLength), InvertMethod.Svd);
            this.XScale = Math.Sqrt(coefRet[0]);
            this.YScale = Math.Sqrt(coefRet[1]);

            double rangeZ = 20;
            double rangeScale = 0.1;
            double bestZ = this.ZOffset;
            double bestX = this.XScale;
            double bestY = this.YScale;
            Func<double, double, double, double> getError = (z, x, y) =>
            {
                double errorSum = 0;
                foreach (var info in infoList)
                {
                    CvPoint3D64f offset1 = new CvPoint3D64f(info.Screen1.X, info.Screen1.Y, info.Screen1.Z + z);
                    CvPoint3D64f offset2 = new CvPoint3D64f(info.Screen2.X, info.Screen2.Y, info.Screen2.Z + z);
                    CvPoint3D64f real1 = GetOriginalRealFromScreenPos(offset1, imageSize);
                    CvPoint3D64f real2 = GetOriginalRealFromScreenPos(offset2, imageSize);
                    CvPoint3D64f scaled1 = new CvPoint3D64f(real1.X * x, real1.Y * y, real1.Z);
                    CvPoint3D64f scaled2 = new CvPoint3D64f(real2.X * x, real2.Y * y, real2.Z);
                    double distance = CvEx.GetLength(scaled1 - scaled2);
                    errorSum += Math.Abs(info.ExpectedLength - distance);
                }
                return errorSum;
            };

            double minError = getError(bestZ, bestX, bestY);
            for (int i = 0; i < 25; i++)
            {
                const int div = 5;
                double newBestZ = bestZ;
                double newBestX = bestX;
                double newBestY = bestY;
                for (double dZ = -rangeZ; dZ <= rangeZ; dZ += rangeZ / div)
                {
                    for (double dX = -rangeScale; dX <= rangeScale; dX += rangeScale / div)
                    {
                        for (double dY = -rangeScale; dY <= rangeScale; dY += rangeScale / div)
                        {
                            double error = getError(bestZ + dZ, bestX + dX, bestY + dY);
                            if (error < minError)
                            {
                                newBestZ = bestZ + dZ;
                                newBestX = bestX + dX;
                                newBestY = bestY + dY;
                                minError = error;
                            }
                        }
                    }
                }
                bestZ = newBestZ;
                bestX = newBestX;
                bestY = newBestY;
                rangeZ /= 2;
                rangeScale /= 2;
            }
            this.ZOffset = bestZ;
            this.XScale = bestX;
            this.YScale = bestY;
            return;
        }

        List<cornerEdgePair> getCornerPairs(IList<CvPoint3D64f[]> cornerPoints, int cols, int rows)
        {
            List<cornerEdgePair> ret = new List<cornerEdgePair>();
            foreach (CvPoint3D64f[] points in cornerPoints)
            {
                for (int x = 0; x < cols; x++)
                {
                    for (int y = 0; y < rows - 1; y++)
                    {
                        ret.Add(new cornerEdgePair(points[x + y * cols], points[x + (y + 1) * cols], false));
                    }
                }
                for (int x = 0; x < cols - 1; x++)
                {
                    for (int y = 0; y < rows; y++)
                    {
                        ret.Add(new cornerEdgePair(points[x + y * cols], points[(x + 1) + y * cols], true));
                    }
                }
            }
            return ret;
        }

        CvPoint[] getCorrectionMap(CvSize imageSize)
        {
            CvPoint[] ret = new CvPoint[imageSize.Width * imageSize.Height];
            for (int i = 0; i < ret.Length; i++)
            {
                ret[i] = new CvPoint(-1, -1);
            }

            double[] minDistanceSq = new double[imageSize.Width * imageSize.Height];
            for (int i = 0; i < minDistanceSq.Length; i++)
            {
                minDistanceSq[i] = double.MaxValue;
            }
            const double step = 0.50;
            for (double y = 0; y < imageSize.Height; y += step)
            {
                for (double x = 0; x < imageSize.Width; x += step)
                {
                    int orgX = (int)Math.Floor(x);
                    int orgY = (int)Math.Floor(y);
                    CvPoint2D64f newPos = getCorrectedImageXY(new CvPoint2D64f(x, y), imageSize);
                    int newX = (int)Math.Floor(newPos.X);
                    int newY = (int)Math.Floor(newPos.Y);
                    if (newX >= 0 && newY >= 0 && newX < imageSize.Width && newY < imageSize.Height)
                    {
                        int offset = newY * imageSize.Width + newX;
                        double centerPosX = newX + 0.5;
                        double centerPosY = newY + 0.5;
                        double dx = newPos.X - centerPosX;
                        double dy = newPos.Y - centerPosY;
                        double distanceSq = dx * dx + dy * dy;
                        if (distanceSq < minDistanceSq[offset])
                        {
                            minDistanceSq[offset] = distanceSq;
                            ret[offset] = new CvPoint(orgX, orgY);
                        }
                    }
                }
            }
            clearUndistorionImageXYCache();
            return ret;
        }

        CvPoint2D64f getCorrectedImageXY(CvPoint2D64f point, CvSize imageSize)
        {
            CvPoint2D64f temp = UndistortImageXY(point, imageSize);
            double px = _cameraStruct.PrincipalX * imageSize.Width / _imageMatSize.Width;
            double py = _cameraStruct.PrincipalY * imageSize.Width / _imageMatSize.Width;
            return new CvPoint2D64f((temp.X - px) * XScale + px, (temp.Y - py) * YScale + py);
        }

        readonly object _lockForCorrectionMap = new object();

        void correctImage(CvMat dest, CvMat src, CvPoint[] correctionMat, CvSize correctionMatSize)
        {
            if (correctionMat == null)
            {
                throw new ArgumentNullException("correctionMat");
            }
            if (correctionMat.Length != correctionMatSize.Width * correctionMatSize.Height)
            {
                throw new ArgumentException("Size mismatch: correctionMat and correctionMatSize");
            }
            CvEx.InitCvMat(ref dest, src);
            dest.Zero();
            if (src.Cols == correctionMatSize.Width && src.Rows == correctionMatSize.Height)
            {
                if (src.ElemChannels == 1 && src.ElemDepth == 16)
                {
                    unsafe
                    {
                        short* srcArr = src.DataInt16;
                        short* destArr = dest.DataInt16;
                        for (int y = 0; y < src.Rows; y++)
                        {
                            int offset = y * correctionMatSize.Width;
                            for (int x = 0; x < src.Cols; x++)
                            {
                                CvPoint srcPosition = correctionMat[offset + x];
                                if (srcPosition.X >= 0 && srcPosition.Y >= 0 && srcPosition.X < src.Cols && srcPosition.Y < src.Rows)
                                {
                                    int srcIndex = srcPosition.Y * src.Cols + srcPosition.X;
                                    destArr[offset + x] = srcArr[srcIndex];
                                }
                            }
                        }
                    }
                }
                else if (src.ElemDepth == 8)
                {
                    unsafe
                    {
                        byte* srcArr = src.DataByte;
                        byte* destArr = dest.DataByte;
                        for (int y = 0; y < src.Rows; y++)
                        {
                            int offset = y * correctionMatSize.Width;
                            for (int x = 0; x < src.Cols; x++)
                            {
                                CvPoint srcPosition = correctionMat[offset + x];
                                if (srcPosition.X >= 0 && srcPosition.Y >= 0 && srcPosition.X < src.Cols && srcPosition.Y < src.Rows)
                                {
                                    int srcIndex = (srcPosition.Y * src.Cols + srcPosition.X) * src.ElemChannels;
                                    for (int i = 0; i < src.ElemChannels; i++)
                                    {
                                        destArr[(offset + x) * src.ElemChannels + i] = srcArr[srcIndex + i];
                                    }
                                }
                            }
                        }
                    }
                }
                else
                {
                    for (int y = 0; y < src.Rows; y++)
                    {
                        int offsetTemp = y * correctionMatSize.Width;
                        for (int x = 0; x < src.Cols; x++)
                        {
                            CvPoint newPosition = correctionMat[offsetTemp + x];
                            if (newPosition.X >= 0 && newPosition.Y >= 0 && newPosition.X < src.Cols && newPosition.Y < src.Rows)
                            {
                                dest.Set2D(y, x, src.Get2D(newPosition.Y, newPosition.X));
                            }
                        }
                    }
                }
            }
            else
            {
                for (int y = 0; y < src.Rows; y++)
                {
                    int matY = y * correctionMatSize.Height / src.Rows;
                    int offsetTemp = matY * correctionMatSize.Width;
                    for (int x = 0; x < src.Cols; x++)
                    {
                        int matX = x * correctionMatSize.Width / src.Cols;
                        CvPoint newPosTemp = correctionMat[offsetTemp + matX];
                        CvPoint newPosition = new CvPoint(newPosTemp.X * src.Cols / correctionMatSize.Width, newPosTemp.Y * src.Rows / correctionMatSize.Height);
                        if (newPosition.X >= 0 && newPosition.Y >= 0 && newPosition.X < src.Cols && newPosition.Y < src.Rows)
                        {
                            dest.Set2D(y, x, src.Get2D(newPosition.Y, newPosition.X));
                        }
                    }
                }
            }
        }


        void undistortDepthMat(ref CvMat dest, CvMat src)
        {
            CvEx.InitCvMat(ref dest, src);
            dest.Zero();
            CvSize imageSize = new CvSize(src.Cols, src.Rows);
            unsafe
            {
                short* srcArr = src.DataInt16;
                short* destArr = dest.DataInt16;
                Parallel.For(0, src.Rows, y =>
                {
                    int offset = y * src.Cols;
                    for (int x = 0; x < src.Cols; x++)
                    {
                        int index = offset + x;
                        short depth = srcArr[index];
                        if (depth != 0)
                        {
                            short newDepth = (short)Math.Round(UndistortDepth(x, y, depth, imageSize) + this.ZOffset);
                            destArr[index] = newDepth;
                        }
                    }
                });
            }
        }

        public void CorrectSingleTrackImage(MotionData dest, MotionData source)
        {

            if (_imageCorrectionMap == null)
            {
                lock (_lockForCorrectionMap)
                {
                    if (_imageCorrectionMap == null)
                    {
                        if (_imageMatSize == CvSize.Empty)
                        {
                            _imageMatSize = source.ImageSize;
                        }
                        _imageCorrectionMap = getCorrectionMap(_imageMatSize);
                    }
                }
            }
            if (_depthCorrectionMap == null)
            {
                lock (_lockForCorrectionMap)
                {
                    if (_depthCorrectionMap == null)
                    {
                        if (_depthMatSize == CvSize.Empty)
                        {
                            _depthMatSize = source.DepthUserSize;
                        }
                        _depthCorrectionMap = getCorrectionMap(_depthMatSize);
                    }
                }
            }

            CvMat depthMat = null;
            CvMat depthMat2 = null;
            CvMat[] _tempMatArr = null;
            if (true)
            {
                CalcEx.MedianDepthMat(ref depthMat, source.DepthMat, 7);
                CalcEx.FillHoleDepthMat(ref depthMat2, depthMat, 15, 0.25, 0.75, 350);
                CalcEx.FillHoleDepthMat(ref depthMat, depthMat2, 9, 0.25, 0.75, 180);
                CalcEx.SmoothDepthStep(ref depthMat2, depthMat, 9);

                CalcEx.TrimEdgeDepthMat(ref depthMat, depthMat2, ref _tempMatArr);
                //  depthMat2.Copy(depthMat);
                undistortDepthMat(ref depthMat2, depthMat);
            }

            EventEx.SimultaneousInvoke(() =>
            {
                correctImage(dest.ImageMat, source.ImageMat, _imageCorrectionMap, _imageMatSize);
            }, () =>
            {
                correctImage(dest.DepthMat, depthMat2, _depthCorrectionMap, _depthMatSize);
            }, () =>
            {
                correctImage(dest.UserMat, source.UserMat, _depthCorrectionMap, _depthMatSize);
            });

            dest.TimeStamp = source.TimeStamp;
            //dest.UserTrackings = new Dictionary<int, UserTrackingState>();
            dest.bodies = new SerializableBody[source.bodies.Length];

            /*
            foreach (SerializableBody body in source.bodies)
            {
                
                UserTrackingState state = dest.UserTrackings[pair.Key] = new UserTrackingState();
                //state.IsCalibrating = pair.Value.IsCalibrating;
                //state.IsTracking = pair.Value.IsTracking;
                state.UserIndex = pair.Value.UserIndex;
                state.OriginalUserIndex = pair.Value.OriginalUserIndex;
                state.Position = KinectUndistortion.GetOriginalScreenPosFromReal(this.GetRealFromScreenPos(pair.Value.Position.ToCvPoint3D(), source.DepthUserSize), source.DepthUserSize).ToOpenNIPoint3D();
                if (pair.Value.OriginalJoints == null)
                {
                    state.OriginalJoints = null;
                }
                else
                {
                    state.OriginalJoints = new Dictionary<OpenNI.SkeletonJoint, OpenNI.SkeletonJointPosition>();
                    foreach (var joint in pair.Value.OriginalJoints)
                    {
                        OpenNI.SkeletonJointPosition newJointPos = new OpenNI.SkeletonJointPosition();
                        newJointPos.Position = KinectUndistortion.GetOriginalScreenPosFromReal(this.GetRealFromScreenPos(joint.Value.Position.ToCvPoint3D(), source.DepthUserSize), source.DepthUserSize).ToOpenNIPoint3D();
                        newJointPos.Confidence = joint.Value.Confidence;
                        state.OriginalJoints[joint.Key] = newJointPos;
                    }
                }
                 
            }*/
        }
    }
}
