using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;
using System.Windows.Forms;
using System.Xml.Serialization;
using System.IO;
using OpenCvSharp;

namespace KinectMotionCapture
{
    /// <summary>
    /// CameraUndistortWindow.xaml の相互作用ロジック
    /// </summary>
    public partial class CameraUndistortWindow : Window
    {
        public CameraUndistortWindow()
        {
            InitializeComponent();
            _dialogOpenTrackImage = new FolderBrowserDialog();
            _dialogOpenUndistortion = new OpenFileDialog();
            _dialogSaveUndistortion = new SaveFileDialog();
            const string filter = "Camera Undistortion Settings (*.undist)|*.undist|All Files (*.*)|*.*";
            _dialogOpenUndistortion.Filter = filter;
            _dialogSaveUndistortion.Filter = filter;
            const string extension = "undist";
            _dialogOpenUndistortion.DefaultExt = extension;
            _dialogSaveUndistortion.DefaultExt = extension;
            _undistortionSerializer = new XmlSerializer(typeof(KinectUndistortion));
        }
        XmlSerializer _undistortionSerializer;

        public bool IsSingleWindowMode = false;


        public string RelatedTrackImageSourcePath = null;
        string _settingsSourcePath = null;

        KinectUndistortion _undistortion;
        readonly object _lockUndistortionInstantiation = new object();
        /// <summary>
        /// カメラの歪み補正データを取得または設定します。
        /// </summary>
        public KinectUndistortion UndistortionData
        {
            get
            {
                if (_undistortion == null)
                {
                    lock (_lockUndistortionInstantiation)
                    {
                        if (_undistortion == null)
                        {
                            _undistortion = new KinectUndistortion();
                        }
                    }
                }
                return _undistortion;
            }
            set { _undistortion = value; }
        }

        bool _isCalibrateScaleOffsetEnabled { get { return this.UndistortionData.UndistortionDepthMat != null && this.UndistortionData.ImageMatSize != CvSize.Empty; } }

        FolderBrowserDialog _dialogOpenTrackImage;
        OpenFileDialog _dialogOpenUndistortion;
        SaveFileDialog _dialogSaveUndistortion;

        private void buttonFlatDepthCalib_Click(object sender, RoutedEventArgs e)
        {
            // 改造
            string path1, path2;
            MotionDataHandler handler1, handler2;
            openMotionData(out handler1, out path1);
            openMotionData(out handler2, out path2);
            IEnumerable<CvMat> depthImages1 = null;
            IEnumerable<CvMat> depthImages2 = null;
            Utility.LoadImages(handler1.GetDepthImagePaths(), out depthImages1);
            Utility.LoadImages(handler2.GetDepthImagePaths(), out depthImages2);

            List<double> errorVarLog = new List<double>();
            CvMat resultMat = null;

            int length = depthImages1.Count();
            DepthUndistortionLinearCalibrator undistort = new DepthUndistortionLinearCalibrator(this.UndistortionData.CameraStruct, 1);
            CvMat mat = null;

            foreach (CvMat depthMat in depthImages1)
            {
                CalcEx.SmoothDepthStep(ref mat, depthMat, 19);
                undistort.PutDepthImage(ref resultMat, mat, _undistortion);
                viewDepthUndistionMat(resultMat, depthMat);
            }
            foreach (CvMat depthMat in depthImages2)
            {
                CalcEx.SmoothDepthStep(ref mat, depthMat, 19);
                undistort.PutDepthImage(ref resultMat, mat, _undistortion);
                viewDepthUndistionMat(resultMat, depthMat);
            }
            this.UndistortionData.SetUndistortionDepthMat(undistort.GetUndistortCoefMat(), path1);
            displayLabels();
            viewDepthUndistionMat(this.UndistortionData.UndistortionDepthMat);
        }
        void viewDepthUndistionMat(CvMat src)
        {
            viewDepthUndistionMat(src, null);
        }
        void viewDepthUndistionMat(CvMat src, CvMat depthMat)
        {
            if (src == null)
            {
                return;
            }
            double pX = this.UndistortionData.CameraStruct.PrincipalX;
            double pY = this.UndistortionData.CameraStruct.PrincipalY;
            int focalX = (int)(this.UndistortionData.CameraStruct.FocalX / 2);
            int focalY = (int)(this.UndistortionData.CameraStruct.FocalY / 2);

            if (src.ElemChannels == 3)
            {
                CvMat quadratic = CvEx.InitCvMat(src, MatrixType.F32C1);
                CvMat scale = CvEx.InitCvMat(src, MatrixType.F32C1);
                CvMat offset = CvEx.InitCvMat(src, MatrixType.F32C1);
                src.Split(quadratic, scale, offset, null);
                float maxQuadratic, minQuadratic;
                float maxScale, minScale;
                float maxOffset, minOffset;
                using (CvMat displayMatQuadratic = getDepthUndistortMatImage(out maxQuadratic, out minQuadratic, quadratic, 0f))
                using (CvMat displayMatScale = getDepthUndistortMatImage(out maxScale, out minScale, scale, 1f))
                using (CvMat displayMatOffset = getDepthUndistortMatImage(out maxOffset, out minOffset, offset, null))
                {
                    using (CvMat black = CvEx.InitCvMat(src, MatrixType.U8C1))
                    {
                        CvMat total = CvEx.InitCvMat(src, MatrixType.U8C3);
                        black.Zero();
                        total.Merge(displayMatQuadratic, displayMatScale, displayMatOffset, null);
                        total.PutText(string.Format("RangeQuadatic: {0:0.000} to {1:0.000}", minQuadratic, maxQuadratic), new CvPoint(0, 20), new CvFont(FontFace.HersheyPlain, 1, 1), new CvScalar(255, 255, 255));
                        total.PutText(string.Format("RangeScale: {0:0.000} to {1:0.000}", minScale, maxScale), new CvPoint(0, 40), new CvFont(FontFace.HersheyPlain, 1, 1), new CvScalar(255, 255, 255));
                        total.PutText(string.Format("RangeOffset: {0:0.000} to {1:0.000}", minOffset, maxOffset), new CvPoint(0, 60), new CvFont(FontFace.HersheyPlain, 1, 1), new CvScalar(255, 255, 255));
                        total.DrawLine(new CvPoint((int)pX, 0), new CvPoint((int)pX, total.Rows), new CvScalar(255, 255, 0));
                        total.DrawLine(new CvPoint(0, (int)pY), new CvPoint(total.Cols, (int)pY), new CvScalar(255, 255, 0));
                        total.DrawEllipse(new CvPoint((int)pX, (int)pY), new CvSize(focalX, focalY), 0, 0, 360, new CvScalar(255, 0, 0));
                        putImage(total, PixelFormats.Rgb24);
                    }
                }
            }
            else if (src.ElemChannels == 2)
            {
                CvMat scale = CvEx.InitCvMat(src, MatrixType.F32C1);
                CvMat offset = CvEx.InitCvMat(src, MatrixType.F32C1);
                src.Split(scale, offset, null, null);
                float maxScale, minScale;
                float maxOffset, minOffset;
                using (CvMat displayMatScale = getDepthUndistortMatImage(out maxScale, out minScale, scale, 1f))
                using (CvMat displayMatOffset = getDepthUndistortMatImage(out maxOffset, out minOffset, offset, null))
                {
                    using (CvMat black = CvEx.InitCvMat(src, MatrixType.U8C1))
                    {
                        CvMat total = CvEx.InitCvMat(src, MatrixType.U8C3);
                        black.Zero();
                        total.Merge(black, displayMatScale, displayMatOffset, null);
                        total.PutText(string.Format("RangeScale: {0:0.000} to {1:0.000}", minScale, maxScale), new CvPoint(0, 20), new CvFont(FontFace.HersheyPlain, 1, 1), new CvScalar(255, 255, 255));
                        total.PutText(string.Format("RangeOffset: {0:0.000} to {1:0.000}", minOffset, maxOffset), new CvPoint(0, 40), new CvFont(FontFace.HersheyPlain, 1, 1), new CvScalar(255, 255, 255));
                        total.DrawLine(new CvPoint((int)pX, 0), new CvPoint((int)pX, total.Rows), new CvScalar(255, 255, 0));
                        total.DrawLine(new CvPoint(0, (int)pY), new CvPoint(total.Cols, (int)pY), new CvScalar(255, 255, 0));
                        total.DrawEllipse(new CvPoint((int)pX, (int)pY), new CvSize(focalX, focalY), 0, 0, 360, new CvScalar(255, 0, 0));
                        putImage(total, PixelFormats.Rgb24);
                    }
                }
            }
            else if (src.ElemChannels == 1)
            {
                float max, min;
                using (CvMat displayMat = getDepthUndistortMatImage(out max, out min, src, null))
                {
                    CvMat total = CvEx.InitCvMat(src, MatrixType.U8C3);
                    total.Merge(displayMat, displayMat, displayMat, null);
                    total.PutText(string.Format("Range: {0:0.000} to {1:0.000}", min, max), new CvPoint(0, 20), new CvFont(FontFace.HersheyPlain, 1, 1), new CvScalar(255, 255, 0));
                    if (depthMat != null)
                    {
                        double? pDepth = CvEx.Get2DSubPixel(depthMat, new CvPoint2D32f(pX, pY), 0);
                        if (pDepth.HasValue)
                        {
                            total.PutText(pDepth.Value.ToString("0"), new CvPoint((int)pX, (int)pY), new CvFont(FontFace.HersheyPlain, 1, 1), new CvScalar(255, 255, 0));
                        }
                        total.DrawLine(new CvPoint((int)pX, 0), new CvPoint((int)pX, total.Rows), new CvScalar(255, 255, 0));
                        total.DrawLine(new CvPoint(0, (int)pY), new CvPoint(total.Cols, (int)pY), new CvScalar(255, 255, 0));
                        total.DrawEllipse(new CvPoint((int)pX, (int)pY), new CvSize(focalX, focalY), 0, 0, 360, new CvScalar(255, 0, 0));

                    }
                    putImage(total, PixelFormats.Rgb24);
                }
            }
        }

        CvMat getDepthUndistortMatImage(out float maxValue, out float minValue, CvMat src, float? center)
        {
            CvMat ret = CvEx.InitCvMat(src, MatrixType.U8C1);
            List<double> values = new List<double>();
            for (int y = 0; y < src.Rows; y++)
            {
                for (int x = 0; x < src.Cols; x++)
                {
                    if (!_undistortion.CameraStruct.IsInFocalLength(x, y))
                        continue;
                    int i = y * src.Cols + x;
                    float value = src.DataArraySingle[i];
                    // if (value < 2)
                    //     continue;
                    values.Add(value);
                }
            }
            float max = 0;
            float min = float.MaxValue;
            if (values.Count >= 1)
            {
                max = (float)CalcEx.GetNth(values, (int)(values.Count * 0.99));
                min = (float)CalcEx.GetNth(values, (int)(values.Count * 0.01));
            }
            max = (max - 1) * 1.5f + 1;
            min = (min - 1) * 1.5f + 1;
            if (center.HasValue)
            {
                float maxRange = Math.Max(max - center.Value, center.Value - min);
                max = center.Value + maxRange;
                min = center.Value - maxRange;
            }

            //max = 1.05f;
            //min = 0.95f;


            maxValue = max;
            minValue = min;
            if (max == min)
            {
                max += 0.5f;
                min -= 0.5f;
            }

            for (int i = 0; i < src.Rows * src.Cols; i++)
            {
                float value = src.DataArraySingle[i];
                float output = 255 * (value - min) / (max - min);
                ret.DataArrayByte[i] = (byte)output;
            }
            return ret;
        }

        WriteableBitmap _bmp;
        void putImage(CvMat mat, PixelFormat format)
        {
            if (!this.Dispatcher.CheckAccess())
            {
                this.Dispatcher.Invoke(new Action<CvMat, PixelFormat>(putImage), mat, format);
                return;
            }
            CvEx.GetBmpFromMat(ref _bmp, mat, format);
            imageTrack.Source = _bmp;
        }

        bool openMotionData(out MotionDataHandler handler, out string path)
        {
            handler = null;
            path = null;
            _dialogOpenTrackImage.SelectedPath = Properties.Settings.Default.InitialRecordPath;
            if (_dialogOpenTrackImage.ShowDialog() != System.Windows.Forms.DialogResult.OK)
                return false;
            Properties.Settings.Default.InitialRecordPath = _dialogOpenTrackImage.SelectedPath;
            Properties.Settings.Default.Save();
            try
            {
                handler = new MotionDataHandler(_dialogOpenTrackImage.SelectedPath);
                path = _dialogOpenTrackImage.SelectedPath;
            }
            catch (NotImplementedException ex)
            {
                System.Windows.MessageBox.Show(ex.Message, ex.GetType().Name);
                return false;
            }
            return true;
        }

        private void buttonExport_Click(object sender, RoutedEventArgs e)
        {
            if (_dialogSaveUndistortion.ShowDialog() == System.Windows.Forms.DialogResult.OK)
            {
                using (FileStream fs = new FileStream(_dialogSaveUndistortion.FileName, FileMode.Create))
                {
                    _undistortionSerializer.Serialize(fs, this.UndistortionData);
                }
            }
        }

        private void buttonImport_Click(object sender, RoutedEventArgs e)
        {
            if (_dialogOpenUndistortion.ShowDialog() == System.Windows.Forms.DialogResult.OK)
            {
                using (FileStream fs = new FileStream(_dialogOpenUndistortion.FileName, FileMode.Open))
                {
                    this.UndistortionData = (KinectUndistortion)_undistortionSerializer.Deserialize(fs);
                }
                _settingsSourcePath = _dialogSaveUndistortion.FileName = _dialogOpenUndistortion.FileName;
                displayLabels();
            }
        }

        void displayLabels()
        {
            if (!this.Dispatcher.CheckAccess())
            {
                this.Dispatcher.BeginInvoke(new Action(displayLabels));
                return;
            }
            string title = typeof(CameraUndistortWindow).Name;
            if (_settingsSourcePath != null)
            {
                title += " - " + _settingsSourcePath;
            }
            string trackPath = this.RelatedTrackImageSourcePath;
            if (trackPath != null)
            {
                title += " / (" + trackPath + ")";
            }
            this.Title = title;
        }

        private void buttonOK_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                this.DialogResult = true;
            }
            catch (InvalidOperationException) { }
            this.Close();
        }

        private void buttonCancel_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                this.DialogResult = null;
            }
            catch (InvalidOperationException) { }

            this.Close();
        }

        private void buttonViewDepthUndistorion_Click(object sender, RoutedEventArgs e)
        {
            viewDepthUndistionMat(this.UndistortionData.UndistortionDepthMat);

        }

        private void textBoxCols_TextChanged(object sender, TextChangedEventArgs e)
        {

        }

        private void buttonCameraCalibration_Click(object sender, RoutedEventArgs e)
        {
            int cols, rows;
            double horizLength, vertLength;
            if (!parseChessboardParameters(out cols, out rows, out horizLength, out vertLength))
            {
                return;
            }

            int imageNum;
            if (!int.TryParse(textCalibrationCameraIteration.Text, out imageNum) || imageNum <= 0)
            {
                System.Windows.MessageBox.Show(string.Format("キャリブレーションに使用するイメージ数が不正です: {0}", textCalibrationCameraIteration.Text));
                return;
            }
            TrackImageRecordData recordData;
            string path;
            if (openMotionData(out recordData, out path))
            {
                CvMat displayMat1 = null;
                CvMat displayMat3 = null;
                CvMat gray = null;
                using (recordData)
                {
                    if (ProgressData.DoAction(progress =>
                    {
                        int length = recordData.FrameCount;
                        if (length == 0) { return; }
                        progress.InitProgress("Find Chessboard...", length * 2);
                        CvSize boardSize = new CvSize(cols, rows);
                        List<CvPoint2D32f[]> list = new List<CvPoint2D32f[]>();
                        CvSize imageSize = new CvSize();
                        CvPoint2D32f[] lastCorners = null;

                        foreach (TrackImageFrame trackImage in recordData.EnumerateTrackImageFrame())
                        {
                            progress.CurrentValue++;

                            imageSize = new CvSize(trackImage.ImageMat.Cols, trackImage.ImageMat.Rows);
                            CvPoint2D32f[] corners;
                            int count;
                            CvEx.InitCvMat(ref gray, trackImage.ImageMat, MatrixType.U8C1);
                            trackImage.ImageMat.CvtColor(gray, ColorConversion.RgbToGray);
                            if (gray.FindChessboardCorners(boardSize, out corners, out count, ChessboardFlag.AdaptiveThresh))
                            {
                                CvEx.CloneCvMat(ref displayMat1, trackImage.ImageMat);
                                CvTermCriteria criteria = new CvTermCriteria(20, 0.1);
                                gray.FindCornerSubPix(corners, count, new CvSize(11, 11), new CvSize(-1, -1), criteria);
                                list.Add(corners);
                                CvEx.DrawChessboardCornerFrame(displayMat1, boardSize, corners, new CvScalar(64, 128, 64));
                                displayMat1.DrawChessboardCorners(boardSize, corners, true);
                                lastCorners = corners;
                                putImage(displayMat1, PixelFormats.Rgb24);
                            }
                            else
                            {
                                CvEx.CloneCvMat(ref displayMat3, trackImage.ImageMat);
                                putImage(displayMat3, PixelFormats.Rgb24);
                            }
                        }

                        progress.SetProgress("Calibrating...", length);

                        this.UndistortionData.CalibrateCamera(list, cols, rows, (horizLength + vertLength) / 2, imageSize, imageNum, path);
                        CvMat displayMat2 = CvEx.InitCvMat(displayMat1);
                        displayMat1.Undistort2(displayMat2, this.UndistortionData.CameraStruct.CreateCvMat(), this.UndistortionData.DistortStruct.CreateCvMat(true));
                        if (lastCorners != null)
                        {
                            drawUndistortedCornerFrame(displayMat2, lastCorners, boardSize);
                        }

                        putImage(displayMat2, PixelFormats.Rgb24);
                    }, "Camera Calib", true))
                    {
                        displayLabels();
                    }
                }
            }

        }

        private void textBoxCols_LostFocus(object sender, RoutedEventArgs e)
        {
            Properties.Settings.Default.Save();
        }

        private void textBoxRows_LostFocus(object sender, RoutedEventArgs e)
        {
            Properties.Settings.Default.Save();
        }

        private void textBoxBlockLength_LostFocus(object sender, RoutedEventArgs e)
        {
            Properties.Settings.Default.Save();
        }

        private void buttonProperties_Click(object sender, RoutedEventArgs e)
        {
            StringBuilder str = new StringBuilder();
            str.AppendLine(string.Format("Camera Calibration: {0}", this.UndistortionData.CalibrationSourcePath ?? "(empty)"));
            str.AppendLine(string.Format("Undistortion Depth: {0}", this.UndistortionData.UndistortionDepthSourcePath ?? "(empty)"));
            System.Windows.MessageBox.Show(str.ToString());
        }

        bool parseChessboardParameters(out int cols, out int rows, out double horizLength, out double vertLength)
        {
            cols = rows = 0;
            horizLength = vertLength = 0;
            if (!int.TryParse(textBoxCols.Text, out cols) || cols < 1)
            {
                System.Windows.MessageBox.Show(string.Format("列数が不正です: {0}", textBoxCols.Text));
                return false;
            }
            if (!int.TryParse(textBoxRows.Text, out rows) || rows < 1)
            {
                System.Windows.MessageBox.Show(string.Format("行数が不正です: {0}", textBoxRows.Text));
                return false;
            }
            if (!double.TryParse(textBoxHorizontalLength.Text, out horizLength) || horizLength <= 0)
            {
                System.Windows.MessageBox.Show(string.Format("コーナー間の水平の長さが不正です: {0}", textBoxHorizontalLength.Text));
                return false;
            }
            if (!double.TryParse(textBoxVerticalLength.Text, out vertLength) || vertLength <= 0)
            {
                System.Windows.MessageBox.Show(string.Format("コーナー間の垂直の長さが不正です: {0}", textBoxVerticalLength.Text));
                return false;
            }
            return true;
        }

        private void buttonCalibrateScaleOffset_Click(object sender, RoutedEventArgs e)
        {
            int cols, rows;
            double horizLength, vertLength;
            if (!parseChessboardParameters(out cols, out rows, out horizLength, out vertLength))
            {
                return;
            }
            TrackImageRecordData recordData;
            string path;
            if (openMotionData(out recordData, out path))
            {
                CvMat displayMat1 = null;
                CvMat displayMat3 = null;
                CvMat gray = null;
                using (recordData)
                {
                    if (ProgressData.DoAction(progress =>
                    {
                        int length = recordData.FrameCount;
                        if (length == 0) { return; }
                        progress.InitProgress("Find Chessboard...", length * 2);
                        CvSize boardSize = new CvSize(cols, rows);
                        List<CvPoint3D32f?[]> list = new List<CvPoint3D32f?[]>();
                        CvSize imageSize = new CvSize();
                        CvPoint2D32f[] lastCorners = null;

                        foreach (TrackImageFrame trackImage in recordData.EnumerateTrackImageFrame())
                        {
                            progress.CurrentValue++;


                            imageSize = new CvSize(trackImage.ImageMat.Cols, trackImage.ImageMat.Rows);
                            CvPoint2D32f[] corners;
                            int count;
                            CvEx.InitCvMat(ref gray, trackImage.ImageMat, MatrixType.U8C1);
                            trackImage.ImageMat.CvtColor(gray, ColorConversion.RgbToGray);
                            if (gray.FindChessboardCorners(boardSize, out corners, out count, ChessboardFlag.AdaptiveThresh))
                            {
                                CvEx.CloneCvMat(ref displayMat1, trackImage.ImageMat);
                                CvTermCriteria criteria = new CvTermCriteria(50, 0.01);
                                gray.FindCornerSubPix(corners, count, new CvSize(3, 3), new CvSize(-1, -1), criteria);
                                CvPoint3D32f?[] cornerPoints = new CvPoint3D32f?[corners.Length];
                                for (int j = 0; j < corners.Length; j++)
                                {
                                    CvPoint2D32f corner = corners[j];
                                    double? value = CalcEx.BilateralFilterDepthMatSinglePixel(corner, trackImage.DepthMat, 100, 4, 9);
                                    if (value.HasValue)
                                    {
                                        cornerPoints[j] = new CvPoint3D32f(corner.X, corner.Y, value.Value);
                                    }
                                }
                                list.Add(cornerPoints);
                                CvEx.DrawChessboardCornerFrame(displayMat1, boardSize, corners, new CvScalar(64, 128, 64));
                                displayMat1.DrawChessboardCorners(boardSize, corners, true);
                                lastCorners = corners;
                                putImage(displayMat1, PixelFormats.Rgb24);
                            }
                            else
                            {
                                CvEx.CloneCvMat(ref displayMat3, trackImage.ImageMat);
                                putImage(displayMat3, PixelFormats.Rgb24);
                            }
                        }

                        progress.SetProgress("Scale Offset Calibrating...", length);

                        this.UndistortionData.CalibrateRealScaleAndOffset(list, cols, rows, horizLength, vertLength, imageSize);
                        CvMat displayMat2 = CvEx.InitCvMat(displayMat1);
                        displayMat1.Undistort2(displayMat2, this.UndistortionData.CameraStruct.CreateCvMat(), this.UndistortionData.DistortStruct.CreateCvMat(true));
                        if (lastCorners != null)
                        {
                            drawUndistortedCornerFrame(displayMat2, lastCorners, boardSize);
                        }

                        displayMat2.PutText(string.Format("XScale: {0}", this.UndistortionData.XScale), new CvPoint(20, 20), new CvFont(FontFace.HersheyPlain, 1, 1), new CvScalar(255, 255, 255));
                        displayMat2.PutText(string.Format("YScale: {0}", this.UndistortionData.YScale), new CvPoint(20, 40), new CvFont(FontFace.HersheyPlain, 1, 1), new CvScalar(255, 255, 255));
                        displayMat2.PutText(string.Format("Zoffset: {0}", this.UndistortionData.ZOffset), new CvPoint(20, 60), new CvFont(FontFace.HersheyPlain, 1, 1), new CvScalar(255, 255, 255));
                        putImage(displayMat2, PixelFormats.Rgb24);
                    }, "Calibrate Scale Offset", true))
                    {
                        displayLabels();
                    }
                }
            }
        }

        void drawUndistortedCornerFrame(CvMat displayMat, CvPoint2D32f[] corners, CvSize boardSize)
        {
            CvMat cornerMat = new CvMat(1, corners.Length, MatrixType.F32C2);
            CvEx.FillCvMat(cornerMat, corners.Select(x => new CvScalar(x.X, x.Y)).ToList());
            CvMat undistMat = CvEx.InitCvMat(cornerMat);
            Cv.UndistortPoints(cornerMat, undistMat, this.UndistortionData.CameraStruct.CreateCvMat(), this.UndistortionData.DistortStruct.CreateCvMat(true), null, this.UndistortionData.CameraStruct.CreateCvMat());
            CvEx.DrawChessboardCornerFrame(displayMat, boardSize, undistMat.Select(x => new CvPoint2D32f(x.Val0, x.Val1)).ToArray(), new CvScalar(216, 216, 216));
        }

        private void buttonScalingScore_Click(object sender, RoutedEventArgs e)
        {
            int cols, rows;
            double horizLength, vertLength;
            if (!parseChessboardParameters(out cols, out rows, out horizLength, out vertLength))
            {
                return;
            }
            TrackImageRecordData recordData;
            string path;
            if (openMotionData(out recordData, out path))
            {
                CvMat displayMat1 = null;
                CvMat displayMat3 = null;
                CvMat displayMat4 = null;
                CvMat gray = null;
                using (recordData)
                {
                    if (ProgressData.DoAction(progress =>
                    {
                        int length = recordData.FrameCount;
                        if (length == 0) { return; }
                        progress.InitProgress("Find Chessboard...", length * 2);
                        CvSize boardSize = new CvSize(cols, rows);
                        CvSize imageSize = new CvSize();
                        List<Tuple<double, double>> pairs = new List<Tuple<double, double>>();
                        CvPoint2D32f[] lastCorners = null;
                        foreach (TrackImageFrame trackImage in recordData.EnumerateTrackImageFrame())
                        {
                            progress.CurrentValue++;
                            if (displayMat4 == null)
                            {
                                displayMat4 = CvEx.InitCvMat(trackImage.ImageMat);
                            }

                            imageSize = new CvSize(trackImage.ImageMat.Cols, trackImage.ImageMat.Rows);
                            CvPoint2D32f[] corners;
                            int count;
                            CvEx.InitCvMat(ref gray, trackImage.ImageMat, MatrixType.U8C1);
                            trackImage.ImageMat.CvtColor(gray, ColorConversion.RgbToGray);
                            if (gray.FindChessboardCorners(boardSize, out corners, out count, ChessboardFlag.AdaptiveThresh))
                            {
                                CvEx.CloneCvMat(ref displayMat1, trackImage.ImageMat);
                                CvTermCriteria criteria = new CvTermCriteria(50, 0.01);
                                gray.FindCornerSubPix(corners, count, new CvSize(3, 3), new CvSize(-1, -1), criteria);
                                CvPoint3D32f?[] cornerPoints = new CvPoint3D32f?[corners.Length];
                                for (int j = 0; j < corners.Length; j++)
                                {
                                    CvPoint2D32f corner = corners[j];
                                    double? value = CalcEx.BilateralFilterDepthMatSinglePixel(corner, trackImage.DepthMat, 100, 4, 9);
                                    if (value.HasValue)
                                    {
                                        cornerPoints[j] = new CvPoint3D32f(corner.X, corner.Y, value.Value);
                                    }
                                }
                                for (int x = 0; x < cols; x++)
                                {
                                    for (int y = 0; y < rows; y++)
                                    {
                                        if (!cornerPoints[x + y * cols].HasValue)
                                            continue;
                                        CvPoint3D32f point1 = cornerPoints[x + y * cols].Value;
                                        CvPoint3D64f undistortPoint1 = this.UndistortionData.GetRealFromScreenPos(point1, imageSize);
                                        foreach (var offset in new[] { new { X = 1, Y = 0, D = horizLength }, new { X = 0, Y = 1, D = vertLength } })
                                        {
                                            int dx = x + offset.X;
                                            int dy = y + offset.Y;
                                            if (dx >= cols || dy >= rows)
                                                continue;
                                            if (!cornerPoints[dx + dy * cols].HasValue)
                                                continue;

                                            CvPoint3D32f point2 = cornerPoints[dx + dy * cols].Value;
                                            CvPoint3D64f undistortPoint2 = this.UndistortionData.GetRealFromScreenPos(point2, imageSize);
                                            double distance = Math.Sqrt(CvEx.GetDistanceSq(undistortPoint1, undistortPoint2));
                                            double scale = distance / offset.D;
                                            CvColor color = CalcEx.HSVtoRGB(Math.Max(0, Math.Min(300, scale * 600 - 450)), scale, 2 - scale);
                                            displayMat4.DrawLine((int)point1.X, (int)point1.Y, (int)point2.X, (int)point2.Y, new CvScalar(color.R, color.G, color.B), 1, LineType.AntiAlias);
                                            pairs.Add(new Tuple<double, double>(distance, offset.D));
                                        }
                                    }
                                }
                                CvEx.DrawChessboardCornerFrame(displayMat1, boardSize, corners, new CvScalar(64, 128, 64));
                                displayMat1.DrawChessboardCorners(boardSize, corners, true);
                                lastCorners = corners;
                                putImage(displayMat1, PixelFormats.Rgb24);
                            }
                            else
                            {
                                CvEx.CloneCvMat(ref displayMat3, trackImage.ImageMat);
                                putImage(displayMat3, PixelFormats.Rgb24);
                            }
                        }
                        progress.SetProgress("Calibrating...", length);

                        CvMat displayMat2 = CvEx.InitCvMat(displayMat1);
                        displayMat1.Undistort2(displayMat2, this.UndistortionData.CameraStruct.CreateCvMat(), this.UndistortionData.DistortStruct.CreateCvMat(true));
                        if (lastCorners != null)
                        {
                            drawUndistortedCornerFrame(displayMat2, lastCorners, boardSize);
                        }
                        displayMat2.PutText(string.Format("Min: {0}", pairs.Min(x => x.Item1 / x.Item2)), new CvPoint(20, 20), new CvFont(FontFace.HersheyPlain, 1, 1), new CvScalar(255, 255, 255));
                        displayMat2.PutText(string.Format("Max: {0}", pairs.Max(x => x.Item1 / x.Item2)), new CvPoint(20, 40), new CvFont(FontFace.HersheyPlain, 1, 1), new CvScalar(255, 255, 255));
                        displayMat2.PutText(string.Format("Avg: {0}", pairs.Average(x => x.Item1 / x.Item2)), new CvPoint(20, 60), new CvFont(FontFace.HersheyPlain, 1, 1), new CvScalar(255, 255, 255));
                        displayMat2.PutText(string.Format("Med: {0}", CalcEx.GetMedian(pairs.Select(x => x.Item1 / x.Item2).ToList())), new CvPoint(20, 80), new CvFont(FontFace.HersheyPlain, 1, 1), new CvScalar(255, 255, 255));
                        putImage(displayMat4, PixelFormats.Rgb24);
                    }, "Camera Calib", true))
                    {
                        displayLabels();
                    }
                }
            }

        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            if (this.IsSingleWindowMode)
            {
                buttonOK.IsEnabled = false;
                buttonCancel.IsEnabled = false;
            }
        }

        private void buttonDepthErrorEvaluation_Click(object sender, RoutedEventArgs e)
        {
            TrackImageRecordData recordData;
            string path;
            if (openMotionData(out recordData, out path))
            {
                CvMat resultMat = null;
                using (recordData)
                {
                    List<double> errorVarList = new List<double>();
                    if (ProgressData.DoAction(progress =>
                    {
                        int length = recordData.FrameCount;
                        progress.InitProgress("Calculating...", length);
                        foreach (TrackImageFrame trackImage in recordData.EnumerateTrackImageFrame())
                        {
                            progress.CurrentValue++;

                            double errorVar = DepthUndistortionLinearCalibrator.EvaluateUndistortion(ref resultMat, trackImage.DepthMat, _undistortion);
                            errorVarList.Add(errorVar);
                            viewDepthUndistionMat(resultMat, trackImage.DepthMat);
                        }
                    }, "Flat Depth Calib", true))
                    {
                        displayLabels();
                        viewDepthUndistionMat(this.UndistortionData.UndistortionDepthMat);
                        if (errorVarList.Count > 0)
                        {
                            double errorVarAvg = errorVarList.Average();
                            System.Windows.MessageBox.Show(string.Format("誤差分散の平均: {0}", errorVarAvg));
                        }
                    }
                }
            }
        }

        private void buttonTest0_Click(object sender, RoutedEventArgs e)
        {
            int cols, rows;
            double horizLength, vertLength;
            if (!parseChessboardParameters(out cols, out rows, out horizLength, out vertLength))
            {
                return;
            }
            TrackImageRecordData recordData;
            string path;
            if (openMotionData(out recordData, out path))
            {
                CvMat displayMat1 = null;
                CvMat displayMat3 = null;
                CvMat displayMat4 = null;
                CvMat gray = null;
                using (recordData)
                {
                    if (ProgressData.DoAction(progress =>
                    {
                        int length = recordData.FrameCount;
                        if (length == 0) { return; }
                        progress.InitProgress("Find Chessboard...", length);
                        CvSize boardSize = new CvSize(cols, rows);
                        CvSize imageSize = new CvSize();
                        double minVarDistance2d = double.MaxValue;
                        foreach (TrackImageFrame trackImage in recordData.EnumerateTrackImageFrame())
                        {
                            progress.CurrentValue++;
                            if (displayMat4 == null)
                            {
                                displayMat4 = CvEx.InitCvMat(trackImage.ImageMat);
                            }

                            imageSize = new CvSize(trackImage.ImageMat.Cols, trackImage.ImageMat.Rows);
                            CvPoint2D32f[] corners;
                            int count;
                            CvEx.InitCvMat(ref gray, trackImage.ImageMat, MatrixType.U8C1);
                            trackImage.ImageMat.CvtColor(gray, ColorConversion.RgbToGray);
                            if (gray.FindChessboardCorners(boardSize, out corners, out count, ChessboardFlag.AdaptiveThresh))
                            {
                                CvEx.CloneCvMat(ref displayMat1, trackImage.ImageMat);
                                CvTermCriteria criteria = new CvTermCriteria(50, 0.01);
                                gray.FindCornerSubPix(corners, count, new CvSize(3, 3), new CvSize(-1, -1), criteria);
                                CvPoint3D32f?[] cornerPoints = new CvPoint3D32f?[corners.Length];
                                for (int j = 0; j < corners.Length; j++)
                                {
                                    CvPoint2D32f corner = new CvPoint2D32f(corners[j].X - 10, corners[j].Y - 10);
                                    double? value = CvEx.Get2DSubPixel(trackImage.DepthMat, corner, 0);
                                    if (value.HasValue)
                                    {
                                        double depth = UndistortionData.UndistortDepth(corner.X, corner.Y, value.Value, trackImage.DepthUserSize);
                                        cornerPoints[j] = new CvPoint3D32f(corner.X, corner.Y, depth);
                                    }
                                }
                                List<double> distance2dList = new List<double>();
                                for (int x = 0; x < cols; x++)
                                {
                                    for (int y = 0; y < rows; y++)
                                    {
                                        if (!cornerPoints[x + y * cols].HasValue)
                                            continue;
                                        int nextX = x + 1;
                                        if (nextX < cols)
                                        {
                                            if (!cornerPoints[nextX + y * cols].HasValue)
                                                continue;
                                            CvPoint3D32f point = cornerPoints[x + y * cols].Value;
                                            CvPoint3D32f nextPoint = cornerPoints[nextX + y * cols].Value;
                                            distance2dList.Add(Math.Sqrt(Math.Pow(point.X - nextPoint.X, 2) + Math.Pow(point.Y - nextPoint.Y, 2)));
                                        }
                                        int nextY = y + 1;
                                        if (nextY < rows)
                                        {
                                            if (!cornerPoints[x + nextY * cols].HasValue)
                                                continue;
                                            CvPoint3D32f point = cornerPoints[x + y * cols].Value;
                                            CvPoint3D32f nextPoint = cornerPoints[x + nextY * cols].Value;
                                            distance2dList.Add(Math.Sqrt(Math.Pow(point.X - nextPoint.X, 2) + Math.Pow(point.Y - nextPoint.Y, 2)));
                                        }
                                    }
                                }
                                if (distance2dList.Count >= 2)
                                {
                                    double stdevDistance2d = CalcEx.GetStdDev(distance2dList);
                                    displayMat1.PutText(string.Format("{0:0.00}/{1:0.00}", stdevDistance2d, minVarDistance2d), new CvPoint(0, 20), new CvFont(FontFace.HersheyPlain, 1, 1), new CvScalar(255, 255, 0));
                                    double avgDepth = cornerPoints.Where(p => p.HasValue).Select(p => p.Value.Z).Average();
                                    for (int x = 0; x < cols; x++)
                                    {
                                        for (int y = 0; y < rows; y++)
                                        {
                                            if (!cornerPoints[x + y * cols].HasValue)
                                                continue;
                                            CvPoint3D32f point = cornerPoints[x + y * cols].Value;
                                            displayMat1.PutText((point.Z - avgDepth).ToString("0.00"), new CvPoint((int)point.X, (int)point.Y), new CvFont(FontFace.HersheyPlain, 0.6, 0.6), new CvScalar(255, 0, 0));
                                            displayMat1.PutText(((point.Z - avgDepth) / avgDepth * 100).ToString("0.000"), new CvPoint((int)point.X, (int)point.Y + 12), new CvFont(FontFace.HersheyPlain, 0.6, 0.6), new CvScalar(0, 255, 255));
                                        }
                                    }
                                    //displayMat1.DrawChessboardCorners(boardSize, corners, true);

                                    if (stdevDistance2d < minVarDistance2d)
                                    {
                                        minVarDistance2d = stdevDistance2d;
                                        CvEx.CloneCvMat(ref displayMat4, displayMat1);
                                    }
                                    //System.Threading.Thread.Sleep(500);
                                }
                                putImage(displayMat1, PixelFormats.Rgb24);
                            }
                            else
                            {
                                CvEx.CloneCvMat(ref displayMat3, trackImage.ImageMat);
                                putImage(displayMat3, PixelFormats.Rgb24);
                            }
                        }

                        putImage(displayMat4, PixelFormats.Rgb24);
                    }, "Camera Calib", true))
                    {
                        displayLabels();
                    }
                }
            }
        }

        private void buttonClearDepth_Click(object sender, RoutedEventArgs e)
        {
            _undistortion.UndistortionDepthMat = null;
        }

        private void button1_Click(object sender, RoutedEventArgs e)
        {

        }

        private void button1_Click_1(object sender, RoutedEventArgs e)
        {
            MemorySavingLeastSquare.Test();
        }

        private void button1_Click_2(object sender, RoutedEventArgs e)
        {
            TrackImageRecordData recordData;
            string path;
            if (openMotionData(out recordData, out path))
            {
                using (recordData)
                {
                    CvMat resultMat = null;
                    if (ProgressData.DoAction(progress =>
                    {
                        int length = recordData.FrameCount;
                        progress.InitProgress("t...", length);
                        foreach (TrackImageFrame trackImage in recordData.EnumerateTrackImageFrame())
                        {
                            CvEx.InitCvMat(ref resultMat, trackImage.DepthMat, MatrixType.U8C3);
                            resultMat.Zero();
                            double avgDepth = trackImage.DepthMat.Select(v => v.Val0).Where(v => v != 0).Average();
                            double pDepth = CvEx.Get2DSubPixel(trackImage.DepthMat, new CvPoint2D32f(_undistortion.CameraStruct.PrincipalX, _undistortion.CameraStruct.PrincipalY), 0) ?? 0;
                            List<double>[] diffs = Enumerable.Range(0, trackImage.DepthUserSize.Width).Select(x => new List<double>()).ToArray();
                            unsafe
                            {
                                short* depthArr = trackImage.DepthMat.DataInt16;
                                for (int y = 0; y < trackImage.DepthUserSize.Height; y++)
                                {
                                    int offset = y * trackImage.DepthUserSize.Width;
                                    for (int x = 0; x < trackImage.DepthUserSize.Width - 1; x++)
                                    {
                                        short l = depthArr[offset + x];
                                        short r = depthArr[offset + x + 1];
                                        if (l != 0 && r != 0)
                                        {
                                            double ll = Math.Log(l);
                                            double rl = Math.Log(r);
                                            diffs[x].Add(ll - rl);
                                        }
                                    }
                                }
                            }
                            double[] median = diffs.Select(x => x.Count > 0 ? CalcEx.GetMedian(x) : 0).ToArray();
                            double max = median.Select(x => Math.Abs(x)).Max();
                            for (int x = 0; x < trackImage.DepthUserSize.Width; x++)
                            {
                                resultMat.DrawLine(new CvPoint(x, 0), new CvPoint(x, resultMat.Rows), new CvScalar(Math.Max(median[x] / max * 255, 0), Math.Max(-median[x] / max * 255, 0), 0));
                            }
                            resultMat.PutText(avgDepth.ToString("0.00000"), new CvPoint(0, 20), new CvFont(FontFace.HersheyPlain, 1, 1), new CvScalar(255, 255, 255));
                            resultMat.PutText(pDepth.ToString("0.00000"), new CvPoint(0, 40), new CvFont(FontFace.HersheyPlain, 1, 1), new CvScalar(255, 255, 255));
                            progress.CurrentValue++;
                            putImage(resultMat, PixelFormats.Rgb24);
                        }
                    }, "t", true))
                    {

                    }
                }
            }
        }
    }
}

