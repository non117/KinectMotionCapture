using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.IO;
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

using Microsoft.Kinect;
using OpenCvSharp;
using OpenCvSharp.Extensions;

namespace KinectMotionCapture
{
    /// <summary>
    /// MergeRecordWindow.xaml の相互作用ロジック
    /// </summary>
    public partial class MergeRecordWindow : Window
    {
        private FrameSequence frameSequence;

        private KinectSensor kinectSensor;
        private CoordinateMapper coordinateMapper;
        private BackgroundWorker worker;

        private int playingIndex;
        private int startIndex;
        private int endIndex;
        private bool isPlaying;

        private bool[] isUserSelected;
        private bool[] isRecordSelected;
        private Slider[] recordSliders;
        private Canvas[] canvases;

        private DrawingGroup drawingGroup1;
        private DrawingImage bodyImageSource1;
        private DrawingGroup drawingGroup2;
        private DrawingImage bodyImageSource2;
        private DrawingGroup drawingGroup3;
        private DrawingImage bodyImageSource3;
        private DrawingGroup drawingGroup4;
        private DrawingImage bodyImageSource4;
        private DrawingGroup drawingGroup5;
        private DrawingImage bodyImageSource5;

        private List<Tuple<JointType, JointType>> bones;

        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));
        private readonly Brush inferredJointBrush = Brushes.Yellow;
        private const double JointThickness = 3;

        /// <summary>
        /// FrameSequenceの構築, そのうちダイアログとか出したい
        /// </summary>
        private void LoadFrames()
        {
            string[] datadir = new string[] {
                                                    @"C:\Users\non\Desktop\1226\1226_kinect1\calib",  
                                                    @"C:\Users\non\Desktop\1226\1226_kinect2\calib", 
                                                    @"C:\Users\non\Desktop\1226\1226_kinect3\calib", 
                                                    @"C:\Users\non\Desktop\1226\1226_kinect4\calib", 
                                                    @"C:\Users\non\Desktop\1226\1226_kinect5\calib", 
            };
            List<string> mapdir = new List<string>() {
                                                    @"C:\Users\non\Desktop\1226\1226_kinect1\coordmap.dump",  
                                                    @"C:\Users\non\Desktop\1226\1226_kinect2\coordmap.dump", 
                                                    @"C:\Users\non\Desktop\1226\1226_kinect3\coordmap.dump", 
                                                    @"C:\Users\non\Desktop\1226\1226_kinect4\coordmap.dump", 
                                                    @"C:\Users\non\Desktop\1226\1226_kinect5\coordmap.dump", 
            };
            List<string> cameradir = new List<string>() {
                                                    @"C:\Users\non\Desktop\1226\1226_kinect1\CameraInfo.dump",  
                                                    @"C:\Users\non\Desktop\1226\1226_kinect2\CameraInfo.dump", 
                                                    @"C:\Users\non\Desktop\1226\1226_kinect3\CameraInfo.dump", 
                                                    @"C:\Users\non\Desktop\1226\1226_kinect4\CameraInfo.dump", 
                                                    @"C:\Users\non\Desktop\1226\1226_kinect5\CameraInfo.dump", 
            };
            this.frameSequence = new FrameSequence(datadir);
            this.frameSequence.LocalCoordinateMappers = mapdir.Select(s => (LocalCoordinateMapper)Utility.LoadFromBinary(s)).ToList();
            this.frameSequence.CameraInfo = cameradir.Select(s => (CameraIntrinsics)Utility.LoadFromBinary(s)).ToList();
        }

        /// <summary>
        /// こんすとらくたん
        /// </summary>
        public MergeRecordWindow()
        {
            this.drawingGroup1 = new DrawingGroup();
            this.bodyImageSource1 = new DrawingImage(this.drawingGroup1);
            this.drawingGroup2 = new DrawingGroup();
            this.bodyImageSource2 = new DrawingImage(this.drawingGroup2);
            this.drawingGroup3 = new DrawingGroup();
            this.bodyImageSource3 = new DrawingImage(this.drawingGroup3);
            this.drawingGroup4 = new DrawingGroup();
            this.bodyImageSource4 = new DrawingImage(this.drawingGroup4);
            this.drawingGroup5 = new DrawingGroup();
            this.bodyImageSource5 = new DrawingImage(this.drawingGroup5);

            this.worker = new BackgroundWorker();
            this.worker.WorkerReportsProgress = true;
            this.worker.DoWork += new DoWorkEventHandler(this.worker_DoWork);
            this.worker.ProgressChanged += new ProgressChangedEventHandler(this.worker_ProgressChanged);
            this.worker.WorkerSupportsCancellation = true;

            // a bone defined as a line between two joints
            this.bones = Utility.GetBones();

            this.DataContext = this;
            InitializeComponent();
        }

        /// <summary>
        /// なんだっけこれ。。。
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void CompositionTargetRendering(object sender, EventArgs e)
        {
            if (this.isPlaying)
            {
                PlayPause.Content = "||";
            }
            else
            {
                PlayPause.Content = "▷";
            }

        }

        /// <summary>
        /// Windowがロードされたら初期化しまくり
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Window_Loaded(object sender, RoutedEventArgs e)
        {

            this.kinectSensor = KinectSensor.GetDefault();
            this.kinectSensor.Open();
            this.coordinateMapper = kinectSensor.CoordinateMapper;

            CompositionTarget.Rendering += this.CompositionTargetRendering;
            this.LoadFrames();

            this.isUserSelected = new bool[this.frameSequence.recordNum];
            this.isRecordSelected = new bool[this.frameSequence.recordNum];
            
            // player関係
            this.playingIndex = 0;
            this.PlaySlider.Minimum = 0;
            this.PlaySlider.Maximum = this.frameSequence.Frames.Count - 1;
            this.startIndex = 0;
            this.endIndex = this.frameSequence.Frames.Count - 1;
            this.PlaySlider.SelectionStart = this.startIndex;
            this.PlaySlider.SelectionEnd = this.endIndex;

            // 各レコードのスライダー
            this.recordSliders = new Slider[] { RecordSelectSlider1, RecordSelectSlider2, RecordSelectSlider3, RecordSelectSlider4, RecordSelectSlider5 };
            foreach (Slider slider in this.recordSliders)
            {
                slider.Minimum = 0;
                slider.Maximum = this.frameSequence.Frames.Count - 1;
            }
            // キャンバス
            this.canvases = new Canvas[] { RecordSelectCanvas1, RecordSelectCanvas2, RecordSelectCanvas3, RecordSelectCanvas4, RecordSelectCanvas5 };

            // UserIdを選択するUI
            ComboBox[] boxes = { UserIdBox1, UserIdBox2, UserIdBox3, UserIdBox4, UserIdBox5 };
            for (int recordNo = 0; recordNo < frameSequence.recordNum; recordNo++)
            {
                foreach (ulong id in frameSequence.userIdList[recordNo])
                {
                    string idStr = id.ToString();
                    if (frameSequence.Segmentations != null)
                    {
                        Dictionary<ulong, int> map = frameSequence.Segmentations[recordNo].Conversions.Last().Value;
                        if (map.ContainsKey(id))
                        {
                            idStr = map[id].ToString();
                        }
                    }
                    boxes[recordNo].Items.Add(idStr);
                }
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>

        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            //Utility.SaveToBinary(this.frameSequence, @"E:frameseq.dump");
        }

        /// <summary>
        /// BackgroundWorkerの処理内容
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void worker_DoWork(object sender, DoWorkEventArgs e)
        {
            BackgroundWorker bw = (BackgroundWorker)sender;
            int interval = (int)(1000 / this.frameSequence.frameRate);
            
            while (this.isPlaying)
            {
                if (this.playingIndex == this.endIndex)
                {
                    this.isPlaying = false;
                    this.playingIndex = this.startIndex;
                    continue;
                }

                Frame frame = this.frameSequence.Frames[this.playingIndex];
                this.playingIndex++;
                bw.ReportProgress(0, frame);
                System.Threading.Thread.Sleep(interval);
            }
        }

        /// <summary>
        /// BackgroundWorkerの描画系の処理
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void worker_ProgressChanged(object sender, ProgressChangedEventArgs e)
        {
            Frame frame = (Frame)e.UserState;
            this.UpdateDisplay(frame);
        }

        /// <summary>
        /// 画面を更新する
        /// TODO : coordinatemapperを使うように変更(3月)
        /// </summary>
        /// <param name="frame"></param>
        private void UpdateDisplay(Frame frame)
        {
            Label[] timeLabels = { Box1Timer, Box2Timer, Box3Timer, Box4Timer, Box5Timer };
            Image[] images = { Image1, Image2, Image3, Image4, Image5 };
            DrawingGroup[] drawings = { drawingGroup1, drawingGroup2, drawingGroup3, drawingGroup4, drawingGroup5 };
            // 使い回しの変数
            List<Dictionary<JointType, Point>> pointsList;
            List<Dictionary<JointType, Joint>> jointsList;
            List<Tuple<ulong, Point>> idPointList;
            FormattedText fmt;

            for (int recordNo = 0; recordNo < frameSequence.recordNum; recordNo++)
            {
                images[recordNo].Source = new BitmapImage(new Uri(frame.ColorImagePathList[recordNo]));
                timeLabels[recordNo].Content = frame.GetMotionData(recordNo).TimeStamp.ToString(@"ss\:fff");

                // 描画
                using (DrawingContext dc = drawings[recordNo].Open())
                {
                    CvSize colorSize = frame.ColorSize[recordNo];
                    dc.DrawRectangle(Brushes.Transparent, null, new Rect(0.0, 0.0, colorSize.Width, colorSize.Height));
                    pointsList = frame.GetBodyColorSpaceJoints(recordNo);
                    jointsList = frame.GetBodyJoints(recordNo);
                    idPointList = frame.GetIdAndPosition(recordNo);

                    for (int user = 0; user < pointsList.Count(); user++)
                    {
                        // Bodyの描画
                        if (pointsList[user] != null && jointsList[user] != null)
                            this.DrawBody(pointsList[user], jointsList[user], dc);
                        // user id の描画
                        if (idPointList[user] != null)
                        {
                            ulong userId = idPointList[user].Item1;
                            string text = userId.ToString();
                            if (frameSequence.UserMapping.ContainsKey(userId))
                            {
                                text = frameSequence.UserMapping[userId].ToString();
                            }
                            // TODO: 縁取りテキスト, http://gushwell.ldblog.jp/archives/52312432.html
                            fmt = new FormattedText(text,
                                System.Globalization.CultureInfo.CurrentCulture,
                                System.Windows.FlowDirection.LeftToRight,
                                new Typeface("Arial"), 50.0, Brushes.Cyan
                                );
                            dc.DrawText(fmt, idPointList[user].Item2);
                        }
                    }
                    drawings[recordNo].ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, colorSize.Width, colorSize.Height));
                }
            }
            // スライダーとか時間表示
            this.PlaySlider.Value = this.playingIndex;
            this.TimeLabel.Content = frame.Time.ToString(frame.Time.ToString(@"mm\:ss\:fff"));
        }

        /// <summary>
        /// Bodyの関節点、boneを描画する
        /// </summary>
        /// <param name="points"></param>
        /// <param name="joints"></param>
        /// <param name="drawingContext"></param>
        private void DrawBody(Dictionary<JointType, Point> points, Dictionary<JointType, Joint> joints, DrawingContext drawingContext)
        {
            Pen drawingPen;
            // Draw the bones
            foreach (var bone in this.bones)
            {
                if (points.Keys.Contains(bone.Item1) && points.Keys.Contains(bone.Item2))
                {
                    int thickness = 6;
                    if (joints[bone.Item1].TrackingState == TrackingState.Inferred || joints[bone.Item2].TrackingState == TrackingState.Inferred)
                    {
                        thickness = 2;
                    }
                    drawingPen = new Pen(Brushes.Red, thickness);
                    if (bone.Item2.ToString().Contains("Right") || bone.Item1.ToString().Contains("Right"))
                    {
                        drawingPen = new Pen(Brushes.Blue, thickness);
                    }
                    drawingContext.DrawLine(drawingPen, points[bone.Item1], points[bone.Item2]);
                }
            }

            // Draw the joints
            foreach (JointType jointType in points.Keys)
            {
                drawingContext.DrawEllipse(this.trackedJointBrush, null, points[jointType], JointThickness, JointThickness);
            }
        }

        #region ImageSources
        public ImageSource BodyImageSource1
        {
            get
            {
                return this.bodyImageSource1;
            }
        }
        public ImageSource BodyImageSource2
        {
            get
            {
                return this.bodyImageSource2;
            }
        }
        public ImageSource BodyImageSource3
        {
            get
            {
                return this.bodyImageSource3;
            }
        }
        public ImageSource BodyImageSource4
        {
            get
            {
                return this.bodyImageSource4;
            }
        }
        public ImageSource BodyImageSource5
        {
            get
            {
                return this.bodyImageSource5;
            }
        }
        #endregion

        /// <summary>
        /// 再生制御
        /// TODO : Spaceで再生制御
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void PlayPause_Click(object sender, RoutedEventArgs e)
        {
            if (this.isPlaying)
            {
                this.isPlaying = false;
            }
            else
            {
                this.isPlaying = true;
                this.worker.RunWorkerAsync();
            }
        }

        /// <summary>
        /// ユーザ選択UI
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void UserIdBox_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            ComboBox box = (ComboBox)sender;
            int i = box.Name.Length - 1;
            int index = box.Name[i] - '0' - 1;
            if (box.SelectedItem != null)
            {
                // ulongかどうかの判定
                if (box.SelectedItem.ToString().Length > 5)
                {
                    ulong bodyId = ulong.Parse(box.SelectedItem.ToString());
                    this.frameSequence.SetUserID(index, bodyId);
                    this.isUserSelected[index] = true;
                }
                else
                {
                    int bodyId = int.Parse(box.SelectedItem.ToString());
                    this.frameSequence.setIntegratedID(index, bodyId);
                    this.isUserSelected[index] = true;
                }
            }
        }

        /// <summary>
        /// 人間が再生スライダーを動かした時の処理
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void PlaySlider_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            // このイベントは全ての変更にフックされるっぽいので、人間が弄った時に再生止めて動かす、みたいな。
            // playingIndexの値をチェックして、PlaySliderが直接動かされた条件のみ拾う
            if (!this.isPlaying && (int)((Slider)sender).Value != this.playingIndex)
            {
                this.playingIndex = (int)((Slider)sender).Value;
                this.UpdateDisplay(this.frameSequence.Frames[this.playingIndex]);

                // 各スライダーを動かす
                foreach (Slider slider in this.recordSliders)
                {
                    slider.Value = this.playingIndex;
                }
            }
        }

        /// <summary>
        /// 現在のフレームでBoneから統合行列計算
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void MenuCalibBoneFrame_Click(object sender, RoutedEventArgs e)
        {
            Frame frame = frameSequence.Frames[playingIndex];
            List<CvMat> convs;
            if (this.isUserSelected.All(b => b))
            {
                if (this.frameSequence.Segmentations == null)
                {
                    ulong[] selectedUsers = frameSequence.selectedOriginalIdList;
                    convs = KinectMerge.GetConvMatrixFromBoneFrame(frame, frameSequence.ToWorldConversions, selectedUsers);
                }
                else
                {
                    int[] selectedUsers = frameSequence.selecteedIntegretedIdList;
                    convs = KinectMerge.GetConvMatrixFromBoneFrame(frame, frameSequence.ToWorldConversions, selectedUsers);
                }                
                frameSequence.ToWorldConversions = convs;
            }
        }

        /// <summary>
        /// 現在のフレーム範囲でBoneから統合行列計算
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void MenuCalibBoneFrameRange_Click(object sender, RoutedEventArgs e)
        {
            ulong[] selectedUsers = frameSequence.selectedOriginalIdList;
            if (this.isUserSelected.All(b => b))
            {
                List<CvMat> convs = KinectMerge.GetConvMatrixFromBoneFrameSequence(frameSequence, startIndex, endIndex);
                frameSequence.ToWorldConversions = convs;
            }
        }

        /// <summary>
        /// 現在のフレームで深度画像から統合行列計算
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void MenuCalibDepthFrame_Click(object sender, RoutedEventArgs e)
        {
            Frame frame = frameSequence.Frames[playingIndex];
            List<CvMat> convs = KinectMerge.GetConvMatrixFromDepthFrame(frame, frameSequence.ToWorldConversions, frameSequence.LocalCoordinateMappers);
            frameSequence.ToWorldConversions = convs;
        }

        /// <summary>
        /// 現在のフレーム範囲で深度画像から統合行列計算
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void MenuCalibDepthFrameRange_Click(object sender, RoutedEventArgs e)
        {
            List<CvMat> convs = KinectMerge.GetConvMatrixFromDepthFrameSequence(frameSequence, startIndex, endIndex);
            frameSequence.ToWorldConversions = convs;
        }

        /// <summary>
        /// 統合IDの選択UIとかをupdateする
        /// </summary>
        private void UpdateIntegratedIds()
        {
            ComboBox[] boxes = { UserIdBox1, UserIdBox2, UserIdBox3, UserIdBox4, UserIdBox5 };
            for (int recordNo = 0; recordNo < frameSequence.recordNum; recordNo++)
            {
                boxes[recordNo].SelectedItem = null;
                boxes[recordNo].Items.Clear();
                this.isUserSelected[recordNo] = false;
                Dictionary<ulong, int> map = frameSequence.Segmentations[recordNo].Conversions.Last().Value;
                foreach (int id in map.Select(pair => pair.Value).OrderBy(num => num).Distinct())
                {
                    boxes[recordNo].Items.Add(id.ToString());
                }
            }
        }

        /// <summary>
        /// ユーザのセグメンテーションと新IDの割り当て
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Segmentation_Click(object sender, RoutedEventArgs e)
        {
            UserSegmentation[] segm = new UserSegmentation[frameSequence.recordNum];
            for (int i = 0; i < frameSequence.recordNum; i++)
            {
                segm[i] = UserSegmentation.Segment(frameSequence.GetMotionDataSequence(i), new TimeSpan(0, 0, 1));
            }
            frameSequence.Segmentations = segm.ToList();
            // 統合IDの生成
            segm = UserSegmentation.Identification(frameSequence, 1);
            frameSequence.Segmentations = segm.ToList();
            // UserIdを選択するUIのリフレッシュ
            this.UpdateIntegratedIds();
        }

        /// <summary>
        /// 全てのユーザを統合してバイナリで出力する
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void ExportAllBodiesAsBinary_Click(object sender, RoutedEventArgs e)
        {
            if (frameSequence.Segmentations != null)
            {
                var res = SkeletonInterpolator.ExportFromProject(frameSequence, startIndex, endIndex);
                Dictionary<int, List<Dictionary<JointType, CvPoint3D64f>>> mergedBodies = res.Item1;
                foreach (int userId in mergedBodies.Keys)
                {
                    string path = System.IO.Path.Combine(Environment.CurrentDirectory, userId.ToString() + @"_Body.dump");
                    Utility.SaveBodySequence(mergedBodies[userId], path);
                }
                //Utility.SaveToBinary(res.Item2, System.IO.Path.Combine(Environment.CurrentDirectory, @"TimeData.dump"));
            }
        }

        /// <summary>
        /// 選択中のユーザを統合してバイナリで出力する
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void ExportSelectedBodiesAsBinary_Click(object sender, RoutedEventArgs e)
        {
            if (frameSequence.Segmentations != null && this.isUserSelected.All(b => b))
            {
                var res = SkeletonInterpolator.ExportFromProject(frameSequence, startIndex, endIndex);
                string path = System.IO.Path.Combine(Environment.CurrentDirectory, @"SelectedUserBody.dump");
                int id = this.frameSequence.selecteedIntegretedIdList[0];
                if (this.frameSequence.selecteedIntegretedIdList.All(i => i == id))
                {
                    if (res.Item1.Keys.Contains(id))
                    {
                        Utility.SaveBodySequence(res.Item1[id], path);
                        Utility.SaveToBinary(res.Item2[id], System.IO.Path.Combine(Environment.CurrentDirectory, @"TimeData.dump"));
                    }
                }
            }
        }

        /// <summary>
        /// 統合行列を保存する
        /// TODO : ダイアログ
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void ExportConversionMatrix_Click(object sender, RoutedEventArgs e)
        {
            string path = System.IO.Path.Combine(Environment.CurrentDirectory, @"ConversionMatrix.dump");
            List<SerializableCvMat> conversions = frameSequence.ToWorldConversions.Select(mat => SerializableCvMat.CreateOrNull(mat)).ToList();
            Utility.SaveToBinary(conversions, path);
        }

        private void ExportConversionMatrixAsCsv_Click(object sender, RoutedEventArgs e)
        {
            for (int recordNo = 0; recordNo < frameSequence.recordNum; recordNo++)
            {
                string path = System.IO.Path.Combine(Environment.CurrentDirectory, @"ConversionMatrix_" + recordNo.ToString() + ".csv");
                CvMat mat = frameSequence.ToWorldConversions[recordNo];
                using (StreamWriter sw = new StreamWriter(path))
                {                    
                    for (int i = 0; i < 4; i++)
                    {
                        for (int j = 0; j < 4; j++)
                        {
                            double val = mat[i, j];
                            sw.Write("{0}, ", val);
                        }
                        sw.WriteLine();
                    }                    
                }
            }
        }

        /// <summary>
        /// 統合行列を読み込む
        /// TODO : ダイアログ
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void ImportConversionMatrix_Click(object sender, RoutedEventArgs e)
        {
            string path = System.IO.Path.Combine(Environment.CurrentDirectory, @"ConversionMatrix.dump");
            List<SerializableCvMat> conversions = (List<SerializableCvMat>)Utility.LoadFromBinary(path);
            frameSequence.ToWorldConversions = conversions.Select(mat => mat.CreateCvMat()).ToList();
        }

        /// <summary>
        /// レコードを選択する
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void RecordSelect_Clicked(object sender, RoutedEventArgs e)
        {
            CheckBox box = (CheckBox)sender;
            int i = box.Name.Length - 1;
            int index = box.Name[i] - '0' - 1;
            this.isRecordSelected[index] = (bool)box.IsChecked;
        }

        /// <summary>
        /// フレーム範囲の選択中ユーザ・レコードを左右反転する
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void MirrorSelectedRecordRange_Click(object sender, RoutedEventArgs e)
        {
            int[] integratedIds = this.frameSequence.selecteedIntegretedIdList;
            ulong[] originalIds = this.frameSequence.selectedOriginalIdList;
            IEnumerable<Frame> frames = this.frameSequence.Slice(startIndex, endIndex);
            for (int recordNo = 0; recordNo < this.frameSequence.recordNum; recordNo++)
            {
                if (this.isRecordSelected[recordNo] && this.isUserSelected[recordNo])
                {
                    foreach (Frame frame in frames)
                    {
                        if (frameSequence.Segmentations == null)
                        {
                            frame.InverseBody(recordNo, originalId: originalIds[recordNo]);
                        }
                        else
                        {
                            frame.InverseBody(recordNo, integratedId: integratedIds[recordNo]);
                        }
                    }
                }
            }
        }

        /// <summary>
        /// 左右反転をリセットする
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void ResetMirroredBodies_Click(object sender, RoutedEventArgs e)
        {
            foreach (Frame frame in this.frameSequence.Frames)
                frame.ResetInversedBody();
        }

        /// <summary>
        /// 現在のレコード・ユーザで左右反転
        /// </summary>
        private void InverseRecordUser(Frame frame)
        {
            int[] integratedIds = this.frameSequence.selecteedIntegretedIdList;
            ulong[] originalIds = this.frameSequence.selectedOriginalIdList;
            for (int recordNo = 0; recordNo < this.frameSequence.recordNum; recordNo++)
            {
                if (this.isRecordSelected[recordNo] && this.isUserSelected[recordNo])
                {
                    if (frameSequence.Segmentations == null)
                    {
                        frame.InverseBody(recordNo, originalId: originalIds[recordNo]);
                    }
                    else
                    {
                        frame.InverseBody(recordNo, integratedId: integratedIds[recordNo]);
                    }

                }
            }
        }

        /// <summary>
        /// 現在のフレームで選択中のユーザ・レコードに対して左右反転処理
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void MirrorSelectedRecordFrame_Click(object sender, RoutedEventArgs e)
        {
            Frame frame = this.frameSequence.Frames[this.playingIndex];
            this.InverseRecordUser(frame);
        }

        /// <summary>
        /// あるフレームの点群を出力する
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void ExportFrameRangePointClouds_Click(object sender, RoutedEventArgs e)
        {
            //List<Frame> frames = frameSequence.Slice(startIndex, endIndex);
            
            //for (int i = 0; i < frameSequence.recordNum; i++)
            //{
            //    List<float[]>[] pointsSequence = new List<float[]>[frames.Count()];
            //    string path = System.IO.Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.DesktopDirectory), i.ToString() + "_PointsCloud.dump");
            //    for (int frameNo = 0; frameNo < frames.Count(); frameNo++)
            //    {
            //        Frame frame = frames[frameNo];
            //        List<Tuple<CvPoint3D64f, CvColor>> colors = frameSequence.LocalCoordinateMappers[i].DepthColorMatToRealPoints(frame.DepthMatList[i], frame.ColorMatList[i]);
            //        colors = colors.Select(t => Tuple.Create(CvEx.ConvertPoint3D(t.Item1, frameSequence.ToWorldConversions[i]), t.Item2)).ToList();
            //        List<float[]> dumpColors = colors.Select(t => new float[] { (float)t.Item1.X, (float)t.Item1.Y, (float)t.Item1.Z, t.Item2.R, t.Item2.G, t.Item2.B }).ToList();
            //        pointsSequence[frameNo] = dumpColors;
            //    }
            //    Utility.SaveToBinary(pointsSequence, path);
            //}
        }

        /// <summary>
        /// あるフレーム範囲の全ユーザの点群を出力する
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void ExportFrameRangeUserPointClouds_Click(object sender, RoutedEventArgs e)
        {
            //List<Frame> frames = frameSequence.Slice(startIndex, endIndex);
            
            //for (int i = 0; i < frameSequence.recordNum; i++)
            //{
            //    List<List<float[]>> pointsSequence = new List<List<float[]>>();
            //    string path = System.IO.Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.DesktopDirectory), i.ToString() + "_UserPointsCloud.dump");
            //    for (int frameNo = 0; frameNo < frames.Count(); frameNo++)
            //    {
            //        // フレームレート半分
            //        if (frameNo % 2 == 0)
            //            continue;
            //        Frame frame = frames[frameNo];
            //        List<Tuple<CvPoint3D64f, CvColor>> colors = frameSequence.LocalCoordinateMappers[i].GetUserColorPoints(frame.DepthMatList[i], frame.ColorMatList[i], frame.UserMatList[i]);
            //        colors = colors.Select(t => Tuple.Create(CvEx.ConvertPoint3D(t.Item1, frameSequence.ToWorldConversions[i]), t.Item2)).ToList();
            //        List<float[]> dumpColors = colors.Select(t => new float[] { (float)t.Item1.X, (float)t.Item1.Y, (float)t.Item1.Z, t.Item2.R, t.Item2.G, t.Item2.B }).ToList();
            //        // 点の数1/10
            //        dumpColors = dumpColors.Where((fs, index) => index % 10 == 0).ToList();
            //        pointsSequence.Add(dumpColors);
            //    }
            //    Utility.SaveToBinary(pointsSequence.ToArray(), path);
            //}
        }

        /// <summary>
        /// あるフレームの全ユーザの点群を出力する
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void ExportFrameUserPointClouds_Click(object sender, RoutedEventArgs e)
        {
            //Frame frame = frameSequence.Frames[playingIndex];
            //for (int i = 0; i < frameSequence.recordNum; i++)
            //{
            //    List<List<float[]>> pointsSequence = new List<List<float[]>>();
            //    string path = System.IO.Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.DesktopDirectory), i.ToString() + "_UserPointsCloud.dump");

            //    List<Tuple<CvPoint3D64f, CvColor>> colors = frameSequence.LocalCoordinateMappers[i].GetUserColorPoints(frame.DepthMatList[i], frame.ColorMatList[i], frame.UserMatList[i]);
            //    colors = colors.Select(t => Tuple.Create(CvEx.ConvertPoint3D(t.Item1, frameSequence.ToWorldConversions[i]), t.Item2)).ToList();
            //    List<float[]> dumpColors = colors.Select(t => new float[] { (float)t.Item1.X, (float)t.Item1.Y, (float)t.Item1.Z, t.Item2.R, t.Item2.G, t.Item2.B }).ToList();
            //    dumpColors = dumpColors.Where((fs, index) => index % 2 == 0).ToList();
            //    pointsSequence.Add(dumpColors);
            //    Utility.SaveToBinary(pointsSequence.ToArray(), path);
            //}
        }

        /// <summary>
        /// 選択中の統合IDを新しいIDに振りかえる
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void CreateNewIntegratedId_Click(object sender, RoutedEventArgs e)
        {
            if (frameSequence.Segmentations != null && this.isUserSelected.All(b => b))
            {
                frameSequence.CreateNewIds();
                this.UpdateIntegratedIds();
            }
        }

        /// <summary>
        /// レコードの選択範囲をクリア
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void ClearRecordRangeClick(object sender,  RoutedEventArgs e)
        {
            Button button = (Button)sender;
            int index = button.Name[button.Name.Length - 1] - '0' - 1;
            Canvas canvas = this.canvases[index];
            canvas.Children.Clear();
            // フレーム範囲を有効に
            IEnumerable<Frame> frames = frameSequence.Slice(this.startIndex, this.endIndex);
            foreach (Frame frame in frames)
            {
                frame.ResetAllValidFlags();
            }

        }

        /// <summary>
        /// レコードのスライダーが変わったとき
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void RecordSlide_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (!this.isPlaying)
            {
                Slider calledSlider = (Slider)sender;

                // すでにplayingIndexがセットされてる、つまり動かされたのはPlaySliderを除外
                if (this.playingIndex != (int)calledSlider.Value)
                {
                    this.playingIndex = (int)calledSlider.Value;
                    this.PlaySlider.Value = this.playingIndex;
                    this.UpdateDisplay(this.frameSequence.Frames[this.playingIndex]);
                }

                // 他のレコードスライダーに反映、すでにplayingIndexは変更済
                foreach (Slider slider in this.recordSliders)
                {
                    if (slider != calledSlider)
                    {
                        slider.Value = this.playingIndex;
                    }
                }
            }
        }

        /// <summary>
        /// 骨の統計情報を記録しておく
        /// TODO : ボタンを使えなくしてしまえば良いのでは？
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void StoreBonesStatistics_Click(object sender, RoutedEventArgs e)
        {
            BodyStatistics bodyStat = new BodyStatistics();
            IEnumerable<Frame> frames = frameSequence.Slice(this.startIndex, this.endIndex);
            foreach (Frame frame in frames)
            {
                List<SerializableBody> bodies = frame.GetSelectedBodyList(this.frameSequence.selecteedIntegretedIdList);
                if (bodies.Count == 0)
                {
                    continue;
                }
                for (int no = 0; no < frame.recordNum; no++)
                {
                    bodyStat.StoreBoneLength(bodies[no].Joints);
                }
            }
            bodyStat.CalcMedianBoneRange();
            this.frameSequence.BodyStat = bodyStat;
            Utility.SaveToBinary(bodyStat.boneLengthSqStatistics, System.IO.Path.Combine(Environment.CurrentDirectory, @"StatData.dump"));
        }

        /// <summary>
        /// 現在のフレームのレコード・ユーザを信頼できるデータとしてタグ付けする
        /// TODO : UI上で特別なデータは色とかで反映する
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void CheckTrustDataFrame_Click(object sender, RoutedEventArgs e)
        {
            // record, userを選択中のやつ
            for (int recordNo = 0; recordNo < frameSequence.recordNum; recordNo++)
            {
                if (this.isRecordSelected[recordNo])
                {
                    int record = recordNo;
                    int user = this.frameSequence.selecteedIntegretedIdList[recordNo];
                    int frame = this.playingIndex;
                    TrustData td = new TrustData(frame, record, user);
                    // こいつをUI上で反映する手がかりにする
                    SerializableBody body = td.GetBody(this.frameSequence.Frames);
                    body.trustData = true;
                    this.frameSequence.trustData.Add(td);
                }
            }

        }

        /// <summary>
        /// 骨格を修正するボタン
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void CorrectMirroredJoints_Click(object sender, RoutedEventArgs e)
        {
            if (this.isUserSelected.All(b => b))
            {
                int userId = this.frameSequence.selecteedIntegretedIdList[0];
                if (this.frameSequence.selecteedIntegretedIdList.All(i => i == userId))
                {
                    JointCorrection jc = new JointCorrection();
                    jc.Correct(this.frameSequence);
                }
                else
                {
                    System.Windows.MessageBox.Show("選択ユーザが一致していません");
                }
            }
        }

        /// <summary>
        /// スライダーのマウスが押されたとき
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void PlaySlider_MouseDown(object sender, MouseButtonEventArgs e)
        {
        }

        /// <summary>
        /// スライダーのマウスが離されたとき
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void PlaySlider_MouseUp(object sender, MouseButtonEventArgs e)
        {
        }

        private double startX;
        private Rectangle rect;
        private int recordInvalidStart;

        /// <summary>
        /// レコードのスライダーでのマウスダウン
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void CanvasMouseDown(object sender, MouseButtonEventArgs e)
        {
        }

        /// <summary>
        /// レコードのスライダーでのマウス操作
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void CanvasMouseMove(object sender, MouseEventArgs e)
        {
        }

        /// <summary>
        /// レコードのスライダーでのマウスアップ
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void CanvasMouseUp(object sender, MouseButtonEventArgs e)
        {
        }

        bool IsSeeking = false;
        List<int> frameInverseList = new List<int>();
        List<int> frameRemoveList = new List<int>();
        private void Window_KeyDown(object sender, KeyEventArgs e)
        {
            // select
            if (e.Key == Key.S)
            {
                var focusedElement = FocusManager.GetFocusedElement(this);
                // select処理の上のスライダー
                if (IsSeeking == false && focusedElement == this.PlaySlider)
                {
                    this.startIndex = (int)this.PlaySlider.Value;
                    this.PlaySlider.SelectionStart = this.startIndex;

                    foreach (Slider slider in this.recordSliders)
                    {
                        slider.Minimum = this.startIndex;
                    }
                    IsSeeking = true;
                }
                // select処理のrecordスライダー
                else if (IsSeeking == false && focusedElement is Slider)
                {
                    Slider slider = (Slider)focusedElement;
                    int index = slider.Name[slider.Name.Length - 1] - '0' - 1;
                    startX = Mouse.GetPosition(slider).X;//slider.ActualWidth * slider.Value / (double)(slider.Maximum - slider.Minimum);
                    rect = new Rectangle { Stroke = Brushes.OrangeRed, StrokeThickness = 6 };
                    Canvas.SetLeft(rect, startX);
                    Canvas.SetTop(rect, 0);
                    this.canvases[index].Children.Add(rect);
                    this.recordInvalidStart = (int)slider.Value;
                    IsSeeking = true;
                }
                // select処理のrecordスライダー塗りつぶし
                else if (IsSeeking == true && focusedElement is Slider)
                {
                    if (rect == null)
                        return;
                    Slider slider = (Slider)focusedElement;
                    double PosX = Mouse.GetPosition(slider).X;//slider.ActualWidth * slider.Value / (double)(slider.Maximum - slider.Minimum);
                    double x = Math.Min(PosX, startX);
                    double w = Math.Max(PosX, startX) - x;
                    rect.Width = w;
                    rect.Height = 5;
                    Canvas.SetLeft(rect, x);
                    Canvas.SetTop(rect, 0);
                }
            }
            // reverse
            else if (e.Key == Key.R)
            {
                this.frameInverseList.Add(this.playingIndex);
            }
            // delete upper body or legs
            else if (e.Key == Key.U || e.Key == Key.D)
            {
                this.frameRemoveList.Add(this.playingIndex);
            }
        }

        private void Window_KeyUp(object sender, KeyEventArgs e)
        {
            // select
            if (e.Key == Key.S)
            {
                var focusedElement = FocusManager.GetFocusedElement(this);
                // 上のスライダー
                if (IsSeeking == true && focusedElement == this.PlaySlider)
                {
                    this.endIndex = (int)this.PlaySlider.Value;
                    this.PlaySlider.SelectionEnd = this.endIndex;

                    foreach (Slider slider in this.recordSliders)
                    {
                        slider.Maximum = this.endIndex;
                    }
                    IsSeeking = false;
                }
                // レコードスライダー
                else if (IsSeeking == true && focusedElement is Slider)
                {
                    rect = null;
                    Slider slider = (Slider)focusedElement;
                    int index = slider.Name[slider.Name.Length - 1] - '0' - 1;
                    // フレーム範囲を無効に
                    IEnumerable<Frame> frames = frameSequence.Slice(this.recordInvalidStart, (int)slider.Value);
                    foreach (Frame frame in frames)
                    {
                        frame.SetDataNotValid(index);
                    }
                    // 初期化
                    this.recordInvalidStart = this.frameSequence.Frames.Count;
                    IsSeeking = false;
                }
            }
            // reverse
            else if (e.Key == Key.R)
            {
                IEnumerable<int> range = Enumerable.Range(frameInverseList.Min(), (frameInverseList.Max() - frameInverseList.Min() + 1));
                foreach (int frameIndex in range)
                {
                    Frame frame = this.frameSequence.Frames[frameIndex];
                    this.InverseRecordUser(frame);
                }
                frameInverseList.Clear();
            }
            // delete upper body or legs
            else if (e.Key == Key.U || e.Key == Key.D)
            {
                IEnumerable<int> range = Enumerable.Range(frameRemoveList.Min(), (frameRemoveList.Max() - frameRemoveList.Min() + 1));
                foreach (int frameIndex in range)
                {
                    for (int recordNo = 0; recordNo < frameSequence.recordNum; recordNo++)
                    {
                        if (this.isRecordSelected[recordNo])
                        {
                            SerializableBody body = this.frameSequence.Frames[frameIndex].GetSelectedBody(recordNo, frameSequence.selecteedIntegretedIdList[recordNo]);
                            if (body != null)
                            {
                                if (e.Key == Key.U)
                                {
                                    body.RemoveJoints(Utility.UpperBody);
                                }
                                else if (e.Key == Key.D)
                                {
                                    body.RemoveJoints(Utility.Legs);
                                }
                            }
                        }
                    }
                }
                frameRemoveList.Clear();
            }
        }
    }
}
