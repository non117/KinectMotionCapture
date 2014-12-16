using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Linq;
using System.IO;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Forms;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using Microsoft.Kinect;

using OpenCvSharp;
using OpenCvSharp.Extensions;

namespace KinectMotionCapture
{
    using PointsPair = Tuple<Dictionary<JointType, Point>, Dictionary<JointType, Point>>;
    public partial class RecordWindow : Window, INotifyPropertyChanged
    {
        // 基本設定
        private int counter = 0;
        private string recordPath = Path.Combine(Environment.CurrentDirectory, Properties.Settings.Default.RecordDirectoryName);
        
        // Kinect関連
        private KinectSensor kinectSensor = null;
        private CoordinateMapper coordinateMapper = null;
        private MultiSourceFrameReader multiFrameSourceReader = null;

        // 描画用
        private DrawingGroup drawingGroup;
        private DrawingImage imageSource;
        public WriteableBitmap colorBitmap = null;

        // データ貯めるやつ
        private readonly int bytesPerPixel = (PixelFormats.Bgr32.BitsPerPixel + 7) / 8;
        private byte[] colorPixels = null;
        private byte[] bodyIndexBuffer = null;
        private ushort[] depthBuffer = null;

        // bone描画関連
        private Body[] bodies = null;
        private List<Tuple<JointType, JointType>> bones;
        private List<Pen> bodyColors;
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));
        private readonly Brush inferredJointBrush = Brushes.Yellow;
        private const double JointThickness = 3;

        // 画面の大きさ
        private int depthWidth = 0;
        private int depthHeight = 0;
        private int colorWidth = 0;
        private int colorHeight = 0;

        // 記録再生制御とか
        private bool isRecording = false;

        // Model
        private MotionDataHandler motionDataHandler = null;

        private string statusText = null;

        public RecordWindow()
        {
            // 基本設定の初期化処理

            // Kinect関連初期化処理
            this.kinectSensor = KinectSensor.GetDefault();
            this.multiFrameSourceReader = this.kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Depth | FrameSourceTypes.Color | FrameSourceTypes.Body | FrameSourceTypes.BodyIndex);
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            FrameDescription deapthFrameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;
            FrameDescription colorFrameDescription = this.kinectSensor.ColorFrameSource.FrameDescription;

            this.depthWidth = deapthFrameDescription.Width;
            this.depthHeight = deapthFrameDescription.Height;
            this.colorWidth = colorFrameDescription.Width;
            this.colorHeight = colorFrameDescription.Height;

            this.motionDataHandler = new MotionDataHandler(this.recordPath , this.colorWidth, this.colorHeight, this.depthWidth, this.depthHeight);            

            // 描画関連
            this.drawingGroup = new DrawingGroup();
            this.imageSource = new DrawingImage(this.drawingGroup);
            this.colorBitmap = new WriteableBitmap(this.colorWidth, this.colorHeight, 96.0, 96.0, PixelFormats.Bgra32, null);

            // allocate space to put the pixels being received
            this.colorPixels = new byte[this.colorWidth * this.colorHeight * this.bytesPerPixel];
            this.depthBuffer = new ushort[this.depthWidth * this.depthHeight];
            this.bodyIndexBuffer = new byte[this.depthWidth * this.depthHeight];

            // a bone defined as a line between two joints
            this.bones = new List<Tuple<JointType, JointType>>();

            // Torso
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // Right Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            // Left Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            // Right Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            // Left Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));

            // populate body colors, one for each BodyIndex
            this.bodyColors = new List<Pen>();
            this.bodyColors.Add(new Pen(Brushes.Red, 6));
            this.bodyColors.Add(new Pen(Brushes.Orange, 6));
            this.bodyColors.Add(new Pen(Brushes.Green, 6));
            this.bodyColors.Add(new Pen(Brushes.Blue, 6));
            this.bodyColors.Add(new Pen(Brushes.Indigo, 6));
            this.bodyColors.Add(new Pen(Brushes.Violet, 6));

            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;
            this.kinectSensor.Open();

            this.DataContext = this;

            this.InitializeComponent();
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        private void OnPropertyChanged(string name)
        {
            if (this.PropertyChanged != null)
            {
                this.PropertyChanged(this, new PropertyChangedEventArgs(name));
            }

        }

        public ImageSource ImageSource
        {
            get
            {
                return this.imageSource;
            }
        }

        public ImageSource BackgroundImage
        {
            get
            {
                return this.colorBitmap;
            }
        }

        public string RecordPath
        {
            get
            {
                return this.recordPath;
            }
        }

        private void OpenFolderButton_Click(object sender, RoutedEventArgs e)
        {
            FolderBrowserDialog fbd = new FolderBrowserDialog();
            fbd.Description = "フォルダを選択してください";
            fbd.SelectedPath = this.recordPath;
            if (fbd.ShowDialog() == System.Windows.Forms.DialogResult.OK )
            {
                this.recordPath = fbd.SelectedPath;
                this.motionDataHandler.DataDir = fbd.SelectedPath;
                this.OnPropertyChanged("RecordPath");
            }
        }
        
        public void RecordButton_Click(object sender, RoutedEventArgs e)
        {
            if (this.isRecording)
            {
                //this.motionDataHandler.Flush();
                this.motionDataHandler.CloseFile();
                this.counter = 0;

                RecordButton.Content = "Record";
                this.isRecording = false;
                this.StatusText = Properties.Resources.RecordingReadyStatusText;
            }else
            {
                RecordButton.Content = "Stop";
                //this.motionDataHandler.ClearAll();
                this.isRecording = true;
                this.StatusText = Properties.Resources.RecordingStatusText;                
            }
        }

        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;
                    this.OnPropertyChanged("StatusText");
                }
            }
        }

        /// <summary>
        /// Execute start up tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            if (this.multiFrameSourceReader != null)
            {
                this.multiFrameSourceReader.MultiSourceFrameArrived += this.Reader_FrameArrived;
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>

        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.multiFrameSourceReader != null)
            {
                this.multiFrameSourceReader.Dispose();
                this.multiFrameSourceReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }

        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            int depthWidth = 0;
            int depthHeight = 0;
            int colorWidth = 0;
            int colorHeight = 0;
            int bodyIndexWidth = 0;
            int bodyIndexHeight = 0;
            
            bool multiSourceFrameProcessed = false;
            bool colorFrameProcessed = false;
            bool depthFrameProcessed = false;
            bool bodyFrameProcessed = false;
            bool bodyIndexFrameProcessed = false;

            // coordinatemapperのルックアップテーブルがどのタイミングで生成されるのかよくわからんので、とりあえずここ
            if (this.motionDataHandler.DepthLUT == null)
            {
                this.motionDataHandler.DepthLUT = this.coordinateMapper.GetDepthFrameToCameraSpaceTable();
                //TODO: world to colorの式を予想する
            }

            MultiSourceFrame multiSourceFrame = e.FrameReference.AcquireFrame();

            if (multiSourceFrame != null)
            {
                using (DepthFrame depthFrame = multiSourceFrame.DepthFrameReference.AcquireFrame())                
                using (ColorFrame colorFrame = multiSourceFrame.ColorFrameReference.AcquireFrame())
                using (BodyFrame bodyFrame = multiSourceFrame.BodyFrameReference.AcquireFrame())
                using (BodyIndexFrame bodyIndexFrame = multiSourceFrame.BodyIndexFrameReference.AcquireFrame())
                {
                    if (depthFrame != null)
                    {
                        FrameDescription depthFrameDescription = depthFrame.FrameDescription;
                        depthWidth = depthFrameDescription.Width;
                        depthHeight = depthFrameDescription.Height;
                        if ((depthWidth * depthHeight) == this.depthBuffer.Length)
                        {
                            depthFrame.CopyFrameDataToArray(this.depthBuffer);
                            depthFrameProcessed = true;
                        }
                    }
                    if (bodyIndexFrame != null)
                    {
                        FrameDescription bodyFrameDescription = bodyIndexFrame.FrameDescription;
                        bodyIndexWidth = bodyFrameDescription.Width;
                        bodyIndexHeight = bodyFrameDescription.Height;
                        if ((bodyIndexWidth * bodyIndexHeight) == this.bodyIndexBuffer.Length)
                        {
                            bodyIndexFrame.CopyFrameDataToArray(this.bodyIndexBuffer);
                            bodyIndexFrameProcessed = true;
                        }
                        
                    }
                    if (colorFrame != null)
                    {
                        FrameDescription colorFrameDescription = colorFrame.FrameDescription;
                        colorWidth = colorFrameDescription.Width;
                        colorHeight = colorFrameDescription.Height;
                        if ((colorWidth == this.colorBitmap.PixelWidth) && (colorHeight == this.colorBitmap.PixelHeight))
                        {
                            if (colorFrame.RawColorImageFormat == ColorImageFormat.Bgra)
                            {
                                colorFrame.CopyRawFrameDataToArray(this.colorPixels);
                            }
                            else
                            {
                                colorFrame.CopyConvertedFrameDataToArray(this.colorPixels, ColorImageFormat.Bgra);
                            }
                            colorFrameProcessed = true;
                        }
                    }
                    if (bodyFrame != null)
                    {
                        if (this.bodies == null)
                        {
                            this.bodies = new Body[bodyFrame.BodyCount];
                        }
                        bodyFrame.GetAndRefreshBodyData(this.bodies);
                        bodyFrameProcessed = true;

                    }
                    multiSourceFrameProcessed = true;
                }
            }

            // we got all frames
            if (multiSourceFrameProcessed && depthFrameProcessed && colorFrameProcessed && bodyFrameProcessed && bodyIndexFrameProcessed)
            {
                Dictionary<ulong, PointsPair> pointPairs = new Dictionary<ulong, PointsPair>();
                this.RenderColorPixels();
                using (DrawingContext dc = this.drawingGroup.Open())
                {                    
                    dc.DrawRectangle(Brushes.Transparent, null, new Rect(0.0, 0.0, this.colorWidth, this.colorHeight));
                    int penIndex = 0;
                    
                    foreach (Body body in this.bodies)
                    {
                        Pen drawPen = this.bodyColors[penIndex++];
                        if (body.IsTracked)
                        {
                            Dictionary<JointType, Joint> joints = (Dictionary<JointType, Joint>)body.Joints;
                            PointsPair pointsPair = this.ConvertCameraPoint(joints);
                            pointPairs[body.TrackingId] = pointsPair;
                            
                            Dictionary<JointType, Point> colorPoints = pointsPair.Item1;
                            this.DrawBody(joints, colorPoints, dc, drawPen);
                        }
                    }
                    this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.colorWidth, this.colorHeight));
                }
                if (this.isRecording)
                {
                    Task.Run(() => this.motionDataHandler.AddData(this.counter++, DateTime.Now, this.bodies, ref this.colorPixels, ref this.depthBuffer, ref this.bodyIndexBuffer, pointPairs));
                }
            }
        }

        /// <summary>
        /// jointsを受け取ってカラー画像 / 深度画像の2次元座標系に変換する
        /// </summary>
        /// <param name="joints"></param>
        /// <returns></returns>
        private PointsPair ConvertCameraPoint(Dictionary<JointType, Joint> joints)
        {
            Dictionary<JointType, Point> colorPoints = new Dictionary<JointType, Point>();
            Dictionary<JointType, Point> depthPoints = new Dictionary<JointType, Point>();
            foreach (JointType jointType in joints.Keys)
            {
                CameraSpacePoint position = joints[jointType].Position;
                if (position.Z < 0)
                {
                    position.Z = 0.1f;
                }
                ColorSpacePoint colorSpacePoint = this.coordinateMapper.MapCameraPointToColorSpace(position);
                colorPoints[jointType] = new Point(colorSpacePoint.X, colorSpacePoint.Y);
                DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                depthPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
            }
            return Tuple.Create(colorPoints, depthPoints);
        }

        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(IDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
        {
            // Draw the bones
            foreach (var bone in this.bones)
            {
                if (bone.Item2.ToString().Contains("Right") || bone.Item1.ToString().Contains("Right"))
                {
                    this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, new Pen(Brushes.WhiteSmoke, 6));
                }
                else
                {
                    this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
                }
            }

            // Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
                Brush drawBrush = null;

                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                }
            }
        }

        /// <summary>
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        private void DrawBone(IDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }

            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }

        /// <summary>
        /// Renders color pixels into the writeableBitmap.
        /// </summary>
        private void RenderColorPixels()
        {
            this.colorBitmap.WritePixels(
                new Int32Rect(0, 0, this.colorBitmap.PixelWidth, this.colorBitmap.PixelHeight),
                this.colorPixels,
                this.colorBitmap.PixelWidth * this.bytesPerPixel,
                0);
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RecordingReadyStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }

        /// <summary>
        /// LocalCoordinateMapperをdumpするやつ
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void DumpCalibButton_Click(object sender, RoutedEventArgs e)
        {
            LocalCoordinateMapper lcm = new LocalCoordinateMapper(this.coordinateMapper, this.depthWidth, this.depthHeight);
            string path = Path.Combine(Path.GetDirectoryName(this.recordPath), "coordmap.dump");
            lcm.dump(path);

            CameraIntrinsics cameraIntrinsics = this.coordinateMapper.GetDepthCameraIntrinsics();
            string path2 = Path.Combine(Path.GetDirectoryName(this.recordPath), "CameraInfo.dump");
            Utility.SaveToBinary(cameraIntrinsics, path2);
        }

    }
}
