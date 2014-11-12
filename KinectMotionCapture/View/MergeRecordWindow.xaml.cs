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
using MsgPack.Serialization;

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
        private BackgroundWorker worker;
        private bool isPlaying;
        private DrawingGroup drawingGroup;
        private DrawingImage bodyImageSource1;
        private List<Tuple<JointType, JointType>> bones;

        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));
        private readonly Brush inferredJointBrush = Brushes.Yellow;
        private const double JointThickness = 3;

        private void NewProject()
        {
            List<string> datadir = new List<string>() {
                                                    @"E:\kinect1",  
                                                    @"E:\kinect2", 
                                                    @"E:\kinect3", 
                                                    @"E:\kinect4", 
            };
            this.frameSequence = new FrameSequence(datadir);
            this.Initialize();
        }

        public MergeRecordWindow()
        {
            this.drawingGroup = new DrawingGroup();
            this.bodyImageSource1 = new DrawingImage(this.drawingGroup);

            this.worker = new BackgroundWorker();
            this.worker.WorkerReportsProgress = true;
            this.worker.DoWork += new DoWorkEventHandler(this.worker_DoWork);
            this.worker.ProgressChanged += new ProgressChangedEventHandler(this.worker_ProgressChanged);
            this.worker.WorkerSupportsCancellation = true;

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

            InitializeComponent();
        }

        private void CompositionTargetRendering(object sender, EventArgs e)
        {
            if (this.isPlaying)
            {
                PlayStop.Content = "□";
            }
            else
            {
                PlayStop.Content = "▷";
            }

        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            CompositionTarget.Rendering += this.CompositionTargetRendering;
            this.NewProject();
            this.frameSequence.LocalCoordinateMapper = (LocalCoordinateMapper)Utility.LoadFromBinary(@"C:\Users\non\Desktop\coordmapper.dump");
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

        private void Initialize()
        {
            ComboBox[] boxes = { UserIdBox1, UserIdBox2, UserIdBox3, UserIdBox4 };
            for (int i = 0; i < frameSequence.recordNum; i++)
            {
                foreach (ulong id in frameSequence.userIdList[i])
                {
                    boxes[i].Items.Add(id);
                }
            }
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
            foreach (Frame frame in frameSequence.Frames)
            {
                if (this.isPlaying)
                {
                    bw.ReportProgress(0, frame);
                    System.Threading.Thread.Sleep(interval);
                }
                else
                {
                    return;
                }
            }
            this.isPlaying = false;
        }

        /// <summary>
        /// BackgroundWorkerの描画系の処理
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void worker_ProgressChanged(object sender, ProgressChangedEventArgs e)
        {
            Frame frame = (Frame)e.UserState;
            Label[] labels = { UserIdLabel1, UserIdLabel2, UserIdLabel3, UserIdLabel4 };
            //Image[] images = { Image1, Image2, Image3, Image4 };
            for (int i = 0; i < frameSequence.recordNum; i++)
            {
                labels[i].Content = String.Join(",", frame.BodyIdList(i));
                //images[i].Source = new BitmapImage(new Uri(frame.ColorImagePathList[i]));
            }

            using (DrawingContext dc = this.drawingGroup.Open())
            {
                CvSize colorSize = frame.ColorSize[0];
                dc.DrawRectangle(Brushes.Transparent, null, new Rect(0.0, 0.0, colorSize.Width, colorSize.Height));

                foreach (Dictionary<JointType, Point> points in frame.GetBodyPoints(0))
                {
                    Pen drawPen = new Pen(Brushes.Red, 6);
                    this.DrawBody(points, dc, drawPen);
                }
                this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, colorSize.Width, colorSize.Height));
            }

        }

        private void DrawBody(Dictionary<JointType, Point> points, DrawingContext drawingContext, Pen drawingPen)
        {
            // Draw the bones
            foreach (var bone in this.bones)
            {
                if (points.Keys.Contains(bone.Item1) && points.Keys.Contains(bone.Item2))
                {
                    drawingContext.DrawLine(drawingPen, points[bone.Item1], points[bone.Item2]);
                }
            }

            // Draw the joints
            foreach (JointType jointType in points.Keys)
            {
                drawingContext.DrawEllipse(this.trackedJointBrush, null, points[jointType], JointThickness, JointThickness);
            }
        }

        public ImageSource BodyImageSource1
        {
            get
            {
                return this.bodyImageSource1;
            }
        }

        /// <summary>
        /// 再生制御
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void PlayStop_Click(object sender, RoutedEventArgs e)
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

        private void UserIdBox_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            ComboBox box = (ComboBox)sender;
            int i = box.Name.Length - 1;
            int index = box.Name[i] - '0' - 1;
            ulong bodyId = (ulong)box.SelectedItem;
            this.frameSequence.SetUserID(index, bodyId);
        }
    }
}
