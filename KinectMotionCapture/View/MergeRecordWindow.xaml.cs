﻿using System;
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
        //private List<Frame> frameList;
        
        private BackgroundWorker worker;

        private int playingIndex;
        private int startIndex;
        private int endIndex;
        private bool isPlaying;


        private DrawingGroup drawingGroup1;
        private DrawingImage bodyImageSource1;
        private DrawingGroup drawingGroup2;
        private DrawingImage bodyImageSource2;
        private DrawingGroup drawingGroup3;
        private DrawingImage bodyImageSource3;
        private DrawingGroup drawingGroup4;
        private DrawingImage bodyImageSource4;

        private List<Tuple<JointType, JointType>> bones;

        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));
        private readonly Brush inferredJointBrush = Brushes.Yellow;
        private const double JointThickness = 3;

        private void LoadFrames()
        {
            string[] datadir = new string[] {
                                                    @"F:\kinect1",  
                                                    @"F:\kinect2", 
                                                    @"F:\kinect3", 
                                                    @"F:\kinect4", 
            };
            List<string> mapdir = new List<string>() {
                                                    @"F:\kinect1_coordmap.dump",  
                                                    @"F:\kinect2_coordmap.dump", 
                                                    @"F:\kinect3_coordmap.dump", 
                                                    @"F:\kinect4_coordmap.dump", 
            };
            //string cameradir = @"E:\CameraInfo.dump";
            this.frameSequence = new FrameSequence(datadir);
            this.frameSequence.LocalCoordinateMappers = mapdir.Select(s => (LocalCoordinateMapper)Utility.LoadFromBinary(s)).ToList();
            //this.frameSequence.CameraInfo = (CameraIntrinsics)Utility.LoadFromBinary(cameradir);
        }

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

            this.DataContext = this;
            InitializeComponent();
        }

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

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            CompositionTarget.Rendering += this.CompositionTargetRendering;
            this.LoadFrames();
            
            // player関係
            this.playingIndex = 0;
            this.PlaySlider.Minimum = 0;
            this.PlaySlider.Maximum = this.frameSequence.Frames.Count() - 1;
            this.startIndex = 0;
            this.endIndex = this.frameSequence.Frames.Count() - 1;
            this.PlaySlider.SelectionStart = this.startIndex;
            this.PlaySlider.SelectionEnd = this.endIndex;

            // UserIdを選択するUI, あとで同様の処理で更新される
            ComboBox[] boxes = { UserIdBox1, UserIdBox2, UserIdBox3, UserIdBox4 };
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
        /// </summary>
        /// <param name="frame"></param>
        private void UpdateDisplay(Frame frame)
        {
            Label[] timeLabels = { Box1Timer, Box2Timer, Box3Timer, Box4Timer };
            Image[] images = { Image1, Image2, Image3, Image4 };
            DrawingGroup[] drawings = { drawingGroup1, drawingGroup2, drawingGroup3, drawingGroup4 };
            // 使い回しの変数
            List<Dictionary<JointType, Point>> pointsList;
            List<Dictionary<JointType, Joint>> jointsList;
            List<Tuple<ulong, Point>> idPointList;
            FormattedText fmt;

            for (int recordNo = 0; recordNo < frameSequence.recordNum; recordNo++)
            {
                images[recordNo].Source = new BitmapImage(new Uri(frame.ColorImagePathList[recordNo]));
                timeLabels[recordNo].Content = frame.GetMotionData(recordNo).TimeStamp.ToString(@"ss\:fff");

                // Boneの描画
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
                        this.DrawBody(pointsList[user], jointsList[user], dc);
                        // user id の描画
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

        private void UserIdBox_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            ComboBox box = (ComboBox)sender;
            int i = box.Name.Length - 1;
            int index = box.Name[i] - '0' - 1;
            if (box.SelectedItem.ToString().Length > 5)
            {
                ulong bodyId = (ulong)box.SelectedItem;
                this.frameSequence.SetUserID(index, bodyId);
            }
            else
            {
                
            }
        }

        private void PlaySlider_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            // このイベントは全ての変更にフックされるっぽいので、人間が弄った時に再生止めて動かす、みたいな。
            if (!this.isPlaying)
            {
                this.playingIndex = (int)((Slider)sender).Value;
                this.UpdateDisplay(this.frameSequence.Frames[this.playingIndex]);
            }
        }

        private void SetMinTime_Click(object sender, RoutedEventArgs e)
        {
            this.startIndex = (int)this.PlaySlider.Value;
            this.PlaySlider.SelectionStart = this.startIndex;
        }

        private void SetMaxTime_Click(object sender, RoutedEventArgs e)
        {           
            this.endIndex = (int)this.PlaySlider.Value;
            this.PlaySlider.SelectionEnd = this.endIndex;
        }

        private void MenuCalibBoneFrame_Click(object sender, RoutedEventArgs e)
        {
            Frame frame = frameSequence.Frames[playingIndex];
            ulong[] selectedUsers = frameSequence.selectedUserIdList;
            if (selectedUsers.Length == frameSequence.recordNum)
            {
                List<CvMat> convs = KinectMerge.GetConvMatrixFromBoneFrame(frame, frameSequence.ToWorldConversions, selectedUsers);
                frameSequence.ToWorldConversions = convs;
            }
        }

        private void MenuCalibBoneFrameRange_Click(object sender, RoutedEventArgs e)
        {
            ulong[] selectedUsers = frameSequence.selectedUserIdList;
            if (selectedUsers.Length == frameSequence.recordNum)
            {
                List<CvMat> convs = KinectMerge.GetConvMatrixFromBoneFrameSequence(frameSequence, startIndex, endIndex);
                frameSequence.ToWorldConversions = convs;
            }
            //var mergedBodies = SkeletonInterpolator.ExportFromProject(frameSequence);

            // DEBUG
            List<Frame> frames = frameSequence.Slice(this.startIndex, this.endIndex);
            for (int i = 0; i < frameSequence.recordNum; i++)
            {
                Dictionary<JointType, Joint> joints = Utility.ApplyConversions(frames[0].GetMotionData(i).bodies[0].Joints, frameSequence.ToWorldConversions[i]);
                CameraSpacePoint p = joints[JointType.SpineBase].Position;
                Debug.WriteLine(string.Format("{0},{1},{2}", p.X, p.Y, p.Z));

                List<Dictionary<JointType, Joint>> body = frames.Select(
                    f => Utility.ApplyConversions(f.GetMotionData(i).bodies[0].Joints, frameSequence.ToWorldConversions[i])).ToList();
                //Utility.SaveBodySequence(body, string.Format(@"C:\Users\non\Desktop\joints{0}.dump", i + 1));
            }
        }

        private void MenuCalibDepthFrame_Click(object sender, RoutedEventArgs e)
        {
            Frame frame = frameSequence.Frames[playingIndex];
            List<CvMat> convs = KinectMerge.GetConvMatrixFromDepthFrame(frame, frameSequence.ToWorldConversions, frameSequence.LocalCoordinateMappers);
            frameSequence.ToWorldConversions = convs;
        }

        private void MenuCalibDepthFrameRange_Click(object sender, RoutedEventArgs e)
        {
            List<CvMat> convs = KinectMerge.GetConvMatrixFromDepthFrameSequence(frameSequence, startIndex, endIndex);
            frameSequence.ToWorldConversions = convs;

        }

        private void Segmentation_Click(object sender, RoutedEventArgs e)
        {
            UserSegmentation[] segm = new UserSegmentation[frameSequence.recordNum];
            for (int i = 0; i < frameSequence.recordNum; i++)
            {
                segm[i] = UserSegmentation.Segment(frameSequence.GetMotionDataSequence(i), new TimeSpan(0, 0, 1));
            }
            frameSequence.Segmentations = segm.ToList();
            segm = UserSegmentation.Identification(frameSequence, 1);
            frameSequence.Segmentations = segm.ToList();
        }
    }
}
