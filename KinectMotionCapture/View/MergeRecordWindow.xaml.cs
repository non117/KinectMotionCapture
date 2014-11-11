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

        private void NewProject()
        {
            List<string> datadir = new List<string>() {
                                                    @"F:\1015\1015_kinect1\master",  
                                                    @"F:\1015\1015_kinect2\master", 
                                                    @"F:\1015\1015_kinect3\master", 
                                                    @"F:\1015\1015_kinect4\master", 
            };
            this.frameSequence = new FrameSequence(datadir);
            this.Initialize();
        }

        public MergeRecordWindow()
        {
            this.NewProject();
            this.frameSequence.LocalCoordinateMapper = (LocalCoordinateMapper)Utility.LoadFromBinary(@"C:\Users\non\Desktop\coordmapper.dump");

            this.worker = new BackgroundWorker();
            this.worker.WorkerReportsProgress = true;
            this.worker.DoWork += new DoWorkEventHandler(this.worker_DoWork);
            this.worker.ProgressChanged += new ProgressChangedEventHandler(this.worker_ProgressChanged);
            this.worker.WorkerSupportsCancellation = true;

            InitializeComponent();
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>

        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            Utility.SaveToBinary(this.frameSequence, @"C:\Users\non\Desktop\frameSeq.dump");
        }

        private void Initialize()
        {
            ComboBox[] boxes = { UserIdBox1, UserIdBox2, UserIdBox3, UserIdBox4 };
            for (int i = 0; i < frameSequence.recordNum; i++)
            {
                boxes[i].Items.Add(frameSequence.userIdList[i]);
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
            PlayStop.Content = "Play";
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
            Image[] images = { Image1, Image2, Image3, Image4 };
            for (int i = 0; i < frameSequence.recordNum; i++)
            {
                labels[i].Content = String.Join(",", frame.BodyIdList(i));
                images[i].Source = new BitmapImage(new Uri(frame.ColorImagePathList[i]));
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
                PlayStop.Content = "Play";
            }
            else
            {
                this.isPlaying = true;
                PlayStop.Content = "Stop";
                this.worker.RunWorkerAsync();
            }
        }

        private void UserIdBox_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            ComboBox box = (ComboBox)sender;
            int index = int.Parse(box.Name.Substring(box.Name.Length - 2));
            ulong bodyId = (ulong)((ComboBoxItem)box.SelectedItem).Content;
            this.frameSequence.SetUserID(index, bodyId);
        }
    }
}
