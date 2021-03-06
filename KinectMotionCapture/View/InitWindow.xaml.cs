﻿using System;
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

using OpenCvSharp;
using Microsoft.Kinect;

namespace KinectMotionCapture
{
    using Bone = Tuple<JointType, JointType>;
    /// <summary>
    /// InitWindow.xaml の相互作用ロジック
    /// </summary>
    public partial class InitWindow : Window
    {
        public InitWindow()
        {
            InitializeComponent();
        }

        private void RecordMotionButton_Click(object sender, RoutedEventArgs e)
        {
            RecordWindow wnd = new RecordWindow();
            wnd.Show();
            this.Close();
        }

        //private void CalibrateCameraButton_Click(object sender, RoutedEventArgs e)
        //{
        //    CameraUndistortWindow wnd = new CameraUndistortWindow();
        //    wnd.IsSingleWindowMode = true;
        //    wnd.Show();
        //    this.Close();
        //}

        private void MergeRecordButton_Click(object sender, RoutedEventArgs e)
        {
            MergeRecordWindow wnd = new MergeRecordWindow();
            wnd.Show();
            this.Close();
        }
    }
}
