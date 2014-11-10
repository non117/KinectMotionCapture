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

using System.IO;
using MsgPack.Serialization;

namespace KinectMotionCapture
{
    /// <summary>
    /// MergeRecordWindow.xaml の相互作用ロジック
    /// </summary>
    public partial class MergeRecordWindow : Window
    {


        public MergeRecordWindow()
        {
            List<string> datadir = new List<string>() {
                                                    @"F:\0922\0922_kinect1\mitemite2",  
                                                    @"F:\0922\0922_kinect2\mitemite2", 
                                                    @"F:\0922\0922_kinect3\mitemite2", 
                                                    @"F:\0922\0922_kinect4\mitemite2", 
            };
            FrameSequence fs = new FrameSequence(datadir);
            Environment.Exit(0);
            InitializeComponent();
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>

        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
        }
    }
}
