using System;
using System.Collections.Generic;
using System.ComponentModel;
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

using System.IO;
using MsgPack.Serialization;

namespace KinectMotionCapture
{
    /// <summary>
    /// MergeRecordWindow.xaml の相互作用ロジック
    /// </summary>
    public partial class MergeRecordWindow : Window
    {
        private void ArewoNaosu(string filepath)
        {
            List<MotionDataOld> md;
            var serializer = MessagePackSerializer.Get<List<MotionDataOld>>();
            using (FileStream fs = File.Open(filepath, FileMode.Open))
            {
                md = (List<MotionDataOld>)serializer.Unpack(fs);
            }

            //ここで変換する

            List<MotionData> newMd;
            using (FileStream fs = File.Open(filepath, FileMode.OpenOrCreate, FileAccess.Write))
            {
                lock (newMd)
                {
                    serializer.Pack(fs, newMd);
                }
            }

        }

        public MergeRecordWindow()
        {
            List<string> datadir = new List<string>() { @"C:\Users\non\Desktop\data\0922_kinect4\master" };
            FrameSequence fs = new FrameSequence(datadir);

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
