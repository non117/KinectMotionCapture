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

        private void UtilityButton_Click(object sender, RoutedEventArgs e)
        {
            string bodyData = @"C:\Users\non\Desktop\Data\1222\L1\Student1SelectedUserBody.dump";
            string timeData = @"C:\Users\non\Desktop\Data\1222\L1\Student1TimeData.dump";
            List<string> statData = new List<string>()
            {
                @"C:\Users\non\Desktop\Data\1222\L1\L1StudentStatData.dump",
                @"C:\Users\non\Desktop\Data\1222\L2\L2StudentStatData.dump",
                @"C:\Users\non\Desktop\Data\1222\L3\L3StudentStatData.dump",
                @"C:\Users\non\Desktop\Data\1222\L4\L4StudentStatData.dump",
                @"C:\Users\non\Desktop\Data\1222\L5\L5StudentStatData.dump",
            };
            List<Dictionary<JointType, CvPoint3D64f>> jointsSeq = Utility.ConvertToCvPoint((List<Dictionary<int, float[]>>)Utility.LoadFromBinary(bodyData));
            List<DateTime> timeSeq = (List<DateTime>)Utility.LoadFromBinary(timeData);
            List<Dictionary<Bone, BoneStatistics>> boneStats = statData.Select(s => (Dictionary<Bone, BoneStatistics>)Utility.LoadFromBinary(s)).ToList();
            MotionMetaData mmd = new MotionMetaData(jointsSeq, timeSeq, boneStats);

            // ほぞーーーーーーーーん
            string desktop = Environment.GetFolderPath(Environment.SpecialFolder.DesktopDirectory);
            string saveBodyPath = System.IO.Path.Combine(desktop, @"Student1FilteredBody.dump");
            string saveTimePath = System.IO.Path.Combine(desktop, @"Student1TimeData.dump");
            Utility.SaveBodySequence(mmd.BodySequence, saveBodyPath);
            Utility.SaveToBinary(mmd.TimeSequence, saveTimePath);
        }
    }
}
