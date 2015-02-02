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
using System.Windows.Navigation;
using System.Windows.Shapes;

using KinectMotionCapture;
using OpenCvSharp;
using Microsoft.Kinect;

namespace KinectDataAnalyzer
{
    using Bone = Tuple<JointType, JointType>;
    /// <summary>
    /// MainWindow.xaml の相互作用ロジック
    /// </summary>
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            InitializeComponent();
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {

        }

        private void CleanMotionButton_Click(object sender, RoutedEventArgs e)
        {
            string bodyData = @"C:\Users\non\Desktop\Data\1222\L1\Student1SelectedUserBody.dump";
            string timeData = @"C:\Users\non\Desktop\Data\1222\L1\Student1TimeData.dump";
            List<string> statData = new List<string>()
            {
                @"C:\Users\non\Desktop\Data\1222\L1\L1StudentStatData.dump",
            };
            List<Dictionary<JointType, CvPoint3D64f>> jointsSeq = Utility.ConvertToCvPoint((List<Dictionary<int, float[]>>)Utility.LoadFromBinary(bodyData));
            List<DateTime> timeSeq = (List<DateTime>)Utility.LoadFromBinary(timeData);
            List<Dictionary<Bone, BoneStatistics>> boneStats = statData.Select(s => (Dictionary<Bone, BoneStatistics>)Utility.LoadFromBinary(s)).ToList();
            MotionMetaData mmd = new MotionMetaData(jointsSeq, timeSeq, boneStats);
        }
    }
}
