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
            //string bodyData = @"C:\Users\non\Desktop\Data\1225\L3\Student40SelectedUserBody.dump";
            //string timeData = @"C:\Users\non\Desktop\Data\1225\L3\Student40TimeData.dump";
            string bodyData = @"C:\Users\non\Desktop\Data\1222\Teacher\Teacher0SelectedUserBody.dump";
            string timeData = @"C:\Users\non\Desktop\Data\1222\Teacher\Teacher0TimeData.dump";
            string bodyFileName = System.IO.Path.GetFileName(bodyData).Replace("SelectedUser", "Filtered");
            string timeFileName = System.IO.Path.GetFileName(timeData);
            //List<string> statData = new List<string>()
            //{
            //    @"C:\Users\non\Desktop\Data\1222\L1\L1StudentStatData.dump",
            //    @"C:\Users\non\Desktop\Data\1222\L2\L2StudentStatData.dump",
            //    @"C:\Users\non\Desktop\Data\1222\L3\L3StudentStatData.dump",
            //    @"C:\Users\non\Desktop\Data\1222\L4\L4StudentStatData.dump",
            //    @"C:\Users\non\Desktop\Data\1222\L5\L5StudentStatData.dump",
            //    @"C:\Users\non\Desktop\Data\1224\L1\L1StudentStatData.dump",
            //    @"C:\Users\non\Desktop\Data\1224\L2\L2StudentStatData.dump",
            //    @"C:\Users\non\Desktop\Data\1224\L3\L3StudentStatData.dump",
            //    @"C:\Users\non\Desktop\Data\1224\L4\L4StudentStatData.dump",
            //    @"C:\Users\non\Desktop\Data\1224\L5\L5StudentStatData.dump",
            //    @"C:\Users\non\Desktop\Data\1225\L1\L1StudentStatData.dump",
            //    @"C:\Users\non\Desktop\Data\1225\L2\L2StudentStatData.dump",
            //    @"C:\Users\non\Desktop\Data\1225\L3\L3StudentStatData.dump",
            //};
            List<string> statData = new List<string>()
            {
                @"C:\Users\non\Desktop\Data\1222\L1\L1TeacherStatData.dump",
                @"C:\Users\non\Desktop\Data\1222\Teacher\TeacherStatData.dump",
                @"C:\Users\non\Desktop\Data\1224\Teacher\TeacherStatData.dump",
                @"C:\Users\non\Desktop\Data\1225\Teacher\TeacherStatData.dump",
            };
            List<Dictionary<JointType, CvPoint3D64f>> jointsSeq = Utility.ConvertToCvPoint((List<Dictionary<int, float[]>>)Utility.LoadFromBinary(bodyData));
            List<DateTime> timeSeq = (List<DateTime>)Utility.LoadFromBinary(timeData);
            List<Dictionary<Bone, BoneStatistics>> boneStats = statData.Select(s => (Dictionary<Bone, BoneStatistics>)Utility.LoadFromBinary(s)).ToList();
            MotionMetaData mmd = new MotionMetaData(jointsSeq, timeSeq, boneStats);

            // ほぞーーーーーーーーん
            string desktop = Environment.GetFolderPath(Environment.SpecialFolder.DesktopDirectory);
            string saveBodyPath = System.IO.Path.Combine(desktop, bodyFileName);
            string saveTimePath = System.IO.Path.Combine(desktop, timeFileName);
            Utility.SaveBodySequence(mmd.BodySequence, saveBodyPath);
            Utility.SaveToBinary(mmd.TimeSequence, saveTimePath);
        }

        private void MotionAnalyzeButton_Click(object sender, RoutedEventArgs e)
        {
            string csvFilePath = @"C:\Users\non\Desktop\Filtered\CutTiming2.csv";
            MotionAnalyzer ma = new MotionAnalyzer(csvFilePath);
            ma.AjustBodyDirection("Teacher6");
            bool legMode = true;
            ma.Slice(legMode);
            string[] teachers = new string[] { "Teacher0", "Teacher4", "Teacher5", "Teacher6", "Teacher7" };
            ma.JoinData(teachers);
        }

        private void SimilaritySort_Click(object sender, RoutedEventArgs e)
        {
            //string filePath = @"C:\Users\non\Desktop\Data\AllSimilaritiesDay3.dump";
            //string filePath = @"C:\Users\non\Desktop\Data\AllSimilaritiesTillDay2.dump";
            //bool legMode = true;
            //bool legMode = false;
            SimilarityAnalyzer sa = new SimilarityAnalyzer();
            //string[] students = Enumerable.Range(1, 32).Select(i => "Student" + i.ToString()).ToArray();
            //string[] students = Enumerable.Range(33, 8).Select(i => "Student" + i.ToString()).ToArray();
            //sa.CalcSpecificStepSimilarities("G1", students, 30);
            //sa.DayVariableVariance();
            //sa.DumpSpecificVariableSimLog();
            //sa.ThirdDayLessonVariableVariance();
            //sa.DumpAllDecreasedCsvs(@"C:\Users\non\Desktop\Data\variancesLeg.dump", @"C:\Users\non\Desktop\LegCsv");
            //sa.DumpAllDecreasedCsvs(@"C:\Users\non\Desktop\Day3ArmVariances.dump", @"C:\Users\non\Desktop\ArmCsv2");
            //sa.SortVarianceFile(@"C:\Users\non\Desktop\Data\Day3ArmVariances.dump");
            //sa.DayVariableVariance();
            //sa.StepSpecificCsvExporter();
            sa.SearchSpecific();
        }
    }
}
