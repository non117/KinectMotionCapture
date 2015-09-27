using System;
using System.Collections.Generic;
using System.Linq;
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

        private void MergeRecordButton_Click(object sender, RoutedEventArgs e)
        {
            NewProjectWindow npw = new NewProjectWindow();
            npw.Show();
            this.Close();
        }

        private void OpenProjectButton_Click(object sender, RoutedEventArgs e)
        {
            string path = Utility.ChooseFileDialog("プロジェクトファイルを選択してください", defaultPath:Environment.CurrentDirectory,  filter:"XML Files (*.xml)|*.xml");
            if (path == null)
            {
                System.Windows.MessageBox.Show("プロジェクトファイルを選択してください");
                return;
            }
            Settings s = Utility.LoadFromXML<Settings>(path);
            MergeRecordWindow wnd = new MergeRecordWindow(s);
            wnd.Show();
            this.Close();
        }

        private void ConvertButton_Click(object sender, RoutedEventArgs e)
        {
            FolderBrowserDialog fbd = new FolderBrowserDialog();
            fbd.SelectedPath = Environment.GetFolderPath(Environment.SpecialFolder.Desktop);
            string src, dest;
            bool flg1 = false;
            bool flg2 = false;
            fbd.Description = "入力フォルダを選択してください";
            if (fbd.ShowDialog() == System.Windows.Forms.DialogResult.OK)
            {
                src = fbd.SelectedPath;
                flg1 = true;

                fbd.Description = "出力フォルダを選択してください";
                if (fbd.ShowDialog() == System.Windows.Forms.DialogResult.OK)
                {
                    dest = fbd.SelectedPath;
                    flg2 = true;
                    DataConverter.Convert(src, dest);
                }
            }
            
        }
    }
}
