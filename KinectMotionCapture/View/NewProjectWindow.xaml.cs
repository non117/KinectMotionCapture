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
using System.Windows.Forms;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;

namespace KinectMotionCapture
{
    /// <summary>
    /// NewProjectWindow.xaml の相互作用ロジック
    /// </summary>
    public partial class NewProjectWindow :  Window, INotifyPropertyChanged
    {
        private string dataPath1, dataPath2, dataPath3, dataPath4, dataPath5;
        private string[] MapFileArr, CameraFileArr;
        private int num;

        public NewProjectWindow()
        {
            num = 5;
            dataPath1 = "";
            dataPath2 = "";
            dataPath3 = "";
            dataPath4 = "";
            dataPath5 = "";
            MapFileArr = new string[num];
            CameraFileArr = new string[num];
            this.DataContext = this;
            InitializeComponent();
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        private void OnPropertyChanged(string name)
        {
            if (this.PropertyChanged != null)
            {
                this.PropertyChanged(this, new PropertyChangedEventArgs(name));
            }

        }

        public string DataPath1
        {
            get { return dataPath1; }
        }
        public string DataPath2
        {
            get { return dataPath2; }
        }
        public string DataPath3
        {
            get { return dataPath3; }
        }
        public string DataPath4
        {
            get { return dataPath4; }
        }
        public string DataPath5
        {
            get { return dataPath5; }
        }
        public string MapFiles
        {
            get
            {
                if (MapFileArr.Where(s => s == null).Count() == num)
                    return "";
                return String.Join(", ", MapFileArr);
            }
        }
        public string CameraFiles
        {
            get
            {
                if (CameraFileArr.Where(s => s == null).Count() == num)
                    return "";
                return String.Join(", ", CameraFileArr);
            }
        }

        private void OpenFolderButton1_Click(object sender, RoutedEventArgs e)
        {
            string path = Utility.ChooseFolderDialog("撮影データのフォルダを選択してください");
            if (path == null)
            {
                System.Windows.MessageBox.Show("撮影データのフォルダを選択してください");
                return;
            }
            dataPath1 = path;
            OnPropertyChanged("DataPath1");
        }
        private void OpenFolderButton2_Click(object sender, RoutedEventArgs e)
        {
            string path = Utility.ChooseFolderDialog("撮影データのフォルダを選択してください");
            if (path == null)
            {
                System.Windows.MessageBox.Show("撮影データのフォルダを選択してください");
                return;
            }
            dataPath2 = path;
            OnPropertyChanged("DataPath2");
        }
        private void OpenFolderButton3_Click(object sender, RoutedEventArgs e)
        {
            string path = Utility.ChooseFolderDialog("撮影データのフォルダを選択してください");
            if (path == null)
            {
                System.Windows.MessageBox.Show("撮影データのフォルダを選択してください");
                return;
            }
            dataPath3 = path;
            OnPropertyChanged("DataPath3");
        }
        private void OpenFolderButton4_Click(object sender, RoutedEventArgs e)
        {
            string path = Utility.ChooseFolderDialog("撮影データのフォルダを選択してください");
            if (path == null)
            {
                System.Windows.MessageBox.Show("撮影データのフォルダを選択してください");
                return;
            }
            dataPath4 = path;
            OnPropertyChanged("DataPath4");
        }
        private void OpenFolderButton5_Click(object sender, RoutedEventArgs e)
        {
            string path = Utility.ChooseFolderDialog("撮影データのフォルダを選択してください");
            if (path == null)
            {
                System.Windows.MessageBox.Show("撮影データのフォルダを選択してください");
                return;
            }
            dataPath5 = path;
            OnPropertyChanged("DataPath5");
        }
        private void OpenMapFileButton_Click(object sender, RoutedEventArgs e)
        {
            OpenFileDialog ofd = new OpenFileDialog();
            ofd.InitialDirectory = Environment.GetFolderPath(Environment.SpecialFolder.Desktop);
            ofd.Title = "coordmapファイルを選択してください";
            ofd.Filter = "Dump Files (*.dump)|*.dump";
            ofd.Multiselect = true;
            if (ofd.ShowDialog() == System.Windows.Forms.DialogResult.OK)
            {
                MapFileArr = ofd.FileNames;
                OnPropertyChanged("MapFiles");
            }
            else
            {
                System.Windows.MessageBox.Show("coordmapファイルを選択してください");
            }
        }
        private void OpenCameraFileButton_Click(object sender, RoutedEventArgs e)
        {
            OpenFileDialog ofd = new OpenFileDialog();
            ofd.InitialDirectory = Environment.GetFolderPath(Environment.SpecialFolder.Desktop);
            ofd.Title = "CameraInfoファイルを選択してください";
            ofd.Filter = "Dump Files (*.dump)|*.dump";
            ofd.Multiselect = true;
            if (ofd.ShowDialog() == System.Windows.Forms.DialogResult.OK)
            {
                CameraFileArr = ofd.FileNames;
                OnPropertyChanged("CameraFiles");
            }
            else
            {
                System.Windows.MessageBox.Show("CameraInfoファイルを選択してください");
            }
        }
        private void SubmitButtonClick(object sender, RoutedEventArgs e)
        {
            Settings s = new Settings();
            string[] dirs = new string[]{ dataPath1, dataPath2, dataPath3, dataPath4, dataPath5 };
            List<string> dirList = dirs.Where(str => str != "").ToList();
            int num = dirList.Count();
            s.dataDirectories = dirList.ToArray();
            s.mapFiles = MapFileArr.Take(num).ToArray();
            s.cameraFiles = CameraFileArr.Take(num).ToArray();
            MergeRecordWindow wnd = new MergeRecordWindow(s);
            wnd.Show();
            this.Close();
        }
    }
}
