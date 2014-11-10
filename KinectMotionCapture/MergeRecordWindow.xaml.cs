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
        private void ConvertOldToNew(string filepath)
        {
            List<MotionDataOld> mdos;
            var serializer = MessagePackSerializer.Get<List<MotionDataOld>>();
            using (FileStream fs = File.Open(filepath, FileMode.Open))
            {
                mdos = (List<MotionDataOld>)serializer.Unpack(fs);
            }

            //ここで変換する
            List<MotionData> newMd = new List<MotionData>();
            foreach (MotionDataOld mdo in mdos)
            {
                MotionData md = new MotionData();
                md.FrameNo = mdo.FrameNo;
                md.ImagePath = mdo.ImagePath;
                md.DepthPath = mdo.DepthPath;
                md.UserPath = mdo.UserPath;
                md.bodies = mdo.bodies.CloneDeep();
                md.TimeStamp = mdo.TimeStamp;
                md.ColorWidth = mdo.ColorWidth;
                md.ColorHeight = mdo.ColorHeight;
                md.DepthUserWidth = mdo.DepthUserWidth;
                md.DepthUserHeight = mdo.DepthUserHeight;
                if (mdo.depthLUT != null)
                {
                    md.depthLUT = mdo.depthLUT.CloneDeep();
                }
                else
                {
                    md.depthLUT = null;
                }
                newMd.Add(md);
            }

            var serializer2 = MessagePackSerializer.Get<List<MotionData>>();
            using (FileStream fs = File.Open(filepath, FileMode.OpenOrCreate, FileAccess.Write))
            {
                lock (newMd)
                {
                    serializer2.Pack(fs, newMd);
                }
            }
        }
        private int LoadTest(string filepath)
        {
            var serializer = MessagePackSerializer.Get<List<MotionData>>();
            List<MotionData> mdns;
            using (FileStream fs = File.Open(filepath, FileMode.Open))
            {
                mdns = (List<MotionData>)serializer.Unpack(fs);
            }
            return mdns.Count();
        }

        private void MotionDataCheckAndReform(string rootDir)
        {
            IEnumerable<string> recordDirectories = Directory.GetDirectories(rootDir, "*", SearchOption.AllDirectories);
            int corrected = 0;
            int check = 0;
            foreach (string record in recordDirectories)
            {
                string metafile = Path.Combine(record, "BodyInfo.mpac");
                if (!File.Exists(metafile))
                {
                    Debug.WriteLine(metafile + " is not exists");
                    continue;
                }
                bool testFailed;
                try
                {
                    this.LoadTest(metafile);
                    testFailed = false;
                }
                catch (Exception e)
                {
                    Debug.WriteLine(e.ToString());
                    testFailed = true;
                }
                if (testFailed)
                {
                    string backupname = Path.Combine(record, "BodyInfo_old.mpac");
                    if (File.Exists(backupname))
                    {
                        continue;
                    }
                    Debug.WriteLine("Correct " + metafile);
                    File.Copy(metafile, backupname);
                    this.ConvertOldToNew(metafile);
                    this.LoadTest(metafile);
                    corrected++;
                    check++;
                }
                else
                {
                    check++;
                    continue;
                }
            }
            Debug.WriteLine("Checked:" + check.ToString());
            Debug.WriteLine("Corrected:" + corrected.ToString());
        }

        public MergeRecordWindow()
        {
            //List<string> datadir = new List<string>() { @"C:\Users\non\Desktop\data\1015_kinect4\mite1" };
            string rootPath = @"F:\1015";
            this.MotionDataCheckAndReform(rootPath);
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
