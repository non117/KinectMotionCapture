using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;

using MsgPack.Serialization;
using Microsoft.Kinect;
using OpenCvSharp;

namespace KinectMotionCapture
{
    using PointsPair = Tuple<Dictionary<JointType, Point>, Dictionary<JointType, Point>>;
    using User = System.UInt64;

    class MotionDataChecker
    {
        public MotionDataChecker(string rootPath)
        {
            this.MotionDataCheckAndReform(rootPath);
        }

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
    }


    public class MotionDataOld
    {
        /// <summary>
        /// for MsgPack
        /// </summary>
        public MotionDataOld() { }

        /// <summary>
        /// コンストラクタ
        /// </summary>
        /// <param name="frameNo"></param>
        /// <param name="dataDir"></param>
        /// <param name="timeStamp"></param>
        /// <param name="bodies"></param>
        public MotionDataOld(int frameNo, string dataDir, DateTime timeStamp, Body[] bodies, Dictionary<ulong, PointsPair> pointPairs)
        {
            this.FrameNo = frameNo;
            this.ImagePath = Path.Combine(dataDir, frameNo.ToString() + "_color.jpg");
            this.DepthPath = Path.Combine(dataDir, frameNo.ToString() + "_depth.png");
            this.UserPath = Path.Combine(dataDir, frameNo.ToString() + "_user.png");
            this.TimeStamp = timeStamp;
            this.bodies = bodies.Where(body => body.IsTracked).Select(body => new SerializableBody(body)).ToArray();
            foreach (SerializableBody body in this.bodies)
            {
                try
                {
                    body.colorSpacePoints = pointPairs[body.TrackingId].Item1;
                    body.depthSpacePoints = pointPairs[body.TrackingId].Item2;
                }
                catch (KeyNotFoundException e)
                {
                    body.colorSpacePoints = null;
                    body.depthSpacePoints = null;
                }
            }
        }

        public int FrameNo { get; set; }
        public string ImagePath { get; set; }
        public string DepthPath { get; set; }
        public string UserPath { get; set; }
        public SerializableBody[] bodies { get; set; }
        public DateTime TimeStamp { get; set; }
        public int ColorWidth { get; set; }
        public int ColorHeight { get; set; }
        public int DepthUserWidth { get; set; }
        public int DepthUserHeight { get; set; }
        public PointF[] depthLUT { get; set; }

        // 互換性のためのメンバ。シリアライズ不可能なので下記のメソッドでロードする。
        [NonSerialized]
        public CvMat depthMat = null;
        [NonSerialized]
        public CvMat imageMat = null;
        [NonSerialized]
        public CvMat userMat = null;

        /// <summary>
        /// setを定義するとデシリアライズできなくなって死ぬ
        /// メモリを食べまくるのであんまりメンバとして画像を持ちたくない
        /// 複数回呼ばれることが前提だったらLoadしておいてもらうとか？
        /// </summary>
        public CvMat DepthMat
        {
            get
            {
                if (this.depthMat != null)
                {
                    return this.depthMat;
                }
                return CvMat.LoadImageM(this.DepthPath, LoadMode.Unchanged);
            }
        }
        public CvMat ImageMat
        {
            get
            {
                if (this.imageMat != null)
                {
                    return this.imageMat;
                }
                return CvMat.LoadImageM(this.ImagePath, LoadMode.Unchanged);
            }
        }
        public CvMat UserMat
        {
            get
            {
                if (this.userMat != null)
                {
                    return this.userMat;
                }
                return CvMat.LoadImageM(this.UserPath, LoadMode.Unchanged);
            }
        }
        public CvSize ImageSize { get; set; }
        public CvSize DepthUserSize { get; set; }

        /// <summary>
        /// pathをフルパスで保存してしまってたのでつくった。つらい。
        /// </summary>
        /// <param name="dataDir"></param>
        public void ReConstructPaths(string dataDir)
        {
            this.ImagePath = Path.Combine(dataDir, Path.GetFileName(this.ImagePath));
            this.DepthPath = Path.Combine(dataDir, Path.GetFileName(this.DepthPath));
            this.UserPath = Path.Combine(dataDir, Path.GetFileName(this.UserPath));
        }

        /// <summary>
        /// undistortionでのみ使われる特殊ケースなので分けたほうが良いかも
        /// </summary>
        public void LoadImages()
        {
            this.depthMat = CvMat.LoadImageM(this.DepthPath, LoadMode.Unchanged);
            this.imageMat = CvMat.LoadImageM(this.ImagePath, LoadMode.Unchanged);
            this.userMat = CvMat.LoadImageM(this.UserPath, LoadMode.Unchanged);
        }
    }
}
