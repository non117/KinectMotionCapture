using System;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using System.Text;
using System.Threading.Tasks;
using System.Windows;

using Microsoft.Kinect;
using OpenCvSharp;

namespace KinectMotionCapture
{
    using PointsPair = Tuple<Dictionary<JointType, Point>, Dictionary<JointType, Point>>;
    using User = System.UInt64;

    class DataConverter
    {
        private static string bodyInfoFilename = @"BodyInfo.dump";

        /// <summary>
        /// レコードのメタデータをデシリアライズしてくる
        /// </summary>
        /// <param name="filepath"></param>
        /// <returns></returns>
        private static IEnumerable<OldMotionData> GetMotionDataFromFile(string filepath)
        {
            string ext = Path.GetExtension(filepath);
            if (ext == ".dump")
            {
                return Utility.LoadFromSequentialBinary(filepath).Select(o => (OldMotionData)o);
            }
            return new List<OldMotionData>();
        }

        /// <summary>
        /// 旧から新へ変換する
        /// </summary>
        /// <param name="data"></param>
        /// <returns></returns>
        private static MotionData ConvertData(OldMotionData data)
        {
            MotionData md = new MotionData();
            md.bodies = data.bodies;
            md.ColorHeight = data.ColorHeight;
            md.ColorWidth = data.ColorWidth;
            md.depthLUT = data.depthLUT;
            md.DepthUserHeight = data.DepthUserHeight;
            md.DepthUserWidth = data.DepthUserWidth;
            md.DepthUserPath = data.DepthPath;
            md.FrameNo = data.FrameNo;
            md.ImagePath = data.ImagePath;
            md.TimeStamp = data.TimeStamp;
            return md;
        }

        /// <summary>
        /// カラー画像をコピーする
        /// </summary>
        /// <param name="destDir"></param>
        /// <param name="imagePaths"></param>
        private static void MoveImages(string destDir, IEnumerable<string> imagePaths)
        {
            Utility.CreateDirectories(destDir);
            foreach (string oldImage in imagePaths)
            {
                string newImage = Path.Combine(destDir, Path.GetFileName(oldImage));
                File.Copy(oldImage, newImage);
            }
        }

        public static void Convert(string srcDir, string destDir)
        {
            string metaDataFilePath = Path.Combine(srcDir, bodyInfoFilename);
            IEnumerable<MotionData> mdList = GetMotionDataFromFile(metaDataFilePath).Select(ConvertData);
            foreach (MotionData md in mdList)
            {
                md.ReConstructPaths(srcDir);
            }
            MoveImages(destDir, mdList.Select(md => md.ImagePath));
            // TODO : 画像の変換と移動処理
        }
    }

    /// <summary>
    /// 動作のメタデータと骨格座標とかのモデルクラス
    /// </summary>
    [Serializable]
    public class OldMotionData
    {
        /// <summary>
        /// for MsgPack
        /// </summary>
        public OldMotionData() { }

        /// <summary>
        /// コンストラクタ
        /// </summary>
        /// <param name="frameNo"></param>
        /// <param name="dataDir"></param>
        /// <param name="timeStamp"></param>
        /// <param name="bodies"></param>
        public OldMotionData(int frameNo, string dataDir, DateTime timeStamp, Body[] bodies, Dictionary<ulong, PointsPair> pointPairs)
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

        // このデータを使うかどうか
        [NonSerialized]
        public bool isValid = true;

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
