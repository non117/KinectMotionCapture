using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.IO;
using System.Reflection;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Media.Media3D;
using Microsoft.Kinect;

using MsgPack.Serialization;
using OpenCvSharp;

namespace KinectMotionCapture
{
    using PointsPair = Tuple<Dictionary<JointType, Point>, Dictionary<JointType, Point>>;
    using User = System.UInt64;

    /// <summary>
    /// Kinectで記録したbody情報とかを記録するやつ
    /// </summary>
    public class MotionDataHandler
    {
        private string dataDir = "";
        private string bodyInfoFilename = @"BodyInfo.mpac";
        private string recordPath = "";

        private int colorWidth = 0;
        private int colorHeight = 0;
        private int depthWidth = 0;
        private int depthHeight = 0;

        public List<MotionData> motionDataList { get; set; }

        /// <summary>
        /// Kinect記録用のコンストラクタ
        /// </summary>
        /// <param name="colorWidth"></param>
        /// <param name="colorHeight"></param>
        /// <param name="depthWidth"></param>
        /// <param name="depthHeight"></param>
        public MotionDataHandler(string dataDir, int colorWidth, int colorHeight, int depthWidth, int depthHeight)
        {
            this.dataDir = dataDir;
            // TODO: 上書き防止機能をつくる
            this.recordPath = Path.Combine(dataDir, this.bodyInfoFilename);
            Utility.CreateDirectories(this.dataDir);
            
            this.motionDataList = new List<MotionData>();
            this.colorWidth = colorWidth;
            this.colorHeight = colorHeight;
            this.depthWidth = depthWidth;
            this.depthHeight = depthHeight;
        }

        /// <summary>
        /// 記録が存在するときのコンストラクタ
        /// </summary>
        /// <param name="dataDir"></param>
        public MotionDataHandler(string dataDir)
        {
            this.LoadAndSetData(dataDir);
        }

        /// <summary>
        /// データを破棄して既存のデータをロードします。
        /// </summary>
        /// <param name="dataDir"></param>
        public void LoadAndSetData(string dataDir)
        {
            this.dataDir = dataDir;
            this.recordPath = Path.Combine(dataDir, bodyInfoFilename);

            this.ClearAll();
            this.motionDataList = this.GetMotionDataFromFile(this.recordPath);
            MotionData md = this.motionDataList[0];
            this.colorWidth = md.ColorWidth;
            this.colorHeight = md.ColorHeight;
            this.depthWidth = md.DepthUserWidth;
            this.depthHeight = md.DepthUserHeight;
        }

        /// <summary>
        /// すべてのデータを破棄する
        /// </summary>
        public void ClearAll()
        {
            if (this.motionDataList != null)
            {
                this.motionDataList.Clear();
            }
        }

        public string DataDir
        {
            get{ return this.dataDir; }
            set
            {
                this.dataDir = value;
                this.recordPath = Path.Combine(this.dataDir, this.bodyInfoFilename);
            }
        }

        public int FrameCount
        {
            get { return this.motionDataList.Count();  }
        }

        public PointF[] DepthLUT { get; set; }

        /// <summary>
        /// ファイルからBodyデータをデシリアライズする
        /// #TODO メモリを食べ過ぎる問題があるため、リアルタイムにBinarySerializeするように変更する。優先度は低。
        /// </summary>
        /// <param name="filepath"></param>
        /// <returns></returns>
        private List<MotionData> GetMotionDataFromFile(string filepath)
        {
            var serializer = MessagePackSerializer.Get<List<MotionData>>();
            using (FileStream fs = File.Open(filepath, FileMode.Open))
            {
                return serializer.Unpack(fs);
            }
        }

        /// <summary>
        /// 各Kinect情報の画像を保存する
        /// </summary>
        /// <param name="frameNo"></param>
        /// <param name="colorPixels"></param>
        /// <param name="depthBuffer"></param>
        /// <param name="bodyIndexBuffer"></param>
        private void SaveImages(int frameNo, ref byte[] colorPixels, ref ushort[] depthBuffer, ref byte[] bodyIndexBuffer)
        {
            string path = Path.Combine(this.dataDir, frameNo.ToString());
            CvMat colorOrigMat = Utility.ColorArrayToCvMat(this.colorWidth, this.colorHeight, ref colorPixels);
            CvMat depthMat = Utility.DpethArrayToCvMat(this.depthWidth, this.depthHeight, ref depthBuffer);
            CvMat bodyIndexMat = Utility.BodyIndexArrayToCvMat(this.depthWidth, this.depthHeight, ref bodyIndexBuffer);
            CvMat colorMat = new CvMat(this.colorHeight / 2, this.colorWidth / 2, MatrixType.U8C4);
            Cv.Resize(colorOrigMat, colorMat, Interpolation.Lanczos4);
            Task.Run(() => colorMat.SaveImage(path + "_color.jpg", new ImageEncodingParam(ImageEncodingID.JpegQuality, 85)));
            Task.Run(() => depthMat.SaveImage(path + "_depth.png", new ImageEncodingParam(ImageEncodingID.PngCompression, 5)));
            Task.Run(() => bodyIndexMat.SaveImage(path + "_user.png", new ImageEncodingParam(ImageEncodingID.PngCompression, 5)));
        }

        /// <summary>
        /// データを追加
        /// </summary>
        /// <param name="frameNo"></param>
        /// <param name="dateTime"></param>
        /// <param name="bodies"></param>
        public void AddData(int frameNo, DateTime dateTime, Body[] bodies, ref byte[] colorPixels, ref ushort[] depthBuffer, ref byte[] bodyIndexBuffer, Dictionary<ulong, PointsPair> pointPairs)
        {
            this.SaveImages(frameNo, ref colorPixels, ref depthBuffer, ref bodyIndexBuffer);
            lock (this.motionDataList)
            {
                MotionData motionData = new MotionData(frameNo, this.dataDir, dateTime, bodies, pointPairs);
                motionData.ColorWidth = this.colorWidth;
                motionData.ColorHeight = this.colorHeight;
                motionData.DepthUserWidth = this.depthWidth;
                motionData.DepthUserHeight = this.depthHeight;
                if (frameNo == 0)
                {
                    motionData.depthLUT = this.DepthLUT;
                }
                this.motionDataList.Add(motionData);
            }
        }

        /// <summary>
        /// 骨格情報をファイルに書き出して、リストを空にする。
        /// </summary>
        public void Flush()
        {
            var serializer = MessagePackSerializer.Get<List<MotionData>>();
            using (FileStream fs = File.Open(this.recordPath, FileMode.OpenOrCreate, FileAccess.Write))
            {
                lock (this.motionDataList)
                {
                    serializer.Pack(fs, this.motionDataList);
                }
            }
            this.motionDataList.Clear();
        }

        /// <summary>
        /// 画面描画用のカラー画像のパスを返す
        /// </summary>
        /// <returns></returns>
        public IEnumerable<string> GetColorImagePaths()
        {
            return this.motionDataList.Select(data => data.ImagePath);
        }

        /// <summary>
        /// Depth画像のパスを返す
        /// </summary>
        /// <returns></returns>
        public IEnumerable<string> GetDepthImagePaths()
        {
            return this.motionDataList.Select(data => data.DepthPath);
        }
    }

    /// <summary>
    /// 動作のメタデータと骨格座標とかのモデルクラス
    /// </summary>
    public class MotionData
    {
        /// <summary>
        /// for MsgPack
        /// </summary>
        public MotionData() { }

        /// <summary>
        /// コンストラクタ
        /// </summary>
        /// <param name="frameNo"></param>
        /// <param name="dataDir"></param>
        /// <param name="timeStamp"></param>
        /// <param name="bodies"></param>
        public MotionData(int frameNo, string dataDir, DateTime timeStamp, Body[] bodies, Dictionary<ulong, PointsPair> pointPairs)
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
                }catch(KeyNotFoundException e){                    
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

        public CvSize ImageSize
        {
            get { return new CvSize(this.ColorWidth, this.ColorHeight); }
            set {
                this.ColorWidth = value.Width;
                this.ColorHeight = value.Height;
            }
        }
        public CvSize DepthUserSize
        {
            get { return new CvSize(this.DepthUserWidth, this.DepthUserHeight); }
            set
            {
                this.DepthUserWidth = value.Width;
                this.DepthUserHeight = value.Height;
            }
        }

        public void LoadImages()
        {
            this.depthMat = CvMat.LoadImageM(this.DepthPath, LoadMode.Unchanged);
            this.imageMat = CvMat.LoadImageM(this.ImagePath, LoadMode.Unchanged);
            this.userMat = CvMat.LoadImageM(this.UserPath, LoadMode.Unchanged);
        }
    }

    /// <summary>
    /// BodyクラスはMsgPackでシリアライズできないので、勝手に定義
    /// </summary>
    [Serializable]
    public class SerializableBody
    {
        /// <summary>
        /// for MsgPack
        /// </summary>
        public SerializableBody() { }

        /// <summary>
        /// bodyがTrackされている場合にのみ格納する
        /// </summary>
        /// <param name="body"></param>
        public SerializableBody(Body body)
        {
            foreach (PropertyInfo bodyPropertyInfo in body.GetType().GetProperties())
            {
                foreach (PropertyInfo propertyInfo in this.GetType().GetProperties())
                {
                    if (bodyPropertyInfo.Name == propertyInfo.Name)
                    {
                        propertyInfo.SetValue(this, bodyPropertyInfo.GetValue(body, null));
                    }
                }
            }
        }

        public Dictionary<Activity, DetectionResult> Activities { get; set; }
        public Dictionary<Appearance, DetectionResult> Appearance { get; set; }
        public FrameEdges ClippedEdges { get; set; }
        public DetectionResult Engaged { get; set; }
        public Dictionary<Microsoft.Kinect.Expression, DetectionResult> Expressions { get; set; }
        public TrackingConfidence HandLeftConfidence { get; set; }
        public HandState HandLeftState { get; set; }
        public TrackingConfidence HandRightConfidence { get; set; }
        public HandState HandRightState { get; set; }
        public bool IsRestricted { get; set; }
        public bool IsTracked { get; set; }
        public static int JointCount { get; set; }
        public Dictionary<JointType, JointOrientation> JointOrientations { get; set; }
        public Dictionary<JointType, Joint> Joints { get; set; }
        public PointF Lean { get; set; }
        public TrackingState LeanTrackingState { get; set; }
        public ulong TrackingId { get; set; }
        public Dictionary<JointType, Point> colorSpacePoints { get; set; }
        public Dictionary<JointType, Point> depthSpacePoints { get; set; }
    }
}
