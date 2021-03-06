﻿using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.IO;
using System.Reflection;
using System.Runtime.Serialization;
using System.Runtime.Serialization.Formatters.Binary;
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
        private string bodyInfoFilename = @"BodyInfo.dump";
        private string recordPath = "";

        private int colorWidth = 0;
        private int colorHeight = 0;
        private int depthWidth = 0;
        private int depthHeight = 0;

        private FileStream fileStream = null;
        private Queue<MotionData> motioDataQueue = null;
        private BackgroundWorker worker = null;

        // キャリブレーションが無くなったらいらない可能性が高い
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
            this.CheckRecordExistsAndCreateOpen(dataDir);
            
            this.motionDataList = new List<MotionData>();
            this.colorWidth = colorWidth;
            this.colorHeight = colorHeight;
            this.depthWidth = depthWidth;
            this.depthHeight = depthHeight;

            this.motioDataQueue = new Queue<MotionData>();
            this.worker = new BackgroundWorker();
            this.worker.WorkerSupportsCancellation = true;
            this.worker.DoWork += new DoWorkEventHandler(this.worker_DoWork);
            this.worker.RunWorkerCompleted += new RunWorkerCompletedEventHandler(this.worker_Completed);
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

            this.motionDataList = this.GetMotionDataFromFile(this.recordPath);
            MotionData md = this.motionDataList[0];
            this.colorWidth = md.ColorWidth;
            this.colorHeight = md.ColorHeight;
            this.depthWidth = md.DepthUserWidth;
            this.depthHeight = md.DepthUserHeight;
        }

        public string DataDir
        {
            get{ return this.dataDir; }
            set
            {
                this.CheckRecordExistsAndCreateOpen(value);
            }
        }

        public bool IsFileClosed
        {
            get { return !this.fileStream.CanWrite; }
        }

        public int FrameCount
        {
            get { return this.motionDataList.Count(); }
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
        /// ディレクトリが上書きになってないかチェックしてかぶってたら新しいの作る。ついでにファイルを開く。
        /// </summary>
        /// <param name="dir"></param>
        private void CheckRecordExistsAndCreateOpen(string dir)
        {
            string dataDir = dir;
            if (File.Exists(Path.Combine(dataDir, this.bodyInfoFilename)))
            {
                dataDir = dataDir + "_";
            }
            this.dataDir = dataDir;
            this.recordPath = Path.Combine(dataDir, this.bodyInfoFilename);
            Utility.CreateDirectories(this.dataDir);
            this.fileStream = new FileStream(this.recordPath, FileMode.Append, FileAccess.Write, FileShare.None);
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
            Cv.Resize(colorOrigMat, colorMat, Interpolation.NearestNeighbor); // THIS IS A BOTTLENECK!!!
            Task.Run(() => colorMat.SaveImage(path + "_color.jpg", new ImageEncodingParam(ImageEncodingID.JpegQuality, 85)));
            Task.Run(() => depthMat.SaveImage(path + "_depth.png", new ImageEncodingParam(ImageEncodingID.PngCompression, 5)));
            Task.Run(() => bodyIndexMat.SaveImage(path + "_user.png", new ImageEncodingParam(ImageEncodingID.PngCompression, 5)));
        }

        /// <summary>
        /// BackgroundWorkerをスタートする
        /// </summary>
        public void StartWorker()
        {
            this.worker.RunWorkerAsync();
        }

        /// <summary>
        /// BackgroundWorkerを止めてファイルを閉じる
        /// </summary>
        public void StopWorker()
        {
            this.worker.CancelAsync();
        }

        /// <summary>
        /// データを追加
        /// TODO : 2回目を同じファイル名でレコードすると確実に落ちる
        /// </summary>
        /// <param name="frameNo"></param>
        /// <param name="dateTime"></param>
        /// <param name="bodies"></param>
        public void AddData(int frameNo, DateTime dateTime, Body[] bodies, ref byte[] colorPixels, ref ushort[] depthBuffer, ref byte[] bodyIndexBuffer, Dictionary<ulong, PointsPair> pointPairs)
        {
            this.SaveImages(frameNo, ref colorPixels, ref depthBuffer, ref bodyIndexBuffer);
            MotionData motionData = new MotionData(frameNo, this.dataDir, dateTime, bodies, pointPairs);
            motionData.ColorWidth = this.colorWidth;
            motionData.ColorHeight = this.colorHeight;
            motionData.DepthUserWidth = this.depthWidth;
            motionData.DepthUserHeight = this.depthHeight;
            if (frameNo == 0)
            {
                motionData.depthLUT = this.DepthLUT;
            }
            
            lock (this.motioDataQueue)
            {
                this.motioDataQueue.Enqueue(motionData);
            }
        }

        /// <summary>
        /// BackgroundWorkerのworker
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void worker_DoWork(object sender, DoWorkEventArgs e)
        {
            BackgroundWorker bw = sender as BackgroundWorker;
            int interval = (int)(1000 / 40);
            IFormatter formatter = new BinaryFormatter();
            MotionData motionData;

            while (true)
            {
                lock (this.motioDataQueue)
                {
                    if (bw.CancellationPending && this.motioDataQueue.Count == 0)
                    {
                        e.Cancel = true;
                        break;
                    }

                    if (this.motioDataQueue.Count() > 0)
                    {
                        motionData = this.motioDataQueue.Dequeue();
                        // 開きっぱなしのファイルに逐次書き込み            
                        formatter.Serialize(this.fileStream, motionData);
                    }

                }
                System.Threading.Thread.Sleep(interval);
            }
        }

        /// <summary>
        /// RunWorkerCompletedのイベントハンドラ
        /// </summary>
        public void worker_Completed(object sender, RunWorkerCompletedEventArgs e)
        {
            this.fileStream.Close();
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
    [Serializable]
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

        /// <summary>
        /// Bodyの関節を反転する
        /// </summary>
        public void InverseJoints()
        {
            Dictionary<JointType, Joint> newJoints = new Dictionary<JointType, Joint>();
            Dictionary<JointType, Point> newColorPoints = new Dictionary<JointType, Point>();
            Dictionary<JointType, Point> newDepthPoints = new Dictionary<JointType, Point>();
            foreach (JointType key in Enum.GetValues(typeof(JointType)))
            {
                JointType newKey = CalcEx.GetMirroredJoint(key);
                if (this.Joints.ContainsKey(key))
                    newJoints[newKey] = this.Joints[key];
                if (this.colorSpacePoints.ContainsKey(key))
                    newColorPoints[newKey] = this.colorSpacePoints[key];
                if (this.depthSpacePoints.ContainsKey(key))
                    newDepthPoints[newKey] = this.depthSpacePoints[key];

            }
            this.Joints = newJoints;
            this.colorSpacePoints = newColorPoints;
            this.depthSpacePoints = newDepthPoints;
        }

        /// <summary>
        /// 骨を取り除く
        /// </summary>
        /// <param name="joints"></param>
        /// <param name="removeJoints"></param>
        public void RemoveJoints(List<JointType> removeJoints)
        {
            Dictionary<JointType, Joint> newJoints = this.Joints.CloneDeep();
            Dictionary<JointType, Point> newColorPoints = this.colorSpacePoints.CloneDeep();
            Dictionary<JointType, Point> newDepthPoints = this.depthSpacePoints.CloneDeep();
            foreach (JointType jointType in removeJoints)
            {
                if (newJoints.ContainsKey(jointType))
                    newJoints.Remove(jointType);
                if (newColorPoints.ContainsKey(jointType))
                    newColorPoints.Remove(jointType);
                if (newDepthPoints.ContainsKey(jointType))
                    newDepthPoints.Remove(jointType);
            }
            this.Joints = newJoints;
            this.colorSpacePoints = newColorPoints;
            this.depthSpacePoints = newDepthPoints;
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
        [NonSerialized]
        public bool mirrored = false;
        [NonSerialized]
        public int integratedId = -1;
        [NonSerialized]
        public bool trustData = false;
    }
}
