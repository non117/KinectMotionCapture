using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using System.Reflection;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;
using Microsoft.Kinect;

using MsgPack.Serialization;
using OpenCvSharp;

namespace KinectMotionCapture
{
    using User = System.UInt64;

    /// <summary>
    /// Kinectで記録したbody情報とかを記録するやつ
    /// </summary>
    public class MotionDataHandler
    {
        private string dataDir = @"Data"; // そのうちPropertyとかUIからsetするようにしたい
        private string filename = @"BodyInfo.mpac";
        private string recordPath = "";
        private List<MotionData> motionDataList = null;

        private int colorWidth = 0;
        private int colorHeight = 0;
        private int depthWidth = 0;
        private int depthHeight = 0;

        /// <summary>
        /// Kinect記録用のコンストラクタ
        /// </summary>
        /// <param name="colorWidth"></param>
        /// <param name="colorHeight"></param>
        /// <param name="depthWidth"></param>
        /// <param name="depthHeight"></param>
        public MotionDataHandler(int colorWidth, int colorHeight, int depthWidth, int depthHeight)
        {
            this.motionDataList = new List<MotionData>();
            this.colorWidth = colorWidth;
            this.colorHeight = colorHeight;
            this.depthWidth = depthWidth;
            this.depthHeight = depthHeight;
            Utility.CreateDirectories(this.dataDir);
            this.recordPath = Path.Combine(dataDir, filename);
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
        public void AddData(int frameNo, DateTime dateTime, ref Body[] bodies, ref byte[] colorPixels, ref ushort[] depthBuffer, ref byte[] bodyIndexBuffer)
        {
            this.SaveImages(frameNo, ref colorPixels, ref depthBuffer, ref bodyIndexBuffer);
            MotionData motionData = new MotionData(frameNo, this.dataDir, dateTime.Ticks, ref bodies);
            motionData.ImageSize = new Tuple<int, int>(this.colorWidth, this.colorHeight);
            motionData.DepthUserSize = new Tuple<int, int>(this.depthWidth, this.depthHeight);
            this.motionDataList.Add(motionData);
        }

        /// <summary>
        /// 骨格情報をファイルに書き出して、リストを空にする。
        /// </summary>
        public void Flush()
        {
            using (MemoryStream ms = new MemoryStream())
            using (FileStream fs = new FileStream(this.recordPath, FileMode.Create, FileAccess.Write))
            {
                var serializer = SerializationContext.Default.GetSerializer<List<MotionData>>();
                serializer.Pack(ms, this.motionDataList);
                byte[] data = ms.ToArray();
                fs.Write(data, 0, data.Length);
            }
            this.motionDataList.Clear();
        }
    }

    public class MotionData
    {
        public MotionData() { }
        public MotionData(int frameNo, string dataDir, long timeStamp, ref Body[] bodies)
        {
            this.FrameNo = frameNo;
            this.ImagePath = Path.Combine(dataDir, frameNo.ToString() + "_image.jpg");
            this.DepthPath = Path.Combine(dataDir, frameNo.ToString() + "_depth.png");
            this.UserPath = Path.Combine(dataDir, frameNo.ToString() + "_user.png");
            this.TimeStamp = timeStamp;
            this.Joints = new Dictionary<User,JointPosition>();
            foreach (Body body in bodies)
            {                                
                if (body.IsTracked)
                {
                    JointPosition joints = new JointPosition(body);
                    this.Joints.Add(body.TrackingId, joints);
                }
            }

        }

        public int FrameNo { get; set; }
        public string ImagePath { get; set; }
        public string DepthPath { get; set; }
        public string UserPath { get; set; }
        public Dictionary<User, JointPosition> Joints { get; set; }
        public long TimeStamp { get; set; }
        public Tuple<int, int> ImageSize { get; set; } // useless?
        public Tuple<int, int> DepthUserSize { get; set; } // useless?
    }

    public class JointPosition
    {
        public JointPosition() { }
        public JointPosition(Body body)
        {
            foreach (Joint joint in body.Joints.Values)
            {
                CameraSpacePoint position = joint.Position;
                PointInfo points = new PointInfo(position.X, position.Y, position.Z, joint.TrackingState);
                this.SetProperty(joint.JointType.ToString(), points);
            }
        }

        private void SetProperty(string key, object data)
        {
            PropertyInfo propertyInfo = this.GetType().GetProperty(key, BindingFlags.Public | BindingFlags.Instance);
            if (propertyInfo.PropertyType == typeof(PointInfo))
            {
                propertyInfo.SetValue(this, (PointInfo)data, null);
            }
        }

        #region JointDefinition

        public User user { get; set; }
        public PointInfo SpineBase { get; set; } // 0
        public PointInfo SpineMid { get; set; } // 1
        public PointInfo Neck { get; set; } // 2
        public PointInfo Head { get; set; } // 3
        public PointInfo ShoulderLeft { get; set; } // 4
        public PointInfo ElbowLeft { get; set; } // 5
        public PointInfo WristLeft { get; set; } // 6
        public PointInfo HandLeft { get; set; } // 7
        public PointInfo ShoulderRight { get; set; } // 8
        public PointInfo ElbowRight { get; set; } // 9
        public PointInfo WristRight { get; set; } // 10
        public PointInfo HandRight { get; set; } // 11
        public PointInfo HipLeft { get; set; } // 12
        public PointInfo KneeLeft { get; set; } // 13
        public PointInfo AnkleLeft { get; set; } // 14
        public PointInfo FootLeft { get; set; } // 15
        public PointInfo HipRight { get; set; } // 16
        public PointInfo KneeRight { get; set; } // 17
        public PointInfo AnkleRight { get; set; } //18
        public PointInfo FootRight { get; set; } // 19
        public PointInfo SpineShoulder { get; set; } // 20
        public PointInfo HandTipLeft { get; set; } // 21
        public PointInfo ThumbLeft { get; set; } // 22
        public PointInfo HandTipRight { get; set; } //23
        public PointInfo ThumbRight { get; set; } //24

        #endregion
    }

    public class PointInfo
    {
        public PointInfo() { }
        public PointInfo(float x, float y, float z, TrackingState state)
        {
            this.X = x;
            this.Y = y;
            this.Z = z;
            this.trackingState = state;
        }

        public float X { get; set; }
        public float Y { get; set; }
        public float Z { get; set; }
        public TrackingState trackingState { get; set; }
        public Point3D GetPoint3D()
        {
            return new Point3D(this.X, this.Y, this.Z);
        }
    }

}
