using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using System.Reflection;
using System.Runtime.Serialization;
using System.Runtime.Serialization.Json;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;
using Microsoft.Kinect;

namespace KinectMotionCapture
{
    using User = System.UInt64;

    public class MotionDataHandler
    {
        private string dataDir = @"Data"; // そのうちPropertyとかUIからsetするようにしたい
        private string filename = @"BoneInfo.json";
        private string recordPath = "";
        private string DataString = "";

        private int colorWidth = 0;
        private int colorHeight = 0;
        private int depthWidth = 0;
        private int depthHeight = 0;

        public MotionDataHandler(int colorWidth, int colorHeight, int depthWidth, int depthHeight)
        {
            this.colorWidth = colorWidth;
            this.colorHeight = colorHeight;
            this.depthWidth = depthWidth;
            this.depthHeight = depthHeight;
            Utility.CreateDirectories(this.dataDir);
            this.recordPath = Path.Combine(dataDir, filename);
        }
        public void addData(int frameNo, DateTime dateTime, ref Body[] bodies)
        {
            MotionData motionData = new MotionData(frameNo, this.dataDir, dateTime.Ticks, ref bodies);
            motionData.ImageSize = new Tuple<int, int>(this.colorWidth, this.colorHeight);
            motionData.DepthUserSize = new Tuple<int, int>(this.depthWidth, this.depthHeight);
            this.DataString += JsonHandler.getJsonFromObject(motionData);
        }
        public async Task writeData()
        {
            byte[] encodedText = Encoding.Unicode.GetBytes(this.DataString);
            using (FileStream sourceStream = new FileStream(this.recordPath, FileMode.Create, FileAccess.Write, FileShare.None, bufferSize:4096, useAsync:true))
            {
                await sourceStream.WriteAsync(encodedText, 0, encodedText.Length);
            }
        }
    }

    public class JsonHandler
    {
        /// <summary>
        /// json文字列からオブジェクトに変換する
        /// </summary>
        /// <param name="jsonString"></param>
        /// <returns></returns>
        public static MotionData getObjectFromeJson(string jsonString)
        {
            DataContractJsonSerializer serializer = new DataContractJsonSerializer(typeof(MotionData));
            byte[] jsonBytes = Encoding.Unicode.GetBytes(jsonString);
            MemoryStream stream = new MemoryStream(jsonBytes);
            return (MotionData)serializer.ReadObject(stream);
        }
        /// <summary>
        /// オブジェクトをjson文字列に変換する
        /// </summary>
        /// <param name="obj"></param>
        /// <returns></returns>
        public static string getJsonFromObject(object obj)
        {
            MemoryStream stream = new MemoryStream();
            DataContractJsonSerializer serializer = new DataContractJsonSerializer(typeof(MotionData));
            serializer.WriteObject(stream, obj);
            stream.Position = 0;
            StreamReader sr = new StreamReader(stream);
            return sr.ReadToEnd();
        }
    }

    [DataContract]
    public class MotionData
    {
        public MotionData(int frameNo, string dataDir, long timeStamp, ref Body[] bodies)
        {
            this.FrameNo = frameNo;
            this.ImagePath = Path.Combine(dataDir, frameNo.ToString() + "_image.jpg");
            this.DepthPath = Path.Combine(dataDir, frameNo.ToString() + "_depth.png");
            this.UserPath = Path.Combine(dataDir, frameNo.ToString() + "_user.png");
            this.TimeStamp = timeStamp;
            this.Tracking = new Dictionary<User,bool>();
            this.Joints = new Dictionary<User,JointPosition>();
            foreach (Body body in bodies)
            {
                JointPosition joints = new JointPosition(body);
                this.Tracking.Add(body.TrackingId, body.IsTracked);
                if (body.IsTracked)
                {
                    this.Joints.Add(body.TrackingId, joints);
                }
            }

        }

        [DataMember]
        public int FrameNo { get; set; }
        [DataMember]
        public string ImagePath { get; set; }
        [DataMember]
        public string DepthPath { get; set; }
        [DataMember]
        public string UserPath { get; set; }
        //[DataMember]
        //public User[] Users { get; set; } // いるんかこれ
        [DataMember]
        public Dictionary<User, bool> Tracking { get; set; }
        [DataMember]
        public Dictionary<User, JointPosition> Joints { get; set; }
        [DataMember]
        public long TimeStamp { get; set; }
        [DataMember]
        public Tuple<int, int> ImageSize { get; set; } // useless?
        [DataMember]
        public Tuple<int, int> DepthUserSize { get; set; } // useless?
    }

    [DataContract]
    public class JointPosition
    {
        public JointPosition(Body body)
        {
            foreach (Joint joint in body.Joints.Values)
            {
                CameraSpacePoint position = joint.Position;
                Point3 points = new Point3(position.X, position.Y, position.Z, joint.TrackingState);
                this.SetProperty(joint.JointType.ToString(), points);
            }
        }

        private void SetProperty(string key, object data)
        {
            PropertyInfo propertyInfo = this.GetType().GetProperty(key, BindingFlags.Public | BindingFlags.Instance);
            if (propertyInfo.PropertyType == typeof(Point3))
            {
                propertyInfo.SetValue(this, (Point3)data, null);
            }
        }

        #region JointDefinition
        [DataMember]
        public User user { get; set; }
        [DataMember]
        public Point3 SpineBase { get; set; } // 0
        [DataMember]
        public Point3 SpineMid { get; set; } // 1
        [DataMember]
        public Point3 Neck { get; set; } // 2
        [DataMember]
        public Point3 Head { get; set; } // 3
        [DataMember]
        public Point3 ShoulderLeft { get; set; } // 4
        [DataMember]
        public Point3 ElbowLeft { get; set; } // 5
        [DataMember]
        public Point3 WristLeft { get; set; } // 6
        [DataMember]
        public Point3 HandLeft { get; set; } // 7
        [DataMember]
        public Point3 ShoulderRight { get; set; } // 8
        [DataMember]
        public Point3 ElbowRight { get; set; } // 9
        [DataMember]
        public Point3 WristRight { get; set; } // 10
        [DataMember]
        public Point3 HandRight { get; set; } // 11
        [DataMember]
        public Point3 HipLeft { get; set; } // 12
        [DataMember]
        public Point3 KneeLeft { get; set; } // 13
        [DataMember]
        public Point3 AnkleLeft { get; set; } // 14
        [DataMember]
        public Point3 FootLeft { get; set; } // 15
        [DataMember]
        public Point3 HipRight { get; set; } // 16
        [DataMember]
        public Point3 KneeRight { get; set; } // 17
        [DataMember]
        public Point3 AnkleRight { get; set; } //18
        [DataMember]
        public Point3 FootRight { get; set; } // 19
        [DataMember]
        public Point3 SpineShoulder { get; set; } // 20
        [DataMember]
        public Point3 HandTipLeft { get; set; } // 21
        [DataMember]
        public Point3 ThumbLeft { get; set; } // 22
        [DataMember]
        public Point3 HandTipRight { get; set; } //23
        [DataMember]
        public Point3 ThumbRight { get; set; } //24
        #endregion
    }

    [DataContract]
    public class Point3
    {
        public Point3(float x, float y, float z, TrackingState state)
        {
            this.X = x;
            this.Y = y;
            this.Z = z;
            this.trackingState = state;
        }
        [DataMember]
        public float X { get; set; }
        [DataMember]
        public float Y { get; set; }
        [DataMember]
        public float Z { get; set; }
        [DataMember]
        public TrackingState trackingState { get; set; }
    }

}
