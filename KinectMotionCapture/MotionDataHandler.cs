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
        private string dataRoot = @"Data"; // そのうちPropertyとかUIからsetするようにしたい
        public MotionDataHandler()
        {
            Utility.CreateDirectories(this.dataRoot);
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
        public MotionData(int frameNo, string dataDir, int imageW, int imageH, int depthW, int depthH, long timeStamp)
        {
            this.FrameNo = frameNo;
            this.ImagePath = Path.Combine(dataDir, frameNo.ToString() + "_image.jpg");
            this.DepthPath = Path.Combine(dataDir, frameNo.ToString() + "_depth.png");
            this.UserPath = Path.Combine(dataDir, frameNo.ToString() + "_user.png");
            this.TimeStamp = timeStamp;
            this.ImageSize = new Tuple<int, int>(imageW, imageH);
            this.DepthUserSize = new Tuple<int, int>(depthW, depthH);
        }
        private void AddBody(ref Body body)
        {
            Joints joints = new Joints(ref body);
            this.Tracking.Add(body.TrackingId, body.IsTracked);
            if (body.IsTracked)
            {
                this.Joints.Add(body.TrackingId, joints);
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
        [DataMember]
        public User[] Users { get; set; } // いるんかこれ
        [DataMember]
        public Dictionary<User, bool> Tracking { get; set; }
        [DataMember]
        public Dictionary<User, Joints> Joints { get; set; }
        [DataMember]
        public long TimeStamp { get; set; }
        [DataMember]
        public Tuple<int, int> ImageSize { get; set; }
        [DataMember]
        public Tuple<int, int> DepthUserSize { get; set; }
    }

    [DataContract]
    public class Joints
    {
        public Joints(ref Body body)
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
