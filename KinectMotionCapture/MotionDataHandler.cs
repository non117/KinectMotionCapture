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
    public class MotionDataHandler
    {
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

        [DataContract]
        public class MotionData
        {
            [DataMember]
            public int FrameNo { get; set; }
            [DataMember]
            public string ImagePath { get; set; }
            [DataMember]
            public string DepthPath { get; set; }
            [DataMember]
            public string UserPath { get; set; }
            [DataMember]
            public int[] Users { get; set; }
            [DataMember]
            public Dictionary<int, bool> Tracking { get; set; }
            [DataMember]
            public Joints[] Joints { get; set; }
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
            public void SetJoint(JointType jointType, CameraSpacePoint position)
            {

            }

            private void SetProperty(string key, object data)
            {
                PropertyInfo propertyInfo = this.GetType().GetProperty(key, BindingFlags.Public | BindingFlags.Instance);
                if (propertyInfo.PropertyType == typeof(float[]))
                {
                    propertyInfo.SetValue(this, (float[])data, null);
                }
            }

            [DataMember]
            public int user { get; set; }
            [DataMember]
            public float[] SpineBase { get; set; } // 0
            [DataMember]
            public float[] SpineMid { get; set; } // 1
            [DataMember]
            public float[] Neck { get; set; } // 2
            [DataMember]
            public float[] Head { get; set; } // 3
            [DataMember]
            public float[] ShoulderLeft { get; set; } // 4
            [DataMember]
            public float[] ElbowLeft { get; set; } // 5
            [DataMember]
            public float[] WristLeft { get; set; } // 6
            [DataMember]
            public float[] HandLeft { get; set; } // 7
            [DataMember]
            public float[] ShoulderRight { get; set; } // 8
            [DataMember]
            public float[] ElbowRight { get; set; } // 9
            [DataMember]
            public float[] WristRight { get; set; } // 10
            [DataMember]
            public float[] HandRight { get; set; } // 11
            [DataMember]
            public float[] HipLeft { get; set; } // 12
            [DataMember]
            public float[] KneeLeft { get; set; } // 13
            [DataMember]
            public float[] AnkleLeft { get; set; } // 14
            [DataMember]
            public float[] FootLeft { get; set; } // 15
            [DataMember]
            public float[] HipRight { get; set; } // 16
            [DataMember]
            public float[] KneeRight { get; set; } // 17
            [DataMember]
            public float[] AnkleRight { get; set; } //18
            [DataMember]
            public float[] FootRight { get; set; } // 19
            [DataMember]
            public float[] SpineShoulder { get; set; } // 20
            [DataMember]
            public float[] HandTipLeft { get; set; } // 21
            [DataMember]
            public float[] ThumbLeft { get; set; } // 22
            [DataMember]
            public float[] HandTipRight { get; set; } //23
            [DataMember]
            public float[] ThumbRight { get; set; } //24
        }
        
    }
}
