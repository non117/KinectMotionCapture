using System;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using System.Text;
using System.Runtime.Serialization.Formatters.Binary;

namespace UnityLib
{
    public class Utility
    {
        /// <summary>
        /// バイナリから読み込む
        /// </summary>
        /// <param name="path"></param>
        /// <param name="T"></param>
        /// <returns></returns>
        public static object LoadFromBinary(string path)
        {
            using (FileStream fs = new FileStream(path, FileMode.Open, FileAccess.Read))
            {
                BinaryFormatter f = new BinaryFormatter();
                object obj = f.Deserialize(fs);
                return obj;
            }
        }

        /// <summary>
        /// JointTYpeを表す数値をJointTypeに変換する
        /// </summary>
        /// <param name="jointNum"></param>
        /// <returns></returns>
        public static JointType ConvertIntToJointTYpe(int jointNum)
        {
            Dictionary<int, JointType> mapping = new Dictionary<int, JointType>();
            foreach (JointType jointType in Enum.GetValues(typeof(JointType)))
            {
                mapping.Add((int)jointType, jointType);
            }
            return mapping[jointNum];
        }
    }

    /// <summary>
    /// Unityで読み込むためのJointType
    /// </summary>
    public enum JointType
    {
        SpineBase = 0,
        SpineMid = 1,
        Neck = 2,
        Head = 3,
        ShoulderLeft = 4,
        ElbowLeft = 5,
        WristLeft = 6,
        HandLeft = 7,
        ShoulderRight = 8,
        ElbowRight = 9,
        WristRight = 10,
        HandRight = 11,
        HipLeft = 12,
        KneeLeft = 13,
        AnkleLeft = 14,
        FootLeft = 15,
        HipRight = 16,
        KneeRight = 17,
        AnkleRight = 18,
        FootRight = 19,
        SpineShoulder = 20,
        HandTipLeft = 21,
        ThumbLeft = 22,
        HandTipRight = 23,
        ThumbRight = 24,
    }
}
