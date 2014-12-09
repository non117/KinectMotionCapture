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
        public static JointType ConvertIntToJointType(int jointNum)
        {
            Dictionary<int, JointType> mapping = new Dictionary<int, JointType>();
            foreach (JointType jointType in Enum.GetValues(typeof(JointType)))
            {
                mapping.Add((int)jointType, jointType);
            }
            return mapping[jointNum];
        }

        /// <summary>
        /// 単純移動平均. 真ん中を基準とする.
        /// </summary>
        /// <param name="sequence"></param>
        /// <param name="windowSize"></param>
        /// <returns></returns>
        public static List<Dictionary<int, float[]>> MovingAverage(List<Dictionary<int, float[]>> sequence, int windowSize)
        {
            List<Dictionary<int, float[]>> res = new List<Dictionary<int, float[]>>();
            int length = sequence.Count();
            // 現在のフレームから前後に探索する窓を定義
            List<int> window = new List<int>();
            for (int i = -(windowSize / 2); i <= (windowSize / 2); i++)
                window.Add(i);

            for (int frameNo = 0; frameNo < length; frameNo++)
            {
                Dictionary<int, float[]> newJoints = new Dictionary<int, float[]>();
                foreach (JointType key in Enum.GetValues(typeof(JointType)))
                {
                    float x = 0, y=0, z=0;
                    int count = 0;
                    foreach (int i in window)
                    {
                        int index = frameNo + i;
                        if (index < 0 || index >= length)
                            continue;
                        float[] points;
                        if (sequence[index].TryGetValue((int)key, out points))
                        {
                            // ダミーデータチェック
                            if (points[0] > 1e10)
                                continue;
                            x += points[0]; y += points[1]; z += points[2];
                            count++;
                        }
                    }
                    if (count == 0)
                    {
                        newJoints[(int)key] = new float[] { float.MaxValue, float.MaxValue, float.MaxValue };
                    }
                    else
                    {
                        newJoints[(int)key] = new float[] { x / count, y / count, z / count };
                    }
                }
                res.Add(newJoints);
            }
            return res;
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
