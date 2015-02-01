using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Linq;
using System.IO;
using System.Text;
using System.Runtime.Serialization;
using System.Runtime.Serialization.Formatters.Binary;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;

using Microsoft.Kinect;
using OpenCvSharp;

namespace KinectDataAnalyzer
{
    /// <summary>
    /// 雑多なツールを集めたライブラリ
    /// </summary>
    public static class Utility
    {
        /// <summary>
        /// ディープコピーを作成する
        /// Serializableが必要。不要なフィールドはNonSerializedAttribute属性を
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="target"></param>
        /// <returns></returns>
        public static T CloneDeep<T>(this T target)
        {
            object clone = null;
            using (MemoryStream stream = new MemoryStream())
            {
                BinaryFormatter formatter = new BinaryFormatter();
                formatter.Serialize(stream, target);
                stream.Position = 0;
                clone = formatter.Deserialize(stream);
            }
            return (T)clone;
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
                    float x = 0, y = 0, z = 0;
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

        /// <summary>
        /// pathのディレクトリを無ければつくる
        /// </summary>
        /// <param name="path"></param>
        public static void CreateDirectories(string path)
        {
            if (Directory.Exists(path))
            {
                return;
            }
            string fullpath = Path.GetFullPath(path);
            string dir = Path.GetDirectoryName(fullpath);
            CreateDirectories(dir);
            Directory.CreateDirectory(fullpath);

        }

        /// <summary>
        /// バイナリとして保存する
        /// </summary>
        /// <param name="obj"></param>
        /// <param name="path"></param>
        public static void SaveToBinary(object obj, string path){
            using (FileStream fs = new FileStream(path, FileMode.Create, FileAccess.Write))
            {
                BinaryFormatter bf = new BinaryFormatter();
                bf.Serialize(fs, obj);
            }
        }

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
        /// byte[]としてファイルから読み込む
        /// </summary>
        /// <param name="path"></param>
        /// <returns></returns>
        public static byte[] LoadAsBytes(string path)
        {
            using (FileStream fs = new FileStream(path, FileMode.Open, FileAccess.Read))
            {
                byte[] bytes = new byte[fs.Length];
                fs.Read(bytes, 0, bytes.Length);
                return bytes;
            }
        }

        /// <summary>
        /// csvファイルとして出力する
        /// </summary>
        /// <param name="path"></param>
        /// <param name="header"></param>
        /// <param name="data"></param>
        public static void SaveToCsv<T>(string path, List<List<T>> data,  List<string> header = null)
        {
            using (StreamWriter sw = new StreamWriter(path))
            {
                string buffer = "";
                if (header != null)
                {
                    foreach (string str in header)
                    {
                        buffer += str + ",";
                    }
                    buffer.Remove(buffer.Count() - 1);
                    sw.WriteLine(buffer);
                }
                foreach(List<T> line in data)
                {
                    buffer = "";
                    foreach(T cell in line)
                    {
                        buffer += cell.ToString() + ",";
                    }
                    buffer.Remove(buffer.Count() - 1);
                    sw.WriteLine(buffer);
                }
                sw.WriteLine();
            }
        }

        /// <summary>
        /// Unityに吐くためのデータ変換
        /// </summary>
        /// <param name="originalJoints"></param>
        /// <returns></returns>
        public static List<Dictionary<int, float[]>> ConverToCompatibleJoint(List<Dictionary<JointType, Joint>> originalJoints)
        {
            List<Dictionary<int, float[]>> newJoints = new List<Dictionary<int, float[]>>();
            foreach (Dictionary<JointType, Joint> joints in originalJoints)
            {
                Dictionary<int, float[]> body = new Dictionary<int, float[]>();
                foreach (JointType jointType in joints.Keys)
                {
                    CameraSpacePoint position = joints[jointType].Position;
                    float[] points = new float[] { position.X, position.Y, position.Z };                    
                    body.Add((int)jointType, points);
                }
                newJoints.Add(body);
            }
            return newJoints;
        }

        /// <summary>
        /// Unityに吐くためのデータ変換
        /// </summary>
        /// <param name="originalJoints"></param>
        /// <returns></returns>
        public static List<Dictionary<int, float[]>> ConverToCompatibleJoint(List<Dictionary<JointType, CvPoint3D64f>> originalJoints)
        {
            List<Dictionary<int, float[]>> newJoints = new List<Dictionary<int, float[]>>();
            foreach (Dictionary<JointType, CvPoint3D64f> joints in originalJoints)
            {
                Dictionary<int, float[]> body = new Dictionary<int, float[]>();
                foreach (JointType jointType in joints.Keys)
                {
                    CvPoint3D64f position = joints[jointType];
                    float[] points = new float[] { (float)position.X, (float)position.Y, (float)position.Z };
                    body.Add((int)jointType, points);
                }
                newJoints.Add(body);
            }
            return newJoints;
        }

        /// <summary>
        /// 全身の骨を返す
        /// </summary>
        /// <returns></returns>
        public static List<Tuple<JointType, JointType>> GetBones()
        {
            List<Tuple<JointType, JointType>> bones = new List<Tuple<JointType, JointType>>();
            // Torso
            bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // Right Arm
            bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            // Left Arm
            bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            // Right Leg
            bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            // Left Leg
            bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));
            return bones;
        }

        /// <summary>
        /// 足の骨を返す
        /// </summary>
        /// <returns></returns>
        public static List<Tuple<JointType, JointType>> GetLegBones()
        {
            List<Tuple<JointType, JointType>> bones = new List<Tuple<JointType, JointType>>();
            // Torso
            bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // Right Leg
            bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            // Left Leg
            bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));
            return bones;
        }

        // 右半身
        public static List<JointType> RightBody = new List<JointType>(){JointType.ShoulderRight, JointType.ElbowRight, JointType.WristRight,
                                                              JointType.HandRight, JointType.ThumbRight, JointType.HandTipRight,
                                                              JointType.HipRight, JointType.KneeRight, JointType.AnkleRight, JointType.FootRight};
        // 左半身
        public static List<JointType> LeftBody = new List<JointType>(){JointType.ShoulderLeft, JointType.ElbowLeft, JointType.WristLeft,
                                                             JointType.HandLeft, JointType.ThumbLeft, JointType.HandTipLeft,
                                                             JointType.HipLeft, JointType.KneeLeft, JointType.AnkleLeft, JointType.FootLeft};
        // 体幹
        public static List<JointType> Spines = new List<JointType>() { JointType.Head, JointType.Neck, JointType.SpineShoulder, JointType.SpineMid, JointType.SpineBase };

        // 上半身
        public static List<JointType> UpperBody = RightBody.Concat(LeftBody).Concat(Spines).ToList();

        // 下半身
        public static List<JointType> Legs = new List<JointType>() {JointType.HipRight, JointType.HipLeft, JointType.KneeRight, JointType.KneeLeft,
                                                                    JointType.AnkleRight, JointType.AnkleLeft, JointType.FootRight, JointType.FootLeft};

        /*
        /// <summary>
        /// DPマッチングのための座標, Point2D使って書き直すべき
        /// </summary>
        public class Point
        {
            public int x, y;
            public Point(){ this.x = 0; this.y = 0; }
            public Point(int x, int y){ this.x = x; this.y = y; }
        }

        /// <summary>
        /// DPマッチングのためのノード
        /// </summary>
        public class Node
        {
            public float cost;
            public Point prev;
            public Point current;
            public Node() { this.cost = float.MaxValue; }
            public Node(float cost, Point prev, Point cur)
            {
                this.cost = cost; this.prev = prev; this.current = cur;
            }
        }

        /// <summary>
        /// DPマッチングをジェネリックを使って。まだバグってる。
        /// </summary>
        /// <typeparam name="Type"></typeparam>
        /// <param name="modelSeq"></param>
        /// <param name="targetSeq"></param>
        /// <param name="costFunc"></param>
        /// <returns></returns>
        public static Tuple<float, int[]> DPmatching<Type>(List<Type> modelSeq, List<Type> targetSeq, Func<Type, Type, float> costFunc)
        {
            int m = modelSeq.Count();
            int n = targetSeq.Count();
            int x, y;
            Node[,] pathMatrix = new Node[m, n];
            for (int i = 0; i < m; i++)
            {
                for (int j = 0; j < n; j++)
                {
                    pathMatrix[i, j] = new Node();
                }
            }

            Node point = new Node(costFunc(modelSeq[0], targetSeq[0]), new Point(-1, -1), new Point(0, 0));
            List<Node> priorityList = new List<Node>();
            priorityList.Add(point);

            int[] dirX = { 0, 1, 1 };
            int[] dirY = { 1, 1, 0 };

            while (priorityList.Count > 0)
            {
                Node curNode = priorityList[0];
                priorityList.RemoveAt(0);
                
                if (pathMatrix[curNode.current.x, curNode.current.y].cost < curNode.cost){ continue; }
                if (curNode.current.x == m - 1 && curNode.current.y == n - 1) { break; }

                for (int i = 0; i < 3; i++)
                {
                    int nX = curNode.current.x + dirX[i];
                    int nY = curNode.current.y + dirY[i];
                    float addCost = costFunc(modelSeq[nX], targetSeq[nY]);
                    if (nX < m && nY < n && pathMatrix[nX, nY].cost > curNode.cost + addCost)
                    {
                        pathMatrix[nX, nY].cost = curNode.cost + addCost;
                        pathMatrix[nX, nY].prev = curNode.current;
                        priorityList.Add(new Node(pathMatrix[nX, nY].cost, curNode.current, new Point(nX, nY)));
                    }                    
                }
                priorityList = priorityList.OrderBy(node => node.cost).ToList();
            }

            List<Point> minPath = new List<Point>();
            x = m - 1;
            y = n - 1;
            while (x != -1)
            {
                Node node = pathMatrix[x, y];
                minPath.Add(new Point(x, y));
                x = node.prev.x;
                y = node.prev.y;
            }
            int[] res = new int[m];
            foreach (Point p in minPath)
            {
                int i = p.x;
                int j = p.y;
                res[i] = j;
            }
            return new Tuple<float, int[]>(pathMatrix[m - 1, n - 1].cost / minPath.Count, res);
        }
         */ 
    }
}