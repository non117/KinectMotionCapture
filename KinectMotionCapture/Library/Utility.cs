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
using KinectMotionCapture;
using OpenCvSharp;

namespace KinectMotionCapture
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
        /// 画像を読みまくり
        /// </summary>
        /// <param name="paths"></param>
        /// <returns></returns>
        public static void LoadImages(IEnumerable<string> paths, out IEnumerable<CvMat> images)
        {
            images = paths.Select(path => CvMat.LoadImageM(path, LoadMode.Unchanged));
        }

        /// <summary>
        /// KinectのカラーデータをCvMatに変換する
        /// </summary>
        /// <param name="height"></param>
        /// <param name="width"></param>
        /// <param name="data"></param>
        /// <returns></returns>
        public static CvMat ColorArrayToCvMat(int width, int height, ref byte[] data)
        {
            CvMat mat = new CvMat(height, width, MatrixType.U8C4, data);
            return mat;
        }

        /// <summary>
        /// Kinectの深度データをCvMatに変換する
        /// </summary>
        /// <param name="width"></param>
        /// <param name="height"></param>
        /// <param name="data"></param>
        /// <returns></returns>
        public static CvMat DpethArrayToCvMat(int width, int height, ref ushort[] data)
        {
            CvMat mat = new CvMat(height, width, MatrixType.U16C1, data);
            return mat;
        }

        /// <summary>
        /// kinectのBodyIndexデータをCvMatに変換する
        /// </summary>
        /// <param name="width"></param>
        /// <param name="height"></param>
        /// <param name="data"></param>
        /// <returns></returns>
        public static CvMat BodyIndexArrayToCvMat(int width, int height, ref byte[] data)
        {
            CvMat mat = new CvMat(height, width, MatrixType.U8C1, data);
            return mat;
        }


        /// <summary>
        /// WritableBitmapをfileName.pngに保存する
        /// </summary>
        /// <param name="bitmap"></param>
        /// <param name="fileName"></param>
        public static void SavePngImage(WriteableBitmap bitmap, string fileName)
        {
            using (FileStream stream = new FileStream(fileName + ".png", FileMode.Create, FileAccess.Write))
            {
                PngBitmapEncoder encoder = new PngBitmapEncoder();
                encoder.Frames.Add(BitmapFrame.Create(bitmap));
                encoder.Save(stream);
            }
        }

        /// <summary>
        /// WritableBitmapをfileName.jpgに保存する
        /// </summary>
        /// <param name="bitmap"></param>
        /// <param name="fileName"></param>
        public static void SaveJpegImage(WriteableBitmap bitmap, string fileName)
        {
            using (FileStream stream = new FileStream(fileName + ".jpg", FileMode.Create, FileAccess.Write))
            {
                JpegBitmapEncoder encoder = new JpegBitmapEncoder();
                encoder.Frames.Add(BitmapFrame.Create(bitmap));
                encoder.Save(stream);
            }
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
        /// バイナリから読み込む
        /// </summary>
        /// <param name="path"></param>
        /// <returns></returns>
        public static List<object> LoadFromSequentialBinary(string path)
        {
            IFormatter formatter = new BinaryFormatter();
            List<object> res = new List<object>();
            using (FileStream fs = new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.None))
            {
                while (fs.Length != fs.Position)
                {
                    res.Add(formatter.Deserialize(fs));
                }
            }
            return res;
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
        /// Frameのリストから時刻のメタデータをUnity用に出力する
        /// </summary>
        /// <param name="Frames"></param>
        public static void SaveTimeMetaData(IEnumerable<Frame> frames, string path)
        {
            DateTime startTime = frames.First().Time;
            List<double> times = frames.Select(f => (f.Time - startTime).TotalMilliseconds).ToList();
            Utility.SaveToBinary(times, path);
        }

        /// <summary>
        /// Bodyの関節点をTrackedなものだけにフィルターして返す
        /// </summary>
        /// <param name="joints"></param>
        /// <returns></returns>
        public static Dictionary<JointType, Joint> GetValidJoints(Dictionary<JointType, Joint> joints)
        {
            return joints.Where(j => j.Value.TrackingState == TrackingState.Tracked).ToDictionary(j => j.Key, j => j.Value);
        }

        /// <summary>
        /// Body[]を入力にとってTrackedな関節点をフィルターして返す
        /// </summary>
        /// <param name="bodies"></param>
        /// <param name="userId"></param>
        /// <returns></returns>
        public static Dictionary<JointType, Joint> GetValidJoints(SerializableBody[] bodies, ulong userId)
        {
            IEnumerable<SerializableBody> selectedBodies = bodies.ToList().Where(b => b.TrackingId == userId);
            if (selectedBodies.Count() == 0)
            {
                return null;
            }
            return GetValidJoints(selectedBodies.First().Joints);
        }

        /// <summary>
        /// JointのDicから座標だけのやつに
        /// </summary>
        /// <param name="joints"></param>
        /// <returns></returns>
        public static Dictionary<JointType, CameraSpacePoint> GetJointPointsFromJoints(Dictionary<JointType, Joint> joints)
        {
            return joints.ToDictionary(p => p.Key, p => p.Value.Position);
        }

        /// <summary>
        /// JointのDicから座標だけのやつに
        /// </summary>
        /// <param name="joints"></param>
        /// <returns></returns>
        public static Dictionary<JointType, CameraSpacePoint> GetValidJointPointsFromJoints(Dictionary<JointType, Joint> joints)
        {
            return GetValidJoints(joints).ToDictionary(p => p.Key, p => p.Value.Position);
        }

        /// <summary>
        /// Body[]を入力にとってTrackedな関節点座標をフィルターして返す
        /// </summary>
        /// <param name="bodies"></param>
        /// <param name="userId"></param>
        /// <returns></returns>
        public static Dictionary<JointType, CameraSpacePoint> GetValidJointPoints(SerializableBody[] bodies, ulong userId)
        {
            Dictionary<JointType, Joint> validJoints = GetValidJoints(bodies, userId);
            if (validJoints != null)
            {
                return GetJointPointsFromJoints(GetValidJoints(bodies, userId));
            }
            return null;
        }

        /// <summary>
        /// 時系列Bodyをシリアライズして保存する
        /// </summary>
        /// <param name="bodies"></param>
        /// <param name="path"></param>
        public static void SaveBodySequence(List<Dictionary<JointType, Joint>> bodies, string path)
        {
            List<Dictionary<int, float[]>> serializableBodies = Utility.ConverToCompatibleJoint(bodies);
            Utility.SaveToBinary(serializableBodies, path);
        }

        /// <summary>
        /// 時系列Bodyをシリアライズして保存する
        /// </summary>
        /// <param name="bodies"></param>
        /// <param name="path"></param>
        public static void SaveBodySequence(List<Dictionary<JointType, CvPoint3D64f>> bodies, string path)
        {
            List<Dictionary<int, float[]>> serializableBodies = Utility.ConverToCompatibleJoint(bodies);
            Utility.SaveToBinary(serializableBodies, path);
        }

        /// <summary>
        /// Jointsに座標変換を適用する
        /// </summary>
        /// <param name="joints"></param>
        /// <param name="conversion"></param>
        /// <returns></returns>
        public static Dictionary<JointType, Joint> ApplyConversions(Dictionary<JointType, Joint> joints, CvMat conversion)
        {
            Dictionary<JointType, Joint> newJoints = new Dictionary<JointType, Joint>();
            foreach (JointType jointType in joints.Keys)
            {
                Joint originalJoint = joints[jointType];
                CvPoint3D64f fromPoint = originalJoint.Position.ToCvPoint3D();
                CameraSpacePoint newPoint = CvEx.ConvertPoint3D(fromPoint, conversion).ToCameraSpacePoint();
                originalJoint.Position = newPoint;
                newJoints[jointType] = originalJoint;
            }
            return newJoints;
        }

        /// <summary>
        /// 平均のPointを返す
        /// </summary>
        /// <param name="ps"></param>
        /// <returns></returns>
        public static Point GetAveragePoint(IEnumerable<Point> ps)
        {
            double x = ps.Select(p => p.X).Average();
            double y = ps.Select(p => p.Y).Average();
            return new Point(x, y);
        }

        /// <summary>
        /// 指定された点から主点までの長さ を 焦点距離/2 で割った値の二乗を返します
        /// </summary>
        /// <param name="cameraInfo"></param>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <returns></returns>
        public static double GetRatioSqOfPrincipalToFocal(CameraIntrinsics cameraInfo, double x, double y)
        {
            double dx = (x + 0.5 - cameraInfo.PrincipalPointX) / (cameraInfo.FocalLengthX / 2);
            double dy = (y + 0.5 - cameraInfo.PrincipalPointY) / (cameraInfo.FocalLengthY / 2);
            return dx * dx + dy * dy;
        }

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