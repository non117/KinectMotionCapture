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
        /// depthとuserIndexをまとめて保存する
        /// </summary>
        /// <param name="width"></param>
        /// <param name="height"></param>
        /// <param name="user"></param>
        /// <param name="depthes"></param>
        /// <returns></returns>
        public static CvMat MergeDepthAndUserMat(int width, int height, ushort[] depthes, byte[] bodyIndexes)
        {
            CvMat dst = Cv.CreateMat(height, width, MatrixType.U16C3);
            CvMat depthMat = new CvMat(height, width, MatrixType.U16C1, depthes);
            ushort[] userIndexes = new ushort[bodyIndexes.Length];
            for (int i = 0; i < bodyIndexes.Length; i++)
            {
                userIndexes[i] = (ushort)BitConverter.ToInt16(new byte[] { bodyIndexes[i], 0 }, 0);
            }
            CvMat userMat = new CvMat(height, width, MatrixType.U16C1, userIndexes);
            Cv.Merge(depthMat, userMat, null, null, dst);
            return dst;
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
        /// デスクトップ直下のパスを作る
        /// </summary>
        /// <param name="filename"></param>
        /// <returns></returns>
        public static string CreateDesktopPath(string filename)
        {
            return System.IO.Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.DesktopDirectory), filename);
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
        /// plyファイルとして保存する
        /// </summary>
        /// <param name="data"></param>
        /// <param name="path"></param>
        public static void SaveToPly(List<Tuple<CvPoint3D64f, CvColor>> data, string path)
        {
            using (StreamWriter fs = new StreamWriter(path, false, Encoding.ASCII))
            {
                int num = data.Count;
                fs.WriteLine("ply\nformat ascii 1.0\ncomment KinectMotionCapture");
                fs.WriteLine("element vertex " + num.ToString());
                fs.WriteLine("property float x\nproperty float y\nproperty float z");
                fs.WriteLine("property uchar red\nproperty uchar green\nproperty uchar blue");
                fs.WriteLine("end_header\n");

                foreach(var tuple in data)
                {
                    string line = String.Format("{0} {1} {2} {3} {4} {5}",
                        tuple.Item1.X, tuple.Item1.Y, tuple.Item1.Z, tuple.Item2.R, tuple.Item2.G, tuple.Item2.B);
                    fs.WriteLine(line);
                }
                
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
                    MotionMetaData md = (MotionMetaData)formatter.Deserialize(fs);
                    md.isValid = true;
                    res.Add(md);
                    //if (md.TimeStamp.Minute < 20)
                    //{
                    //    res.Add(md);
                    //}
                    //if (md.TimeStamp.Hour == 11 && md.TimeStamp.Minute >= 20)
                    //{
                    //    break;
                    //}
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
                    buffer = buffer.Remove(buffer.Count() - 1);
                    sw.WriteLine(buffer);
                }
                sw.WriteLine();
            }
        }

        /// <summary>
        /// Vectorをcsvファイルとして出力する
        /// </summary>
        /// <param name="points"></param>
        /// <param name="times"></param>
        /// <param name="filename"></param>
        public static void SaveToCsv(string filename, List<CvPoint3D64f> points, List<DateTime> times)
        {
            List<List<string>> outputs = new List<List<string>>();
            DateTime start = times[0];
            foreach (var pair in times.Zip(points, (time, point) => new { time, point }))
            {
                List<string> line = new List<string>();
                line.Add((pair.time - start).TotalSeconds.ToString());
                line.Add(pair.point.X.ToString());
                line.Add(pair.point.Y.ToString());
                line.Add(pair.point.Z.ToString());
                outputs.Add(line);
            }
            string path = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.DesktopDirectory), filename + ".csv");
            Utility.SaveToCsv(path, outputs);
        }

        /// <summary>
        /// Vectorをcsvファイルとして出力する
        /// </summary>
        /// <param name="points"></param>
        /// <param name="times"></param>
        /// <param name="filename"></param>
        public static void SaveToCsv(string filename, List<CvPoint3D32f> points, List<DateTime> times)
        {
            List<List<string>> outputs = new List<List<string>>();
            DateTime start = times[0];
            foreach (var pair in times.Zip(points, (time, point) => new { time, point }))
            {
                List<string> line = new List<string>();
                line.Add((pair.time - start).TotalSeconds.ToString());
                line.Add(pair.point.X.ToString());
                line.Add(pair.point.Y.ToString());
                line.Add(pair.point.Z.ToString());
                outputs.Add(line);
            }
            string path = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.DesktopDirectory), filename + ".csv");
            Utility.SaveToCsv(path, outputs);
        }

        /// <summary>
        /// csvファイルを読み込む
        /// </summary>
        /// <param name="path"></param>
        /// <returns></returns>
        public static List<List<string>> LoadFromCsv(string path)
        {
            List<List<string>> data = new List<List<string>>();
            using (StreamReader reader = new StreamReader(File.OpenRead(path)))
            {
                while (!reader.EndOfStream)
                {
                    string line = reader.ReadLine();
                    data.Add(line.Split(',').Select(s => s.Trim()).ToList());
                }
            }
            return data;
        }

        /// <summary>
        /// JointをCvPointに変換するやつ
        /// </summary>
        /// <param name="joints"></param>
        /// <returns></returns>
        public static Dictionary<JointType, CvPoint3D64f> ToCvJoints(this Dictionary<JointType, Joint> joints)
        {
            return joints.ToDictionary(p => p.Key, p => (CvPoint3D64f)p.Value.Position.ToCvPoint3D());
        }

        /// <summary>
        /// JointをCvPointに変換するやつ
        /// </summary>
        /// <param name="joints"></param>
        /// <returns></returns>
        public static Dictionary<JointType, CvPoint3D64f> ToCvJoints(this Dictionary<JointType, Joint> joints, CvMat conv)
        {
            return joints.ToDictionary(p => p.Key, p => CvEx.ConvertPoint3D(p.Value.Position.ToCvPoint3D(), conv));
        }

        /// <summary>
        /// JointsをあるJointTypeの座標系に変換する
        /// </summary>
        /// <param name="joints"></param>
        /// <param name="jointType"></param>
        /// <returns></returns>
        public static Dictionary<JointType, CvPoint3D64f> ConvertJointsByJointType(this Dictionary<JointType, CvPoint3D64f> joints, JointType jointType)
        {
            CvPoint3D64f originJointPos = joints[jointType];
            return joints.ToDictionary(p => p.Key, p => p.Value - originJointPos);
        }
        
        /// <summary>
        /// 変換するやつ
        /// </summary>
        /// <param name="joints"></param>
        /// <param name="conv"></param>
        /// <returns></returns>
        public static Dictionary<JointType, CvPoint3D64f> ApplyConversion(this Dictionary<JointType, CvPoint3D64f> joints, CvMat conv)
        {
            return joints.ToDictionary(p => p.Key, p => CvEx.ConvertPoint3D(p.Value, conv));
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
        public static List<JointType> UpperBody = new List<JointType>(){JointType.ShoulderLeft, JointType.ElbowLeft, JointType.WristLeft, JointType.HandLeft, JointType.ThumbLeft,
                                                                        JointType.ShoulderRight, JointType.ElbowRight, JointType.WristRight, JointType.HandRight, JointType.ThumbRight,
                                                                        JointType.Head, JointType.Neck, JointType.SpineShoulder, JointType.SpineMid};
        // 下半身
        public static List<JointType> Legs = new List<JointType>() {JointType.HipRight, JointType.HipLeft, JointType.KneeRight, JointType.KneeLeft,
                                                                    JointType.AnkleRight, JointType.AnkleLeft, JointType.FootRight, JointType.FootLeft};

        // 手
        public static List<JointType> Hands = new List<JointType>(){JointType.HandRight, JointType.ThumbRight, JointType.HandTipRight,
                                                                    JointType.HandLeft, JointType.ThumbLeft, JointType.HandTipLeft};

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
        /// MedianAverage
        /// </summary>
        /// <param name="seq"></param>
        /// <param name="rate"></param>
        /// <returns></returns>
        public static double CalcMedianAverage(List<double> seq, double rate = 0.3)
        {
            int skip = (int)(seq.Count() * rate);
            int take = seq.Count() - skip * 2;
            seq.Sort();
            // 上下30%を削除
            seq = seq.Skip(skip).Take(take).ToList();
            return seq.Average();            
        }

        /// <summary>
        /// CvPointの平均
        /// </summary>
        /// <param name="seq"></param>
        /// <param name="rate"></param>
        /// <returns></returns>
        public static CvPoint3D64f CalcMedianAverage(List<CvPoint3D64f> seq, double rate = 0.3)
        {
            double x = CalcMedianAverage(seq.Select(p => p.X).ToList());
            double y = CalcMedianAverage(seq.Select(p => p.Y).ToList());
            double z = CalcMedianAverage(seq.Select(p => p.Z).ToList());
            return new CvPoint3D64f(x, y, z);
        }

        /// <summary>
        /// 中央値移動平均. 真ん中を基準とする.
        /// </summary>
        /// <param name="sequence"></param>
        /// <param name="windowSize"></param>
        /// <returns></returns>
        public static List<CvPoint3D64f> MovingMedianAverage(List<CvPoint3D64f> sequence, int windowSize)
        {
            List<CvPoint3D64f> res = new List<CvPoint3D64f>();
            int length = sequence.Count();
            // 現在のフレームから前後に探索する窓を定義
            List<int> window = new List<int>();
            for (int i = -(windowSize / 2); i <= (windowSize / 2); i++)
                window.Add(i);

            List<double> xs = new List<double>();
            List<double> ys = new List<double>();
            List<double> zs = new List<double>();
            for (int frameNo = 0; frameNo < length; frameNo++)
            {
                xs.Clear(); ys.Clear(); zs.Clear();
                foreach (int i in window)
                {
                    int index = frameNo + i;
                    if (index < 0 || index >= length)
                        continue;
                    xs.Add(sequence[index].X);
                    ys.Add(sequence[index].Y);
                    zs.Add(sequence[index].Z);
                }
                double x = CalcMedianAverage(xs);
                double y = CalcMedianAverage(ys);
                double z = CalcMedianAverage(zs);
                res.Add(new CvPoint3D64f(x, y, z));
            }
            return res;
        }

        /// <summary>
        /// double sequenceの頻度分布を集計するための関数
        /// </summary>
        /// <param name="seq"></param>
        /// <param name="cutUnit"></param>
        /// <returns></returns>
        public static SortedDictionary<double, int> CountFrequencyOfNumbers(IEnumerable<double> seq, int cutUnit)
        {
            SortedDictionary<double, int> map = new SortedDictionary<double, int>();
            foreach(double point in seq)
            {
                double key = Math.Round(point, cutUnit, MidpointRounding.AwayFromZero);
                if(map.ContainsKey(key))
                {
                    map[key]++;
                }
                else
                {
                    map[key] = 0;
                }
            }
            return map;
        }

        /// <summary>
        /// トラジェクトリを生成する
        /// </summary>
        /// <param name="sequence"></param>
        /// <returns></returns>
        public static List<CvPoint3D64f> MakeTrajectory(List<CvPoint3D64f> sequence)
        {
            List<CvPoint3D64f> res = new List<CvPoint3D64f>();
            for (int index = 0; index < sequence.Count() - 1; index++)
            {
                double x = sequence[index + 1].X - sequence[index].X;
                double y = sequence[index + 1].Y - sequence[index].Y;
                double z = sequence[index + 1].Z - sequence[index].Z;
                res.Add(new CvPoint3D64f(x, y, z));
            }
            res.Add(res.Last());
            return res;
        }

        /// <summary>
        /// 微分する. 無理やり値を突っ込むことで長さを変えない
        /// </summary>
        /// <param name="sequence"></param>
        /// <param name="times"></param>
        /// <returns></returns>
        public static List<CvPoint3D64f> Difference(List<CvPoint3D64f> sequence, List<DateTime> times)
        {
            List<CvPoint3D64f> res = new List<CvPoint3D64f>();
            for (int index = 1; index < sequence.Count() - 1; index++)
            {
                double h = 2 * (times[index + 1] - times[index - 1]).TotalSeconds;
                CvPoint3D64f prev = sequence[index - 1];
                CvPoint3D64f next = sequence[index + 1];
                double x = (next.X - prev.X) / h;
                double y = (next.Y - prev.Y) / h;
                double z = (next.Z - prev.Z) / h;
                res.Add(new CvPoint3D64f(x, y, z));
            }
            res.Insert(0, res.First());
            res.Add(res.Last());
            return res;
        }

        /// <summary>
        /// スムージング有りの二次微分
        /// </summary>
        /// <param name="sequence"></param>
        /// <param name="times"></param>
        /// <returns></returns>
        public static List<CvPoint3D64f> SecondaryDifferenceAndSmoothing(List<CvPoint3D64f> sequence, List<DateTime> times)
        {
            List<CvPoint3D64f> firstDiff = MovingMedianAverage(Difference(sequence, times), 10);
            List<CvPoint3D64f> secondDiff = MovingMedianAverage(Difference(firstDiff, times), 10);
            return secondDiff;
        }

        /// <summary>
        /// 中央値の骨の長さを得る
        /// </summary>
        /// <param name="stats"></param>
        /// <returns></returns>
        public static double CalcMedianLength(List<BoneStatistics> stats)
        {
            List<double> minSqs = new List<double>();
            List<double> maxSqs = new List<double>();
            foreach (BoneStatistics stat in stats)
            {
                minSqs.Add(stat.minLengthSq);
                maxSqs.Add(stat.maxLengthSq);
            }
            minSqs.Sort();
            maxSqs.Sort();
            double minSq = CalcMedianAverage(minSqs, 0.2);
            double maxSq = CalcMedianAverage(maxSqs, 0.2);
            double length = (Math.Sqrt(minSq) + Math.Sqrt(maxSq)) / 2;
            return length;
        }

        /// <summary>
        /// Unity用に吐かれたデータを変換しておく
        /// </summary>
        /// <param name="rawJointsSeq"></param>
        /// <returns></returns>
        public static List<Dictionary<JointType, CvPoint3D64f>> ConvertToCvPoint(List<Dictionary<int, float[]>> rawJointsSeq)
        {
            List<Dictionary<JointType, CvPoint3D64f>> newJointsSeq = new List<Dictionary<JointType, CvPoint3D64f>>();
            foreach (Dictionary<int, float[]> joints in rawJointsSeq)
            {
                Dictionary<JointType, CvPoint3D64f> newJoints = new Dictionary<JointType, CvPoint3D64f>();
                foreach (int jointNo in joints.Keys)
                {
                    JointType jointType = ConvertIntToJointType(jointNo);
                    float[] jointAry = joints[jointNo];
                    CvPoint3D64f point = new CvPoint3D64f(jointAry[0], jointAry[1], jointAry[2]);
                    newJoints[jointType] = point;
                }
                newJointsSeq.Add(newJoints);
            }
            return newJointsSeq;
        }

        /// <summary>
        /// CvPointを時間で線形補間
        /// </summary>
        /// <param name="data"></param>
        /// <returns></returns>
        public static List<CvPoint3D64f?> LinearInterpolate(List<DateTime> times, List<CvPoint3D64f?> points)
        {
            List<CvPoint3D64f?> res = new List<CvPoint3D64f?>();
            DateTime start = times.First();
            double t0 = 0;
            double t1 = (times.Last() - times.First()).TotalMilliseconds;
            CvPoint3D64f p0 = (CvPoint3D64f)points.First();
            CvPoint3D64f p1 = (CvPoint3D64f)points.Last();
            res.Add(p0);
            for (int i = 1; i < times.Count() - 1; i++)
            {
                DateTime now = times[i];
                double t = (now - start).TotalMilliseconds;
                double x = LinearInterpolate(t0, p0.X, t1, p1.X, t);
                double y = LinearInterpolate(t0, p0.Y, t1, p1.Y, t);
                double z = LinearInterpolate(t0, p0.Z, t1, p1.Z, t);
                res.Add(new CvPoint3D64f(x, y, z));
            }
            res.Add(p1);
            return res;
        }

        /// <summary>
        /// 線形補間
        /// </summary>
        /// <param name="x0"></param>
        /// <param name="y0"></param>
        /// <param name="x1"></param>
        /// <param name="y1"></param>
        /// <param name="x"></param>
        /// <returns></returns>
        public static double LinearInterpolate(double x0, double y0, double x1, double y1, double x)
        {
            double alpha = (x - x0) / (x1 - x0);
            return y0 + alpha * (y1 - y0);
        }

        /// <summary>
        /// 2つのベクトルの角度をラジアンで返す
        /// </summary>
        /// <param name="one"></param>
        /// <param name="two"></param>
        /// <returns></returns>
        public static double GetVectorRadian(CvPoint3D64f one, CvPoint3D64f two)
        {
            // masterにslaveを回転させるコード
            //CvPoint3D64f slave = new CvPoint3D64f(0.7071, 0, 0.7071);
            //CvPoint3D64f master = new CvPoint3D64f(0, 0, 1);
            //double rad = Utility.GetVectorRadian(master, slave);
            //CvMat mat = CvEx.GetRotation3D(new CvPoint3D64f(0, 1, 0), rad);
            //CvPoint3D64f to = CvEx.RotatePoint3D(slave, mat);
            one.Y = 0;
            two.Y = 0;
            CvPoint3D64f norm = CvEx.Cross(two, one);
            double cos = CvEx.Cos(two, one);
            double rad = Math.Acos(cos);
            if (norm.Y < 0)
            {
                rad = -1 * rad;
            }            
            return rad;
        }
    }
}