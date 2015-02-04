using System;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using System.Text;
using System.Threading.Tasks;

using OpenCvSharp;
using Microsoft.Kinect;

namespace KinectMotionCapture
{
    using Bone = Tuple<JointType, JointType>;

    /// <summary>
    /// あるJointの座標時系列
    /// </summary>
    public class PointSequence
    {
        public List<CvPoint3D64f?> points;
        public List<DateTime> times;
        public JointType jointType;
        
        /// <summary>
        /// こんすとらくたん
        /// </summary>
        /// <param name="points"></param>
        /// <param name="times"></param>
        /// <param name="jointType"></param>
        public PointSequence(List<CvPoint3D64f?> points, List<DateTime> times, JointType jointType)
        {
            this.points = points;
            this.times = times;
            this.jointType = jointType;
        }
        /// <summary>
        /// データを補完する
        /// </summary>
        public void Interpolate()
        {
            List<CvPoint3D64f?> filledPoints = new List<CvPoint3D64f?>();
            List<CvPoint3D64f?> interpolatePoints = new List<CvPoint3D64f?>();
            List<DateTime> interpolateTimes = new List<DateTime>();
            bool isPrevNull = false;
            // 連続するnullな領域を検出して、両端の値をもとに線形補間する
            for (int index = 0; index < this.points.Count(); index++)
            {
                if (points[index] == null)
                {
                    // index 0の場合をアレ
                    if (isPrevNull == false)
                    {
                        if (index > 0)
                        {
                            interpolatePoints.Add(points[index - 1]);
                            interpolateTimes.Add(times[index - 1]);
                        }
                        isPrevNull = true;
                    }
                    interpolatePoints.Add(points[index]);
                    interpolateTimes.Add(times[index]);
                }
                else
                {
                    if (isPrevNull == true)
                    {
                        interpolatePoints.Add(points[index]);
                        interpolateTimes.Add(times[index]);
                        isPrevNull = false;
                        // ここで補完はじめる
                        List<CvPoint3D64f?> fixedPoints = new List<CvPoint3D64f?>();
                        // 頭からの区間で補完できない場合は、代わりの点で埋める
                        if (interpolatePoints.First() == null)
                        {
                            fixedPoints = interpolatePoints.Select(p => interpolatePoints.Last()).ToList();
                        }
                        else
                        {
                            fixedPoints = Utility.LinearInterpolate(interpolateTimes, interpolatePoints);
                        }
                        // 補正済みを追加
                        filledPoints.AddRange(fixedPoints);
                        // clear
                        interpolatePoints.Clear();
                        interpolatePoints.Clear();
                    }
                    filledPoints.Add(points[index]);
                }
            }
            //　末尾の区間で補完できていない場合は代わりの点で埋める
            if (interpolatePoints.Count() > 0)
            {
                List<CvPoint3D64f?> fixedPoints = interpolatePoints.Select(p => interpolatePoints.First()).ToList();
                filledPoints.AddRange(fixedPoints);
            }
            // update
            this.points = filledPoints;
        }

        /// <summary>
        /// 平滑化
        /// </summary>
        public void Smoothing(int n = 10)
        {
            this.Interpolate();
            this.points = Utility.MovingMedianAverage(this.points, n);
        }
        /// <summary>
        /// csvに吐く
        /// </summary>
        public void Dump()
        {
            List<List<string>> outputs = new List<List<string>>();
            DateTime start = this.times[0];
            foreach(var pair in times.Zip(points, (time, point) => new {time, point}))
            {
                List<string> line = new List<string>();
                line.Add((pair.time - start).TotalSeconds.ToString());
                if (pair.point == null)
                {
                    line.Add("");
                    line.Add("");
                    line.Add("");
                }
                else
                {
                    line.Add(pair.point.Value.X.ToString());
                    line.Add(pair.point.Value.Y.ToString());
                    line.Add(pair.point.Value.Z.ToString());
                }
                outputs.Add(line);
            }
            Utility.SaveToCsv(jointType.ToString() + ".csv", outputs);
        }
    }

    /// <summary>
    /// ある瞬間の姿勢
    /// </summary>
    public struct Pose
    {
        public Dictionary<JointType, CvPoint3D64f> joints;
        public DateTime timeStamp;
        
        /// <summary>
        /// こんすとらくたん
        /// </summary>
        /// <param name="joints"></param>
        /// <param name="time"></param>
        public Pose(Dictionary<JointType, CvPoint3D64f> joints, DateTime time)
        {
            this.joints = joints;
            this.timeStamp = time;
        }
        /// <summary>
        /// あるjointTypeの点を返す。null許容。
        /// </summary>
        /// <param name="jointType"></param>
        /// <returns></returns>
        public CvPoint3D64f? GetPoint(JointType jointType)
        {
            if (this.joints.ContainsKey(jointType))
            {
                return this.joints[jointType];
            }
            return null;
        }
        /// <summary>
        /// 胴体中央の座標系に変換する
        /// </summary>
        public void TranslateToTorsoCoordinate()
        {
            Dictionary<JointType, CvPoint3D64f> newJoints = new Dictionary<JointType, CvPoint3D64f>();
            foreach (JointType jointType in this.joints.Keys)
            {
                newJoints[jointType] = this.joints[jointType] - this.joints[JointType.SpineMid];
            }
            this.joints = newJoints;
        }
        /// <summary>
        /// 胴体の方向を計算する
        /// </summary>
        /// <returns></returns>
        public CvPoint3D64f GetBodyDirection()
        {
            CvPoint3D64f spine = joints[JointType.SpineShoulder] - joints[JointType.SpineMid];
            CvPoint3D64f torsoToRightShoulder = joints[JointType.ShoulderRight] - joints[JointType.SpineMid];            
            CvPoint3D64f rightbodyCross = CvEx.Cross(spine, torsoToRightShoulder);            
            CvPoint3D64f torsoToLeftShoulder = joints[JointType.ShoulderLeft] - joints[JointType.SpineMid];
            CvPoint3D64f leftbodyCross = CvEx.Cross(torsoToLeftShoulder, spine);
            CvPoint3D64f rightDirection = CvEx.Normalize(rightbodyCross);
            CvPoint3D64f leftDirection = CvEx.Normalize(leftbodyCross);
            return CvEx.Normalize(rightDirection + leftDirection);
        }
        /// <summary>
        /// Y軸まわりにradで回転させる
        /// </summary>
        /// <param name="rad"></param>
        public void RotateToY(double rad)
        {
            Dictionary<JointType, CvPoint3D64f> newJoints = new Dictionary<JointType, CvPoint3D64f>();
            CvMat mat = CvEx.GetRotation3D(new CvPoint3D64f(0, 1, 0), rad);
            foreach (JointType jointType in this.joints.Keys)
            {
                newJoints[jointType] = CvEx.RotatePoint3D(this.joints[jointType], mat);
            }
            this.joints = newJoints;
        }
    }

    /// <summary>
    /// Jointの接続関係をアレする
    /// </summary>
    public struct JointNode
    {
        public JointNode[] nextNodes;
        public JointType from;
        public JointType to;
        public JointNode(JointType from, JointType to, JointNode[] next)
        {
            this.from = from;
            this.to = to;
            this.nextNodes = next;
        }
        public bool isRoot()
        {
            return this.from == default(JointType) && this.to == default(JointType);
        }
    }

    /// <summary>
    /// 統括するメタデータ
    /// </summary>
    public class MotionMetaData
    {
        private List<Pose> motionLog;
        private Dictionary<JointType, PointSequence> pointSeqs;
        private List<Dictionary<Bone, BoneStatistics>> stats;
        private Dictionary<Bone, double> boneLengthes;
        private JointNode rootNode;

        /// <summary>
        /// 出力用
        /// </summary>
        public List<Dictionary<JointType, CvPoint3D64f>> BodySequence
        {
            get
            {
                return this.motionLog.Select(m => m.joints).ToList();
            }
        }

        /// <summary>
        /// 出力用
        /// </summary>
        public List<DateTime> TimeSequence
        {
            get
            {
                return this.motionLog.Select(m => m.timeStamp).ToList();
            }
        }

        /// <summary>
        /// 骨の長さの代表値を決定する
        /// </summary>
        private void CalcBoneLength()
        {
            foreach (Bone bone in Utility.GetBones())
            {
                // 左右反転を考慮すべきでは
                double length = Utility.CalcMedianLength(stats.Select(d => d[bone]).ToList());
                this.boneLengthes[bone] = length;
            }
        }

        /// <summary>
        /// 骨の長さを探してくる
        /// </summary>
        /// <param name="from"></param>
        /// <param name="to"></param>
        /// <returns></returns>
        private double SearchBoneLength(JointType from, JointType to)
        {
            Bone fromTo = Tuple.Create(from, to);
            Bone toFrom = Tuple.Create(to, from);
            double length;
            if (this.boneLengthes.TryGetValue(fromTo, out length))
            {
                return length;
            }
            else
            {
                return this.boneLengthes[toFrom];
            }
        }

        /// <summary>
        /// 長さの調整された関節点をつくる
        /// </summary>
        /// <param name="origFrom"></param>
        /// <param name="origTo"></param>
        /// <param name="normedFrom"></param>
        /// <param name="length"></param>
        /// <returns></returns>
        private CvPoint3D64f GetFixedJoint(CvPoint3D64f origFrom, CvPoint3D64f origTo, CvPoint3D64f normedFrom, double length)
        {
            return CvEx.Normalize(origTo - origFrom) * length + normedFrom;
        }

        /// <summary>
        /// 骨の長さをなおす。ぜんぶ。
        /// </summary>
        private void FixBoneLength()
        {
            Stack<JointNode> stack = new Stack<JointNode>();
            List<Pose> newLog = new List<Pose>();
            foreach (Pose pose in this.motionLog)
            {
                stack.Clear();
                stack.Push(this.rootNode);
                Dictionary<JointType, CvPoint3D64f> origJoints = pose.joints;
                Dictionary<JointType, CvPoint3D64f> newJoints = new Dictionary<JointType, CvPoint3D64f>();
                newJoints[JointType.SpineBase] = origJoints[JointType.SpineBase];
                while (stack.Count() != 0)
                {
                    JointNode currentNode = stack.Pop();
                    // rootじゃなかったら今のノードに対して骨の修正
                    if (!currentNode.isRoot())
                    {
                        JointType from = currentNode.from;
                        JointType to = currentNode.to;
                        double length = this.SearchBoneLength(from, to);
                        CvPoint3D64f newToPoint = this.GetFixedJoint(origJoints[from], origJoints[to], newJoints[from], length);
                        newJoints[to] = newToPoint;
                    }
                    foreach (JointNode childNode in currentNode.nextNodes)
                    {
                        stack.Push(childNode);
                    }
                }
                newLog.Add(new Pose(newJoints, pose.timeStamp));
            }
            this.motionLog = newLog;
        }

        /// <summary>
        /// SequenceをPoseに戻す
        /// </summary>
        private void SequenceToPose()
        {
            int n = motionLog.Count();
            motionLog.Clear();
            for (int i = 0; i < n; i++)
            {
                Dictionary<JointType, CvPoint3D64f> joints = new Dictionary<JointType, CvPoint3D64f>();
                DateTime time = new DateTime();
                foreach (JointType jointType in Enum.GetValues(typeof(JointType)))
                {
                    joints[jointType] = (CvPoint3D64f)pointSeqs[jointType].points[i];
                    time = pointSeqs[jointType].times[i];
                }
                motionLog.Add(new Pose(joints, time));
            }
        }

        /// <summary>
        /// いにしゃらいず
        /// </summary>
        /// <returns></returns>
        private JointNode InitializeNodes()
        {
            JointNode neckHead = new JointNode(JointType.Neck, JointType.Head, new JointNode[] { });
            JointNode spineShoulderNeck = new JointNode(JointType.SpineShoulder, JointType.Neck, new JointNode[] { neckHead });

            JointNode elbowWristLeft = new JointNode(JointType.ElbowLeft, JointType.WristLeft, new JointNode[] { });
            JointNode shoulderElbowLeft = new JointNode(JointType.ShoulderLeft, JointType.ElbowLeft, new JointNode[] { elbowWristLeft });
            JointNode spineShoulderLeft = new JointNode(JointType.SpineShoulder, JointType.ShoulderLeft, new JointNode[] { shoulderElbowLeft });

            JointNode elbowWristRight = new JointNode(JointType.ElbowRight, JointType.WristRight, new JointNode[] { });
            JointNode shoulderElbowRight = new JointNode(JointType.ShoulderRight, JointType.ElbowRight, new JointNode[] { elbowWristRight });
            JointNode spineShoulderRight = new JointNode(JointType.SpineShoulder, JointType.ShoulderRight, new JointNode[] { shoulderElbowRight });

            JointNode ankleFootLeft = new JointNode(JointType.AnkleLeft, JointType.FootLeft, new JointNode[] { });
            JointNode kneeAnkleLeft = new JointNode(JointType.KneeLeft, JointType.AnkleLeft, new JointNode[] { ankleFootLeft });
            JointNode hipKneeLeft = new JointNode(JointType.HipLeft, JointType.KneeLeft, new JointNode[] { kneeAnkleLeft });
            JointNode spineHipLeft = new JointNode(JointType.SpineBase, JointType.HipLeft, new JointNode[] { hipKneeLeft });

            JointNode ankleFootRight = new JointNode(JointType.AnkleRight, JointType.FootRight, new JointNode[] { });
            JointNode kneeAnkleRight = new JointNode(JointType.KneeRight, JointType.AnkleRight, new JointNode[] { ankleFootRight });
            JointNode hipKneeRight = new JointNode(JointType.HipRight, JointType.KneeRight, new JointNode[] { kneeAnkleRight });
            JointNode spineHipRight = new JointNode(JointType.SpineBase, JointType.HipRight, new JointNode[] { hipKneeRight });

            JointNode spineMidShoulder = new JointNode(JointType.SpineMid, JointType.SpineShoulder, new JointNode[] { spineShoulderNeck, spineShoulderRight, spineShoulderLeft });
            JointNode spineBaseMid = new JointNode(JointType.SpineBase, JointType.SpineMid, new JointNode[] { spineMidShoulder });
            JointNode root = new JointNode(default(JointType), default(JointType), new JointNode[] { spineBaseMid, spineHipRight, spineHipLeft });

            return root;
        }

        /// <summary>
        /// こんすとらくたん
        /// </summary>
        /// <param name="jointsSeq"></param>
        /// <param name="timeSeq"></param>
        /// <param name="stats"></param>
        public MotionMetaData(List<Dictionary<JointType, CvPoint3D64f>> jointsSeq, List<DateTime> timeSeq, List<Dictionary<Bone, BoneStatistics>> stats)
        {
            this.rootNode = new JointNode();
            // poseのデータを生成
            this.motionLog = new List<Pose>();
            foreach (var pair in jointsSeq.Zip(timeSeq, (joints, time) => new { joints, time }))
            {
                Pose pose = new Pose(pair.joints, pair.time);
                this.motionLog.Add(pose);
            }
            // seqに詰め込む
            this.pointSeqs = new Dictionary<JointType, PointSequence>();
            foreach (JointType jointType in Enum.GetValues(typeof(JointType)))
            {
                List<CvPoint3D64f?> points = this.motionLog.Select(m => m.GetPoint(jointType)).ToList();
                pointSeqs[jointType] = new PointSequence(points, timeSeq, jointType);
            }

            // すむーじんぐ            
            foreach (PointSequence pointSeq in this.pointSeqs.Values)
            {
                pointSeq.Smoothing();
            }

            this.stats = stats;
            this.boneLengthes = new Dictionary<Bone, double>();
            this.CalcBoneLength();

            // nodeの接続関係をイニシャライズ            
            this.rootNode = this.InitializeNodes();

            // 書き戻し
            this.SequenceToPose();

            // のーまらいず
            this.FixBoneLength();
        }
    }

    public class User
    {
        public int day;
        public int lesson;
        public string userName;
        public string motionDataPath;
        public string timeDataPath;
        public List<Pose> motionLog;
        public Dictionary<JointType, PointSequence> pointSeqs;
        public Dictionary<string, Tuple<DateTime, DateTime>> timeSlices;
        /// <summary>
        /// こんすとらくた
        /// </summary>
        /// <param name="day"></param>
        /// <param name="lesson"></param>
        /// <param name="name"></param>
        public User(string day, string lesson, string name, string dir)
        {
            this.day = int.Parse(day);
            this.lesson = int.Parse(lesson);
            this.userName = name;
            this.motionDataPath = Path.Combine(dir, name + "FilteredBody.dump");
            this.timeDataPath = Path.Combine(dir, name + "TimeData.dump");
            
            this.motionLog = new List<Pose>();
            this.pointSeqs = new Dictionary<JointType, PointSequence>();
            this.LoadData();
            // type, starttime, endtime
            this.timeSlices = new Dictionary<string, Tuple<DateTime, DateTime>>();
        }
        /// <summary>
        /// でーたをよみこむ
        /// </summary>
        public void LoadData()
        {
            List<Dictionary<JointType, CvPoint3D64f>> jointsSeq = Utility.ConvertToCvPoint((List<Dictionary<int, float[]>>)Utility.LoadFromBinary(this.motionDataPath));
            List<DateTime> timeSeq = (List<DateTime>)Utility.LoadFromBinary(this.timeDataPath);
            foreach (var pair in jointsSeq.Zip(timeSeq, (joints, time) => new { joints, time }))
            {
                Pose pose = new Pose(pair.joints, pair.time);
                this.motionLog.Add(pose);
            }
            // ここでTorso中心にして、肩をそろえる

            foreach (JointType jointType in Enum.GetValues(typeof(JointType)))
            {
                List<CvPoint3D64f?> points = this.motionLog.Select(m => m.GetPoint(jointType)).ToList();
                pointSeqs[jointType] = new PointSequence(points, timeSeq, jointType);
            }
        }

        /// <summary>
        /// 切る時間を追加する
        /// </summary>
        /// <param name="keyMotionType"></param>
        /// <param name="start"></param>
        /// <param name="end"></param>
        public void AddTimeSlices(string keyMotionType, string start, string end)
        {
            string modelTimeString = this.motionLog.First().timeStamp.ToString("yyyy/MM/dd HH:");
            DateTime startTime = DateTime.Parse(modelTimeString + start);
            DateTime endTime = DateTime.Parse(modelTimeString + end);
            this.timeSlices[keyMotionType] = Tuple.Create(startTime, endTime);
        }
    }

    public class MotionAnalyzer
    {
        List<User> users = new List<User>();
        public MotionAnalyzer(string csvFilePath)
        {
            string folder = Path.GetDirectoryName(csvFilePath);
            List<List<string>> csvData = Utility.LoadFromCsv(csvFilePath);
            List<string> userNames = new List<string>();
            foreach (List<string> line in csvData)
            {
                if (line.Contains("Day"))
                {
                    continue;
                }
                string userName = line[2];
                if (!userNames.Contains(userName))
                {
                    User user = new User(line[0], line[1], line[2], folder);
                    user.AddTimeSlices(line[3], line[4], line[5]);
                    this.users.Add(user);
                    userNames.Add(userName);
                }
                else
                {
                    User user = this.users.Where(u => u.userName == userName).First();
                    user.AddTimeSlices(line[3], line[4], line[5]);
                }
            }
        }
    }
}
