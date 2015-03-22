using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Microsoft.Kinect;
using OpenCvSharp;

namespace KinectMotionCapture
{
    class PointRefiner
    {
        Dictionary<JointType, CvPoint3D64f> standardJoints;
        // 骨と表面情報を受け取って、標準骨格のまわりにマッピングしていく。
        // 基準となる標準骨格はとりあえず、最初のフレームの最初のレコード
        // とりあえず胴体近傍を出力することが目標
        // 最近傍を計算するアレ. ヒューリスティックを導入するか、全探索か。
        // 可視化を急ぐ
    }
}
