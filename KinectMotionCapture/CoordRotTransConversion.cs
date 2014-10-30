using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using OpenCvSharp;
using System.Windows;
using System.Collections.Concurrent;

namespace KinectMotionCapture
{
    /// <summary>
    /// 3次元列ベクトル用座標変換
    /// </summary>
    public interface ICoordConversion3D
    {
        void PutPoint(OpenCvSharp.CvPoint3D64f from, OpenCvSharp.CvPoint3D64f to, double weight);
        OpenCvSharp.CvMat Solve();
        int PointCount { get; }
        void Clear();
    }

    public class CoordRotTransConversion : ICoordConversion3D
    {
        ConcurrentBag<Tuple<CvPoint3D64f, CvPoint3D64f, double>> _correspondings = new ConcurrentBag<Tuple<CvPoint3D64f, CvPoint3D64f, double>>();
        public int PointCount { get { return _correspondings.Count; } }
        public CoordRotTransConversion()
        {

        }

        public void PutPoint(CvPoint3D64f from, CvPoint3D64f to, double weight)
        {
            _correspondings.Add(new Tuple<CvPoint3D64f, CvPoint3D64f, double>(from, to, weight));
        }

        public static void Test()
        {
            CoordRotTransConversion crtc = new CoordRotTransConversion();
            Random rand = new Random();
            CvMat cov = new CvMat(4, 4, MatrixType.F64C1);
            cov.Zero();
            cov[0, 3] = rand.NextDouble() * 200 - 500;
            cov[1, 3] = rand.NextDouble() * 200 - 500;
            cov[2, 3] = rand.NextDouble() * 200 - 500;
            cov[3, 3] = 1.0;
            CvMat rotateConversion;
            cov.GetSubRect(out rotateConversion, new CvRect(0, 0, 3, 3));
            CvMat rotVector = new CvMat(1, 3, MatrixType.F64C1);
            rotVector[0, 0] = rand.NextDouble() * 10 - 5;
            rotVector[0, 1] = rand.NextDouble() * 10 - 5;
            rotVector[0, 2] = rand.NextDouble() * 10 - 5;
            Cv.Rodrigues2(rotVector, rotateConversion);

            for (int i = 0; i < 100000; i++)
            {
                CvPoint3D64f from = new CvPoint3D64f(rand.NextDouble() * rand.NextDouble() * rand.NextDouble() * 200 - 500, rand.NextDouble() * 200 - 500, rand.NextDouble() * 200 - 500);
                CvMat fromMat = new CvMat(4, 1, MatrixType.F64C1);
                CvEx.FillCvMat(fromMat, new double[] { from.X, from.Y, from.Z, 1.0 });
                CvMat toMat = cov * fromMat;
                CvPoint3D64f to = new CvPoint3D64f(toMat[0, 0], toMat[0, 1], toMat[0, 2]);
                crtc.PutPoint(from, to, 1.0);
            }
            CvMat ret = crtc.Solve();
            Func<CvMat, CvMat, string> show = (i, o) =>
            {
                StringBuilder str = new StringBuilder();
                str.AppendFormat("{0} = {1} / {2}\n", "11", i[0, 0].ToString("0.000"), o[0, 0].ToString("0.000"));
                str.AppendFormat("{0} = {1} / {2}\n", "12", i[0, 1].ToString("0.000"), o[0, 1].ToString("0.000"));
                str.AppendFormat("{0} = {1} / {2}\n", "13", i[0, 2].ToString("0.000"), o[0, 2].ToString("0.000"));
                str.AppendFormat("{0} = {1} / {2}\n", "14", i[0, 3].ToString("0.000"), o[0, 3].ToString("0.000"));
                str.AppendFormat("{0} = {1} / {2}\n", "21", i[1, 0].ToString("0.000"), o[1, 0].ToString("0.000"));
                str.AppendFormat("{0} = {1} / {2}\n", "22", i[1, 1].ToString("0.000"), o[1, 1].ToString("0.000"));
                str.AppendFormat("{0} = {1} / {2}\n", "23", i[1, 2].ToString("0.000"), o[1, 2].ToString("0.000"));
                str.AppendFormat("{0} = {1} / {2}\n", "24", i[1, 3].ToString("0.000"), o[1, 3].ToString("0.000"));
                str.AppendFormat("{0} = {1} / {2}\n", "31", i[2, 0].ToString("0.000"), o[2, 0].ToString("0.000"));
                str.AppendFormat("{0} = {1} / {2}\n", "32", i[2, 1].ToString("0.000"), o[2, 1].ToString("0.000"));
                str.AppendFormat("{0} = {1} / {2}\n", "33", i[2, 2].ToString("0.000"), o[2, 2].ToString("0.000"));
                str.AppendFormat("{0} = {1} / {2}\n", "34", i[2, 3].ToString("0.000"), o[2, 3].ToString("0.000"));
                str.AppendFormat("{0} = {1} / {2}\n", "41", i[3, 0].ToString("0.000"), o[3, 0].ToString("0.000"));
                str.AppendFormat("{0} = {1} / {2}\n", "42", i[3, 1].ToString("0.000"), o[3, 1].ToString("0.000"));
                str.AppendFormat("{0} = {1} / {2}\n", "43", i[3, 2].ToString("0.000"), o[3, 2].ToString("0.000"));
                str.AppendFormat("{0} = {1} / {2}\n", "44", i[3, 3].ToString("0.000"), o[3, 3].ToString("0.000"));
                return str.ToString();
            };
            MessageBox.Show(show(cov, ret));
        }

        public CvMat Solve()
        {
            // 重心の計算
            CvPoint3D64f fromCenter = new CvPoint3D64f();
            CvPoint3D64f toCenter = new CvPoint3D64f();
            double weightSum = 0;
            foreach (var tuple in _correspondings)
            {
                fromCenter += tuple.Item1 * tuple.Item3;
                toCenter += tuple.Item2 * tuple.Item3;
                weightSum += tuple.Item3;
            }
            if (weightSum != 0)
            {
                fromCenter *= 1.0 / weightSum;
                toCenter *= 1.0 / weightSum;
            }
            // q: quaternion; 4x1
            // fn, tn: from[n], to[n]; 3x1                
            // Xn: (tn - fn, (tn+fn)×[1,0,0], (tn+fn)×[0,1,0], (tn+fn)×[0,0,1]); 3x4
            // M: Σi(Xi^t Wi Xi); 4x4
            // Wi: I; 3x3
            // J = q^t Mq -> min

            // 最小二乗法
            using (CvMat M = new CvMat(4, 4, MatrixType.F64C1))
            {
                M.Zero();
                foreach (var tuple in _correspondings)
                {
                    // 重心からの距離
                    CvPoint3D64f fromVector = tuple.Item1 - fromCenter;
                    CvPoint3D64f toVector = tuple.Item2 - toCenter;

                    using (CvMat Xi = new CvMat(3, 4, MatrixType.F64C1))
                    {
                        CvPoint3D64f diff = toVector - fromVector;
                        CvPoint3D64f sum = toVector + fromVector;
                        CvPoint3D64f second = CvEx.Cross(sum, new CvPoint3D64f(1, 0, 0));
                        CvPoint3D64f third = CvEx.Cross(sum, new CvPoint3D64f(0, 1, 0));
                        CvPoint3D64f fourth = CvEx.Cross(sum, new CvPoint3D64f(0, 0, 1));
                        CvEx.FillCvMat(Xi, new double[] { diff.X, second.X, third.X, fourth.X, diff.Y, second.Y, third.Y, fourth.Y, diff.Z, second.Z, third.Z, fourth.Z });
                        using (CvMat XiTranspose = Xi.Transpose())
                        using (CvMat addend = XiTranspose * Xi * tuple.Item3)
                        {
                            M.Add(addend, M);
                        }
                    }
                }
                using (CvMat MTemp = CvEx.CloneCvMat(M))
                using (CvMat eVals = new CvMat(4, 1, MatrixType.F64C1))
                using (CvMat eVects = new CvMat(4, 4, MatrixType.F64C1))
                {

                    //Cv.EigenVV(MTemp, eVects, eVals, 0.000001);
                    Cv.SVD(MTemp, eVals, eVects, null, SVDFlag.U_T | SVDFlag.ModifyA);
                    int minEIndex = 3;
                    /*
                    if (false)
                    {
                        double minE = double.MaxValue;
                        for (int i = 0; i < 4; i++)
                        {
                            double eVal = Math.Abs(eVals[i, 0]);
                            if (eVal < minE)
                            {
                                minE = eVal;
                                minEIndex = i;
                            }
                        }
                    }
                     */
                    CvMat ret = new CvMat(4, 4, MatrixType.F64C1);
                    ret.Zero();
                    ret[3, 3] = 1.0;
                    CvMat rotateConversion;
                    /*
                    if (false)
                    {
                        // こっちの変換はほとんど恒等のときに誤差が大きい
                        CvMat q = eVects.GetRow(minEIndex);

                        // クォータニオンから回転ベクトルを計算
                        double theta = Math.Acos(q[0, 0]) * 2;
                        double sin = Math.Sin(theta / 2);
                        CvPoint3D64f rot = new CvPoint3D64f(q[0, 1] / sin * theta, q[0, 2] / sin * theta, q[0, 3] / sin * theta);
                        // 回転ベクトルから回転行列を計算
                        ret.GetSubRect(out rotateConversion, new CvRect(0, 0, 3, 3));
                        using (CvMat rotVector = new CvMat(1, 3, MatrixType.F64C1))
                        {
                            rotVector[0, 0] = rot.X;
                            rotVector[0, 1] = rot.Y;
                            rotVector[0, 2] = rot.Z;
                            Cv.Rodrigues2(rotVector, rotateConversion);
                        }
                    }
                    else
                    {*/
                        CvMat rotationMat = CvEx.QuaternionToMat3D(eVects[minEIndex, 0], eVects[minEIndex, 1], eVects[minEIndex, 2], eVects[minEIndex, 3]);
                        ret.GetSubRect(out rotateConversion, new CvRect(0, 0, 3, 3));
                        rotationMat.Copy(rotateConversion);
                    //}
                    // 回転後の重心の並進成分の計算
                    using (CvMat fromCenterMat = new CvMat(3, 1, MatrixType.F64C1))
                    {
                        CvEx.FillCvMat(fromCenterMat, new double[] { fromCenter.X, fromCenter.Y, fromCenter.Z });
                        using (CvMat rotFromCenterMat = rotateConversion * fromCenterMat)
                        {
                            CvPoint3D64f rotFromCenter = new CvPoint3D64f(rotFromCenterMat[0, 0], rotFromCenterMat[1, 0], rotFromCenterMat[2, 0]);
                            CvPoint3D64f offset = toCenter - rotFromCenter;

                            ret[0, 3] = offset.X;
                            ret[1, 3] = offset.Y;
                            ret[2, 3] = offset.Z;

                            return ret;
                        }
                    }
                }
            }
        }

        #region ICoordConversion3D メンバー


        public void Clear()
        {
            _correspondings = new ConcurrentBag<Tuple<CvPoint3D64f, CvPoint3D64f, double>>();
        }

        #endregion
    }
}