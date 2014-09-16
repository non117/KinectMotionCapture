using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using OpenCvSharp;

namespace KinectMotionCapture
{
    /// <summary>
    /// メモリ節約用の最小二乗法クラス
    /// </summary>
    public class MemorySavingLeastSquare
    {
        CvMat _left;
        CvMat _right;
        int _dimension;
        int _count;
        public int Count { get { return _count; } }
        /// <summary>
        /// 次元を指定するコンストラクタ
        /// </summary>
        /// <param name="dimension"></param>
        public MemorySavingLeastSquare(int dimension)
        {
            _dimension = dimension;
            _left = CvEx.InitCvMat(dimension, dimension, MatrixType.F64C1);
            _right = CvEx.InitCvMat(dimension, 1, MatrixType.F64C1);
            _left.Zero();
            _right.Zero();
        }

        /// <summary>
        /// 係数列と実測値をサンプルに加えます
        /// </summary>
        /// <param name="left">係数列</param>
        /// <param name="right">実測値</param>
        public void PutPoint(double[] left, double right)
        {
            _count++;
            for (int i = 0; i < _dimension; i++)
            {
                for (int j = 0; j < _dimension; j++)
                {
                    _left[i, j] += left[i] * left[j];
                }
                _right[i] += left[i] * right;
            }
        }
        /// <summary>
        /// 最小二乗法で解を得ます
        /// </summary>
        /// <returns></returns>
        public double[] Solve()
        {
            CvMat invLeft = CvEx.InitCvMat(_left);
            _left.Invert(invLeft, InvertMethod.Cholesky);
            CvMat ret = invLeft * _right;
            return ret.Select(r => r.Val0).ToArray();
        }

        public static void Test()
        {
            Random rand = new Random();
            int dim = rand.Next(2, 7);
            MemorySavingLeastSquare msls = new MemorySavingLeastSquare(dim);
            double[] ans = Enumerable.Range(0, dim).Select(x => rand.NextDouble() * 200 - 100).ToArray();
            List<double[]> leftArr = new List<double[]>();
            List<double> rightArr = new List<double>();
            for (int k = 0; k < 200; k++)
            {
                double[] left = Enumerable.Range(0, dim).Select(x => rand.NextDouble() * 200 - 100).ToArray();
                double right = Enumerable.Range(0, dim).Select(i => left[i] * ans[i]).Sum() + rand.NextDouble() * 10 - 5;
                msls.PutPoint(left, right);
                leftArr.Add(left);
                rightArr.Add(right);
            }
            double[] ans1 = CvEx.Solve(leftArr, rightArr, InvertMethod.Svd);
            double[] ans2 = msls.Solve();
            System.Windows.MessageBox.Show(string.Join(", ", ans.Select(x => x.ToString("0.00000"))) + "\n" + string.Join(", ", ans1.Select(x => x.ToString("0.00000"))) + "\n" + string.Join(", ", ans2.Select(x => x.ToString("0.00000"))));
        }
    }
}
