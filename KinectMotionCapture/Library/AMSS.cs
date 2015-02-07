using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using OpenCvSharp;

namespace KinectMotionCapture
{
    class AMSS
    {
        /// <summary>
        /// DPマッチングのための座標
        /// </summary>
        private class Point
        {
            public int x, y;
            public Point(){ this.x = 0; this.y = 0; }
            public Point(int x, int y){ this.x = x; this.y = y; }
        }

        /// <summary>
        /// DPマッチングのためのノード
        /// </summary>
        private class Node
        {
            public double cost;
            public Point prev;
            public Point current;
            public Node() { this.cost = double.MaxValue; }
            public Node(double cost, Point prev, Point cur)
            {
                this.cost = cost; this.prev = prev; this.current = cur;
            }
        }

        /// <summary>
        /// CvPoint用のコサイン類似度コスト関数
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        public static float CvPointCostFunction(CvPoint3D32f a, CvPoint3D32f b)
        {
            float cos = (float)CvEx.Cos(a, b);
            if (cos < 0)
            {
                cos = 0;
            }
            return 1 - cos;
        }

        public static float DPmatching<T>(List<T> modelSeq, List<T> targetSeq, Func<T, T, float> costFunc)
        {
            int m = modelSeq.Count();
            int n = targetSeq.Count();
            int x, y;
            float maxValue = 1000.0f;
            float[,] costMatrix = new float[m, n];
            for (x = 0; x < m; x++)
            {
                for (y = 0; y < n; y++)
                {
                    costMatrix[x, y] = costFunc(modelSeq[x], targetSeq[y]);
                }
            }
            float[,] similarities = new float[m, n];
            float[,] trans_num = new float[m, n];
            int loopNum = System.Math.Min(m, n);

            //初期処理　S(0,0) = sim(0,0) S(i,0) = S(0,i) = 1000000f;
            similarities[0, 0] = costMatrix[0, 0];
            for (int num = 1; num < m; num++)
            {
                similarities[num, 0] = maxValue;
            }
            for (int num = 1; num < n; num++)
            {
                similarities[0, num] = maxValue;
            }

            //初期処理 ステップ数をすべてゼロに
            trans_num[0, 0] = 1; //`１回しか試行できない場合
            for (int num = 1; num < m; num++)
            {
                trans_num[num, 0] = 0;
            }
            for (int num = 1; num < n; num++)
            {
                trans_num[0, num] = 0;
            }
            //それ以外の部分を埋める　s(i,j)
            //ループ回数だけ、n行目、n列目のマスを埋める,ポインタは斜めに移動していくイメージ
            for (int num = 1; num < loopNum; num++)
            {
                //num + num2行目の数値を決める,ポインタを上方向に移動していくイメージ
                for (int num2 = 0; num2 < m - num; num2++)
                {
                    float n1 = similarities[num + num2 - 1, num - 1] + costMatrix[num + num2, num]; //
                    float n2 = similarities[num + num2 - 1, num] + costMatrix[num + num2, num];
                    float n3 = similarities[num + num2, num - 1] + costMatrix[num + num2, num];
                    if ((n1 <= n2) && (n1 <= n3))
                    {
                        similarities[num2 + num, num] = n1;
                        trans_num[num2 + num, num] = trans_num[num2 + num - 1, num - 1] + 1;
                    }
                    else if ((n2 <= n1) && (n2 <= n3))
                    {
                        similarities[num2 + num, num] = n2;
                        trans_num[num2 + num, num] = trans_num[num2 + num - 1, num] + 1;
                    }
                    else if ((n3 <= n1) && (n3 <= n2))
                    {
                        similarities[num2 + num, num] = n3;
                        trans_num[num2 + num, num] = trans_num[num2 + num, num - 1] + 1;
                    }
                }

                //num + num2列目の数値を決める num2 = 0の時は上と重複計算になるのでやらない,ポインタを右方向に移動していくイメージ
                for (int num2 = 1; num2 < n - num; num2++)
                {
                    float n1 = similarities[num - 1, num2 + num - 1] + costMatrix[num, num2 + num];
                    float n2 = similarities[num - 1, num2 + num] + costMatrix[num, num2 + num];
                    float n3 = similarities[num, num2 + num - 1] + costMatrix[num, num2 + num];
                    if ((n1 <= n2) && (n1 <= n3))
                    {
                        similarities[num, num2 + num] = n1;
                        trans_num[num, num2 + num] = trans_num[num - 1, num2 + num - 1] + 1;
                    }
                    else if ((n2 <= n1) && (n2 <= n3))
                    {
                        similarities[num, num2 + num] = n2;
                        trans_num[num, num2 + num] = trans_num[num - 1, num2 + num] + 1;
                    }
                    else if ((n3 <= n1) && (n3 <= n2))
                    {
                        similarities[num, num2 + num] = n3;
                        trans_num[num, num2 + num] = trans_num[num, num2 + num - 1] + 1;
                    }
                }
            }
            return similarities[m - 1, n - 1] / trans_num[m - 1, n - 1];
        }


        /// <summary>
        /// DPマッチングをジェネリックを使って。まだバグってる。
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="modelSeq"></param>
        /// <param name="targetSeq"></param>
        /// <param name="costFunc"></param>
        /// <returns></returns>
        public static Tuple<double, int[]> DPmatching2<T>(List<T> modelSeq, List<T> targetSeq, Func<T, T, float> costFunc)
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
            for (x = 0; x < m; x++)
            {
                pathMatrix[x, 0].cost = 0;
            }
            for (y = 0; y < n; y++)
            {
                pathMatrix[0, y].cost = 0;
            }


            Node point = new Node(costFunc(modelSeq[0], targetSeq[0]), new Point(-1, -1), new Point(0, 0));
            pathMatrix[0, 0] = point;
            List<Node> priorityList = new List<Node>();
            priorityList.Add(point);

            int[] dirX = { 0, 1, 1 };
            int[] dirY = { 1, 1, 0 };

            while (priorityList.Count > 0)
            {
                Node curNode = priorityList[0];
                priorityList.RemoveAt(0);

                if (pathMatrix[curNode.current.x, curNode.current.y].cost < curNode.cost)
                {
                    continue;
                }
                if (curNode.current.x == m - 1 && curNode.current.y == n - 1)
                {
                    break;
                }

                for (int i = 0; i < 3; i++)
                {
                    int nX = curNode.current.x + dirX[i];
                    int nY = curNode.current.y + dirY[i];
                    //double addCost = costFunc(modelSeq[nX], targetSeq[nY]);
                    if (nX < m && nY < n && pathMatrix[nX, nY].cost > curNode.cost + costFunc(modelSeq[nX], targetSeq[nY]))
                    {
                        pathMatrix[nX, nY].cost = curNode.cost + costFunc(modelSeq[nX], targetSeq[nY]);
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
            int[] shortestPath = new int[m];
            foreach (Point p in minPath)
            {
                int i = p.x;
                int j = p.y;
                shortestPath[i] = j;
            }
            return new Tuple<double, int[]>(pathMatrix[m - 1, n - 1].cost / minPath.Count, shortestPath);
        }
    }
}
