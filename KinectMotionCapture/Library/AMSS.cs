using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using OpenCvSharp;

namespace KinectMotionCapture.Library
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
        public static double CvPointCostFunction(CvPoint3D64f a, CvPoint3D64f b)
        {
            double cos = CvEx.Cos(a, b);
            if (cos < 0)
            {
                cos = 0;
            }
            return 1 - cos;
        }

        /// <summary>
        /// DPマッチングをジェネリックを使って。まだバグってる。
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="modelSeq"></param>
        /// <param name="targetSeq"></param>
        /// <param name="costFunc"></param>
        /// <returns></returns>
        public static Tuple<double, int[]> DPmatching<T>(List<T> modelSeq, List<T> targetSeq, Func<T, T, double> costFunc)
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
                    double addCost = costFunc(modelSeq[nX], targetSeq[nY]);
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
