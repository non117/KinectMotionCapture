using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using OpenCvSharp;
using OpenCvSharp.CPlusPlus.Flann;
using OpenCvSharp.CPlusPlus;

namespace KinectMotionCapture
{
    class FlannColoredModelPoints : IDisposable
    {
        List<Tuple<CvPoint3D64f, CvColor>> _modelPoints;
        public List<Tuple<CvPoint3D64f, CvColor>> ModelPoints { get { return _modelPoints; } }

        Index _flannIndex;
        IndexParams _indexParam;
        Mat _modelDataMat;
        CvMat _modelMat;
        SearchParams _searchParam;
        double _colorScale;
        public static List<Tuple<CvPoint3D64f, CvColor>> DepthColorMatToRealPoints(CvMat depthMat, CvMat colorMat, Func<CvPoint3D64f, CvPoint3D64f> project)
        {
            List<Tuple<CvPoint3D64f, CvColor>> list = new List<Tuple<CvPoint3D64f, CvColor>>();
            unsafe
            {
                short* depthArr = depthMat.DataInt16;
                byte* colorArr = colorMat.DataByte;
                for (int y = 0; y < depthMat.Rows; y++)
                {
                    int offset = y * depthMat.Cols;
                    int offsetColor = (int)Math.Round(CalcEx.ScaleYOnOpenNIImage(y, depthMat, colorMat)) * colorMat.Cols * colorMat.ElemChannels;
                    for (int x = 0; x < depthMat.Cols; x++)
                    {
                        short depth = depthArr[offset + x];
                        if (depth == 0)
                            continue;
                        CvPoint3D64f fromPoint = new CvPoint3D64f(x, y, depth);
                        CvPoint3D64f toPoint = project(fromPoint);
                        int offsetXColor = (int)Math.Round(CalcEx.ScaleXOnOpenNIImage(x, depthMat, colorMat)) * colorMat.ElemChannels;
                        CvColor col = new CvColor(colorArr[offsetColor + offsetXColor + 0], colorArr[offsetColor + offsetXColor + 1], colorArr[offsetColor + offsetXColor + 2]);
                        list.Add(new Tuple<CvPoint3D64f, CvColor>(toPoint, col));
                    }
                }
            }
            return list;
        }

        public FlannColoredModelPoints(CvMat modelDepthMat, CvMat modelColorMat, LocalCoordinateMapper localCoordinateMapper, IndexParams indexParams, SearchParams searchParams, double colorScale)
        {
            _modelPoints = localCoordinateMapper.DepthColorMatToRealPoints(modelDepthMat, modelColorMat);

            _modelMat = new CvMat(_modelPoints.Count, 6, MatrixType.F32C1);
            unsafe
            {
                float* modelArr = _modelMat.DataSingle;
                foreach (var tuple in _modelPoints)
                {
                    *(modelArr++) = (float)tuple.Item1.X;
                    *(modelArr++) = (float)tuple.Item1.Y;
                    *(modelArr++) = (float)tuple.Item1.Z;
                    *(modelArr++) = (float)(tuple.Item2.R * colorScale / 255);
                    *(modelArr++) = (float)(tuple.Item2.G * colorScale / 255);
                    *(modelArr++) = (float)(tuple.Item2.B * colorScale / 255);
                }
            }
            _colorScale = colorScale;
            _modelDataMat = new Mat(_modelMat);
            _indexParam = indexParams;
            _searchParam = searchParams;
            _indexParam.IsEnabledDispose = false;
            _searchParam.IsEnabledDispose = false;
            _flannIndex = new Index(_modelDataMat, _indexParam);
        }

        public FlannColoredModelPoints(CvMat modelDepthMat, CvMat modelColorMat, Func<CvPoint3D64f, CvPoint3D64f> modelMatProjectToRealWorld, IndexParams indexParams, SearchParams searchParams, double colorScale)
        {
            _modelPoints = DepthColorMatToRealPoints(modelDepthMat, modelColorMat, modelMatProjectToRealWorld);

            _modelMat = new CvMat(_modelPoints.Count, 6, MatrixType.F32C1);
            unsafe
            {
                float* modelArr = _modelMat.DataSingle;
                foreach (var tuple in _modelPoints)
                {
                    *(modelArr++) = (float)tuple.Item1.X;
                    *(modelArr++) = (float)tuple.Item1.Y;
                    *(modelArr++) = (float)tuple.Item1.Z;
                    *(modelArr++) = (float)(tuple.Item2.R * colorScale / 255);
                    *(modelArr++) = (float)(tuple.Item2.G * colorScale / 255);
                    *(modelArr++) = (float)(tuple.Item2.B * colorScale / 255);
                }
            }
            _colorScale = colorScale;
            _modelDataMat = new Mat(_modelMat);
            _indexParam = indexParams;
            _searchParam = searchParams;
            _indexParam.IsEnabledDispose = false;
            _searchParam.IsEnabledDispose = false;
            _flannIndex = new Index(_modelDataMat, _indexParam);
        }

        public void KnnSearch(CvPoint3D64f point, CvColor color, out int[] indices, out float[] dists, int knn)
        {
            if (knn == 0)
            {
                indices = new int[0];
                dists = new float[0];
                return;
            }
            float[] input = new float[] { (float)point.X, (float)point.Y, (float)point.Z, (float)(color.R * _colorScale / 255), (float)(color.G * _colorScale / 255), (float)(color.B * _colorScale / 255) };
            _flannIndex.KnnSearch(input, out indices, out dists, knn, _searchParam);
        }

        public void KnnSearch(CvPoint3D64f point, CvColor color, out int[] indices, out float[] dists, float radius, int maxResult)
        {
            float[] input = new float[] { (float)point.X, (float)point.Y, (float)point.Z, (float)(color.R * _colorScale / 255), (float)(color.G * _colorScale / 255), (float)(color.B * _colorScale / 255) };
            int count = 0;
            const int divCount = 2;
            int maxResult2 = maxResult * 4;
            for (int k = divCount - 1; k >= 0; k--)
            {
                float coef = (float)Math.Pow(0.1, k);
                int[] indices2 = new int[maxResult2];
                float[] dists2 = new float[maxResult2];
                _flannIndex.RadiusSearch(input, indices2, dists2, radius * coef, maxResult2, _searchParam);
                for (count = 0; count < maxResult2; count++)
                {
                    if (dists2[count] == 0 && dists2[count] == 0f && ModelPoints[0].Item1 != point)
                        break;
                }
                if (count >= maxResult)
                {
                    indices = new int[maxResult];
                    dists = new float[maxResult];
                    for (int j = 0; j < maxResult; j++)
                    {
                        indices[j] = indices2[j];
                        dists[j] = dists2[j];
                    }
                    return;
                }
                if (k == 0)
                {
                    indices = new int[count];
                    dists = new float[count];
                    for (int j = 0; j < count; j++)
                    {
                        indices[j] = indices2[j];
                        dists[j] = dists2[j];
                    }
                    return;
                }
            }
            this.KnnSearch(point, color, out indices, out dists, count);
        }

        public void Dispose()
        {
            _modelDataMat.Dispose();
            _flannIndex.Dispose();
            _searchParam.Dispose();
        }
    }

    public class ColoredIterativePointMatching : IDisposable
    {
        List<FlannColoredModelPoints> _flannModels = new List<FlannColoredModelPoints>();
        List<CvMat> _modelTransforms = new List<CvMat>();
        Func<float, double> _weightFromDistanceSq;
        public float SearchDistance = 200f;
        public ColoredIterativePointMatching(IList<CvMat> modelDepthMatList, IList<CvMat> modelColorMatList, IList<Func<CvPoint3D64f, CvPoint3D64f>> modelMatProjectToRealWorldList, IList<CvMat> initialModelTransforms, Func<float, double> weightFromDistanceSq, double colorScale)
        {
            if (weightFromDistanceSq == null)
            {
                throw new ArgumentNullException("weightFromDistanceSq");
            }
            if (modelDepthMatList.Count != modelMatProjectToRealWorldList.Count)
            {
                throw new ArgumentException("list length mismatch");
            }
            if (modelDepthMatList.Count != modelColorMatList.Count)
            {
                throw new ArgumentException("list length mismatch");
            }
            for (int i = 0; i < modelDepthMatList.Count; i++)
            {
                FlannColoredModelPoints model = new FlannColoredModelPoints(modelDepthMatList[i], modelColorMatList[i], modelMatProjectToRealWorldList[i], new KDTreeIndexParams(), new SearchParams(), colorScale);
                _flannModels.Add(model);
            }
            if (_flannModels.Count != initialModelTransforms.Count)
            {
                throw new ArgumentException("transform list length must be same as model list length");
            }
            _modelTransforms = initialModelTransforms.ToList();
            _weightFromDistanceSq = weightFromDistanceSq;
        }

        public ColoredIterativePointMatching(IList<CvMat> modelDepthMatList, IList<CvMat> modelColorMatList, List<LocalCoordinateMapper> localCoordinateMappers, IList<CvMat> initialModelTransforms, Func<float, double> weightFromDistanceSq, double colorScale)
        {
            if (weightFromDistanceSq == null)
            {
                throw new ArgumentNullException("weightFromDistanceSq");
            }
            if (modelDepthMatList.Count != modelColorMatList.Count)
            {
                throw new ArgumentException("list length mismatch");
            }
            for (int i = 0; i < modelDepthMatList.Count; i++)
            {
                FlannColoredModelPoints model = new FlannColoredModelPoints(modelDepthMatList[i], modelColorMatList[i], localCoordinateMappers[i], new KDTreeIndexParams(), new SearchParams(), colorScale);
                _flannModels.Add(model);
            }
            if (_flannModels.Count != initialModelTransforms.Count)
            {
                throw new ArgumentException("transform list length must be same as model list length");
            }
            _modelTransforms = initialModelTransforms.ToList();
            _weightFromDistanceSq = weightFromDistanceSq;
        }

        public void SetModelTransforms(IList<CvMat> initialModelTransforms)
        {
            if (_flannModels.Count != initialModelTransforms.Count)
            {
                throw new ArgumentException("transform list length must be same as model list length");
            }
            _modelTransforms = initialModelTransforms.ToList();
        }

        public CvMat CalculateTransform(int targetModelIndex, bool updateInternalModelTransform)
        {
            return this.CalculateTransform(targetModelIndex, updateInternalModelTransform, 1);
        }

        public CvMat CalculateTransform(int targetModelIndex, bool updateInternalModelTransform, double randomSamplingRatio)
        {
            if (targetModelIndex < 0 || targetModelIndex >= _flannModels.Count)
                throw new ArgumentOutOfRangeException("targetModelIndex");
            CoordRotTransConversion coordConverter = new CoordRotTransConversion();
            //CoordConvertSpring coordConverter = new CoordConvertSpring(_modelTransforms[targetModelIndex]);
            //foreach (var point in dataPointListInWorldCoordinate) {
            List<Tuple<CvPoint3D64f, CvColor>> tuples = _flannModels[targetModelIndex].ModelPoints;
            if (randomSamplingRatio < 1)
            {
                Random rand = new Random();
                tuples = tuples.Where(x => rand.NextDouble() < randomSamplingRatio).ToList();
            }
            CvMat targetTransform = _modelTransforms[targetModelIndex];

            List<CvMat> inverseTransforms = new List<CvMat>();
            foreach (CvMat transform in _modelTransforms)
            {
                CvMat inv = CvEx.InitCvMat(transform);
                transform.Invert(inv);
                inverseTransforms.Add(inv);
            }
            float searchDistanceSq = this.SearchDistance * this.SearchDistance;
            Parallel.ForEach(tuples, tuple =>
            {
                CvPoint3D64f point = tuple.Item1;
                CvColor color = tuple.Item2;
                //foreach (var point in points) {
                CvPoint3D64f worldPoint = CvEx.ConvertPoint3D(point, targetTransform);
                int minModelIndex = -1;
                int minPointIndex = -1;
                float minDistanceSq = float.MaxValue;
                for (int modelIndex = 0; modelIndex < _flannModels.Count; modelIndex++)
                {
                    if (modelIndex == targetModelIndex)
                        continue;
                    CvPoint3D64f inversePoint = CvEx.ConvertPoint3D(worldPoint, inverseTransforms[modelIndex]);
                    int[] indices;
                    float[] distances;
                    _flannModels[modelIndex].KnnSearch(inversePoint, color, out indices, out distances, 1);
                    if (indices.Length >= 1)
                    {
                        float distanceSq = distances[0];
                        if (distanceSq <= searchDistanceSq)
                        {
                            if (distanceSq < minDistanceSq)
                            {
                                minModelIndex = modelIndex;
                                minPointIndex = indices[0];
                                minDistanceSq = distanceSq;
                            }
                        }
                    }
                }
                if (minModelIndex != -1)
                {
                    Tuple<CvPoint3D64f, CvColor> bestModelPoint = _flannModels[minModelIndex].ModelPoints[minPointIndex];
                    double weightTo = 1.0 / (Math.Abs(bestModelPoint.Item1.Z - 1500 / 1000f) + 5000 / 1000f);
                    double weightFrom = 1.0 / (Math.Abs(point.Z - 1500 / 1000f) + 5000 / 1000f);
                    //weightFrom = weightTo = 1;
                    double weight = _weightFromDistanceSq(minDistanceSq) * weightFrom * weightTo;
                    CvPoint3D64f from = CvEx.ConvertPoint3D(point, targetTransform);
                    CvPoint3D64f to = CvEx.ConvertPoint3D(bestModelPoint.Item1, _modelTransforms[minModelIndex]);

                    coordConverter.PutPoint(from, to, weight);
                }
            });
            CvMat ret = coordConverter.Solve() * targetTransform;
            if (updateInternalModelTransform)
            {
                _modelTransforms[targetModelIndex] = ret.Clone();
            }
            return ret;
        }
        public List<CvMat> CalculateTransformSequntially(double randomSamplingRatio, int repetition)
        {
            for (int j = 0; j < repetition; j++)
            {
                List<CvMat> ret = new List<CvMat>();
                for (int i = 0; i < _flannModels.Count; i++)
                {
                    ret.Add(CalculateTransform(i, true, randomSamplingRatio));
                }
                this.SetModelTransforms(ret);
                this.SearchDistance = (this.SearchDistance - 50 / 1000f) * 0.997f + 50 / 1000f;
            }
            return _modelTransforms.ToList();
        }

        public void Dispose()
        {
            _flannModels.ForEach(x => x.Dispose());
            _flannModels.Clear();
        }
    }
    public class ColoredMultiplePointMatching : IDisposable
    {
        List<FlannColoredModelPoints> _flannModels = new List<FlannColoredModelPoints>();
        List<CvMat> _modelTransforms = new List<CvMat>();
        Func<float, double> _weightFromDistanceSq;
        double _colorScale;
        public ColoredMultiplePointMatching(Func<float, double> weightFromDistanceSq, double colorScale)
        {
            if (weightFromDistanceSq == null)
            {
                throw new ArgumentNullException("weightFromDistanceSq");
            }
            _colorScale = colorScale;
            _weightFromDistanceSq = weightFromDistanceSq;
        }

        public void PushModel(CvMat depthMat, CvMat colorMat, Func<CvPoint3D64f, CvPoint3D64f> modelMatProjectToRealWorld, CvMat modelTransform)
        {
            FlannColoredModelPoints model = new FlannColoredModelPoints(depthMat, colorMat, modelMatProjectToRealWorld, new KDTreeIndexParams(), new SearchParams(), _colorScale);
            _modelTransforms.Add(modelTransform);
            _flannModels.Add(model);
        }
        public int ModelCount { get { return _flannModels.Count; } }
        public void PopModel()
        {
            if (_flannModels.Count == 0)
                throw new InvalidOperationException("No Model Data");

            int index = _flannModels.Count - 1;
            FlannColoredModelPoints model = _flannModels[index];
            model.Dispose();
            _flannModels.RemoveAt(index);
            _modelTransforms.RemoveAt(index);
        }


        public CvMat CalculateTransform(CvMat depthMat, CvMat colorMat, Func<CvPoint3D64f, CvPoint3D64f> targetProjectToRealWorld, CvMat targetTransform, int repetition, double randomSamplingRatio)
        {

            List<Tuple<CvPoint3D64f, CvColor>> tuples = FlannColoredModelPoints.DepthColorMatToRealPoints(depthMat, colorMat, targetProjectToRealWorld);

            if (randomSamplingRatio < 1)
            {
                Random rand = new Random();
                tuples = tuples.Where(x => rand.NextDouble() < randomSamplingRatio).ToList();
            }

            List<CvMat> inverseTransforms = new List<CvMat>();
            foreach (CvMat transform in _modelTransforms)
            {
                CvMat inv = CvEx.InitCvMat(transform);
                transform.Invert(inv);
                inverseTransforms.Add(inv);
            }

            CoordRotTransConversion coordConverter = new CoordRotTransConversion();
            Parallel.ForEach(tuples, tuple =>
            {
                CvPoint3D64f point = tuple.Item1;
                CvColor color = tuple.Item2;
                CvPoint3D64f worldPoint = CvEx.ConvertPoint3D(point, targetTransform);
                int minModelIndex = -1;
                int minPointIndex = -1;
                float minDistanceSq = float.MaxValue;
                for (int modelIndex = 0; modelIndex < _flannModels.Count; modelIndex++)
                {
                    CvPoint3D64f inversePoint = CvEx.ConvertPoint3D(worldPoint, inverseTransforms[modelIndex]);
                    int[] indices;
                    float[] distances;
                    _flannModels[modelIndex].KnnSearch(inversePoint, color, out indices, out distances, 1);
                    if (indices.Length >= 1)
                    {
                        float distanceSq = distances[0];
                        if (distanceSq <= 200 * 200)
                        {
                            if (distanceSq < minDistanceSq)
                            {
                                minModelIndex = modelIndex;
                                minPointIndex = indices[0];
                                minDistanceSq = distanceSq;
                            }
                        }
                    }
                }
                if (minModelIndex != -1)
                {
                    Tuple<CvPoint3D64f, CvColor> bestModelPoint = _flannModels[minModelIndex].ModelPoints[minPointIndex];
                    double weightTo = 1.0 / (Math.Abs(bestModelPoint.Item1.Z - 1500) + 5000);
                    double weightFrom = 1.0 / (Math.Abs(point.Z - 1500) + 5000);
                    double weight = _weightFromDistanceSq(minDistanceSq) * weightFrom * weightTo;
                    CvPoint3D64f from = CvEx.ConvertPoint3D(point, targetTransform);
                    CvPoint3D64f to = CvEx.ConvertPoint3D(bestModelPoint.Item1, _modelTransforms[minModelIndex]);

                    coordConverter.PutPoint(from, to, weight);
                }
            });
            if (coordConverter.PointCount < 10)
                return targetTransform.Clone();
            CvMat ret = coordConverter.Solve() * targetTransform;

            return ret;
        }

        public void Dispose()
        {
            _flannModels.ForEach(x => x.Dispose());
            _flannModels.Clear();
        }
    }
}

