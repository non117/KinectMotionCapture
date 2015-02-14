using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using MathNet.Numerics;
using Microsoft.Kinect;

namespace KinectMotionCapture
{
    class SimilarityAnalyzer
    {
        List<Result> results;
        List<JointType> validJoints;
        string[] armVars;
        string[] legVars;

        public SimilarityAnalyzer()
        {
            string tempData = @"tempSimData.dump";
            if (System.IO.File.Exists(tempData))
            {
                this.results = (List<Result>)Utility.LoadFromBinary(tempData);
            }
            else
            {
                string path0 = @"C:\Users\non\Desktop\Data\AllSimilaritiesTillDay2.dump";
                string path1 = @"C:\Users\non\Desktop\Data\AllSimilaritiesDay3.dump";
                this.results = (List<Result>)Utility.LoadFromBinary(path0);
                this.results.AddRange((List<Result>)Utility.LoadFromBinary(path1));
                Utility.SaveToBinary(this.results, tempData);
            }
            this.results = this.results.Where(r => r.variableName != "Position_SpineMid").ToList();

            this.armVars = (string[])Utility.LoadFromBinary(@"C:\Users\non\Desktop\Data\ArmVariableNames.dump");
            this.legVars = (string[])Utility.LoadFromBinary(@"C:\Users\non\Desktop\Data\LegVariableNames.dump");
            this.validJoints = new List<JointType>();
        }

        public SimilarityAnalyzer(string filePath, bool legMode)
        {
            this.results = (List<Result>)Utility.LoadFromBinary(filePath);
            this.results = this.results.Where(r => r.variableName != "Acceleration_SpineMid" && r.variableName != "Position_SpineMid").ToList();
            List<JointType> invalidJoints = Utility.Hands;
            invalidJoints.Add(JointType.FootRight);
            invalidJoints.Add(JointType.FootLeft);
            // 手を教えてもらっていない場合
            if (legMode)
            {
                invalidJoints.AddRange(new JointType[] { JointType.ElbowLeft, JointType.ElbowRight, JointType.WristRight, JointType.WristLeft });
            }
            this.validJoints = Utility.RightBody.Concat(Utility.LeftBody).Concat(Utility.Spines).Where(j => !invalidJoints.Contains(j)).ToList();
            // 変数取得
            this.armVars = (string[])Utility.LoadFromBinary(@"C:\Users\non\Desktop\Data\ArmVariableNames.dump");
            this.legVars = (string[])Utility.LoadFromBinary(@"C:\Users\non\Desktop\Data\LegVariableNames.dump");
        }
        public void CalcSpecificStepSimilarities(string stepName, string[] students, int maxOrder)
        {
            IEnumerable<Result> tempResults = this.results.Where(r => r.stepName == stepName && students.Contains(r.userName));
            List<List<Result>> results = new List<List<Result>>();
            List<List<string>> tempRankings = new List<List<string>>();
            foreach (string student in students)
            {
                IEnumerable<Result> variables = tempResults.Where(r => r.userName == student).OrderBy(r => r.similarities.ToList().Average()).Take(maxOrder);
                results.Add(variables.ToList());
                if (variables.Count() != maxOrder)
                {
                    List<string> line = new List<string>();
                    for (int i = 0; i < maxOrder; i++)
                    {
                        line.Add("NaN");
                    }
                    tempRankings.Add(line);
                }
                else
                {
                    List<string> line = new List<string>();
                    foreach (Result result in variables)
                    {
                        string name = this.SwapLR(result.variableName);
                        line.Add(name);
                    }
                    tempRankings.Add(line);
                    //tempRankings.Add(variables.Select(r => r.variableName).ToList());
                }
            }
            List<List<string>> rankings = new List<List<string>>();
            


            string path =  System.IO.Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.DesktopDirectory), "RankingOf" + stepName + ".csv");
            Utility.SaveToCsv(path, tempRankings, students.ToList());
        }

        public string SwapLR(string name)
        {
            name = name.Replace("Right", "tempL");
            name = name.Replace("Left", "tempR");
            name = name.Replace("tempL", "Left");
            name = name.Replace("tempR", "Right");
            return name;
        }

        public bool IsDecrease(double one, double two, double three)
        {
            if (one >= two)
            {
                return true;
            }
            return false;
        }

        [Serializable]
        public struct VarAndVariance
        {
            public string variable;
            public string step;
            public double var1;
            public double var2;
            public double var3;
            public double decreaseScore;
            public VarAndVariance(string variable, string stepName, double var1, double var2, double var3)
            {
                this.step = stepName;
                this.variable = variable;
                this.var1 = var1;
                this.var2 = var2;
                this.var3 = var3;
                // ちっちゃい方から引くことでマイナスにしてソートしやすく
                this.decreaseScore = var3 - var1;
            }
        }

        public void DayVariableVariance()
        {
            string[] stepNames = new string[] { "A", "B1", "C1", "D1", "E", "B2", "C2", "D2", "F", "G1", "H1",
                                                "I", "J", "G2", "H2", "K", "L"};
            string[] day1Students = Enumerable.Range(1, 22).Select(i => "Student" + i.ToString()).ToArray();
            string[] day2Students = Enumerable.Range(23, 10).Select(i => "Student" + i.ToString()).ToArray();
            string[] day3Students = Enumerable.Range(33, 8).Select(i => "Student" + i.ToString()).ToArray();
            List<VarAndVariance> temp = new List<VarAndVariance>();
            foreach (string stepName in stepNames)
            {
                foreach (string variable in this.legVars)
                {
                    IEnumerable<Result> tempResults = this.results.Where(r => r.variableName == variable && r.stepName == stepName);
                    var day1 = tempResults.Where(r => day1Students.Contains(r.userName));
                    var day2 = tempResults.Where(r => day2Students.Contains(r.userName));
                    var day3 = tempResults.Where(r => day3Students.Contains(r.userName));

                    if (day1.Count() == 0 || day2.Count() == 0 || day3.Count() == 0)
                        continue;

                    double day1Variance = CalcEx.GetStdDev(day1.Select(r => (double)r.similarities.ToList().Average()));
                    double day2Variance = CalcEx.GetStdDev(day2.Select(r => (double)r.similarities.ToList().Average()));
                    double day3Variance = CalcEx.GetStdDev(day3.Select(r => (double)r.similarities.ToList().Average()));
                    if (this.IsDecrease(day1Variance, day2Variance, day3Variance))
                    {
                        temp.Add(new VarAndVariance(variable, stepName, day1Variance, day2Variance, day3Variance));
                    }
                }
            }
            temp = temp.OrderBy(v => v.decreaseScore).ToList();
            List<List<string>> outputs = new List<List<string>>();
            foreach (VarAndVariance v in temp)
            {
                List<string> line = new List<string>();
                line.Add(this.SwapLR(v.variable));
                line.Add(v.step);
                line.Add(v.var1.ToString());
                line.Add(v.var2.ToString());
                line.Add(v.var3.ToString());
                outputs.Add(line);
            }
            string path =  System.IO.Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.DesktopDirectory), "variances.dump");
            string csvpath = System.IO.Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.DesktopDirectory), "variances.csv");
            Utility.SaveToCsv(csvpath, outputs);
            Utility.SaveToBinary(temp, path);
        }

        public void ThirdDayLessonVariableVariance()
        {
            string[] stepNames = new string[] { "A", "B1", "C1", "D1", "E", "B2", "C2", "D2", "F", "G1", "H1",
                                                "I", "J", "G2", "H2", "K", "L"};
            string[] lesson1Students = Enumerable.Range(33, 3).Select(i => "Student" + i.ToString()).ToArray();
            string[] lesson2Students = Enumerable.Range(36, 3).Select(i => "Student" + i.ToString()).ToArray();
            string[] lesson3Students = Enumerable.Range(39, 2).Select(i => "Student" + i.ToString()).ToArray();
            List<VarAndVariance> temp = new List<VarAndVariance>();
            foreach (string stepName in stepNames)
            {
                foreach (string variable in this.armVars)
                {
                    IEnumerable<Result> tempResults = this.results.Where(r => r.variableName == variable && r.stepName == stepName);
                    var l1 = tempResults.Where(r => lesson1Students.Contains(r.userName));
                    var l2 = tempResults.Where(r => lesson2Students.Contains(r.userName));
                    var l3 = tempResults.Where(r => lesson3Students.Contains(r.userName));

                    if (l1.Count() == 0 || l2.Count() == 0 || l3.Count() == 0)
                        continue;

                    double day1Variance = CalcEx.GetStdDev(l1.Select(r => (double)r.similarities.ToList().Average()));
                    double day2Variance = CalcEx.GetStdDev(l2.Select(r => (double)r.similarities.ToList().Average()));
                    double day3Variance = CalcEx.GetStdDev(l3.Select(r => (double)r.similarities.ToList().Average()));
                    if (this.IsDecrease(day1Variance, day2Variance, day3Variance))
                    {
                        temp.Add(new VarAndVariance(variable, stepName, day1Variance, day2Variance, day3Variance));
                    }
                }
            }
            temp = temp.OrderBy(v => v.decreaseScore).ToList();
            List<List<string>> outputs = new List<List<string>>();
            foreach (VarAndVariance v in temp)
            {
                List<string> line = new List<string>();
                line.Add(this.SwapLR(v.variable));
                line.Add(v.step);
                line.Add(v.var1.ToString());
                line.Add(v.var2.ToString());
                line.Add(v.var3.ToString());
                outputs.Add(line);
            }
            string dumppath = System.IO.Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.DesktopDirectory), "Day3ArmVariances.dump");
            string csvpath = System.IO.Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.DesktopDirectory), "Day3ArmVariances.csv");
            Utility.SaveToCsv(csvpath, outputs);
            Utility.SaveToBinary(temp, dumppath);
        }

        public void SortVarianceFile(string varianceFile)
        {
            List<VarAndVariance> temp = (List<VarAndVariance>)Utility.LoadFromBinary(varianceFile);
            temp = temp.OrderBy(v => v.decreaseScore).OrderBy(v => v.var3).ToList();
            List<List<string>> outputs = new List<List<string>>();
            foreach (VarAndVariance v in temp)
            {
                List<string> line = new List<string>();
                line.Add(this.SwapLR(v.variable));
                line.Add(v.step);
                line.Add(v.var1.ToString());
                line.Add(v.var2.ToString());
                line.Add(v.var3.ToString());
                outputs.Add(line);
            }
            string path = System.IO.Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.DesktopDirectory), "variances.csv");
            Utility.SaveToCsv(path, outputs);
        }

        public void DumpAllDecreasedCsvs(string varianceFile, string outputFolder)
        {
            List<VarAndVariance> temp = (List<VarAndVariance>)Utility.LoadFromBinary(varianceFile);
            foreach (VarAndVariance vv in temp)
            {
                this.DumpSpecificVariableSimLog(vv.variable, vv.step, outputFolder);
            }
        }

        public void DumpSpecificVariableSimLog(string variableName, string stepName, string outputFolder)
        {
            IEnumerable<Result> tempResults = this.results.Where(r => r.variableName == variableName && r.stepName == stepName);
            tempResults = tempResults.OrderBy(r => r.end);
            List<List<string>> outputs = new List<List<string>>();
            foreach(Result res in tempResults)
            {
                List<string> line = new List<string>();
                line.Add(res.end.ToString());
                line.Add(res.similarities.Average().ToString());
                outputs.Add(line);
            }
            string path = System.IO.Path.Combine(outputFolder, this.SwapLR(variableName) + "_" + stepName + ".csv");
            Utility.SaveToCsv(path, outputs);
        }

        /// <summary>
        /// あるステップの類似度を全て出力する -> csv
        /// </summary>
        public void StepSpecificCsvExporter()
        {
            string stepName = "G1";
            IEnumerable<Result> temp = this.results.Where(r => r.stepName == stepName && r.variableName.Contains("Position"));
            IEnumerable<string> variables = temp.Select(r => r.variableName).Distinct();
            List<List<string>> outputs = new List<List<string>>();
            foreach (string variable in variables)
            {
                List<string> line = new List<string>();
                line.Add(this.SwapLR(variable));
                IEnumerable<Result> hoge = temp.Where(r => r.variableName == variable);
                foreach (string userName in Enumerable.Range(1, 40).Select(i => "Student" + i.ToString()))
                {
                    IEnumerable<Result> pue = hoge.Where(r => r.userName == userName);
                    // データがあったら突っ込む。なかったら-1をエラー値とする。
                    if (pue.Count() == 1)
                    {
                        line.Add(pue.First().similarities.Average().ToString());
                    }
                    else
                    {
                        line.Add("-1");
                    }
                }
                outputs.Add(line);
            }
            string csvpath = System.IO.Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.DesktopDirectory), stepName + "Sims.csv");
            Utility.SaveToCsv(csvpath, outputs);
        }

        /// <summary>
        /// xsがwindowSizeでthreshold以下で連続する区間があるかどうか
        /// </summary>
        /// <param name="xs"></param>
        /// <param name="windowSize"></param>
        /// <param name="threshold"></param>
        /// <returns></returns>
        public bool IsSeqDetermine(double[] xs, int windowSize = 3, double threshold = 0.05)
        {
            int[] windows = Enumerable.Range(0, windowSize).ToArray();
            List<bool> flags = new List<bool>();
            for (int index = 0; index <= xs.Length - windowSize; index++)
            {
                double[] vals = new double[windowSize];
                // window sizeぶんのデータをつっこむ
                foreach (int offset in windows)
                {
                    vals[offset] = xs[index + offset];
                }
                // エラー値があればfalse
                if (vals.Any(d => d > 1))
                {
                    flags.Add(false);
                }
                // エラー値がない場合
                else
                {
                    bool seqFlag = true;
                    foreach (int offset in windows.Take(windowSize - 1))
                    {
                        // 今と次の値の差が閾値より大きいかどうか
                        if (Math.Abs(vals[offset] - vals[offset + 1]) > threshold)
                        {
                            seqFlag = false;
                        }
                    }
                    flags.Add(seqFlag);
                }
            }
            // どっかにあればtrue
            return flags.Any(b => b == true);
        }

        /// <summary>
        /// 条件のindexをとってくる. indexは0-origin
        /// </summary>
        /// <param name="xs"></param>
        /// <param name="windowSize"></param>
        /// <param name="threshold"></param>
        /// <returns></returns>
        public List<int> FindSeq(double[] xs, int windowSize = 3, double threshold = 0.05)
        {
            int[] windows = Enumerable.Range(0, windowSize).ToArray();
            List<int> indexes = new List<int>();
            for (int index = 0; index <= xs.Length - windowSize; index++)
            {
                double[] vals = new double[windowSize];
                // window sizeぶんのデータをつっこむ
                foreach (int offset in windows)
                {
                    vals[offset] = xs[index + offset];
                }
                // エラー値がある
                if (vals.Any(d => d > 1))
                {
                    continue;
                }
                // エラー値がない場合
                else
                {
                    bool seqFlag = true;
                    foreach (int offset in windows.Take(windowSize - 1))
                    {
                        // 今と次の値の差が閾値より大きいかどうか
                        if (Math.Abs(vals[offset] - vals[offset + 1]) > threshold)
                        {
                            seqFlag = false;
                        }
                    }
                    if (seqFlag)
                    {
                        indexes.AddRange(windows.Select(i => i + index));
                    }
                }
            }
            return indexes;
        }

        /// <summary>
        /// あるステップ・変数のグラフいっこ
        /// </summary>
        [Serializable]
        public struct SimSeq
        {
            public string stepName;
            public string varName;
            public double[] times;
            public double[] sims;
            public List<double> estimates;
            public SimSeq(string stepName, string varName, int[] times, float[] sims)
            {
                this.stepName = stepName;
                this.varName = varName;
                this.times =  new double[41];
                this.sims = new double[41];

                foreach (int dataNo in Enumerable.Range(1, 40))
                {
                    // データ番号と合致するやつがあるかどうか
                    var hoge = this.times.Select((d, i) => new { d, i }).Where(pair => (int)pair.d == dataNo);
                    if (hoge.Count() > 0)
                    {
                        int index = hoge.ToList()[0].i;
                        this.times[dataNo] = times[index];
                        this.sims[dataNo] = sims[index];
                    }
                    else
                    {
                        this.times[dataNo] = dataNo;
                        this.sims[dataNo] = 1000;
                    }
                }

                // 線を引いてその値を求める
                var res = Fit.Line(this.times, this.sims);
                this.estimates = new List<double>();
                foreach (int dataNo in Enumerable.Range(1, 40))
                {
                    this.estimates.Add(res.Item2 * (double)dataNo + res.Item1);
                }
            }
            /// <summary>
            /// もじれつ～
            /// </summary>
            /// <returns></returns>
            public List<string> ToString()
            {
                List<string> res = new List<string>();

                foreach (int dataNo in Enumerable.Range(1, 40))
                {
                    // データ番号と合致するやつがあるかどうか
                    var hoge = this.times.Select((d, i) => new { d, i }).Where(pair => (int)pair.d == dataNo);
                    if (hoge.Count() > 0)
                    {
                        int index = hoge.ToList()[0].i;
                        res.Add(sims[index].ToString());
                    }
                    else
                    {
                        res.Add("-1");
                    }
                }
                return res;
            }

            /// <summary>
            /// 推定値より上の群をフィルタして返す。10埋め
            /// </summary>
            /// <returns></returns>
            public double[] GetUpper(double offset = 0.05)
            {
                double[] res = new double[this.times.Length];
                for (int index = 0; index < this.times.Length; index++)
                {
                    double real = this.sims[index];
                    double est = this.estimates[index];
                    if (real > est + offset)
                    {
                        res[index] = real;
                    }
                    else
                    {
                        res[index] = -10;
                    }
                }
                return res;
            }
            /// <summary>
            /// 推定値より下の群をフィルタして返す。10埋め
            /// </summary>
            /// <param name="offset"></param>
            /// <returns></returns>
            public double[] GetDowner(double offset = 0.05)
            {
                double[] res = new double[this.times.Length];
                for (int index = 0; index < this.times.Length; index++)
                {
                    double real = this.sims[index];
                    double est = this.estimates[index];
                    if (real + offset < est)
                    {
                        res[index] = real;
                    }
                    else
                    {
                        res[index] = 10;
                    }
                }
                return res;
            }
        }

        public void Hoge(ref List<SimSeq> res, string variable, string stepName)
        {
            IEnumerable<Result> tempResults = this.results.Where(r => r.variableName == variable && r.stepName == stepName).OrderBy(r => r.start);
            int[] times = tempResults.Select(r => int.Parse(r.userName.Replace("Student", ""))).ToArray();
            float[] sims = new float[tempResults.Count()];
            try
            {
                sims = tempResults.Select(r => r.similarities.Average()).ToArray();
            }
            catch (Exception e)
            {
                sims = new float[0];
            }
            if (sims.Length > 0)
            {
                SimSeq s = new SimSeq(stepName, variable, times, sims);
                lock (res)
                {
                    res.Add(s);
                }
            }
        }

        /// <summary>
        /// simが連続して下がってた領域を探索し、csvで吐き出す
        /// </summary>
        public void SearchImprovedRange()
        {
            IEnumerable<string> variables = this.legVars.Where(s => s.Contains("Position"));
            string[] stepNames = new string[] { "A", "B1", "C1", "D1", "E", "B2", "C2", "D2", "F", "G1", "H1",
                                                "I", "J", "G2", "H2", "K", "L"};
            List<SimSeq> simSeqs = new List<SimSeq>();

            foreach (string variable in variables)
            {
                foreach (string stepName in stepNames)
                {
                    this.Hoge(ref simSeqs, variable, stepName);
                }
            }

            Utility.SaveToBinary(simSeqs, "SimilaritySequenceList.dump");
            simSeqs = simSeqs.Where(s => this.IsSeqDetermine(s.GetDowner())).ToList();
            List<List<string>> outputs = new List<List<string>>();
            foreach (SimSeq sim in simSeqs)
            {
                List<string> line = new List<string>();
                line.Add(sim.varName);
                line.Add(sim.stepName);
                line.AddRange(sim.ToString());
                outputs.Add(line);
            }
            string csvpath = System.IO.Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.DesktopDirectory), "DownerSims.csv");
            Utility.SaveToCsv(csvpath, outputs);
        }

        public void SearchImprovedRange2()
        {
            List<SimSeq> simSeqs = (List<SimSeq>)Utility.LoadFromBinary("SimilaritySequenceList.dump");
            double offset = 0.1;
            double plus = 0.01;
            while (simSeqs.Count() != 0)
            {
                Debug.WriteLine(offset);
                simSeqs = simSeqs.Where(s => this.IsSeqDetermine(s.GetDowner(offset))).ToList();
                if (simSeqs.Count() < 30 && simSeqs.Count() >= 5)
                {
                    break;
                }
                offset += plus;
            }
            List<List<string>> outputs = new List<List<string>>();
            foreach (SimSeq sim in simSeqs)
            {
                List<string> line = new List<string>();
                line.Add(sim.varName);
                line.Add(sim.stepName);
                line.AddRange(sim.ToString());
                outputs.Add(line);
            }
            string csvpath = System.IO.Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.DesktopDirectory), "DownerSims.csv");
            Utility.SaveToCsv(csvpath, outputs);
        }

        public void SearchImprovedRange3()
        {
            List<SimSeq> simSeqs = (List<SimSeq>)Utility.LoadFromBinary("SimilaritySequenceList.dump");
            List<int> indexRange = new List<int>() { 29, 30, 31, 32 };
            simSeqs = simSeqs.Where(s => this.IsSeqDetermine(s.GetDowner(0.1))).ToList();
            List<List<int>> fuge = simSeqs.Where(s => s.stepName == "G1" || s.stepName == "I").Select(s => this.FindSeq(s.GetDowner(0.11), 3, 0.1)).ToList();
            List<SimSeq> res = new List<SimSeq>();
            int[] indexHoge = new int[41];
            foreach (SimSeq s in simSeqs)
            {
                List<int> indexes = this.FindSeq(s.GetDowner(0.11), 3, 0.05);
                foreach (int index in indexes.Distinct())
                {
                    indexHoge[index]++;
                }
                if (indexes.Contains(26))
                {
                    res.Add(s);
                }
            }
            
            List<List<string>> outputs = new List<List<string>>();
            foreach (SimSeq sim in res)
            {
                List<string> line = new List<string>();
                line.Add(sim.varName);
                line.Add(sim.stepName);
                line.AddRange(sim.ToString());
                outputs.Add(line);
            }
            string csvpath = System.IO.Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.DesktopDirectory), "DownerSims.csv");
            Utility.SaveToCsv(csvpath, outputs);
        }
    }
}
