using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace KinectMotionCapture
{
    /// <summary>
    /// ノード間の同一関係を追加していくことでノードの同一性クラスタを作成するためのクラス
    /// </summary>
    public class IdentificationSet<T>
    {
        /// <summary>
        /// 要素からIDを得る
        /// </summary>
        Dictionary<T, int> _identificationMap;
        /// <summary>
        /// IDから属する要素郡を得る
        /// </summary>
        Dictionary<int, List<T>> _enumerationMap;
        /// <summary>
        /// 最大ID
        /// </summary>
        int _maxIdent = 0;
        /// <summary>
        /// デフォルトの比較器でのコンストラクタ
        /// </summary>
        public IdentificationSet()
        {
            _identificationMap = new Dictionary<T, int>();
            _enumerationMap = new Dictionary<int, List<T>>();
        }
        /// <summary>
        /// 比較器を指定したコンストラクタ
        /// </summary>
        /// <param name="equalityComparer"></param>
        public IdentificationSet(IEqualityComparer<T> equalityComparer)
        {
            _identificationMap = new Dictionary<T, int>(equalityComparer);
            _enumerationMap = new Dictionary<int, List<T>>();
        }
        /// <summary>
        /// 指定された要素と等価な要素群を返します
        /// </summary>
        /// <param name="element"></param>
        /// <returns></returns>
        public IList<T> GetEquivalentElements(T element)
        {
            int ident;
            if (!_identificationMap.TryGetValue(element, out ident))
            {
                return new T[] { element };
            }
            List<T> ret;
            if (!_enumerationMap.TryGetValue(ident, out ret))
            {
                return new T[] { element };
            }
            return ret.ToArray();
        }
        /// <summary>
        /// 指定された二つの要素を等価とします
        /// </summary>
        /// <param name="element1"></param>
        /// <param name="element2"></param>
        public void MakeEquivalent(T element1, T element2)
        {
            this.Add(element1);
            this.Add(element2);

            int ident1 = _identificationMap[element1];
            int ident2 = _identificationMap[element2];
            if (ident1 == ident2)
                return;
            int newIdent = Math.Min(ident1, ident2);
            int oldIdent = Math.Max(ident1, ident2);
            List<T> oldList = _enumerationMap[oldIdent];
            foreach (T oldElement in oldList)
            {
                _identificationMap[oldElement] = newIdent;
            }
            List<T> newList = _enumerationMap[newIdent];
            newList.AddRange(oldList);
            _enumerationMap.Remove(oldIdent);
        }
        /// <summary>
        /// リストに新しい要素を加え、新しいIDを与えます
        /// </summary>
        /// <param name="element"></param>
        public void Add(T element)
        {
            if (_identificationMap.ContainsKey(element))
                return;
            List<T> list = _enumerationMap[_maxIdent] = new List<T>();
            list.Add(element);
            _identificationMap[element] = _maxIdent;
            _maxIdent++;
        }
        /// <summary>
        /// 使われていないIDを再利用してIDを圧縮します
        /// </summary>
        public void CompactIdentificationNumber()
        {
            int newMaxIdent = 0;
            Dictionary<int, int> toNewIdent = new Dictionary<int, int>();
            foreach (int ident in _enumerationMap.Keys.OrderBy(x => x))
            {
                toNewIdent[ident] = newMaxIdent;
                newMaxIdent++;
            }
            List<KeyValuePair<T, int>> identMap = _identificationMap.ToList();
            _identificationMap.Clear();
            foreach (var pair in identMap)
            {
                _identificationMap[pair.Key] = toNewIdent[pair.Value];
            }
            List<KeyValuePair<int, List<T>>> enumMap = _enumerationMap.ToList();
            _enumerationMap.Clear();
            foreach (var pair in enumMap)
            {
                _enumerationMap[toNewIdent[pair.Key]] = pair.Value;
            }
        }
        /// <summary>
        /// 要素からIDを取得します。要素がなければ-1
        /// </summary>
        /// <param name="element"></param>
        /// <returns></returns>
        public int ConvertToIdentificationNumber(T element)
        {
            int ident;
            if (!_identificationMap.TryGetValue(element, out ident))
            {
                return -1;
            }
            return ident;
        }

        public static void Test()
        {
            IdentificationSet<int> set = new IdentificationSet<int>();
            // 1 = 4 = 6 = 7 = 8 
            // 2 = 5 = 9 = 10 = 11
            // 3
            set.MakeEquivalent(1, 4);
            set.MakeEquivalent(5, 2);
            set.MakeEquivalent(3, 3);
            set.MakeEquivalent(5, 10);
            set.MakeEquivalent(11, 5);
            set.MakeEquivalent(5, 9);
            set.MakeEquivalent(6, 8);
            set.MakeEquivalent(7, 1);
            set.MakeEquivalent(7, 4);
            set.MakeEquivalent(7, 6);
            set.CompactIdentificationNumber();
            StringBuilder ret = new StringBuilder();
            for (int i = 1; i <= 11; i++)
            {
                ret.AppendLine(string.Format("{0} => {1}", i, set.ConvertToIdentificationNumber(i)));
            }
            System.Windows.MessageBox.Show(ret.ToString());
        }
    }
}
