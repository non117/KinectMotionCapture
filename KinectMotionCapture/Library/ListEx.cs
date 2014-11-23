using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace KinectMotionCapture
{
    public static class ListEx
    {
        #region BinarySearch
        // from http://philosopherdeveloper.wordpress.com/2010/05/24/whats-annoying-about-sortedlisttkey-tvalue-indexofkey-a-k-a-whats-sweet-about-reflector/
        public static int BinarySearch<T>(this IList<T> list, int index, int length, T value, IComparer<T> comparer)
        {
            if (list == null)
                throw new ArgumentNullException("list");
            else if (index < 0 || length < 0)
                throw new ArgumentOutOfRangeException(
                    (index < 0) ? "playerIndex" : "length"
                );
            else if (list.Count - index < length)
                throw new ArgumentException();

            int lower = index;
            int upper = (index + length) - 1;

            while (lower <= upper)
            {
                int adjustedIndex = lower + ((upper - lower) >> 1);
                int comparison = comparer.Compare(list[adjustedIndex], value);
                if (comparison == 0)
                    return adjustedIndex;
                else if (comparison < 0)
                    lower = adjustedIndex + 1;
                else
                    upper = adjustedIndex - 1;
            }

            return ~lower;
        }

        public static int BinarySearch<T>(this IList<T> list, T value, IComparer<T> comparer)
        {
            return list.BinarySearch(0, list.Count, value, comparer);
        }

        public static int BinarySearch<T>(this IList<T> list, T value) where T : IComparable<T>
        {
            return list.BinarySearch(value, Comparer<T>.Default);
        }
        #endregion

        /// <summary>
        /// 二分探索の結果から、二分探索の基準値以下の値を持つインデックスの中で最大のものを求めます。重複がなければ list[ret] &lt;= search &lt; list[ret+1]
        /// </summary>
        /// <param name="returnValueOfBinarySearch">二分探索の結果</param>
        /// <returns></returns>
        public static int GetMaxLessEqualIndexFromBinarySearch(int returnValueOfBinarySearch)
        {
            if (returnValueOfBinarySearch >= 0)
                return returnValueOfBinarySearch;
            return (~returnValueOfBinarySearch) - 1;
        }
        /// <summary>
        /// 二分探索の結果から、二分探索の基準値以上の値を持つインデックスの中で最小のものを求めます。重複がなければ list[ret-1] &lt; search &lt;= list[ret]
        /// </summary>
        /// <param name="returnValueOfBinarySearch">二分探索の結果</param>
        /// <returns></returns>
        public static int GetMinGreaterEqualIndexFromBinarySearch(int returnValueOfBinarySearch)
        {
            if (returnValueOfBinarySearch >= 0)
                return returnValueOfBinarySearch;
            return (~returnValueOfBinarySearch);
        }

        public static void InsertFirst<T>(IList<T> list, T newItem, int maxItems)
        {
            list.Remove(newItem);
            list.Insert(0, newItem);
            while (list.Count > maxItems)
            {
                list.RemoveAt(maxItems);
            }
        }
        public static void InsertFirst(System.Collections.IList list, object newItem, int maxItems)
        {
            list.Remove(newItem);
            list.Insert(0, newItem);
            while (list.Count > maxItems)
            {
                list.RemoveAt(maxItems);
            }
        }
    }
}
