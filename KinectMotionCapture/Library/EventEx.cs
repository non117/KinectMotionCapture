using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace KinectMotionCapture
{
    public static class EventEx
    {
        /// <summary>
        /// マルチキャストデリゲートイベントハンドラを一斉に呼び出し、完了するのを待ちます
        /// </summary>
        /// <param name="handler">イベントハンドラ</param>
        /// <param name="sender">呼び出しもとのオブジェクト</param>
        /// <param name="e">イベント引数</param>
        public static void SimultaneousInvoke(this EventHandler handler, object sender, EventArgs e)
        {
            if (handler != null)
            {
                List<Tuple<EventHandler, IAsyncResult>> list = new List<Tuple<EventHandler, IAsyncResult>>();
                foreach (EventHandler item in handler.GetInvocationList())
                {
                    list.Add(new Tuple<EventHandler, IAsyncResult>(item, item.BeginInvoke(sender, e, null, null)));
                }
                foreach (var tuple in list)
                {
                    tuple.Item1.EndInvoke(tuple.Item2);
                }
            }
        }
        /// <summary>
        /// マルチキャストデリゲートイベントハンドラを一斉に呼び出し、完了するのを待ちます
        /// </summary>
        /// <param name="handler">イベントハンドラ</param>
        /// <param name="sender">呼び出しもとのオブジェクト</param>
        /// <param name="e">イベント引数</param>
        public static void SimultaneousInvoke<T>(this EventHandler<T> handler, object sender, T e) where T : EventArgs
        {
            if (handler != null)
            {
                List<Tuple<EventHandler<T>, IAsyncResult>> list = new List<Tuple<EventHandler<T>, IAsyncResult>>();
                foreach (EventHandler<T> item in handler.GetInvocationList())
                {
                    list.Add(new Tuple<EventHandler<T>, IAsyncResult>(item, item.BeginInvoke(sender, e, null, null)));
                }
                foreach (var tuple in list)
                {
                    tuple.Item1.EndInvoke(tuple.Item2);
                }
            }
        }
        /// <summary>
        /// System.Threding.Tasks.Parallel.Invoke()と同じようなもの、System.Threading.Tasks,Taskに頼らない版
        /// </summary>
        /// <param name="acts"></param>
        public static void SimultaneousInvoke(params Action[] acts)
        {

            if (acts != null)
            {
                List<Tuple<Action, IAsyncResult>> list = new List<Tuple<Action, IAsyncResult>>();
                foreach (Action item in acts)
                {
                    list.Add(new Tuple<Action, IAsyncResult>(item, item.BeginInvoke(null, null)));
                }
                foreach (var tuple in list)
                {
                    tuple.Item1.EndInvoke(tuple.Item2);
                }
            }
        }

    }
}
