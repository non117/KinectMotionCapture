using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;
using System.Threading;
using System.Windows.Threading;

namespace KinectMotionCapture
{
    /// <summary>
    /// ProgressWindow.xaml の相互作用ロジック
    /// </summary>
    public partial class ProgressWindow : Window
    {
        public ProgressWindow()
        {
            InitializeComponent();
        }
        DispatcherTimer _pollingTimer;
        Thread _actionThread;
        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            if (_action == null)
            {
                throw new InvalidOperationException("Please Attach Action before showing this windows");
            }
            _pollingTimer = new DispatcherTimer(new TimeSpan(0, 0, 0, 0, 200), DispatcherPriority.Background, pollProgressChanged, this.Dispatcher);
            _pollingTimer.Start();
            _actionThread = new Thread(doAction);
            _actionThread.Start();
        }

        void doAction()
        {
            try
            {
                setDialogResult(null);
                _progressData = new ProgressData("Processing...", 0);
                _progressData.ProgressChanged += onProgressChanged;
                _action(_progressData);
                setDialogResult(true);
            }
            catch (ThreadAbortException)
            {
                setDialogResult(false);
            }
            closeAsync();
        }

        void closeAsync()
        {
            if (!this.Dispatcher.CheckAccess())
            {
                this.Dispatcher.BeginInvoke(new Action(closeAsync));
                return;
            }
            this.Close();
        }

        void setDialogResult(bool? dialogResult)
        {
            if (!this.Dispatcher.CheckAccess())
            {
                this.Dispatcher.BeginInvoke(new Action<bool?>(setDialogResult), dialogResult);
                return;
            }
            try
            {
                this.DialogResult = dialogResult;
            }
            catch (InvalidOperationException) { }
        }

        void pollProgressChanged(object sender, EventArgs e)
        {
            ProgressData p = _progressData;
            if (p == null)
                return;
            if (p.IsProgessChanged)
            {
                p.IsProgessChanged = false;
                onProgressChanged(this, new EventArgs());
            }
        }

        ProgressData _progressData;
        Action<ProgressData> _action;
        public void AttachAction(Action<ProgressData> action, bool isCancelEnabled)
        {
            _action = action;
            buttonCancel.IsEnabled = isCancelEnabled;
        }

        void onProgressChanged(object sender, EventArgs e)
        {
            if (!this.Dispatcher.CheckAccess())
            {
                this.Dispatcher.BeginInvoke(new EventHandler(onProgressChanged), sender, e);
                return;
            }
            ProgressData p = _progressData;
            if (p == null)
                return;
            double value = p.CurrentValue;
            double max = p.MaximumValue;
            string msg = p.Message;
            value = Math.Max(0, Math.Min(max, value));
            if (max > 0)
            {
                progressBar.Value = progressBar.Maximum * value / max;
                textBlockMessage.Text = string.Format("{0} ({1:0}/{2:0})", msg, value, max);
            }
            else
            {
                progressBar.Value = 0;
                textBlockMessage.Text = string.Format("{0}", msg, value, max);
            }
        }

        private void buttonCancel_Click(object sender, RoutedEventArgs e)
        {
            _actionThread.Abort();
        }


    }
}
