using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace KinectMotionCapture
{
    public class ProgressData
    {
        private string _message;

        public string Message
        {
            get { return _message; }
            set { _message = value; requestDoProgressChanged(); }
        }
        double _currentValue, _maximumValue;

        public double MaximumValue
        {
            get { return _maximumValue; }
            set { _maximumValue = value; requestDoProgressChanged(); }
        }

        public double CurrentValue
        {
            get { return _currentValue; }
            set { _currentValue = value; requestDoProgressChanged(); }
        }
        public TimeSpan MinimumUpdateInterval;

        public bool IsProgessChanged;

        DateTime _lastInvoke;
        public ProgressData(string initialMessage, double maximumValue, double initialValue, TimeSpan minimumUpdateInterval)
        {
            this.InitProgress(initialMessage, maximumValue, initialValue);
            this.MinimumUpdateInterval = minimumUpdateInterval;
        }
        public ProgressData(string initialMessage, double maximumValue, double initialValue)
            : this(initialMessage, maximumValue, initialValue, new TimeSpan(0, 0, 0, 0, 10))
        {
        }
        public ProgressData(string initialMessage, double maximumValue)
            : this(initialMessage, maximumValue, 0)
        {
        }
        public ProgressData()
            : this("Processing...", 0)
        {
        }

        public void InitProgress(string initialMessage, double maximumValue, double initialValue)
        {
            _message = initialMessage;
            _maximumValue = maximumValue;
            _currentValue = initialValue;
            requestDoProgressChanged();
        }
        public void InitProgress(string initialMessage, double maximumValue)
        {
            this.InitProgress(initialMessage, maximumValue, 0);
        }

        public void SetProgress(string message, double currentValue)
        {
            _currentValue = currentValue;
            _message = message;
            requestDoProgressChanged();
        }
        public void SetProgress(double currentValue)
        {
            _currentValue = currentValue;
            requestDoProgressChanged();
        }

        void requestDoProgressChanged()
        {
            this.IsProgessChanged = true;
            DateTime now = DateTime.Now;
            if (now - _lastInvoke >= this.MinimumUpdateInterval)
            {
                _lastInvoke = now;
                doProgressChanged();
            }
        }

        void doProgressChanged()
        {
            this.ProgressChanged.SimultaneousInvoke(this, new EventArgs());
        }

        public event EventHandler ProgressChanged;


        public static bool DoAction(Action<ProgressData> action, string title, bool isCancelEnabled)
        {
            ProgressWindow wnd = new ProgressWindow();
            wnd.Title = title;
            wnd.AttachAction(action, isCancelEnabled);
            return wnd.ShowDialog() ?? false;
        }

        public ProgressData GetSubProgress(double startValue, double valueRange, Func<ProgressData, string> getMessage)
        {
            ProgressData sub = new ProgressData(this.Message, 0);
            sub.ProgressChanged += (sender, e) =>
            {
                double subMax = sub.MaximumValue;
                double subValue = Math.Max(0, Math.Min(sub.CurrentValue, subMax));
                if (subMax > 0)
                {
                    this.SetProgress(getMessage(sub), startValue + subValue * valueRange / subMax);
                }
                else
                {
                    this.SetProgress(getMessage(sub), startValue);
                }
            };
            return sub;
        }
    }
}
