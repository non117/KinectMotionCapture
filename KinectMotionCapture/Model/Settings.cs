using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace KinectMotionCapture
{
    [Serializable]
    public class Settings
    {
        public string[] dataDirectories
        {
            get;
            set;
        }
        public string[] mapFiles
        {
            get;
            set;
        }
        public string[] cameraFiles
        {
            get;
            set;
        }
    }
}
