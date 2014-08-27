using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Linq;
using System.IO;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Microsoft.Kinect;

namespace KinectMotionCapture
{
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        // Kinect関連
        private readonly uint bytesPerPixel = 0;
        private KinectSensor kinectSensor = null;
        private CoordinateMapper coordinateMapper = null;
        private MultiSourceFrameReader multiFrameSourceReader = null;

        // 描画用
        private DrawingGroup drawingGroup;
        private DrawingImage imageSource;
        private WriteableBitmap bitmap = null;
        
        // データ貯めるやつ
        private ushort[] depthFrameData = null;
        private byte[] colorPixels = null;
        private byte[] bodyIndexFrameData = null;
        private byte[] displayPixels = null;
        private ColorSpacePoint[] colorPoints = null;
        private Body[] bodies = null;

        private string statusText = null;

        public MainWindow()
        {
            // Kinect関連初期化処理
            this.kinectSensor = KinectSensor.GetDefault();
            this.multiFrameSourceReader = this.kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Depth | FrameSourceTypes.Color | FrameSourceTypes.Body);
            this.multiFrameSourceReader.MultiSourceFrameArrived += this.Reader_FrameArrived;
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;
            FrameDescription deapthFrameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;
            FrameDescription colorFrameDescription = this.kinectSensor.ColorFrameSource.FrameDescription;

            int depthWidth = deapthFrameDescription.Width;
            int depthHeight = deapthFrameDescription.Height;

            int colorWidth = colorFrameDescription.Width;
            int colorHeight = colorFrameDescription.Height;

            // allocate space to put the pixels being received and coverted
            //this.depthFrameData = new ushort[depthWidth * depthHeight];
            //this.displayPixels = new byte[depthWidth * depthHeight * this.bytesPerPixel];
            //this.colorPoints = new ColorSpacePoint[depthWidth * depthHeight];

            // 描画関連
            this.drawingGroup = new DrawingGroup();
            this.imageSource = new DrawingImage(this.drawingGroup);
            this.bitmap = new WriteableBitmap(colorWidth, colorHeight, 96.0, 96.0, PixelFormats.Bgra32, null);           

            // allocate space to put the pixels being received
            this.bytesPerPixel = colorFrameDescription.BytesPerPixel;
            this.colorPixels = new byte[colorWidth * colorHeight * this.bytesPerPixel];

            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;
            this.kinectSensor.Open();

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // initialize the components (controls) of the window
            this.InitializeComponent();
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        public ImageSource ImageSource
        {
            get
            {
                return this.imageSource;
            }
        }

        public ImageSource BackgroundImage
        {
            get
            {
                return this.bitmap;
            }
        }

        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.multiFrameSourceReader != null)
            {
                this.multiFrameSourceReader.Dispose();
                this.multiFrameSourceReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }

        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            int depthWidth = 0;
            int depthHeight = 0;

            int colorWidth = 0;
            int colorHeight = 0;

            bool multiSourceFrameProcessed = false;
            bool colorFrameProcessed = false;
            bool depthFrameProcessed = false;
            bool bodyFrameProcessed = false;

            MultiSourceFrame multiSourceFrame = e.FrameReference.AcquireFrame();

            if (multiSourceFrame != null)
            {
                using (DepthFrame depthFrame = multiSourceFrame.DepthFrameReference.AcquireFrame())
                {
                    using (ColorFrame colorFrame = multiSourceFrame.ColorFrameReference.AcquireFrame())
                    {
                        using (BodyFrame bodyFrame = multiSourceFrame.BodyFrameReference.AcquireFrame())
                        {
                            if (depthFrame != null)
                            {
                                FrameDescription depthFrameDescription = depthFrame.FrameDescription;
                                depthWidth = depthFrameDescription.Width;
                                depthHeight = depthFrameDescription.Height;
                                if ((depthWidth * depthHeight) == this.depthFrameData.Length)
                                {
                                    depthFrame.CopyFrameDataToArray(this.depthFrameData);
                                    depthFrameProcessed = true;
                                }
                            }
                            if (colorFrame != null)
                            {
                                FrameDescription colorFrameDescription = colorFrame.FrameDescription;
                                colorWidth = colorFrameDescription.Width;
                                colorHeight = colorFrameDescription.Height;
                                if ((colorWidth * colorHeight * this.bytesPerPixel) == this.colorPixels.Length)
                                {
                                    if (colorFrame.RawColorImageFormat == ColorImageFormat.Bgra)
                                    {
                                        colorFrame.CopyRawFrameDataToArray(this.colorPixels);
                                    }
                                    else
                                    {
                                        colorFrame.CopyConvertedFrameDataToArray(this.colorPixels, ColorImageFormat.Bgra);
                                    }
                                    colorFrameProcessed = true;
                                }
                            }
                            if (bodyFrame != null)
                            {
                                if (this.bodies == null)
                                {
                                    this.bodies = new Body[bodyFrame.BodyCount];
                                }
                                bodyFrame.GetAndRefreshBodyData(this.bodies);
                                bodyFrameProcessed = true;
                                
                            }
                            multiSourceFrameProcessed = true;
                        }
                    }
                }
            }

            // we got all frames
            if (multiSourceFrameProcessed && depthFrameProcessed && colorFrameProcessed && bodyFrameProcessed)
            {
                this.RenderColorPixels();
                using (DrawingContext dc = this.drawingGroup.Open())
                {

                }
            }

        }

        /// <summary>
        /// Renders color pixels into the writeableBitmap.
        /// </summary>
        private void RenderColorPixels()
        {
            this.bitmap.WritePixels(
                new Int32Rect(0, 0, this.bitmap.PixelWidth, this.bitmap.PixelHeight),
                this.colorPixels,
                this.bitmap.PixelWidth * (int)this.bytesPerPixel,
                0);
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }
    }
}
