using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Microsoft.Kinect;

namespace KinectMotionCapture
{
    [Serializable]
    class LocalCoordinateMapper
    {
        private PointF[] _depthFrameToCameraSpaceTable = null;
        private int depthWidth = 0;
        private int depthHeight = 0;


        public CameraSpacePoint MapDepthPointToCameraSpace(DepthSpacePoint depthSpacePoint, ushort depth)
        {
            float distance = depth / 1000;
            int index = (int)(this.depthWidth * depthSpacePoint.Y + depthSpacePoint.X);
            PointF cameraPoint = this._depthFrameToCameraSpaceTable[index];
            return new CameraSpacePoint() { X = cameraPoint.X * distance, Y = cameraPoint.Y * distance, Z = distance };
        }

        public LocalCoordinateMapper(CoordinateMapper originalCoordinateMapper, int depthWidth, int depthHeight)
        {
            this.depthWidth = depthWidth;
            this.depthHeight = depthHeight;
            this._depthFrameToCameraSpaceTable = originalCoordinateMapper.GetDepthFrameToCameraSpaceTable();
        }
    }
}
