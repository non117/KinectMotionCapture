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
using Microsoft.Kinect;

using KinectMotionCapture;
using OpenCvSharp;

namespace KinectMotionCapture
{
    /// <summary>
    /// 雑多なツールを集めたライブラリ
    /// </summary>
    public static class Utility
    {
        /// <summary>
        /// KinectのカラーデータをCvMatに変換する
        /// </summary>
        /// <param name="height"></param>
        /// <param name="width"></param>
        /// <param name="data"></param>
        /// <returns></returns>
        public static CvMat ColorArrayToCvMat(int width, int height, ref byte[] data)
        {
            CvMat mat = new CvMat(height, width, MatrixType.U8C4, data);
            return mat;
        }

        /// <summary>
        /// Kinectの深度データをCvMatに変換する
        /// </summary>
        /// <param name="width"></param>
        /// <param name="height"></param>
        /// <param name="data"></param>
        /// <returns></returns>
        public static CvMat DpethArrayToCvMat(int width, int height, ref ushort[] data)
        {
            CvMat mat = new CvMat(height, width, MatrixType.U16C1, data);
            return mat;
        }

        /// <summary>
        /// kinectのBodyIndexデータをCvMatに変換する
        /// </summary>
        /// <param name="width"></param>
        /// <param name="height"></param>
        /// <param name="data"></param>
        /// <returns></returns>
        public static CvMat BodyIndexArrayToCvMat(int width, int height, ref byte[] data)
        {
            CvMat mat = new CvMat(height, width, MatrixType.U8C1, data);
            return mat;
        }


        /// <summary>
        /// WritableBitmapをfileName.pngに保存する
        /// </summary>
        /// <param name="bitmap"></param>
        /// <param name="fileName"></param>
        public static void SavePngImage(WriteableBitmap bitmap, string fileName)
        {
            using (FileStream stream = new FileStream(fileName + ".png", FileMode.Create, FileAccess.Write))
            {
                PngBitmapEncoder encoder = new PngBitmapEncoder();
                encoder.Frames.Add(BitmapFrame.Create(bitmap));
                encoder.Save(stream);
            }
        }

        /// <summary>
        /// WritableBitmapをfileName.jpgに保存する
        /// </summary>
        /// <param name="bitmap"></param>
        /// <param name="fileName"></param>
        public static void SaveJpegImage(WriteableBitmap bitmap, string fileName)
        {
            using (FileStream stream = new FileStream(fileName + ".jpg", FileMode.Create, FileAccess.Write))
            {
                JpegBitmapEncoder encoder = new JpegBitmapEncoder();
                encoder.Frames.Add(BitmapFrame.Create(bitmap));
                encoder.Save(stream);
            }
        }

        /// <summary>
        /// pathのディレクトリを無ければつくる
        /// </summary>
        /// <param name="path"></param>
        public static void CreateDirectories(string path)
        {
            if (Directory.Exists(path))
            {
                return;
            }
            string fullpath = Path.GetFullPath(path);
            string dir = Path.GetDirectoryName(fullpath);
            CreateDirectories(dir);
            Directory.CreateDirectory(fullpath);

        }
    }

}