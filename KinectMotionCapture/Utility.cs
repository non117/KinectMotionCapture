﻿using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Linq;
using System.IO;
using System.Text;
using System.Runtime.Serialization;
using System.Runtime.Serialization.Formatters.Binary;
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
        /// ディープコピーを作成する
        /// Serializableが必要。不要なフィールドはNonSerializedAttribute属性を
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="target"></param>
        /// <returns></returns>
        public static T CloneDeep<T>(this T target)
        {
            object clone = null;
            using (MemoryStream stream = new MemoryStream())
            {
                BinaryFormatter formatter = new BinaryFormatter();
                formatter.Serialize(stream, target);
                stream.Position = 0;
                clone = formatter.Deserialize(stream);
            }
            return (T)clone;
        }

        /// <summary>
        /// 画像を読みまくり
        /// </summary>
        /// <param name="paths"></param>
        /// <returns></returns>
        public static void LoadImages(IEnumerable<string> paths, out IEnumerable<CvMat> images)
        {
            images = paths.Select(path => CvMat.LoadImageM(path, LoadMode.Unchanged));
        }

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

        /// <summary>
        /// バイナリとして保存する
        /// </summary>
        /// <param name="obj"></param>
        /// <param name="path"></param>
        public static void SaveToBinary(object obj, string path){
            using (FileStream fs = new FileStream(path, FileMode.Create, FileAccess.Write))
            {
                BinaryFormatter bf = new BinaryFormatter();
                bf.Serialize(fs, obj);
            }
        }

        /// <summary>
        /// バイナリから読み込む
        /// </summary>
        /// <param name="path"></param>
        /// <param name="T"></param>
        /// <returns></returns>
        public static object LoadFromBinary(string path, Type T)
        {
            using (FileStream fs = new FileStream(path, FileMode.Open, FileAccess.Read))
            {
                BinaryFormatter f = new BinaryFormatter();
                object obj = f.Deserialize(fs);
                return Convert.ChangeType(obj, T);
            }
        }

    }

}