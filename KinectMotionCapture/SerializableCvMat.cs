using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using OpenCvSharp;
using System.IO;
using System.Runtime.InteropServices;
using System.IO.Compression;

namespace KinectMotionCapture
{
    /// <summary>
    /// CvMatを数値データ列のXMLとしてシリアライズ可能な形で保持するクラス
    /// </summary>
    public class SerializableCvMat
    {
        /// <summary>
        /// 内部データのCvScalar配列
        /// </summary>
        public CvScalar[] Array;
        public int Cols, Rows;
        public MatrixType ElemType;

        public SerializableCvMat()
        {
        }
        public SerializableCvMat(CvMat mat)
        {
            this.Cols = mat.Cols;
            this.Rows = mat.Rows;
            this.ElemType = mat.ElemType;
            this.Array = mat.ToArray();
        }
        public CvMat CreateCvMat()
        {
            CvMat ret = new CvMat(this.Rows, this.Cols, this.ElemType);
            int index = 0;
            for (int y = 0; y < this.Rows; y++)
            {
                for (int x = 0; x < this.Cols; x++)
                {
                    ret.Set2D(y, x, this.Array[index++]);
                }
            }
            return ret;
        }
        public static SerializableCvMat CreateOrNull(CvMat mat)
        {
            if (mat == null)
                return null;
            return new SerializableCvMat(mat);
        }
        public static CvMat CreateOrNull(SerializableCvMat mat)
        {
            if (mat == null)
                return null;
            return mat.CreateCvMat();
        }


    }
    /// <summary>
    /// バイナリデータが文字列エンコーディングされた状態で、CvMatをシリアライズ可能な形で保持するクラス
    /// </summary>
    public class SerializableBinaryMat
    {
        public int Cols, Rows;
        public MatrixType ElemType;
        public string EncodeType = "Base64-GZip";
        /// <summary>
        /// 内部データの文字列エンコーディングされたバイナリデータ
        /// </summary>
        public string ArrayBinaryEncoded;

        public SerializableBinaryMat()
        {
        }

        public static byte[] GetBytesFromData(CvMat mat)
        {
            int byteLength = mat.ElemDepth * mat.Cols * mat.Rows * mat.ElemChannels / 8;
            byte[] ret = new byte[byteLength];
            Marshal.Copy(mat.Data, ret, 0, byteLength);
            return ret;
        }
        public static void CopyDataFromBytes(CvMat dest, byte[] bytes)
        {
            int byteLength = dest.ElemDepth * dest.Cols * dest.Rows * dest.ElemChannels / 8;
            Marshal.Copy(bytes, 0, dest.Data, byteLength);
        }
        public SerializableBinaryMat(CvMat mat)
        {
            this.Cols = mat.Cols;
            this.Rows = mat.Rows;
            this.ElemType = mat.ElemType;
            if (this.EncodeType == "Base64-GZip")
            {
                using (MemoryStream mem = new MemoryStream())
                {
                    using (GZipStream gz = new GZipStream(mem, CompressionMode.Compress))
                    {
                        byte[] bytes = GetBytesFromData(mat);
                        gz.Write(bytes, 0, bytes.Length);
                    }
                    this.ArrayBinaryEncoded = Convert.ToBase64String(mem.ToArray());
                }
            }
            else
            {
                this.ArrayBinaryEncoded = Convert.ToBase64String(GetBytesFromData(mat));
            }
        }
        public static SerializableBinaryMat CreateOrNull(CvMat mat)
        {
            if (mat == null)
                return null;
            return new SerializableBinaryMat(mat);
        }
        public static CvMat CreateOrNull(SerializableBinaryMat mat)
        {
            if (mat == null)
                return null;
            return mat.CreateCvMat();
        }

        public CvMat CreateCvMat()
        {
            CvMat ret = new CvMat(this.Rows, this.Cols, this.ElemType);
            byte[] bytes = Convert.FromBase64String(this.ArrayBinaryEncoded);
            if (this.EncodeType == "Base64-GZip")
            {
                using (MemoryStream decomp = new MemoryStream())
                {
                    using (MemoryStream mem = new MemoryStream(bytes))
                    using (GZipStream gz = new GZipStream(mem, CompressionMode.Decompress))
                    {
                        byte[] buf = new byte[16384];
                        while (true)
                        {
                            int length = gz.Read(buf, 0, buf.Length);
                            if (length <= 0)
                                break;
                            decomp.Write(buf, 0, length);
                        }
                    }
                    bytes = decomp.ToArray();
                }
            }
            CopyDataFromBytes(ret, bytes);
            return ret;
        }
    }
}
