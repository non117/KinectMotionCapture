using System;
using System.Net;
using System.Net.Sockets;

namespace KinectMotionCapture
{
	public class SocketServer
	{
		private TcpListener server = null;
		public SocketServer (string addr, int port)
		{
			try{
				IPAddress localAddr = IPAddress.Parse (addr);
				this.server = new TcpListener (localAddr, port);
				server.Start ();

				while (true) {
					Console.WriteLine ("Waiting for connection...");
					TcpClient client = server.AcceptTcpClient ();
					Console.WriteLine ("Client connected.");
					NetworkStream stream = client.GetStream ();

					Byte[] bytes = new Byte[1024];
					int siz;

					while ((siz = stream.Read (bytes, 0, bytes.Length)) != 0) {
						String data = System.Text.Encoding.GetEncoding (932).GetString (bytes, 0, siz);
						Console.WriteLine (String.Format ("Accepted: {0}bytes {1}", siz, data));
						byte[] msg = System.Text.Encoding.UTF8.GetBytes (data);
						stream.Write (msg, 0, msg.Length);
						Console.WriteLine ("Sended: {0}bytes {1}", msg.Length, data);
					}

					stream.Close ();
					client.Close ();
				}
			}catch(SocketException e){
				Console.WriteLine (e.Message);
			}
		}

		public static void Main()
		{
			SocketServer server = new SocketServer ("127.0.0.1", 8888);
		}
	}
}

