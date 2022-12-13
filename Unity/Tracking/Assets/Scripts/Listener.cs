using System;
using System.Collections.Concurrent;
using System.Threading;
using NetMQ;
using NetMQ.Sockets;

namespace PubSub
{
    public class Listener
    {
        private Thread _clientThread;
        private readonly string _host;
        private readonly string _port;
        private readonly Action<string> _eventAction;
        private bool _clientCancelled;

        private readonly ConcurrentQueue<string> _messageQueue = new ConcurrentQueue<string>();

        public Listener(string host, string port, Action<string> eventAction)
        {
            _host = host;
            _port = port;
            _eventAction = eventAction;
        }

        public void Start()
        {
            _clientCancelled = false;
            _clientThread = new Thread(ListenerWork);
            _clientThread.Start();
        }

        public void Stop()
        {
            _clientCancelled = true;
            _clientThread?.Join();
            _clientThread = null;
        }

        private void ListenerWork()
        {
            AsyncIO.ForceDotNet.Force();
            using (var subSocket = new SubscriberSocket())
            {
                subSocket.Options.ReceiveHighWatermark = 1000;
                subSocket.Connect($"tcp://{_host}:{_port}");
                subSocket.SubscribeToAnyTopic();
                while (!_clientCancelled)
                {
                    if (!subSocket.TryReceiveFrameString(out var message)) continue;
                    _messageQueue.Enqueue(message);
                }
                subSocket.Close();
            }
            NetMQConfig.Cleanup();
        }

        public void DigestMessage()
        {
            while (!_messageQueue.IsEmpty)
            {
                if (_messageQueue.TryDequeue(out var message))
                    _eventAction(message);
                else
                    break;
            }
        }
    }
}