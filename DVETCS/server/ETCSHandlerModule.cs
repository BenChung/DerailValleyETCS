using System;
using System.Threading;
using System.Threading.Tasks;
using EmbedIO;
using EmbedIO.WebSockets;

namespace DVETCS
{
    internal class ETCSHandlerModule : WebSocketModule 
    {

        public ETCSHandlerModule(string uriScheme) : base(uriScheme, true)
        {
        }
        

        protected override Task OnMessageReceivedAsync(IWebSocketContext context, byte[] buffer, IWebSocketReceiveResult result)
        {
            throw new System.NotImplementedException();
        }

        public void NotifySpeed(float currentSpeed)
        {
            BroadcastAsync(currentSpeed.ToString());
        }
    }
}