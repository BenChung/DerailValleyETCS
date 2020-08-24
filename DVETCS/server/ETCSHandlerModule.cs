using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using EmbedIO;
using EmbedIO.WebSockets;
using Newtonsoft.Json;

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

        public void NotifyState(float currentSpeed, SpeedProfile.SpeedStateInfo? speedLimit,
            List<SpeedProfile.MRSPElem> mrspProfile)
        {
            var json = "";
            if (speedLimit.HasValue)
            {
                json = JsonConvert.SerializeObject(
                    new StatusMessage(currentSpeed, speedLimit.Value.P, speedLimit.Value.T, speedLimit.Value.SBI, mrspProfile), Formatting.None);
            }
            else
            {
                json = JsonConvert.SerializeObject(
                    new StatusMessage(currentSpeed, 40.0f, 40.0f, 40.0f, new List<SpeedProfile.MRSPElem>()), Formatting.None);
            }
            BroadcastAsync(json);
        }
    }
    public class StatusMessage
    {
        public float speed;
        public float vperm;
        public float vtarget;
        public float vsbi;
        public List<SpeedProfile.MRSPElem> mrsp;
        public StatusMessage(float speed, float vperm, float vtarget, float vsbi, List<SpeedProfile.MRSPElem> mrsp)
        {
            this.speed = speed;
            this.vperm = vperm;
            this.vtarget = vtarget;
            this.vsbi = vsbi;
            this.mrsp = mrsp;
        }
    }
}