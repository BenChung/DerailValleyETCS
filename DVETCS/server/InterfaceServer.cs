using System;
using System.Collections.Generic;
using System.Text;
using EmbedIO;
using EmbedIO.Routing;
using EmbedIO.WebApi;
using System.Resources;
using System.Threading.Tasks;

namespace DVETCS
{
    class InterfaceServer : IDisposable
    {
        private WebServer server;
        private ETCSHandlerModule etcsHandler;
        
        public InterfaceServer()
        {
            etcsHandler = new ETCSHandlerModule("/etcs");
            server = new WebServer(o => o.WithUrlPrefix("http://localhost:8088/")
                .WithMode(HttpListenerMode.EmbedIO))
                .WithModule(etcsHandler)
                .WithEmbeddedResources("/", typeof(InterfaceServer).Assembly, "DVETCS.web");
            server.RunAsync();
        }

        public void NotifySpeed(float currentSpeed,
            (SpeedProfile.SpeedStateInfo? currentSSI, SpeedProfile.TargetStateInfo currentTSI) speedLimit,
            List<SpeedProfile.MRSPElem> mrspProfile)
        {
            var (ssi, tsi) = speedLimit;
            etcsHandler.NotifyState(currentSpeed, ssi, mrspProfile);
        }

        public void Dispose()
        {
            server.Dispose();
        }
    }
}
