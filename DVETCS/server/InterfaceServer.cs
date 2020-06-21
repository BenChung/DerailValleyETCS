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

        public void NotifySpeed(float currentSpeed)
        {
            etcsHandler.NotifySpeed(currentSpeed);
        }

        public void Dispose()
        {
            server.Dispose();
        }
    }
}
