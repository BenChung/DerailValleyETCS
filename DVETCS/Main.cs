using System;
using System.Linq;
using System.Collections.Generic;
using UnityModManagerNet;
using DV.Signs;
using UnityEngine;
using HarmonyLib;

namespace DVETCS
{
    [EnableReloading]
    class ETCSMain
    {
        static bool Load(UnityModManager.ModEntry modEntry)
        {
            server = new InterfaceServer();
            modEntry.OnToggle = OnToggle;
            modEntry.OnUnload = Unload;
            modEntry.OnFixedUpdate = OnFixedUpdate;
            return true;
        }

        private static bool Unload(UnityModManager.ModEntry arg)
        {
            Stop();
            return true;
        }

        private static bool enabled;
        private static bool OnToggle(UnityModManager.ModEntry arg1, bool active)
        {
            if (active)
            {
                Start();
            }
            else
            {
                Stop();
            }

            enabled = active;
            return true;
        }

        private static InterfaceServer server;
        private static void Stop()
        {
            if (trackManager != null)
                trackManager.RemoveLines();
            server.Dispose();
        }

        private static void Start()
        {
            if (server != null)
                server.Dispose();
            server = new InterfaceServer();
            trackManager = new TrackManager();
        }

        // app logic
        private static LocoControllerBase PlayerLoco()
        {
            return PlayerManager.Car?.GetComponent<LocoControllerBase>();
        }


        private static long nframe = 0;
        private static TrackManager trackManager;
        private static void OnFixedUpdate(UnityModManager.ModEntry entry, float dt)
        {
            if (!enabled)
                return;
            LocoControllerBase loco = PlayerLoco();
            float speed = 0.0f;
            if (loco == null)
            {
                speed = 0.0f;
            } else
            {
                speed = loco.GetSpeedKmH();
                
            }
            if (nframe % 10 == 0)
            {
                trackManager.LoadSituation(loco);
            }
            nframe++;
            server.NotifySpeed(speed);
        }
    }
}
