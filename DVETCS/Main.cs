using System;
using System.Linq;
using System.Collections.Generic;
using UnityModManagerNet;
using DV.Signs;
using UnityEngine;
using HarmonyLib;
using System.Reflection;

namespace DVETCS
{
    [EnableReloading]
    class ETCSMain
    {
        static Harmony harmony;
        static bool Load(UnityModManager.ModEntry modEntry)
        {
            server = new InterfaceServer();
            harmony = new Harmony(modEntry.Info.Id);
            harmony.PatchAll(Assembly.GetExecutingAssembly());
            modEntry.OnToggle = OnToggle;
            modEntry.OnUnload = Unload;
            modEntry.OnFixedUpdate = OnFixedUpdate;
            return true;
        }

        private static bool Unload(UnityModManager.ModEntry arg)
        {
            Stop();
            harmony.UnpatchAll(arg.Info.Id);
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
            Lines.ClearLines();
            server.Dispose();
            foreach (var tm in GameObject.FindObjectsOfType<TrackManager>())
            {
                UnityEngine.Object.Destroy(tm);
            }
        }

        private static void Start()
        {
            if (server != null)
                server.Dispose();
            server = new InterfaceServer();
        }

        // app logic
        private static LocoControllerBase PlayerLoco()
        {
            return PlayerManager.Car?.GetComponent<LocoControllerBase>();
        }

        private static float lastSpeed = 0.0f;
        private static float netDt = 0.0f;
        private static Queue<float> lastAccels = new Queue<float>();
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
                speed = loco.GetForwardSpeed();
                if (trackManager == null)
                {
                    trackManager = loco.gameObject.AddComponent<TrackManager>();
                    return;
                } 
                if (trackManager != loco.GetComponent<TrackManager>())
                {
                    Lines.ClearLines();
                    UnityEngine.Object.Destroy(loco.GetComponent<TrackManager>());
                    trackManager = loco.gameObject.AddComponent<TrackManager>();
                    return;
                }
                lastAccels.Enqueue((loco.train.GetVelocity().magnitude - lastSpeed) / dt);
                if (lastAccels.Count() > 10) lastAccels.Dequeue();
                server.NotifySpeed(speed*3.6f, trackManager.CurrentSpeedLimit(speed), trackManager.MRSPProfile(speed));
            }
            if (loco != null && nframe % 30 == 0)
            {
                //Lines.ClearLines();
                trackManager.DrawDebug();
                /*
                BrakingCurves curves = new BrakingCurves(trackManager.bogiePositions);
                var stoppingPoint = curves.CalculateStoppingPoint(trackManager.trackSection, (float)trackManager.bogiePositions.Values.Max(), loco.train.GetVelocity().magnitude, loco.train.brakeSystem.brakingFactor);
                if (!float.IsInfinity(stoppingPoint))
                {
                    var (stoppingPos, stoppingTangent) = trackManager.trackSection.CurveAt(stoppingPoint);
                    Lines.RegisterLine(Lines.DrawLine(stoppingPos, stoppingPos + Vector3.up * 15.0f, Color.red));
                    Debug.Log($"stopping time {stoppingPoint} front bogie {trackManager.bogiePositions.Values.Max()}");
                }
                */
            }
            nframe++;
        }
    }
    /*
    [HarmonyPatch(typeof(ShunterLocoSimulation), "SimulateEngineRPM")]
    public class ShunterLocoSimulationWSS
    {
        private static void ImplementAntiSlip(ShunterLocoSimulation instance)
        {
            List<double> possibleSolutions = new List<double>();
            var controller = instance.GetComponent<LocoControllerShunter>();
            var car = controller.train;
            if (car != PlayerManager.Car) return;
            if (controller.GetSpeedKmH() == 0.0) return;
            var bogies = car.Bogies;
            var frictionCoefficient = controller.drivingForce.wheelslipToFrictionModifierCurve.Evaluate(Mathf.Clamp01(controller.drivingForce.wheelslip)) *
                ((double)Mathf.Sign(car.GetForwardSpeed()) == (double)Mathf.Sign(controller.reverser) || (double)Mathf.Abs(car.GetForwardSpeed()) <= 1.0 ? controller.drivingForce.sandCoefMax : 1f);
            var slopeMod = 1f - Mathf.Clamp01(controller.drivingForce.slopeCoeficientMultiplier * Mathf.Clamp01(((double)Mathf.Abs(car.transform.localEulerAngles.x) > 180.0 ? Mathf.Abs(car.transform.localEulerAngles.x - 360f) : Mathf.Abs(car.transform.localEulerAngles.x)) / 90f));
            var tractionForceWheelslipLimits = bogies.Select(b => b.rb.mass * slopeMod * frictionCoefficient * 9.8f);
            var braking = bogies.Select(b => b.brakingForce);

            var tfwsl = typeof(DrivingForce).GetField("tractionForceWheelslipLimit", (BindingFlags)(-1)).GetValue(controller.drivingForce);
            var tractionForceWheelslipLimit = (float)tfwsl;
            //cubic setup
            var k = (braking.Average() + tractionForceWheelslipLimit) * bogies.Length / controller.tractionTorqueMult;
            var keyframes = controller.tractionTorqueCurve.keys;
            double speed = controller.GetSpeedKmH();
            var rpmValue = instance.engineRPM.value;
            for (var i = 0; i < keyframes.Length - 1; i++)
            {
                Keyframe p1 = keyframes[i], p2 = keyframes[i + 1];
                Debug.Log($"{p1.time}:{p1.value}:{p1.outTangent} {p2.time}:{p2.value}:{p2.outTangent} {speed / rpmValue}");
                
                double P0 = p1.value, P1 = p2.value;
                var s = speed;

                var dt = p2.time - p1.time;
                var off = p1.time;
                double cubic = 2 * P0 * s - 2 * P1 * s;
                double quadratic = -3 * P0 * s + 3 * P1 * s;
                double linear = -dt * k;
                double constant = -k * off + P0 * s;
                Debug.Log($"qeqns {cubic} {quadratic} {linear} {constant}");
                double s1 = double.NaN, s2 = double.NaN, s3=double.NaN;
                if (Math.Abs(cubic) > double.Epsilon)
                    (s1, s2, s3) = CurveUtils.RealRoots(constant / cubic, linear / cubic, quadratic / cubic);
                else if (Math.Abs(quadratic) > double.Epsilon)
                    (s1, s2) = ((-linear + Math.Sqrt(linear * linear - 4 * quadratic * constant)) / (2 * quadratic), (-linear - Math.Sqrt(linear * linear - 4 * quadratic * constant)) / (2 * quadratic));
                else
                    s1 = -linear / constant;
                double c1 = s1 * dt + off, c2 = s2 * dt + off, c3 = s3 * dt + off;
                if (c1 >= p1.time && c1 <= p2.time) possibleSolutions.Add(s/c1);
                if (c2 >= p1.time && c2 <= p2.time) possibleSolutions.Add(s/c2);
                if (c3 >= p1.time && c3 <= p2.time) possibleSolutions.Add(s/c3);
                if (p1.time < speed / rpmValue && speed / rpmValue < p2.time)
                {
                    var t = (float)(speed / rpmValue - off) / dt;
                    var rpt = P0 * Mathf.Pow(1 - t, 3) + 3 * P0 * Mathf.Pow(1 - t, 2) * t + 3 * P1 * (1 - t) * Mathf.Pow(t, 2) + P1 * Mathf.Pow(t, 3);
                    var refv = controller.tractionTorqueCurve.Evaluate((float)(speed / instance.engineRPM.value));
                    Debug.Log($"speed {speed} reference {speed / rpmValue} {refv} ours {rpt} test {rpmValue * refv - k} wsl {String.Join(", ", car.Bogies.Select(b=>b.wheelslip))} solns {c1} {c2} {c3} tsols {s/c1} {s/c2} {s/c3}");
                }
            }
            Debug.Log($"crpm {rpmValue} ref {speed / rpmValue} nsols {String.Join(", ", possibleSolutions)}");
            if (possibleSolutions.Count > 0 && speed > 1.0)
            {
                var soln = possibleSolutions.Min()*0.9;
                Debug.Log($"intervene? {speed/rpmValue} {speed/soln} {rpmValue} {soln}");
                if ((instance.engineRPM.value > soln || instance.engineRPM.nextValue > soln))
                {
                    Debug.Log($"intervention {soln}");
                    instance.engineRPM.SetNextValue((float)(soln));
                    instance.engineRPM.SetValue((float)(soln));
                }
                if (instance.throttle.value * Mathf.Lerp(0.8f, 1f, Mathf.InverseLerp(instance.engineTemp.min, 75f, instance.engineTemp.value)) > soln)
                {
                    instance.throttle.SetValue((float)soln);
                    instance.throttle.SetValue((float)soln);
                }
                var fpb = controller.GetTotalAppliedForcePerBogie();
                //Debug.Log($"speed {controller.GetSpeedKmH()} {controller.GetEngineRPM()} {instance.engineRPM.value} {controller.drivingForce.wheelslip} {String.Join(", ", possibleSolutions)} {fpb} {tfwsl} {controller.tractionTorqueCurve.Evaluate((float)speed / instance.engineRPM.value)} {controller.tractionTorqueCurve.Evaluate((float)(speed / soln))}");
            }

        }
        public static void Postfix(ShunterLocoSimulation __instance)
        {
            ImplementAntiSlip(__instance);
        }
    }
    */
}
