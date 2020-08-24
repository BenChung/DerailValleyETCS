using DV.Signs;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MoreLinq;
using TMPro;
using UnityEngine;

namespace DVETCS
{
    public class SpeedProfile
    {
        public class TargetStateInfo
        {
            public readonly bool exists;
            public readonly float dEBI;
            public readonly float dSBI;
            public readonly float dW;
            public readonly float dP;
            public readonly float dI;

            public TargetStateInfo()
            {
                this.exists = false;
                this.dI = float.NaN;
                this.dP = float.NaN;
                this.dW = float.NaN;
                this.dSBI = float.NaN;
                this.dEBI = float.NaN;
            }

            public TargetStateInfo(float dI, float dP, float dW, float dSBI, float dEBI)
            {
                this.exists = true;
                this.dI = dI;
                this.dP = dP;
                this.dW = dW;
                this.dSBI = dSBI;
                this.dEBI = dEBI;
            }

            public TargetStateInfo Min(TargetStateInfo tsi)
            {
                if (tsi == null)
                    return this;
                return new TargetStateInfo(Mathf.Min(dI, tsi.dI), Mathf.Min(dP, tsi.dP), Mathf.Min(dW, tsi.dW), 
                    Mathf.Min(dSBI, tsi.dSBI), Mathf.Min(dEBI, tsi.dEBI));
            }
        }
        public struct SpeedStateInfo
        {
            public readonly float EBI;
            public readonly float SBI;
            public readonly float W;
            public readonly float P;
            public readonly float T;

            public SpeedStateInfo(float T, float P, float W, float SBI, float EBI)
            {
                this.T = T;
                this.P = P;
                this.W = W;
                this.SBI = SBI;
                this.EBI = EBI;
            }

            public SpeedStateInfo Min(SpeedStateInfo ssi)
            {
                return new SpeedStateInfo(Mathf.Min(T, ssi.T), Mathf.Min(P, ssi.P), Mathf.Min(W, ssi.W), Mathf.Min(SBI, ssi.SBI), Mathf.Min(EBI, ssi.EBI));
            }
        }
        private interface ISpeedElement
        {
            (SpeedStateInfo ssi, TargetStateInfo tsi) SpeedsAt(float pos, float vest, float accel);
        }
        private struct BlockSegment : ISpeedElement
        {
            public BlockSegment(float speed) => this.speed = speed;
            public float speed;

            private const float dv_ebi_min = 7.5f/3.6f;
            private const float dv_ebi_max = 15f / 3.6f;
            private const float v_ebi_min = 110f / 3.6f;
            private const float v_ebi_max = 210f / 3.6f;
            private const float dv_sbi_min = 5.5f / 3.6f;
            private const float dv_sbi_max = 10f / 3.6f;
            private const float v_sbi_min = 110f / 3.6f;
            private const float v_sbi_max = 210f / 3.6f;
            private const float dv_warning_min = 4f / 3.6f;
            private const float dv_warning_max = 5f / 3.6f;
            private const float v_warning_min = 110f / 3.6f;
            private const float v_warning_max = 140f / 3.6f;

            public static float CalculateEBI(float MRSP)
            {
                if (MRSP < v_ebi_min)
                    return dv_ebi_min + MRSP;
                var Cebi = (dv_ebi_max - dv_ebi_min) / (v_ebi_max - v_ebi_min);
                return Mathf.Min(dv_ebi_min + Cebi * (MRSP - v_ebi_min), dv_ebi_max) + MRSP;
            }

            public static float CalculateSBI(float MRSP)
            {
                if (MRSP < v_sbi_min)
                    return dv_sbi_min + MRSP;
                var Csbi = (dv_sbi_max - dv_sbi_min) / (v_sbi_max - v_sbi_min);
                return Mathf.Min(dv_sbi_min + Csbi * (MRSP - v_sbi_min), dv_sbi_max) + MRSP;
            }

            public static float CalculateWarning(float MRSP)
            {
                if (MRSP < v_warning_min)
                    return dv_warning_min + MRSP;
                var Cwarning = (dv_warning_max - dv_warning_min) / (v_warning_max - v_warning_min);
                return Mathf.Min(dv_warning_min + Cwarning * (MRSP - v_warning_min), dv_warning_max) + MRSP;
            }

            public (SpeedStateInfo ssi, TargetStateInfo tsi) SpeedsAt(float pos, float vest, float accel)
            {
                return (new SpeedStateInfo(speed, speed, CalculateWarning(speed), CalculateSBI(speed), CalculateEBI(speed)),
                    null);
            }

            public override string ToString()
            {
                return
                    $"BlockSegment(perm:{speed}, wrn:{CalculateWarning(speed)}, wrn:{CalculateSBI(speed)}, wrn:{CalculateEBI(speed)})";
            }
        }
        private enum CurveType
        {
            SBD,
            EBD
        }
        private struct BrakingCurveSegment : ISpeedElement
        {
            private const float Tbe = 5.0f; // from 10.38 seconds
            private const float Tbs = 5.0f; // from 10.38 seconds
            private const float Ttraction = 2.0f; // guess
            private const float Twarning = 2.0f;
            private const float Tdriver = 2.0f;
            private BrakingCurves.BrakingCurve curve;
            private readonly float vtarget;
            private readonly CurveType curveType;

            public BrakingCurveSegment(BrakingCurves.BrakingCurve curve, CurveType curveType, float vtarget)
            {
                this.curve = curve;
                this.vtarget = vtarget;
                this.curveType = curveType;
            }

            public float Target => curve.EoA;
            public BrakingCurves.BrakingCurve Curve => curve; 

            public static float IndicationPosition(float vest, float ebiStart)
            {
                var Tindication = Mathf.Max(0.8f * Tbs, 5.0f) + Tdriver;
                return ebiStart - vest * Tbs - vest * Tdriver - vest * Tindication;
            }

            public (SpeedStateInfo ssi, TargetStateInfo tsi) SpeedsAt(float pos, float vest, float accel)
            {
                var Tbrem = Tbe - Ttraction;
                var vdelta0 = 0.0f;
                var vdelta1 = accel*Ttraction;
                var vdelta2 = Mathf.Clamp(accel, 0.0f,0.4f) * Tbrem;
                var vbec = Mathf.Max(vest + vdelta0 + vdelta1, vtarget) + vdelta2;
                var dbec = Mathf.Max(vest + vdelta0 + vdelta1 / 2, vtarget) * Ttraction +
                           (Mathf.Max(vest + vdelta0 + vdelta1, vtarget) + vdelta2 / 2) * Tbrem;

                Debug.Log($"EBI calc from pos {pos} at {pos+dbec} r vel {curve.VelocityForPosition(pos+dbec)} offs {vdelta0 + vdelta1 + vdelta2}");

                float vebi, debi;
                if (curveType == CurveType.SBD)
                {
                    vebi = float.NaN;
                    debi = float.NaN;
                }
                else
                {
                    vebi = curve.VelocityForPosition(pos + dbec) - (vdelta0 + vdelta1 + vdelta2);
                    debi = curve.PositionForVelocity(vbec) - dbec;
                }


                var Tbs1 = Tbs;
                var Tbs2 = Tbs;
                var dsbi = 0.0f;
                if (curveType == CurveType.SBD)
                    dsbi = curve.PositionForVelocity(vest) - vest * Tbs1;
                else 
                    dsbi = curve.PositionForVelocity(vest) - vest * Tbs2;

                var Dbedisplay = (vest + vdelta0 + vdelta1 / 2) * Ttraction +
                                 (vest + vdelta0 + vdelta1 + vdelta2 / 2) * Tbrem;
                Debug.Log($"dbec_display {Dbedisplay} dbec {dbec}");
                var vsbi = ComputeVSBI(pos, vest, Tbs1, Tbs2, Dbedisplay, vdelta0, vdelta1, vdelta2);

                var vwarning = ComputeVSBI(pos + Twarning*vest, vest, Tbs1, Tbs2, Dbedisplay, vdelta0, vdelta1, vdelta2);
                var dwarning = dsbi - vest * Twarning;
                var dperm = dsbi - vest * Tdriver;// TODO: implement the guidance curve

                var vperm = 0.0f;
                if (pos + vest * (Tdriver + Tbs2) + Dbedisplay > curve.PositionForVelocity(vtarget))
                {
                    vperm = vtarget;
                }
                else
                {
                    vperm = Mathf.Max(
                        curve.VelocityForPosition(pos + vest * (Tdriver + Tbs2) + Dbedisplay) -
                        (vdelta0 + vdelta1 + vdelta2), vtarget);
                }

                var Tindication = Mathf.Max(0.8f * Tbs, 5.0f) + Tdriver;
                var dindication = dperm - vest * Tindication;
                Debug.Log($"vtarget {vtarget*3.6} vperm {vperm*3.6f} vwarning {vwarning*3.6} vsbi {vsbi * 3.6} vebi {vebi * 3.6}");
                return (new SpeedStateInfo(vtarget * 3.6f, vperm * 3.6f, vwarning * 3.6f, vsbi * 3.6f, vebi * 3.6f), new TargetStateInfo(dindication, dperm, dwarning, dsbi, debi));
            }

            private float ComputeVSBI(float pos, float vest, float Tbs1, float Tbs2, float Dbedisplay, float vdelta0, float vdelta1,
                float vdelta2)
            {
                float vsbi;
                if (curveType == CurveType.SBD)
                {
                    if (pos + vest * Tbs1 < curve.EoA)
                        vsbi = 0.0f;
                    else
                        vsbi = curve.VelocityForPosition(pos + vest * Tbs1);
                }
                else
                {
                    var basepos = pos + vest * Tbs2 + Dbedisplay;
                    var basevel = curve.VelocityForPosition(basepos);
                    Debug.Log(
                        $"SBI calc {basevel - (vdelta0 + vdelta1 + vdelta2)} from pos {pos} at pos {basepos} w/ basevel {basevel} and modifier {vdelta0 + vdelta1 + vdelta2} target {vtarget} maxvel {BlockSegment.CalculateSBI(vtarget)}");
                    if (pos + vest * Tbs2 + Dbedisplay > curve.PositionForVelocity(vtarget))
                    {
                        vsbi = BlockSegment.CalculateSBI(vtarget);
                    }
                    else
                    {
                        vsbi = Mathf.Max(
                            curve.VelocityForPosition(pos + vest * Tbs2 + Dbedisplay) -
                            (vdelta0 + vdelta1 + vdelta2), BlockSegment.CalculateSBI(vtarget));
                    }
                }

                return vsbi;
            }
            public override string ToString()
            {
                return $"BrakingCurve(EoA:{curve.EoA}, target:{vtarget}, curveType:{curveType})";
            }
        }

        private interface ISpeedSign
        {
            void Removed();
            event EventHandler SpeedChanged;
            float Speed { get; }
        }

        private class NormalSign : ISpeedSign
        {

            public float Speed { get; }

            public NormalSign(float speed)
            {
                this.Speed = speed;
            }
            public void Removed()
            {
            }

            public event EventHandler SpeedChanged;
        }

        private class JunctionSign : ISpeedSign
        {
            private Junction junction;
            private int selectedBranch;
            private float[] speeds;
            private Action<Junction.SwitchMode, int> junctionSwitchedAction;

            public JunctionSign(float[] speeds, Junction junction)
            {
                this.speeds = speeds;
                this.junction = junction;
                selectedBranch += junction.selectedBranch;
                junctionSwitchedAction = (mode, i) =>
                {
                    selectedBranch = i;
                    SpeedChanged?.Invoke(this, null);
                };
                junction.Switched += junctionSwitchedAction;
            }

            public float Speed => this.speeds[this.selectedBranch];

            public void Removed()
            {
                junction.Switched -= junctionSwitchedAction;
            }

            public event EventHandler SpeedChanged;
        }

        private SortedDictionary<double, ISpeedSign> speedLimits;
        private RangeTree.RangeTree<double, ISpeedElement> MRSP;
        private TrackBlock block;
        private SignDebug[] signs;
        private BrakingCurves curves;
        private bool dirty = true;
        private float trainLength;
        public SpeedProfile(TrackBlock block, BrakingCurves curves, float trainLength)
        {
            this.block = block;
            this.signs = UnityEngine.Object.FindObjectsOfType<SignDebug>();
            this.speedLimits = new SortedDictionary<double, ISpeedSign>();
            foreach (var ti in block) AddSigns(ti);
            block.TrackAdded += AddedTrack;
            block.TrackRemoved += TrackRemoved;
            this.trainLength = trainLength;
            this.MRSP = new RangeTree.RangeTree<double, ISpeedElement>();
            this.curves = curves;
        }

        private void TrackRemoved(TrackBlock block, TrackBlock.TrackInfo removed)
        {
            Debug.Log($"Removing signs at track {removed.track.GetInstanceID()} at {removed.offset} to {removed.track.curve.length + removed.offset}");
            double[] toRemove = new double[speedLimits.Count]; int rmp = 0;
            double blockStart = removed.offset, blockEnd = removed.offset + removed.track.curve.length;
            foreach (var limit in speedLimits)
            {
                Debug.Log($"removing {limit.Key} {limit.Value} torem {limit.Key > blockStart && limit.Key < blockEnd}");
                if (limit.Key > blockStart && limit.Key < blockEnd)
                    toRemove[rmp++] = limit.Key;
            }
            for (int trm = 0; trm < rmp; trm++)
            {
                Debug.Log($"Removed {toRemove[trm]}");
                speedLimits.Remove(toRemove[trm]);
            }
            dirty = true;
        }

        private void AddedTrack(TrackBlock block, TrackBlock.TrackInfo added, bool sideAdded, Junction via)
        {
            Debug.Log("New track");
            AddSigns(added);
        }

        public void Clear() { speedLimits.Clear(); }
        

        private int tryParseInt(string text)
        {
            int i = -1;
            if (int.TryParse(text, out i))
                return i;
            return -1;
        }
        private bool AlignedDirections(Vector3 a, Vector3 b, bool direction, float tolerance)
        {
            a = direction ? a : -a;
            a = new Vector3(a.x, 0.0f, a.z);
            b = new Vector3(b.x, 0.0f, b.z);
            float det = Vector3.Dot(a.normalized, b.normalized);
            return det < tolerance;
        }
        Dictionary<SignDebug, Lines.VectorLine> signLines = new Dictionary<SignDebug, Lines.VectorLine>();
        private void AddSigns(TrackBlock.TrackInfo along)
        {
            Debug.Log("Add Signs");
            var curveSpans = along.track.curve.PointSpans();
            var planes = signs.Select(s => (sign: s, offset: s.transform.position, normal: s.transform.TransformDirection(Vector3.forward)));
            var intersections = planes.Select(inter => (
                    sign: inter.sign,
                    intersections: along.track.curve.PlaneIntersect(inter.normal, inter.offset)
                                                    .Where(i => AlignedDirections(along.track.curve.GetTangent(i.p1, i.p2, (float)i.span), inter.normal, along.relativeDirection, -0.98f))
                                                    .Where(i => Vector3.Distance(BezierCurve.GetPoint(i.p1, i.p2, (float)i.span), inter.offset) < 3f),
                    speeds: inter.sign.GetComponentsInChildren<TextMeshPro>()
                                     .Where(tm => !String.IsNullOrWhiteSpace(tm.text) && tryParseInt(tm.text) >= 0)
                                     .Select(tm => int.Parse(tm.text) * 10))).ToList();

            foreach (var intersection in intersections)
            {
                if (intersection.intersections.Count() == 0) continue; // sign not relevant
                var primeInt = intersection.intersections.OrderBy(e => Vector3.Distance(BezierCurve.GetPoint(e.p1, e.p2, (float)e.span), intersection.sign.transform.position)).First();
                var curvePos = curveSpans[primeInt.p1] + primeInt.span * (curveSpans[primeInt.p2] - curveSpans[primeInt.p1]);
                var signSpan = along.offset + (along.relativeDirection ? curvePos : along.track.curve.length - curvePos);
                var signSpeed = 0;
                if (intersection.speeds.Count() == 1)
                {
                    signSpeed = intersection.speeds.First();
                    AddLimitRaw((float)signSpan, new NormalSign(signSpeed));
                }
                else // sign for a junction
                {
                    var selJunction = along.relativeDirection ? along.track.outJunction : along.track.inJunction;
                    if (selJunction == null)
                    {
                        throw new Exception($"Null junction following junction speed sign {signSpan}");
                    }
                    if (selJunction.inBranch.track != along.track)
                        throw new Exception("Invalid junction construction (junction following junction speed sign not properly aligned)");
                    signSpeed = intersection.speeds.ToList()[selJunction.selectedBranch];
                    var junctionSign = new JunctionSign(intersection.speeds.Select(Convert.ToSingle).ToArray(), selJunction);
                    junctionSign.SpeedChanged += (sender, args) => dirty = true;
                    AddLimitRaw((float)signSpan, junctionSign);
                }
            }
        }

        public float FirstLimit()
        {
            if (speedLimits.Count == 0) return float.PositiveInfinity;
            return (float)speedLimits.Keys.Min();
        }

        private void AddLimitRaw(float at, ISpeedSign limit)
        {
            Debug.Log("New limit");
            speedLimits[at] = limit;
            dirty = true;
        }

        private void AddLimit(float at, ISpeedSign limit)
        {
            if (block.GetTrackInfo(at) == null)
                throw new Exception("Attempt to add invalid speed limit");
            AddLimitRaw(at, limit);
            UpdateMRSP();
        }

        public void RefreshSigns()
        {
            speedLimits.Clear();
            this.signs = UnityEngine.Object.FindObjectsOfType<SignDebug>();
            foreach (var track in block)
            {
                AddSigns(track);
            }
        }

        private Queue<(double at, double speed)> addedSpeeds = new Queue<(double, double)>();
        private Queue<(double at, double from, double to)> speedReductions = new Queue<(double at, double from, double to)>();
        private SortedDictionary<double, double> MRSPdict = new SortedDictionary<double, double>();
        public void UpdateMRSP()
        {
            if (!dirty) return; // if we haven't updated the speed limits since we last updated the MRSP we don't need to do it again
            var speedInfoArr = speedLimits.ToArray();
            var currentLimit = 40.0f;
            MRSP.Clear();
            MRSPdict.Clear();
            addedSpeeds.Clear();
            Debug.Log($"Update MRSP {speedInfoArr.Length}");
            for (int lIdx = 0; lIdx < speedInfoArr.Length; lIdx++)
            {
                var speedInfo = speedInfoArr[lIdx];
                var speedLimit = speedInfo.Value; var speedAt = speedInfo.Key;
                Debug.Log($"Limit {speedAt}@{speedLimit}");
                while (addedSpeeds.Count > 0 && addedSpeeds.Peek().at < speedAt)
                {
                    var MRSPchange = addedSpeeds.Dequeue();
                    currentLimit = (float)MRSPchange.speed;
                }
                if (speedLimit.Speed < currentLimit)
                {
                    currentLimit = (float)speedLimit.Speed;
                    MRSPdict.Add(speedAt, speedLimit.Speed); // decreases happen immediately
                } else
                {
                    bool useless = false;
                    for (int j = lIdx + 1; j < speedInfoArr.Length && speedInfoArr[j].Key < speedAt + trainLength; j++)
                    {
                        if (speedInfoArr[j].Value.Speed < speedLimit.Speed)
                        {
                            // the limit decreases again within a train length of the current sign
                            useless = true;
                            break;
                        }
                    }
                    if (useless) continue; // the current sign will not affect the MRSP
                    MRSPdict.Add(speedAt + trainLength, speedLimit.Speed);
                    addedSpeeds.Enqueue((speedAt + trainLength, speedLimit.Speed));
                }
            }
            currentLimit = 40.0f; var currentPos = block.Select(b => b.offset).Min();
            foreach (var mrsp_change in MRSPdict)
            {
                Debug.Log($"adding {mrsp_change.Key}:{mrsp_change.Value} from {currentPos}:{currentLimit}");
                var newSegment = new BlockSegment(currentLimit);
                MRSP.Add(currentPos, mrsp_change.Key, newSegment);
                if (mrsp_change.Value < currentLimit)
                    speedReductions.Enqueue((mrsp_change.Key, currentLimit, mrsp_change.Value));
                currentLimit = (float)mrsp_change.Value; currentPos = mrsp_change.Key;
            }
            var finalSegment = new BlockSegment(currentLimit);
            MRSP.Add(currentPos, double.PositiveInfinity, finalSegment);
            while (speedReductions.Count > 0)
            {
                var (rpos, ospeed, nspeed) = speedReductions.Dequeue();
                var startSpeed = BlockSegment.CalculateEBI((float)(ospeed/3.6));
                var targetSpeed = BlockSegment.CalculateEBI((float)(nspeed/3.6));
                var finalSpeed = (float) (nspeed / 3.6);
                var brakingCurveOpt = curves.CalculateBrakingCurve(block, (float)rpos, targetSpeed, startSpeed, 1.0f, finalSpeed);
                if (!brakingCurveOpt.HasValue)
                    continue;
                var brakingCurve = brakingCurveOpt.Value;
                MRSP.Add(BrakingCurveSegment.IndicationPosition((float)(ospeed/3.6), brakingCurve.Start),brakingCurve.End,
                    new BrakingCurveSegment(brakingCurve, CurveType.EBD, (float)(nspeed / 3.6)));
                Debug.Log("Braking curve");
            }
            dirty = false;
        }

        public (SpeedStateInfo currentSSI, TargetStateInfo currentTSI) GetSpeedLimit(double pos, float vest, float accel)
        {
            var res = MRSP.Query(pos);
            SpeedStateInfo? currentSSI = null;
            TargetStateInfo currentTSI = null;

            // calculate the MRDT
            var speedInfos = res.Select(x => (tgt: x, ti: x.SpeedsAt((float) pos, vest, accel))).ToList();
            var mrdt0s = speedInfos.MinBy(s => s.ti.ssi.P);
            if (!mrdt0s.Any())
                throw new Exception("No speed region found!");

            var mrdt = mrdt0s.First();
            var unused = speedInfos.ToHashSet();
            var updated = true;
            unused.Remove(mrdt);
            while (unused.Any() && updated)
            {
                updated = false;
                foreach (var tgt in unused)
                {
                    if (tgt.ti.tsi.dI < mrdt.ti.tsi.dP)
                    {
                        mrdt = tgt;
                        unused.Remove(mrdt);
                        updated = true;
                        break;
                    }
                }
            }
            


            foreach (var ts in res)
            {
                var (ssi, tsi) = ts.SpeedsAt((float)pos, vest, accel);
                if (currentSSI == null)
                    currentSSI = ssi;
                if (currentTSI == null && tsi != null)
                    currentTSI = tsi;
                
                currentSSI = currentSSI.Value.Min(ssi);
                currentTSI = currentTSI != null ? currentTSI.Min(tsi) : tsi;
            }
            return (currentSSI.Value, currentTSI);
        }

        public struct MRSPElem
        {
            public MRSPElem(float distance, float speed)
            {
                this.distance = distance;
                this.speed = speed;
            }
            public float distance;
            public float speed;
        }
        public List<MRSPElem> GetClientMRSP(float front)
        {
            float lastSpeedPos = (float)block.Select(b => b.offset).Min();
            float lastSpeed = 40.0f;
            foreach (var kv in MRSPdict)
            {
                if (kv.Key > front) continue;
                if (kv.Key > lastSpeedPos)
                {
                    lastSpeedPos = (float)kv.Key;
                    lastSpeed = (float)kv.Value;
                }
            }

            var mrsps = new List<MRSPElem>();
            mrsps.Add(new MRSPElem(lastSpeedPos - front, lastSpeed));

            mrsps.AddRange(MRSPdict.Where(k => k.Key > front)
                .Select(k => new MRSPElem((float) k.Key - front, (float) k.Value)));
            return mrsps;
        }



        private List<Lines.VectorLine> myLines = new List<Lines.VectorLine>();
        public void DrawDebug()
        {
            myLines.ForEach(l=>l.Remove());
            myLines.Clear();
            foreach (var speedPos in speedLimits)
            {
                var ti = block.GetTrackInfo((float)speedPos.Key);
                if (ti == null)
                {
                    Debug.Log("Invalid sign positions");
                    continue;
                }
                var mappedPosition = (speedPos.Key - ti.Value.offset) / ti.Value.track.curve.length;
                var speedWorldPos = ti.Value.track.curve.GetPointAt(ti.Value.relativeDirection ? (float)mappedPosition : 1 - (float)mappedPosition);
                myLines.Add(Lines.RegisterLine(Lines.DrawLine(speedWorldPos, speedWorldPos + Vector3.up * 10.0f, Color.green)));
            }

            foreach (var speedSection in MRSP)
            {
                if (!(speedSection.Value is BrakingCurveSegment) || speedSection.From < block.Min) continue;
                var startPos = block.CurveAt((float)speedSection.From);
                myLines.Add(Lines.RegisterLine(Lines.DrawLine(startPos.pos, startPos.pos + Vector3.up*10.0f, Color.red)));
            }
            Debug.Log($"speeds {String.Join(", ", speedLimits.OrderBy(x=>x.Key).Select(l => $"{l.Value}@{l.Key}"))}");
            Debug.Log($"MRSP {String.Join(", ", MRSP.Select(s=>$"{s.From} -> {s.To} : {s.Value}"))}");
            
        }
    }
}
