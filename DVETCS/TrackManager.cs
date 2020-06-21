using DV.PointSet;
using DV.Signs;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;
using TMPro;
using UnityEngine;

namespace DVETCS
{
    /**
     * The actual coordinate system that this uses is weird. It adopts each RailTrack's dt directly - which is not constant.
     * Concequently there's no actual way to go from position in this to position in the "real world"
     */
    class TrackManager
    {

        private SignPlacer placer;
        private Dictionary<Bogie, double> bogiePositions;
        private SortedDictionary<double, double> speedLimits;
        private TrackBlock trackSection;
        public TrackManager()
        {
            this.placer = UnityEngine.Object.FindObjectOfType<SignPlacer>();
            this.bogiePositions = new Dictionary<Bogie, double>();
            this.speedLimits = new SortedDictionary<double, double>();
        }

        private int TrackDirection(Bogie bogie)
        {
            var trackDirAtBogie = bogie.track.curve.GetTangentAt((float)bogie.traveller.Span / bogie.track.curve.length);
            return Vector3.Dot(trackDirAtBogie.normalized, bogie.transform.forward) < 0.0 ? -1 : 1;
        }
        private void AddBogie(Bogie bogie)
        {
            if (bogiePositions.ContainsKey(bogie)) return;
            TrackBlock.TrackInfo? currentTrack = trackSection.AddTrack(bogie.track);
            if (!currentTrack.HasValue) return; // this bogie isn't on contigous track
            double trackOffset = (double)currentTrack?.offset;
            bool inTravelDir = (bool)currentTrack?.relativeDirection;
            var bogiePos = inTravelDir ? bogie.traveller.Span : bogie.traveller.Span;
            bogiePositions[bogie] = bogiePos + currentTrack.Value.offset;
        }
        private List<Bogie> GetTrainBogies(LocoControllerBase loco)
        {
            var bogies = new List<Bogie>();
            bogies.AddRange(Enumerable.Reverse(loco.forwardCars).SelectMany(c => c.Bogies));
            bogies.AddRange(loco.train.Bogies);
            bogies.AddRange(loco.rearCars.SelectMany(c => c.Bogies));
            return bogies;
        }

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
        List<Lines.VectorLine> lastLines = new List<Lines.VectorLine>();
        private void AddSigns(TrackBlock.TrackInfo along, Dictionary<BezierPoint, float> curveSpans, SignDebug[] signs)
        {
            var stopwatch = new System.Diagnostics.Stopwatch();
            stopwatch.Start();
            var planes = signs.Select(s => (sign:s, offset:s.transform.position, normal: s.transform.TransformDirection(Vector3.forward)));
            var intersections = planes.Select(inter=>(
                    sign:inter.sign, 
                    intersections: along.track.curve.PlaneIntersect(inter.normal, inter.offset)
                                                    .Where(i => AlignedDirections(along.track.curve.GetTangent(i.p1, i.p2, (float)i.span), inter.normal, along.relativeDirection, -0.98f))
                                                    .Where(i => Vector3.Distance(BezierCurve.GetPoint(i.p1, i.p2, (float)i.span), inter.offset) < 3f), 
                    speeds:inter.sign.GetComponentsInChildren<TextMeshPro>()
                                     .Where(tm=>!String.IsNullOrWhiteSpace(tm.text) && tryParseInt(tm.text) >= 0)
                                     .Select(tm=>int.Parse(tm.text)*10))).ToList();

            foreach (var intersection in intersections)
            {
                if (intersection.intersections.Count() == 0) continue; // sign not relevant
                var primeInt = intersection.intersections.OrderBy(e => Vector3.Distance(BezierCurve.GetPoint(e.p1, e.p2, (float)e.span), intersection.sign.transform.position)).First();
                var signSpan = along.offset + curveSpans[primeInt.p1] + primeInt.span*(curveSpans[primeInt.p2] - curveSpans[primeInt.p1]);
                var signSpeed = 0;
                if (intersection.speeds.Count() == 1)
                {
                    signSpeed = intersection.speeds.First();
                } else // sign for a junction
                {
                    var selJunction = along.relativeDirection ? along.track.outJunction : along.track.inJunction;
                    if (selJunction == null)
                    {
                        throw new Exception($"Null junction following junction speed sign {signSpan}");
                    }
                    if (selJunction.inBranch.track != along.track)
                        throw new Exception("Invalid junction construction (junction following junction speed sign not properly aligned)");
                    signSpeed = intersection.speeds.ToList()[selJunction.selectedBranch];
                }
                speedLimits[signSpan] = signSpeed;
            }
            stopwatch.Stop();
        }
        public void LoadSituation(LocoControllerBase loco)
        {
            if (loco == null) return;
            // we affix the "zero" of our coordinate system to the 0 point of the RailTrack that the locomotive's front bogie is currently sitting on
            // the new axis will be forward-aligned with the bogie
            var baseBogie = loco.train.Bogies[0];
            var baseTrack = baseBogie.track;
            bogiePositions.Clear();
            speedLimits.Clear();
            var direction = TrackDirection(baseBogie);
            trackSection = new TrackBlock(baseTrack, direction > 0);

            // add the bogies
            var trainBogies = GetTrainBogies(loco);
            Debug.Log("bogies");
            trainBogies.ForEach(AddBogie);

            // we now know where all the bogies are. We now need to figure out where the signs are.

            RemoveLines();
            var signs = UnityEngine.Object.FindObjectsOfType<SignDebug>();
            foreach (var track in trackSection)
            {
                var spans = track.track.curve.PointSpans();
                AddSigns(track, spans, signs);
            }

            int niters = 0;
            while (speedLimits.Count == 0 || bogiePositions.Values.Min() < speedLimits.Keys.Min()) // if there's no sign controlling the rear of the train
            {
                var firstTrack = trackSection.GetFirstTrack();
                if (!firstTrack.HasValue) break; // empty list, we have other problems
                var track = firstTrack.Value;
                var newTrack = TrackBlock.TrackAlongDir(track.track, !track.relativeDirection, false);
                if (newTrack != null)
                {
                    var newTrackInfo = trackSection.AddTrack(newTrack);
                    var spans = newTrack.curve.PointSpans();
                    AddSigns(newTrackInfo.Value, spans, signs);
                } else
                {
                    // we've ran out of track
                    speedLimits[track.offset] = 40.0f; // pick a reasonable default
                    Debug.Log($"default end");
                    break;
                }
                if (niters > 10) break;
                niters++;
            }
            Debug.Log(String.Join(",", trackSection.Select(ts => ts.offset.ToString())));

            foreach (var track in trackSection)
            {
                var startColor = track.relativeDirection ? Color.magenta : Color.blue;
                var endColor = track.relativeDirection ? Color.blue : Color.magenta;
                lastLines.Add(Lines.DrawCurve(track.track.curve, Vector3.up * 10.0f, startColor, endColor));
            }
            foreach (var bogiePos in bogiePositions)
            {
                var bogieTrackInfo = trackSection.GetTrackInfo(bogiePos.Key.track).Value;
                var mappedPosition = (bogiePos.Value - bogieTrackInfo.offset) / bogiePos.Key.track.curve.length;
                var bogieWorldPos = bogiePos.Key.track.curve.GetPointAt((float)mappedPosition);
                lastLines.Add(Lines.DrawLine(bogieWorldPos, bogieWorldPos + Vector3.up * 10.0f, Color.yellow));
            }
            foreach (var speedPos in speedLimits)
            {
                var ti = trackSection.GetTrackInfo((float)speedPos.Key);
                if (ti == null)
                {
                    Debug.Log("Invalid sign positions");
                    continue;
                }
                var mappedPosition = (speedPos.Key - ti.Value.offset) / ti.Value.track.curve.length;
                var speedWorldPos = ti.Value.track.curve.GetPointAt((float)mappedPosition);
                lastLines.Add(Lines.DrawLine(speedWorldPos, speedWorldPos + Vector3.up * 10.0f, Color.green));

            }
            Debug.Log("signs " + String.Join(",", speedLimits.Select(s => $"{s.Key} : {s.Value}")));
        }

        public void RemoveLines()
        {
            foreach (var line in lastLines) line.Remove();
            lastLines.Clear();
        }
    }
}
