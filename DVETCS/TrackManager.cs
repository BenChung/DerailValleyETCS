using DV.PointSet;
using DV.Signs;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.Statistics;
using TMPro;
using UnityEngine;

namespace DVETCS
{
    /**
     * The actual coordinate system that this uses is weird. It adopts each RailTrack's dt directly - which is not constant.
     * Concequently there's no actual way to go from position in this to position in the "real world"
     */
    [RequireComponent(typeof(LocoControllerBase))]
    class TrackManager : MonoBehaviour
    {
        private LocoControllerBase boundLoco = null;
        private TrackBlock trackSection;
        private SpeedProfile speedProfile;

        private Dictionary<Junction, Action<Junction.SwitchMode, int>> switchlisteners = new Dictionary<Junction, Action<Junction.SwitchMode, int>>();
        private Dictionary<Bogie, double> bogiePositions;
        private BrakingCurves brakingCurves;

        private int TrackDirection(Bogie bogie)
        {
            var trackDirAtBogie = bogie.track.curve.GetTangentAt((float)bogie.traveller.Span / bogie.track.curve.length);
            return Vector3.Dot(trackDirAtBogie.normalized, bogie.transform.forward) < 0.0 ? -1 : 1;
        }
        private void AddBogie(Bogie bogie)
        {
            if (bogie.track == null)
            {
                Debug.Log("Null bogie track");
                return;
            }
            TrackBlock.TrackInfo? currentTrack = trackSection.AddTrack(bogie.track, true);
            if (!currentTrack.HasValue) return; // this bogie isn't on contigous track
            double trackOffset = (double)currentTrack?.offset;
            bool inTravelDir = (bool)currentTrack?.relativeDirection;
            var bogiePos = inTravelDir ? bogie.traveller.Span : currentTrack.Value.track.curve.length - bogie.traveller.Span;
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

        private void SetupLoco()
        {
            // we affix the "zero" of our coordinate system to the 0 point of the RailTrack that the locomotive's front bogie is currently sitting on
            // the new axis will be forward-aligned with the bogie
            var baseBogie = boundLoco.train.Bogies[0];
            var baseTrack = baseBogie.track;
            bogiePositions.Clear();
            var direction = TrackDirection(baseBogie);
            trackSection = new TrackBlock(baseTrack, direction > 0);
        }

        public void LoadSituation(LocoControllerBase loco)
        {
            if (loco == null) return;

            // add the bogies
            var trainBogies = GetTrainBogies(loco);
            trainBogies.ForEach(AddBogie);
            brakingCurves = new BrakingCurves(bogiePositions);
            Debug.Log($"Load {bogiePositions.Count}");
            this.speedProfile = new SpeedProfile(trackSection, brakingCurves, (float)(bogiePositions.Values.Max() - bogiePositions.Values.Min()));

            EnsureLimit((float)bogiePositions.Values.Min());
            EnsureDistance(1000f);
            speedProfile.UpdateMRSP();
        }

        public void Start()
        {
            this.bogiePositions = new Dictionary<Bogie, double>();
            boundLoco = GetComponent<LocoControllerBase>();
            SetupLoco();
            trackSection.JunctionAdded += JunctionAdded;
            trackSection.JunctionRemoved += JunctionRemoved;
            trackSection.TrackAdded += (block, added, sideAdded, via) => drawDirty = true;
            trackSection.TrackRemoved += (block, removed) => drawDirty = true;
            WorldMover.Instance.WorldMoved += (mover, vector3) => drawDirty = true;
            void OnLoadingNewScenes() => speedProfile.RefreshSigns();
            foreach (var s in GameObject.FindGameObjectsWithTag(Streamer.STREAMERTAG))
            {
                s.GetComponent<Streamer>().LoadingNewScenes += OnLoadingNewScenes;
            }

            LoadSituation(boundLoco);
        }

        private void JunctionRemoved(TrackBlock block, Junction removed, TrackBlock.JunctionInfo info)
        {
            if (!switchlisteners.ContainsKey(removed)) return;
            var listener = switchlisteners[removed];
            switchlisteners.Remove(removed);
            removed.Switched -= listener;
        }

        private void JunctionAdded(TrackBlock block, Junction added, TrackBlock.JunctionInfo info)
        {
            Debug.Log($"junction {added.GetInstanceID()} added");
            void invalidateDelegate(Junction.SwitchMode arg1, int arg2)
            {
                Debug.Log($"junction {added.GetInstanceID()} switched");
                if (info.position < bogiePositions.Values.Max()) return;// if the newly added track is behind the front bogie we don't need to watch the switch position
                trackSection.RemoveFollowing(info.prev, true);
            }
            switchlisteners.Add(added, invalidateDelegate);
            added.Switched += invalidateDelegate;
        }

        private float lastvel = 0.0f;
        private float lasttime = 0.0f;

        private bool drawDirty = true;
        private List<Lines.VectorLine> trackLines = new List<Lines.VectorLine>();
        private List<Lines.VectorLine> otherLines = new List<Lines.VectorLine>();
        public void DrawDebug()
        {
            var startPos = trackSection.Select(t => t.offset).Min();
            TrackBlock.TrackInfo? lastTrack = null; double largestOffset = double.MinValue;
            foreach (var track in trackSection)
            {
                if (track.offset > largestOffset)
                {
                    lastTrack = track;
                    largestOffset = track.offset;
                }
            }
            var distance = largestOffset + lastTrack.Value.track.curve.length - startPos;
            var minTrack = startPos;
            if (drawDirty)
            {
                Debug.Log("Draw dirty");
                trackLines.ForEach(t=>t.Remove());
                trackLines.Clear();
                foreach (var track in trackSection)
                {
                    //Debug.Log($"track {track.track.GetInstanceID()} distance {distance} wsstart {track.offset} wsend {track.offset + track.track.curve.length} start {((track.offset - startPos) / distance)} end {(track.offset + track.track.curve.length - startPos) / distance} start {((track.offset - startPos) / distance)} end {(track.offset + track.track.curve.length - startPos) / distance}");
                    var iColor = Color.Lerp(Color.red, Color.green, (float) ((track.offset - startPos) / distance));
                    var eColor = Color.Lerp(Color.red, Color.green,
                        (float) ((track.offset + track.track.curve.length - startPos) / distance));
                    var startColor = track.relativeDirection ? iColor : eColor;
                    var endColor = track.relativeDirection ? eColor : iColor;
                    trackLines.Add(Lines.RegisterLine(Lines.DrawCurve(track.track.curve, Vector3.up * 10.0f, startColor, endColor)));
                }

                drawDirty = false;
            }

            otherLines.ForEach(t => t.Remove());
            otherLines.Clear();
            foreach (var bogiePos in bogiePositions)
            {
                var bogieTrackInfo = trackSection.GetTrackInfo(bogiePos.Key.track).Value;
                var mappedPosition = (bogiePos.Value - bogieTrackInfo.offset) / bogiePos.Key.track.curve.length;
                var bogieWorldPos = bogiePos.Key.track.curve.GetPointAt(bogieTrackInfo.relativeDirection ? (float)mappedPosition : (float)(1.0 - mappedPosition));
                otherLines.Add(Lines.RegisterLine(Lines.DrawLine(bogieWorldPos, bogieWorldPos + Vector3.up * 10.0f, Color.yellow)));
            }
            foreach (var junction in trackSection.Junctions)
            {
                otherLines.Add(Lines.RegisterLine(Lines.DrawLine(junction.Key.position, junction.Key.position + Vector3.up * 10.0f, Color.red)));
            }
            Debug.Log("bogies " + String.Join(",", bogiePositions.Select(s => $"{s.Key} : {s.Value}")));
            speedProfile.DrawDebug();
        }

        private float lastSignDistance = 0.0f;
        private float maxBogiePos = 0.0f;
        public void FixedUpdate()
        {
            //update bogie positions
            bogiePositions.Clear();
            var baseBogie = boundLoco.train.Bogies[0];
            AddBogie(baseBogie);
            if (TrackDirection(baseBogie) > 0 !=
                trackSection.GetSpan((float) bogiePositions[baseBogie]).Item1.relativeDirection)
            {
                speedProfile.Clear();
                trackSection.Clear(baseBogie.track,
                    TrackDirection(baseBogie) > 0); // rebuild the track section, since we're backwards
            }

            var trainBogies = GetTrainBogies(boundLoco);
            trainBogies.ForEach(AddBogie);
            if (maxBogiePos == 0.0f && bogiePositions.Count > 0)
            {
                maxBogiePos = (float)bogiePositions.Values.Max();
            }

            //ensure that forward and backward state is maintained
            var minPosition = (float)bogiePositions.Values.Min();
            //Debug.Log("update trkconfig");
            UpdateTrailing(minPosition);
            EnsureLimit(minPosition);
            EnsureDistance(1000f);

            if (lastSignDistance > 500.0f)
            {
                speedProfile.RefreshSigns();
                lastSignDistance = 0.0f;
            }

            //update the MRSP
            speedProfile.UpdateMRSP();

            if (bogiePositions.Count > 0)
            {
                float newHead = (float) bogiePositions.Values.Max();
                float deltaDist = Mathf.Abs(maxBogiePos - newHead);
                lastSignDistance += deltaDist;
                maxBogiePos = newHead;
            }

            //DrawDebug();
        }

        // clears rear track that is behind a branch-switch (e.g. where we came in along a branch)
        private void UpdateTrailing(float minPosition)
        {
            Junction toRemove = null; TrackBlock.JunctionInfo? remInfo = null; float maxPos = float.MinValue;
            //Debug.Log($"test junctions {trackSection.Junctions.Count()} range {minPosition}");
            foreach (var junction in trackSection.Junctions)
            {
                //Debug.Log($"testing junction {junction.Key.GetInstanceID()} {junction.Value.position} {junction.Value.branch == junction.Key.defaultSelectedBranch}");
                if (junction.Value.position < minPosition && junction.Value.position > maxPos && junction.Value.prev.track != junction.Key.inBranch.track && junction.Value.branch != junction.Key.defaultSelectedBranch)
                {
                    toRemove = junction.Key;
                    remInfo = junction.Value;
                    maxPos = junction.Value.position;
                }
            }
            if (toRemove != null)
            {
                //Debug.Log($"Removing junction {toRemove.GetInstanceID()} with info {remInfo.Value.position} {remInfo.Value.branch}");
                trackSection.RemoveFollowing(remInfo.Value.next, false);
            }
        }

        public void EnsureLimit(float at)
        {
            int niters = 0;
            while (at < speedProfile.FirstLimit()) // if there's no sign controlling the rear of the train
            {
                var firstTrack = trackSection.GetFirstTrack();
                if (!firstTrack.HasValue) break; // empty list, we have other problems
                var track = firstTrack.Value;
                var newTrack = TrackBlock.TrackAlongDir(track.track, !track.relativeDirection, false);
                if (newTrack != null)
                {
                    trackSection.AddTrack(newTrack);
                }
                else
                {
                    // we've ran out of track
                   // speedProfile.AddLimit((float)track.offset + float.Epsilon, 40.0f);
                    break;
                }
                if (niters > 10) break;
                niters++;
            }
        }

        public void EnsureDistance(float dist)
        {
            int niters = 0;
            var target = dist + (float)bogiePositions.Values.Max();
            //Debug.Log($"target dist {target}");
            while (true)
            {
                var lastTrack = trackSection.GetLastTrack();
                if (!lastTrack.HasValue) break; // empty list, we have other problems
                if (lastTrack.Value.offset + lastTrack.Value.track.curve.length > target) break;
                var track = lastTrack.Value;
                var newTrack = TrackBlock.TrackAlongDir(track.track, track.relativeDirection, true);
                if (newTrack != null)
                {
                    var res = trackSection.AddTrack(newTrack);
                    if (res == null)
                    {
                        break;
                    } else
                    {
                        if (res.Value.offset + res.Value.track.curve.length > target) break;
                    }
                }
                else
                {
                    // add braking curve at end of track
                    break;
                }
                if (niters > 20) break;
                niters++;
            }
        }


        public (SpeedProfile.SpeedStateInfo? currentSSI, SpeedProfile.TargetStateInfo currentTSI) CurrentSpeedLimit(float speed)
        {
            if (bogiePositions.Count == 0 || speedProfile == null) return (null, null);
            return speedProfile.GetSpeedLimit(bogiePositions.Values.Max(), speed, 0.0f);
        }

        public List<SpeedProfile.MRSPElem> MRSPProfile(float vest)
        {
            return speedProfile.GetClientMRSP((float)bogiePositions.Values.Max(), vest);
        }
    }
}
