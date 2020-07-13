using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using RangeTree;
using UnityEngine;

namespace DVETCS
{
    class TrackBlock : IEnumerable<TrackBlock.TrackInfo>
    {
        public struct TrackInfo
        {
            public readonly RailTrack track;
            public readonly bool relativeDirection;
            public readonly double offset;
            public TrackInfo(RailTrack track, bool relativeDirection, double offset)
            {
                this.track = track;
                this.relativeDirection = relativeDirection;
                this.offset = offset;
            }
        }
        public struct JunctionInfo
        {
            public readonly float position;
            public readonly CrossingType crossing;
            public readonly int? branch;
            public JunctionInfo(float position, CrossingType crossing, int? branch)
            {
                this.position = position;
                this.crossing = crossing;
                this.branch = branch;
            }
        }
        public class TrackUnit
        {
            public readonly TrackInfo info;
            public Joiner next, prev;
            public TrackUnit(TrackInfo info)
            {
                this.info = info;
            }
        }
        public class Joiner
        {
            public readonly JunctionInfo info;
            public Junction junction; public Junction.Branch branch;
            public Joiner(JunctionInfo info)
            {
                this.info = info;
            }
        }
        public enum CrossingType
        {
            Forward,
            Backward,
            Branch
        }
        private Dictionary<RailTrack, TrackInfo> tracks;
        private Dictionary<Junction, JunctionInfo> junctions;
        private RangeTree<float, TrackInfo> trackRanges;
        public TrackBlock(RailTrack baseTrack, bool direction)
        {
            tracks = new Dictionary<RailTrack, TrackInfo>();
            trackRanges = new RangeTree<float, TrackInfo>();
            var trackInfo = new TrackInfo(baseTrack, direction, 0.0);
            tracks[baseTrack] = trackInfo;
            junctions = new Dictionary<Junction, JunctionInfo>();
            trackRanges.Add(0.0f, baseTrack.curve.length, trackInfo);
            TrackAdded += delegate { };
            TrackRemoved += delegate { };
            JunctionAdded += delegate { };
            JunctionRemoved += delegate { };
        }

        private (LinkedList<(RailTrack track, bool direction, CrossingType crossing, Junction junction, int? branch)>, bool, RailTrack, Junction) SearchFromtrack(RailTrack track)
        {
            HashSet<(RailTrack, bool)> seen = new HashSet<(RailTrack, bool)>();
            Queue<(RailTrack, bool, Junction, LinkedList<(RailTrack, bool, CrossingType, Junction, int?)>)> frontier = new Queue<(RailTrack, bool, Junction, LinkedList<(RailTrack, bool, CrossingType, Junction, int?)>)>();
            frontier.Enqueue((track, true, null, new LinkedList<(RailTrack, bool, CrossingType, Junction, int?)>()));
            frontier.Enqueue((track, false, null, new LinkedList<(RailTrack, bool, CrossingType, Junction, int?)>()));
            while (frontier.Count > 0)
            {
                var (ctrack, cdirection, cjunction, history) = frontier.Dequeue();

                if (tracks.ContainsKey(ctrack))
                {
                    Debug.Log($"njunct {String.Join(", ", history.Select(s=>s.Item4))} ctype {String.Join(", ", history.Select(s => s.Item3))}");
                    return (history, cdirection, ctrack, cjunction);
                }
                
                if (seen.Contains((ctrack, cdirection))) continue;
                seen.Add((ctrack, cdirection));
                
                var nextJunction = cdirection ? ctrack.outJunction : ctrack.inJunction;
                if (nextJunction == null)
                {
                    var nextBranch = cdirection ? ctrack.outBranch : ctrack.inBranch;
                    if (nextBranch == null || nextBranch.track == null) continue;
                    var newList = new LinkedList<(RailTrack, bool, CrossingType, Junction, int?)>(history);
                    newList.AddLast((ctrack, cdirection, CrossingType.Branch, cjunction, null));
                    frontier.Enqueue((nextBranch.track, nextBranch.first, null, newList));
                    continue;
                }
                if (nextJunction.inBranch.track == ctrack) // in branch -> one of the out branches
                {
                    int nbranch = 0;
                    foreach (var branch in nextJunction.outBranches)
                    {
                        if (branch.track == null) continue;
                        var newList = new LinkedList<(RailTrack, bool, CrossingType, Junction, int?)>(history);
                        newList.AddLast((ctrack, cdirection, CrossingType.Forward, nextJunction, nbranch));
                        frontier.Enqueue((branch.track, branch.first, nextJunction, newList));
                        nbranch++;
                    }
                }
                else // out branch -> in branch
                {
                    var next = nextJunction.inBranch;
                    if (next.track == null) continue;
                    var newList = new LinkedList<(RailTrack, bool, CrossingType, Junction, int?)>(history);
                    int nbranch = 0;
                    foreach (var branch in nextJunction.outBranches)
                    {
                        if (branch.track == ctrack) break;
                        nbranch++;
                    }
                    newList.AddLast((ctrack, cdirection, CrossingType.Backward, nextJunction, nbranch));
                    frontier.Enqueue((next.track, next.first, nextJunction, newList));
                }
            }
            throw new Exception("Invalid track configuration under train");
        }

        private bool GetDirectionOfSourceTrack(RailTrack source, bool outDirection, CrossingType type)
        {
            Junction.Branch branch = null;
            var junction = outDirection ? source.outJunction : source.inJunction;
            switch (type)
            {
                case CrossingType.Branch:
                    var nextDir = (outDirection ? source.outBranch : source.inBranch).first;
                    var nextTrack = (outDirection ? source.outBranch : source.inBranch).track;
                    return (nextDir ? nextTrack.inBranch : nextTrack.outBranch).first;
                case CrossingType.Backward:
                    return junction.outBranches.Where(b => b.track == source).Select(b => b.first).First();
                case CrossingType.Forward:
                    return junction.inBranch.first;
            }
            throw new Exception("Invalid junction type");
        }

        public void RemoveFrom(float position, bool removeDirection)
        {
            Debug.Log($"Removing from {position} in {removeDirection}");
            foreach (var trk in tracks)
            {
                Debug.Log($"track {trk.Key.GetInstanceID()}@{trk.Value.offset} to {trk.Value.offset + trk.Key.curve.length} remove {(removeDirection ? trk.Value.offset >= position : trk.Value.offset + trk.Key.curve.length - float.Epsilon <= position)}");
            }
            foreach (var jn in junctions)
            {
                Debug.Log($"junction {jn.Key.GetInstanceID()} at {jn.Value.position} remove {(removeDirection ? jn.Value.position >= position + float.Epsilon : jn.Value.position - float.Epsilon <= position)}");
            }
            var rmTracks = tracks.Where(t => removeDirection ? t.Value.offset >= position : t.Value.offset + t.Key.curve.length - float.Epsilon <= position).ToArray();
            var rmJunctions = junctions.Where(j => removeDirection ? j.Value.position >= position + float.Epsilon : j.Value.position - float.Epsilon <= position).ToArray();
            foreach (var track in rmTracks)
            {
                tracks.Remove(track.Key);
                trackRanges.Remove(track.Value);
                TrackRemoved.Invoke(this, track.Value);
            }
            foreach (var junction in rmJunctions)
            {
                junctions.Remove(junction.Key);
                JunctionRemoved.Invoke(this, junction.Key, junction.Value);
            }
        }

        public void RemoveJunction(Junction junction, bool removeDirection)
        {
            if (!junctions.ContainsKey(junction)) return;
            RemoveFrom(junctions[junction].position, removeDirection);
        }

        public void RemoveTrack(RailTrack track, bool removeDirection)
        {
            if (!tracks.ContainsKey(track)) return; // track does not exist in the current block
            RemoveFrom((float)tracks[track].offset + (removeDirection ? 0.0f : (float)track.curve.length), removeDirection);
        }

        public TrackInfo? AddTrack(RailTrack track)
        {
            return AddTrack(track, false);
        }
        public TrackInfo? AddTrack(RailTrack track, bool removeExisting)
        {
            if (tracks.ContainsKey(track))
                return tracks[track];
            var (path, idir, incident, ijunction) = SearchFromtrack(track);
            Debug.Log($"adding track {track.GetInstanceID()} ijunction {(ijunction ? ijunction.GetInstanceID() : -1 )}");
            if (path == null) return null;

            // we know that the tracks will appear in reverse order, so we walk "back" along the path
            var lastEl = path.Last.Value;
            var junction = (lastEl.Item2 ? lastEl.Item1.outJunction : lastEl.Item1.inJunction);
            RailTrack pairedTo = incident;

            var attachInfo = tracks[pairedTo];
            var extendAction = idir != attachInfo.relativeDirection;
            double offset = attachInfo.offset + (extendAction ? pairedTo.curve.length : 0.0);

            var connectTo = pairedTo;
            if (removeExisting && ijunction != null && junctions.ContainsKey(ijunction))
            {
                JunctionRemoved.Invoke(this, ijunction, junctions[ijunction]);
                junctions.Remove(ijunction);
            }
            foreach ((RailTrack track, bool direction, CrossingType crossing, Junction junction, int? branch) result in path.Reverse())
            {
                var relDir = !GetDirectionOfSourceTrack(result.track, result.direction, result.crossing);
                var computedOffset = extendAction ? offset : offset - result.track.curve.length;
                Debug.Log($"EA {result.track.GetInstanceID()} {relDir != extendAction} {computedOffset} -> {computedOffset + result.track.curve.length} junction: {result.junction}");
                var ti = new TrackInfo(result.track, relDir != extendAction, computedOffset);

                if (removeExisting)
                {
                    var toRemove = trackRanges.Query((float)computedOffset + (extendAction?float.Epsilon:0.0f), (float)computedOffset + result.track.curve.length - (extendAction ? 0.0f : float.Epsilon));
                    if (toRemove.Count() > 0)
                    {
                        Debug.Log($"rem cands {String.Join(", ", toRemove.Select(tr=>$"{tr.offset} -> {tr.offset + tr.track.curve.length}"))} for {computedOffset} to {computedOffset + result.track.curve.length}");
                        var removeExtents = toRemove.Select(r => extendAction ? r.offset : r.offset + r.track.curve.length);
                        var removeFrom = extendAction ? removeExtents.Min() : removeExtents.Max();
                        Debug.Log("Remove action");
                        RemoveFrom((float)removeFrom + (extendAction?1.0f:-1.0f), extendAction);
                    }
                }

                Debug.Log($"extend 2 {ti.track.GetInstanceID()}");

                tracks[result.track] = ti;
                trackRanges.Add((float)ti.offset, (float)ti.offset + result.track.curve.length, ti);
                if (result.junction != null)
                {
                    var junctionPos = (float)(offset);
                    var ji = new JunctionInfo(junctionPos, result.crossing, result.branch);
                    if (junctions.ContainsKey(result.junction))
                        Debug.Log($"re-adding junction {result.junction.GetInstanceID()} at {junctions[result.junction].position} w/ track {result.track.GetInstanceID()}");
                    else
                        Debug.Log($"adding junction {result.junction.GetInstanceID()}");
                    junctions.Add(result.junction, ji);
                    JunctionAdded.Invoke(this, result.junction, ji);
                }
                Debug.Log($"extend 3");
                TrackAdded.Invoke(this, ti, extendAction, result.junction);
                pairedTo = result.track;
                if (extendAction)
                    offset += result.track.curve.length;
                else
                    offset -= result.track.curve.length;
            }
            Debug.Log($"extend 4 id {path.Last.Value.track.GetInstanceID()}");
            return tracks[path.Last.Value.track]; // we've just ensured that it exists
        }
        

        public TrackInfo? GetLastTrack()
        {
            float maxOffset = float.MinValue;
            TrackInfo? output = null;
            foreach (var ti in tracks)
            {
                var extent = ti.Value.offset + ti.Value.track.curve.length;
                if (extent > maxOffset)
                {
                    maxOffset = (float)extent;
                    output = ti.Value;
                }
            }
            return output;
        }

        public static bool ConnectedForward(RailTrack from, RailTrack to)
        {
            return from.outBranch != null ? from.outBranch.track == to : (from.outJunction != null ? from.outJunction == to.inJunction || from.outJunction == to.outJunction : false);
        }

        public TrackInfo? RearmostTrack()
        {
            var minOffset = double.MaxValue;
            TrackInfo? found = null;
            foreach (var track in tracks)
                if (track.Value.offset < minOffset)
                {
                    found = track.Value;
                    minOffset = track.Value.offset;
                }
            return found;
        }

        public static RailTrack TrackAlongDir(RailTrack start, bool relativeDirection, bool directionOfTravel)
        {
            Junction nextJunction = relativeDirection ? start.outJunction : start.inJunction;
            Junction.Branch nextBranch = relativeDirection ? start.outBranch : start.inBranch;
            if (nextJunction == null && nextBranch == null)
                return null;
            if (nextJunction == null && nextBranch != null)
                return nextBranch.track;
            if (start == nextJunction.inBranch.track) // coming in along the inBranch
            {
                if (directionOfTravel)
                    return nextJunction.outBranches[nextJunction.selectedBranch].track;
                else
                    return nextJunction.outBranches[nextJunction.defaultSelectedBranch].track;
            }
            else
            {
                return nextJunction.inBranch.track;
            }
        }

        public static Junction JunctionAlongDir(RailTrack start, bool relativeDirection)
        {
            return relativeDirection ? start.outJunction : start.inJunction;
        }

        public TrackInfo? GetTrackInfo(RailTrack track)
        {
            return tracks[track];
        }
        public TrackInfo? GetTrackInfo(float distanceAlong)
        {
            var search = tracks.Where(track => track.Value.offset < distanceAlong && track.Value.offset + track.Value.track.curve.length > distanceAlong);
            if (search.Count() == 0) return null;
            return search.First().Value;
        }
        public TrackInfo? GetFirstTrack()
        {
            float minOffset = float.MaxValue;
            TrackInfo? output = null;
            foreach (var ti in tracks)
            {
                if (ti.Value.offset < minOffset)
                {
                    minOffset = (float)ti.Value.offset;
                    output = ti.Value;
                }
            }
            return output;
        }
        
        public bool HasPoint(float along)
        {
            return along < trackRanges.Max && along > trackRanges.Min; 
        }
        public (TrackInfo, float) GetSpan(float along)
        {
            var tracks = trackRanges.Query(along);
            var track = tracks.First();
            var span = (along - track.offset) / track.track.curve.length;
            if (!track.relativeDirection) span = 1 - span;
            return (track, (float)span);
        }
        public (Vector3 pos,Vector3 tang) CurveAt(float along)
        {
            var (track, span) = GetSpan(along);
            return (pos: track.track.curve.GetPointAt(span), tang: track.track.curve.GetTangentAt(span));
        }

        public delegate void TrackAddedHandler(TrackBlock block, TrackInfo added, bool sideAdded, Junction via);
        public event TrackAddedHandler TrackAdded;

        public delegate void TrackRemovedHandler(TrackBlock block, TrackInfo removed);
        public event TrackRemovedHandler TrackRemoved;

        public delegate void JunctionAddedHandler(TrackBlock block, Junction added, JunctionInfo info);
        public event JunctionAddedHandler JunctionAdded;

        public delegate void JunctionRemovedHandler(TrackBlock block, Junction removed, JunctionInfo info);
        public event JunctionRemovedHandler JunctionRemoved;

        public IEnumerator<TrackInfo> GetEnumerator()
        {
            return tracks.Values.GetEnumerator();
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return tracks.Values.GetEnumerator();
        }

        public IEnumerable<KeyValuePair<Junction,JunctionInfo>> Junctions
        {
            get
            {
                return junctions.AsEnumerable();
            }
        }
    }
}
