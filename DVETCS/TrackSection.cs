using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
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
        public enum CrossingType
        {
            Forward,
            Backward,
            Branch
        }
        private Dictionary<RailTrack, TrackInfo> tracks;
        public TrackBlock(RailTrack baseTrack, bool direction)
        {
            tracks = new Dictionary<RailTrack, TrackInfo>();
            tracks[baseTrack] = new TrackInfo(baseTrack, direction, 0.0);
        }


        private LinkedList<(RailTrack, bool, CrossingType, int?)> SearchFromtrack(RailTrack track)
        {
            HashSet<(RailTrack, bool)> seen = new HashSet<(RailTrack, bool)>();
            Queue<(RailTrack, bool, LinkedList<(RailTrack, bool, CrossingType, int?)>)> frontier = new Queue<(RailTrack, bool, LinkedList<(RailTrack, bool, CrossingType, int?)>)>();
            frontier.Enqueue((track, true, new LinkedList<(RailTrack, bool, CrossingType, int?)>()));
            frontier.Enqueue((track, false, new LinkedList<(RailTrack, bool, CrossingType, int?)>()));
            while (frontier.Count > 0)
            {
                var (ctrack, cdirection, history) = frontier.Dequeue();

                if (tracks.ContainsKey(ctrack))
                    return history;

                if (seen.Contains((ctrack, cdirection))) continue;
                seen.Add((ctrack, cdirection));

                var nextJunction = cdirection ? ctrack.outJunction : ctrack.inJunction;
                if (nextJunction == null)
                {
                    var nextBranch = cdirection ? ctrack.outBranch : ctrack.inBranch;
                    if (nextBranch == null) continue;
                    var newList = new LinkedList<(RailTrack, bool, CrossingType, int?)>(history);
                    newList.AddLast((ctrack, cdirection, CrossingType.Branch, null));
                    frontier.Enqueue((nextBranch.track, nextBranch.track.inBranch.track == ctrack, newList));
                    continue;
                }
                if (nextJunction.inBranch.track == ctrack) // in branch -> one of the out branches
                {
                    int nbranch = 0;
                    foreach (var branch in nextJunction.outBranches)
                    {
                        if (branch.track == null) continue;
                        var newList = new LinkedList<(RailTrack, bool, CrossingType, int?)>(history);
                        newList.AddLast((ctrack, cdirection, CrossingType.Forward, nbranch));
                        frontier.Enqueue((branch.track, branch.track.inJunction == nextJunction, newList));
                        nbranch++;
                    }
                }
                else // out branch -> in branch
                {
                    var next = nextJunction.inBranch;
                    if (next.track == null) continue;
                    var newList = new LinkedList<(RailTrack, bool, CrossingType, int?)>(history);
                    newList.AddLast((ctrack, cdirection, CrossingType.Backward, null));
                    frontier.Enqueue((next.track, next.track.inJunction == nextJunction, newList));
                }
            }
            throw new Exception("Invalid track configuration under train");
        }

        public TrackInfo? AddTrack(RailTrack track)
        {
            if (tracks.ContainsKey(track))
                return tracks[track];
            var path = SearchFromtrack(track);
            if (path == null) return null;

            // we know that the tracks will appear in reverse order, so we walk "back" along the path
            var lastEl = path.Last.Value;
            var junction = (lastEl.Item2 ? lastEl.Item1.outJunction : lastEl.Item1.inJunction);
            RailTrack pairedTo = null;
            if (lastEl.Item3 == CrossingType.Forward)
            {
                pairedTo = junction.outBranches[lastEl.Item4.Value].track;
            }
            else if (lastEl.Item3 == CrossingType.Backward)
            {
                pairedTo = junction.inBranch.track;
            }
            else
            {
                pairedTo = lastEl.Item2 ? lastEl.Item1.outBranch.track : lastEl.Item1.inBranch.track;
            }
            var attachInfo = tracks[pairedTo];
            var inDirection = ConnectedForward(pairedTo, lastEl.Item1);
            var extendAction = inDirection == attachInfo.relativeDirection;
            double offset = attachInfo.offset + (extendAction ? pairedTo.curve.length : 0.0);
            foreach ((RailTrack, bool, CrossingType, int?) result in path.Reverse())
            {
                tracks[result.Item1] = new TrackInfo(result.Item1, extendAction ? !result.Item2:result.Item2, extendAction ? offset : offset - result.Item1.curve.length);
                if (extendAction)
                    offset += result.Item1.curve.length;
                else
                    offset -= result.Item1.curve.length;
            }
            return tracks[path.Last.Value.Item1]; // we've just ensured that it exists
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
            } else
            {
                return nextJunction.inBranch.track;
            }
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
        
        public IEnumerator<TrackInfo> GetEnumerator()
        {
            return tracks.Values.GetEnumerator();
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return tracks.Values.GetEnumerator();
        }
    }
}
