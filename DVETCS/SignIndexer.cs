/**
Copyright 2020 Miles Spielberg

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject
to the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 */
using System.Collections.Generic;
using System.Linq;
using DV.PointSet;
using DV.Signs;
using HarmonyLib;
using UnityEngine;

namespace DVETCS
{
    interface ISign
    {
        double Span { get; }
        bool Direction { get; }
    }
    class NormalSign : ISign
    {
        public int Limit { get; }
        public bool Direction { get; }
        public double Span { get; }

        public NormalSign(int limit, bool direction, double span)
        {
            Limit = limit;
            Direction = direction;
            Span = span;
        }
    }

    class JunctionSign : ISign
    {
        public int[] Limits { get; }
        public bool Direction { get; }
        public double Span { get; }

        public JunctionSign(int[] limits, bool direction, double span)
        {
            Limits = limits;
            Direction = direction;
            Span = span;
        }
    }

    static class TrackIndexer
    {
        const int SIGN_COLLIDER_LAYER = 30;
        const float SIMPLIFIED_RESOLUTION = 10f;

        static Dictionary<RailTrack, List<ISign>> indexedTracks =
            new Dictionary<RailTrack, List<ISign>>();

        public static List<ISign> GetSignData(RailTrack track)
        {
            List<ISign> data;
            if (!indexedTracks.TryGetValue(track, out data))
                data = indexedTracks[track] = LookForSigns(track.GetPointSet()).ToList();
            return data;
        }

        private static IEnumerable<ISign> ParseSign(string colliderName, bool direction, double span)
        {
            string[] parts = colliderName.Split('\n');
            switch (parts.Length)
            {
                case 1:
                    yield return new NormalSign(int.Parse(parts[0]) * 10, direction, span);
                    break;
                case 2:
                    yield return new JunctionSign(new[] {int.Parse(parts[0]) * 10, int.Parse(parts[1]) * 10}, direction, span);
                    break;
            }
        }

        static IEnumerable<ISign> LookForSigns(EquiPointSet pointSet)
        {
            EquiPointSet simplified = EquiPointSet.ResampleEquidistant(
                pointSet,
                Mathf.Min(SIMPLIFIED_RESOLUTION, (float)pointSet.span / 2));

            foreach (var point in simplified.points)
            {
                var hits = Physics.RaycastAll(
                    new Ray((Vector3)point.position + WorldMover.currentMove, point.forward),
                    (float)point.spanToNextPoint,
                    1 << SIGN_COLLIDER_LAYER);

                foreach (var hit in hits)
                {
                    foreach (var data in ParseSign(hit.collider.name, Vector3.Dot(hit.collider.transform.forward, point.forward) < 0f, point.span + hit.distance))
                        yield return data;
                }
            }
        }

        [HarmonyPatch(typeof(Streamer), nameof(Streamer.AddSceneGO))]
        static class AddSceneGOPatch
        {
            static void Postfix(GameObject sceneGO)
            {
                foreach (var signDebug in sceneGO.GetComponentsInChildren<SignDebug>())
                {
                    signDebug.gameObject.layer = SIGN_COLLIDER_LAYER;
                    var collider = signDebug.gameObject.AddComponent<SphereCollider>();
                    collider.name = signDebug.text;
                    collider.center = new Vector3(2f, 0f, 0f);
                    collider.radius = 1f;
                }
            }
        }
    }
}
