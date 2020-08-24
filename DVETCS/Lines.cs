using DV.PointSet;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TMPro;
using UnityEngine;

namespace DVETCS
{
    class Lines
    {
        public class VectorLine
        {
            private bool removed = false;
            private LineRenderer renderer;

            public VectorLine(LineRenderer renderer)
            {
                this.renderer = renderer;
            }

            public void Remove(bool keepStatic=false)
            {
                if (removed || this.renderer.gameObject == null)
                    return;
                removed = true;
                if (!keepStatic)
                    Lines.UnregisterLine(this);
                UnityEngine.Object.Destroy(this.renderer.gameObject);
            }
        }


        public static VectorLine DrawLine(Vector3 start, Vector3 end, Color color)
        {
            GameObject go = new GameObject("LineDrawer");
            var renderer = go.AddComponent<LineRenderer>();
            renderer.material = new Material(Shader.Find("Legacy Shaders/Particles/Additive"));
            renderer.useWorldSpace = true;
            renderer.SetVertexCount(2);
            renderer.SetPosition(0, start);
            renderer.SetPosition(1, end);
            renderer.SetColors(color, color);
            renderer.SetWidth(0.1f, 0.1f);
            renderer.enabled = true;
            
            return new VectorLine(renderer);
        }
        public static VectorLine DrawCurve(BezierCurve curve, Vector3 offset, Color colorStart, Color colorEnd)
        {
            GameObject go = new GameObject("LineDrawer");
            var renderer = go.AddComponent<LineRenderer>();
            renderer.material = new Material(Shader.Find("Legacy Shaders/Particles/Additive"));
            renderer.useWorldSpace = true;
            var eps = EquiPointSet.FromBezierEquidistant(curve, 0.1f);
            renderer.SetVertexCount(eps.points.Length);
            int npoint = 0;
            foreach (var point in eps.points)
            {
                var posd = point.position;
                renderer.SetPosition(npoint++, new Vector3((float)posd.x, (float)posd.y, (float)posd.z) + offset);
            }
            renderer.SetColors(colorStart, colorEnd);
            renderer.SetWidth(0.1f, 0.1f);
            renderer.enabled = true;

            return new VectorLine(renderer);
        }

        private static HashSet<Lines.VectorLine> lastLines = new HashSet<Lines.VectorLine>();
        public static VectorLine RegisterLine(VectorLine line)
        {
            lastLines.Add(line);
            return line;
        }
        private static void UnregisterLine(VectorLine vectorLine)
        {
            lastLines.Remove(vectorLine);
        }
        public static void ClearLines()
        {
            foreach (var l in lastLines)
            {
                l.Remove(true);
            }
            lastLines.Clear();
            
        }
    }
}
