using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace DVETCS
{
    public static class CurveUtils
    {
        private static (double, double?, double?) IntersectCubic(Vector3 normal, Vector3 offset, Vector3 P0, Vector3 P1, Vector3 P2, Vector3 P3)
        {
            double n1 = normal.x, n2 = normal.y, n3 = normal.z;
            double r1 = offset.x, r2 = offset.y, r3 = offset.z;
            double P01 = P0.x, P02 = P0.y, P03 = P0.z;
            double P11 = P1.x, P12 = P1.y, P13 = P1.z;
            double P21 = P2.x, P22 = P2.y, P23 = P2.z;
            double P31 = P3.x, P32 = P3.y, P33 = P3.z;
            
            double a = -(n1 * P01) - n2 * P02 - n3 * P03 + 3 * n1 * P11 + 3 * n2 * P12 + 3 * n3 * P13 - 3 * n1 * P21 - 3 * n2 * P22 - 3 * n3 * P23 + n1 * P31 + n2 * P32 + n3 * P33;
            double b = (3*n1*P01 + 3*n2*P02 + 3*n3*P03 - 6*n1*P11 - 6*n2*P12 - 6*n3*P13 + 3*n1*P21 + 3*n2*P22 + 3*n3*P23);
            double c = (-3*n1*P01 - 3*n2*P02 - 3*n3*P03 + 3*n1*P11 + 3*n2*P12 + 3*n3*P13);
            double d = (n1*P01 + n2*P02 + n3*P03 - n1*r1 - n2*r2 - n3*r3);
            var (x1,x2,x3) = MathNet.Numerics.RootFinding.Cubic.RealRoots(d/a, c/a, b/a);
            return (x1, Double.IsNaN(x2) ? (double?)null : x2, Double.IsNaN(x2) ? (double?)null : x3);
        }

        private static (double, double)? IntersectQuadratic(Vector3 normal, Vector3 offset, Vector3 P0, Vector3 P1, Vector3 P2)
        {
            double n1 = normal.x, n2 = normal.y, n3 = normal.z;
            double r1 = offset.x, r2 = offset.y, r3 = offset.z;
            double P01 = P0.x, P02 = P0.y, P03 = P0.z;
            double P11 = P1.x, P12 = P1.y, P13 = P1.z;
            double P21 = P2.x, P22 = P2.y, P23 = P2.z;
            double a = n1*P01 + n2*P02 + n3*P03 - 2*n1*P11 - 2*n2*P12 - 2*n3*P13 + n1*P21 + n2*P22 + n3*P23;
            double b = -2*n1*P01 - 2*n2*P02 - 2*n3*P03 + 2*n1*P11 + 2*n2*P12 + 2*n3*P13;
            double c = n1*P01 + n2*P02 + n3*P03 - n1*r1 - n2*r2 - n3*r3;
            double inner = Math.Pow(b, 2) - 4*a*c;
            if (inner < 0) return null;
            if (2*a < Double.Epsilon) return null;
            return ((-b + Math.Sqrt(inner)) / (2*a), (-b - Math.Sqrt(inner)) / (2*a));
        }

        private static double? IntersectLinear(Vector3 normal, Vector3 offset, Vector3 P0, Vector3 P1)
        {
            double n1 = normal.x, n2 = normal.y, n3 = normal.z;
            double r1 = offset.x, r2 = offset.y, r3 = offset.z;
            double P01 = P0.x, P02 = P0.y, P03 = P0.z;
            double P11 = P1.x, P12 = P1.y, P13 = P1.z;
            double a = -(n1*P01) - n2*P02 - n3*P03 + n1*P11 + n2*P12 + n3*P13;
            double b = n1*P01 + n2*P02 + n3*P03 - n1*r1 - n2*r2 - n3*r3;
            if (a < Double.Epsilon) return null;
            return -b / a;
             
        }
        public static List<(BezierPoint p1, BezierPoint p2, double span)> PlaneIntersect(this BezierCurve curve, Vector3 normal, Vector3 offset)
        {
            var outp = new List<(BezierPoint p1, BezierPoint p2, double span)>();
            for (int i = 0; i < curve.pointCount - 1; i++)
            {
                var p1 = curve[i];
                var p2 = curve[i + 1];

                Vector3 p1h = p1.position + p1.handle2;
                Vector3 p2h = p2.position + p2.handle1;
                if (p1.handle2 != Vector3.zero)
                {
                    if (p2.handle1 != Vector3.zero)
                    {
                        var (x,y,z) = IntersectCubic(normal, offset, p1.position, p1h, p2h, p2.position);
                        if (0.0 <= x && x <= 1.0) outp.Add((p1, p2, x));
                        if (y.HasValue && 0.0 <= y && y <= 1.0) outp.Add((p1: p1, p2: p2, span: (double)y));
                        if (z.HasValue && 0.0 <= z && z <= 1.0) outp.Add((p1, p2, (double)z));
                        continue;
                    }
                    var solns = IntersectQuadratic(normal, offset, p1.position, p1h, p2.position);
                    if (!solns.HasValue) continue; var (w, b) = solns.Value;
                    if (0.0 <= w && w <= 1.0) outp.Add((p1, p2, w));
                    if (0.0 <= b && b <= 1.0) outp.Add((p1, p2, b));
                    continue;
                }
                if (p2.handle1 != Vector3.zero)
                {
                    var solns = IntersectQuadratic(normal, offset, p1.position, p2h, p2.position);
                    if (!solns.HasValue) continue; var (x, y) = solns.Value;
                    if (0.0 <= x && x <= 1.0) outp.Add((p1, p2, x));
                    if (0.0 <= y && y <= 1.0) outp.Add((p1, p2, y));
                    continue;
                }
                var soln = IntersectLinear(normal, offset, p1.position, p2.position);
                if (!soln.HasValue) continue; var a = soln.Value;
                if (0.0 <= a && a <= 1.0) outp.Add((p1, p2, a));
            }
            return outp;
        }
        public static Dictionary<BezierPoint, float> PointSpans(this BezierCurve curve)
        {
            Dictionary<BezierPoint, float> output = new Dictionary<BezierPoint, float>();
            float currentSpan = 0.0f;
            for (int i = 0; i < curve.pointCount - 1; i++)
            {
                var p1 = curve[i];
                var p2 = curve[i + 1];
                output[p1] = currentSpan;
                currentSpan += BezierCurve.ApproximateLength(p1, p2, 0.25f);
            }
            output[curve[curve.pointCount - 1]] = currentSpan;
            return output;
        }
    }
}
