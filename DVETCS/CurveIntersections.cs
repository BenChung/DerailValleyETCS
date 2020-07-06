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
        // from Math.NET Numerics. License:
        // <copyright file="Cubic.cs" company="Math.NET">
        // Math.NET Numerics, part of the Math.NET Project
        // http://numerics.mathdotnet.com
        // http://github.com/mathnet/mathnet-numerics
        //
        // Copyright (c) 2009-2014 Math.NET
        //
        // Permission is hereby granted, free of charge, to any person
        // obtaining a copy of this software and associated documentation
        // files (the "Software"), to deal in the Software without
        // restriction, including without limitation the rights to use,
        // copy, modify, merge, publish, distribute, sublicense, and/or sell
        // copies of the Software, and to permit persons to whom the
        // Software is furnished to do so, subject to the following
        // conditions:
        //
        // The above copyright notice and this permission notice shall be
        // included in all copies or substantial portions of the Software.
        //
        // THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
        // EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
        // OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
        // NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
        // HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
        // WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
        // FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
        // OTHER DEALINGS IN THE SOFTWARE.
        // </copyright>
        static void QR(double a2, double a1, double a0, out double Q, out double R)
        {
            Q = (3*a1 - a2*a2) / 9.0;
            R = (9.0*a2*a1 - 27*a0 - 2*a2*a2*a2) / 54.0;
        }
        static double PowThird(double n)
        {
            return Math.Pow(Math.Abs(n), 1d / 3d)*Math.Sign(n);
        }
        public static (double, double, double) RealRoots(double a0, double a1, double a2)
        {
            double Q, R;
            QR(a2, a1, a0, out Q, out R);

            var Q3 = Q*Q*Q;
            var D = Q3 + R*R;
            var shift = -a2 / 3d;

            double x1;
            double x2 = double.NaN;
            double x3 = double.NaN;

            if (D >= 0)
            {
                // when D >= 0, use eqn (54)-(56) where S and T are real
                double sqrtD = Math.Pow(D, 0.5);
                double S = PowThird(R + sqrtD);
                double T = PowThird(R - sqrtD);
                x1 = shift + (S + T);
                if (D == 0)
                {
                    x2 = shift - S;
                }
            }
            else
            {
                // 3 real roots, use eqn (70)-(73) to calculate the real roots
                double theta = Math.Acos(R / Math.Sqrt(-Q3));
                x1 = 2d*Math.Sqrt(-Q)*Math.Cos(theta / 3.0) + shift;
                x2 = 2d*Math.Sqrt(-Q)*Math.Cos((theta + 2.0*Math.PI) / 3d) + shift;
                x3 = 2d*Math.Sqrt(-Q)*Math.Cos((theta - 2.0*Math.PI) / 3d) + shift;
            }

            return (x1,x2,x3);
        }
        // end from Math.NET
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
            var (x1,x2,x3) = RealRoots(d/a, c/a, b/a);
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
