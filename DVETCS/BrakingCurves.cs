using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using DV.Logic.Job;
using MathNet.Numerics.Interpolation;
using Unity.Jobs;
using UnityEngine;

namespace DVETCS
{
    public class BrakingCurves
    {
        /*
         * time to 1.05 bar = 10.23s
         * throttle response lag = 1s
         * brakingFactor = InverseLerp(4.0, 0.7, brakePipePressure)
         * brakingForce = brakingFactor* 2.0 * brakingForcePerBar * 8.69565192260779E-05 * bogie.rb.mass
         * totalBrakingForce = sum_bogie brakingFactor * 2.0 * brakingForcePerBar * 8.69565192260779E-05 * bogie.rb.mass
         * totalFriction = sum_bogie rollingResistanceCoefficient * (bogie.rb.mass * -Physics.gravity.y)
         * force = totalBrakingForce + totalFriction
         * mass = loco.train.trainset.cars.Select(s => s.totalMass).Aggregate(0.0f, (x,y) => x + y);
         */
        /* Curve algorithm
         * Start at target velocity, with the front of train sitting on the LOA/new MSRP.
         * Integrate backwards using RK4
         *  dx = v
         *  dv = f_net/mass
         *  f_net = f_traction + f_friction + f_slope + f_braking
         *  f_traction = 0 (EBI) [current traction force] (SBI)
         *  f_friction = rollingResistanceCoefficient * (sum_bogie bogie.rb.mass * -Physics.gravity.y)
         *  f_slope = sum_bogie cos(bogie_theta)*bogie_force
         *  bogie_theta = angle to horizontal of tangent at bogie pos
         *  bogie_force = -Physics.gravity.y * car_weight(bi/L + h/L theta) + bogie_weight using small angle approximation
         *  f_braking = [totalBrakingForce]
         */
        private struct CarBrakingInfo
        {
            public float frontAxleOffset;
            public float axleDist;
            public float cgHeight;
            public short sign;
            public float mass;
            public CarBrakingInfo(float frontAxleOffset, float axleDist, float cgHeight, short sign, float mass)
            {
                this.frontAxleOffset = frontAxleOffset; this.axleDist = axleDist; this.cgHeight = cgHeight; this.sign = sign; this.mass = mass;
            }
        }
        private struct BogieInfo
        {
            public float mass;
            public float pos;
            public BogieInfo(float mass, float pos)
            {
                this.mass = mass; this.pos = pos;
            }
        }
        private BogieInfo[] bogieOffsets;
        private CarBrakingInfo[] cars;
        private float trainMass;

        public BrakingCurves(Dictionary<Bogie, double> bogieReference)
        {
            var maxBogie = (float)bogieReference.Values.Max();
            bogieOffsets = new BogieInfo[bogieReference.Count]; int bogieIdx = 0;
            this.cars = new CarBrakingInfo[bogieReference.Count/2]; int carIdx = 0;
            var bogieMapping = new Dictionary<TrainCar, (Bogie, Bogie)>();
            foreach (var bogie in bogieReference.OrderByDescending(k=>k.Value))
            {
                var car = bogie.Key.Car;
                if (!bogieMapping.ContainsKey(car)) bogieMapping[car] = (bogie.Key, null);
                else bogieMapping[car] = (bogieMapping[car].Item1, bogie.Key);
                var (first, second) = bogieMapping[car];
          
                bogieOffsets[bogieIdx++] = new BogieInfo(bogie.Key.rb.mass, maxBogie - (float)bogie.Value);
                if (second == null)
                    continue;
                Vector3 bogieRelPos = first.transform.position - second.transform.position;
                Vector3 carAxis = (bogieRelPos).normalized;
                Vector3 rPos = first.transform.position - (car.transform.position + car.rb.centerOfMass);
                float alongAxis = Vector3.Dot(carAxis, rPos);
                float height = (rPos - alongAxis * carAxis).magnitude;
                var sign = (short)Mathf.Sign(Vector3.Dot((car.frontCoupler.transform.position - car.rearCoupler.transform.position).normalized, carAxis));
                this.cars[carIdx++] = new CarBrakingInfo(Mathf.Abs(alongAxis), bogieRelPos.magnitude, height, sign, car.rb.mass);
                trainMass += car.rb.mass + first.rb.mass + second.rb.mass;
            }
        }
        private bool GetBogiePositions(TrackBlock block, float start, Vector3[] bogiePoses, Vector3[] bogieTangents)
        {
            for (int i = 0; i < bogieOffsets.Length; i++)
            {
                var pt = start - bogieOffsets[i].pos;
                if (!block.HasPoint(pt)) return false;
                (bogiePoses[i], bogieTangents[i]) = block.CurveAt(pt);
            }
            return true;
        }
        private float GetAcceleration(TrackBlock block, float point, float brakingFactor, Vector3[] bogiePos, Vector3[] bogieTangents)
        {
            float netForce = 0.0f;
            for (int car = 0; car < cars.Length; car++)
            {
                var bogieDiff = bogiePos[car * 2] - bogiePos[car * 2 + 1];
                var bogieDir = bogieDiff.normalized;
                var b1t = bogieTangents[car * 2].normalized; var b2t = bogieTangents[car * 2 + 1].normalized;
                float carSinTh = Vector3.Dot(bogieDir, Vector3.up);
                float carCosTh = Mathf.Sqrt(Mathf.Clamp01(1 - carSinTh * carSinTh));
                var trackDir = bogieDir;
                trackDir.y = 0; trackDir.Normalize();
                float b1CosTh = cars[car].sign * Vector3.Dot(b1t, trackDir), b2CosTh = cars[car].sign * Vector3.Dot(b2t, trackDir);
                float b1SinTh = Mathf.Sqrt(Mathf.Clamp01(1 - b1CosTh * b1CosTh)), b2SinTh = Mathf.Sqrt(Mathf.Clamp01(1 - b2CosTh * b2CosTh));
                float b1TanTh = b1SinTh / b1CosTh, b2TanTh = b2SinTh / b2CosTh;

                var carObj = cars[car];
                float bbf1 = b1TanTh * cars[car].mass * -Physics.gravity.y * (carObj.frontAxleOffset * carCosTh + carObj.cgHeight * carSinTh) / carObj.axleDist;
                float bbf2 = b2TanTh * cars[car].mass * -Physics.gravity.y * ((carObj.axleDist - cars[car].frontAxleOffset) * carCosTh + carObj.cgHeight * carSinTh) / carObj.axleDist;
                float bmf1 = bogieOffsets[car * 2].mass * -Physics.gravity.y * b1SinTh, bmf2 = bogieOffsets[car * 2 + 1].mass * -Physics.gravity.y * b2SinTh;
                netForce -= bbf1 + bbf2 + bmf1 + bmf2;
                //Debug.Log($"net force 0 {netForce} {bbf1} {bbf2} {bmf1} {bmf2} {b1CosTh} {Mathf.Sqrt(1 - b1CosTh * b1CosTh)} body {1 - b1CosTh * b1CosTh}");
            }
            //Debug.Log($"net force 1 {netForce}");
            for (int bogie = 0; bogie < bogiePos.Length; bogie++)
            {
                netForce += brakingFactor * 3.0f * 10000f * 8.69565192260779E-05f * bogieOffsets[bogie].mass;
                netForce += 0.004f * bogieOffsets[bogie].mass * -Physics.gravity.y;
            }
            //Debug.Log($"net force 2 {netForce} over {trainMass}");
            return netForce / trainMass;
        }
        /// <summary>
        /// A BrakingCurve describes the velocity of a braking curve as a series of equidistant velocity-at-distance points.
        /// </summary>
        public struct BrakingCurve
        {
            private LinearSpline velocityByPositionCurve;
            private LinearSpline positionByVelocityCurve;
            public float EoA { get; }
            public float StartSpeed { get; }
            public float TargetSpeed { get; }
            public float Start { get; }
            public float End { get; }

            public BrakingCurve(float start, float end, float EoA, float startSpeed, float targetSpeed, LinearSpline velocityByPositionCurve, LinearSpline positionByVelocityCurve)
            {
                this.Start = start;
                this.End = end;
                this.EoA = EoA;
                StartSpeed = startSpeed;
                TargetSpeed = targetSpeed;
                this.velocityByPositionCurve = velocityByPositionCurve;
                this.positionByVelocityCurve = positionByVelocityCurve;
            }

            public float VelocityForPosition(float pos)
            {
                return (float)Mathf.Clamp((float)velocityByPositionCurve.Interpolate(pos), TargetSpeed, StartSpeed);
            }

            public float PositionForVelocity(float vel)
            {
                return (float) positionByVelocityCurve.Interpolate(vel);
            }
        }

        public float PredictAcceleration(TrackBlock block, float pos, float brakingFactor)
        {
            Vector3[] bogiePos = new Vector3[bogieOffsets.Length];
            Vector3[] bogieTangents = new Vector3[bogieOffsets.Length];
            GetBogiePositions(block, pos, bogiePos, bogieTangents);
            return GetAcceleration(block, pos, brakingFactor, bogiePos, bogieTangents);
        }

        public struct CurvePointBlittable
        {
            public float A;
            public float B;
            public float C;
            public float D;
        }

        public struct BrakingCurveJob : IJob
        {
            public void Execute()
            {
                throw new NotImplementedException();
            }
        }

        public BrakingCurve? CalculateBrakingCurve(TrackBlock block, float loa, float targetSpeed, float startSpeed, float brakingFactor, float finalSpeed=float.NaN)
        {
            Vector3[] bogiePos = new Vector3[bogieOffsets.Length];
            Vector3[] bogieTangents = new Vector3[bogieOffsets.Length];
            const int maxiter = 10000;
            float currentPos = loa, currentVel = targetSpeed; float dt = 0.1f; int iter = 0; float t = 0.0f, distance = 0.0f;
            List<float> velocities = new List<float>();
            List<float> positions = new List<float>();
            List<float> times = new List<float>();
            Debug.Log($"iter {currentVel < startSpeed} {bogiePos.Length} block extent {block.Min} - {block.Max}");
            while (currentVel < startSpeed && iter++ < maxiter)
            {
                if (!GetBogiePositions(block, currentPos, bogiePos, bogieTangents))
                {
                    Debug.Log($"Early termination due to out of track at {currentPos}");
                    break;
                }
                var accel = GetAcceleration(block, currentPos, brakingFactor, bogiePos, bogieTangents);
                velocities.Add(currentVel);
                positions.Add(currentPos);
                times.Add(t);
                distance += currentVel * dt;
                currentPos -= currentVel * dt;
                currentVel += accel * dt;
                t += dt;
            }

            positions.Reverse();
            velocities.Reverse();
            times.Reverse();
            if (iter < 2)
            {
                return null;
            }
            var curveStart = currentPos;
            if (float.IsNaN(finalSpeed))
            {
                Debug.Log($"[{String.Join("; ", positions.Zip(times, (p,v)=>$"{p} {v}"))}]");
                Debug.Log($"Braking Curve calculation A len {positions.Count} start speed {startSpeed} target speed {targetSpeed} currentvel {currentVel} iters {iter}");
                return new BrakingCurve(curveStart, loa, loa, startSpeed, targetSpeed,
                    LinearSpline.InterpolateSorted(positions.Select(Convert.ToDouble).ToArray(), velocities.Select(Convert.ToDouble).ToArray()),
                    LinearSpline.InterpolateSorted(velocities.Select(Convert.ToDouble).Reverse().ToArray(), positions.Select(Convert.ToDouble).Reverse().ToArray()));
            }
            List<float> bydVels = new List<float>();
            List<float> bydPos = new List<float>();
            currentPos = loa; currentVel = targetSpeed; iter = 0; t = 0.0f; distance = 0.0f;
            while (currentVel > finalSpeed && iter++ < maxiter)
            {
                if (!GetBogiePositions(block, currentPos, bogiePos, bogieTangents)) break;
                var accel = GetAcceleration(block, currentPos, brakingFactor, bogiePos, bogieTangents);
                bydVels.Add(currentVel);
                bydPos.Add(currentPos);
                distance += currentVel * dt;
                currentPos += currentVel * dt;
                currentVel -= accel * dt;
                t += dt;
            }
            positions.AddRange(bydPos);
            velocities.AddRange(bydVels);
            Debug.Log($"Braking Curve calculation B len {positions.Count}");
            return new BrakingCurve(curveStart, currentPos, loa, startSpeed, currentVel,
                LinearSpline.InterpolateSorted(positions.Select(Convert.ToDouble).ToArray(), velocities.Select(Convert.ToDouble).ToArray()),
                LinearSpline.InterpolateSorted(velocities.Select(Convert.ToDouble).Reverse().ToArray(), positions.Select(Convert.ToDouble).Reverse().ToArray()));
        }
        public float CalculateStoppingPoint(TrackBlock block, float loa, float vel, float brakingFactor)
        {
            if (brakingFactor < 0.25) return float.PositiveInfinity;
            Vector3[] bogiePos = new Vector3[bogieOffsets.Length];
            Vector3[] bogieTangents = new Vector3[bogieOffsets.Length];
            const int maxiter = 10000;
            float currentPos = loa, currentVel = vel; float dt = 0.1f; int iter = 0; float t = 0.0f;
            while (currentVel > 0.1f && iter++ < maxiter)
            {
                if (!GetBogiePositions(block, currentPos, bogiePos, bogieTangents))
                {
                    Debug.Log("Ranout of track");
                    return float.PositiveInfinity;
                }
                var accel = GetAcceleration(block, currentPos, brakingFactor, bogiePos, bogieTangents);
                Debug.Log($"iter vel {currentVel} accel {accel}");
                currentPos += currentVel * dt;
                currentVel -= accel * dt;
                t += dt;
            }
            Debug.Log($"stopping time {t} bogies {String.Join(", ", bogieOffsets.Select(bp=>bp.pos))}");
            return currentPos;
        }
    }
}
