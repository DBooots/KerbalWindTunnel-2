﻿using System;
using UnityEngine;

namespace KerbalWindTunnel
{
    public abstract class AeroPredictor
    {
        public virtual bool ThreadSafe => false;
        public virtual AeroPredictor GetThreadSafeObject() => ThreadSafe ? this : throw new NotImplementedException();

        public abstract float Mass { get; }
        public abstract bool ThrustIsConstantWithAoA { get; }
        public Vector3 CoM;
        public Vector3 CoM_dry;

        public abstract float Area { get; }

        public virtual float GetMaxAoA(Conditions conditions, float tolerance = 0.0003f)
        {
            return (float)Accord.Math.Optimization.BrentSearch.Maximize((aoa) => GetLiftForceMagnitude(conditions, (float)aoa, 1), 10 * Mathf.Deg2Rad, 60 * Mathf.Deg2Rad, tolerance);
        }
        public virtual float GetMaxAoA(Conditions conditions, out float lift, float guess = float.NaN, float tolerance = 0.0003f)
        {
#if ENABLE_PROFILER
            UnityEngine.Profiling.Profiler.BeginSample("AeroPredictor.GetMaxAoA()");
#endif
            Accord.Math.Optimization.BrentSearch maximizer = new Accord.Math.Optimization.BrentSearch((aoa) => GetLiftForceMagnitude(conditions, (float)aoa, 1), 10 * Mathf.Deg2Rad, 60 * Mathf.Deg2Rad, tolerance);
            if (float.IsNaN(guess) || float.IsInfinity(guess))
                maximizer.Maximize();
            else
            {
                maximizer.LowerBound = guess - 5 * Mathf.Deg2Rad;
                maximizer.UpperBound = guess + 5 * Mathf.Deg2Rad;
                if (!maximizer.Maximize())
                {
                    maximizer.LowerBound = guess - 10 * Mathf.Deg2Rad;
                    maximizer.UpperBound = guess + 10 * Mathf.Deg2Rad;
                    if (!maximizer.Maximize())
                    {
                        maximizer.LowerBound = Math.Min(10 * Mathf.Deg2Rad, guess - 15 * Mathf.Deg2Rad);
                        maximizer.UpperBound = Mathf.Clamp(guess + 15 * Mathf.Deg2Rad, 60 * Mathf.Deg2Rad, 90 * Mathf.Deg2Rad);
                        maximizer.Maximize();
                    }
                }
            }
            lift = (float)maximizer.Value;
#if ENABLE_PROFILER
            UnityEngine.Profiling.Profiler.EndSample();
#endif
            return (float)maximizer.Solution;
        }
        public virtual float GetMinAoA(Conditions conditions, float guess = float.NaN, float tolerance = 0.0003f)
        {
            Accord.Math.Optimization.BrentSearch minimizer = new Accord.Math.Optimization.BrentSearch((aoa) => GetLiftForceMagnitude(conditions, (float)aoa, 1), -60 * Mathf.Deg2Rad, -10 * Mathf.Deg2Rad, tolerance);
            if (float.IsNaN(guess) || float.IsInfinity(guess))
                minimizer.Maximize();
            else
            {
                minimizer.LowerBound = guess - 2 * Mathf.Deg2Rad;
                minimizer.UpperBound = guess + 2 * Mathf.Deg2Rad;
                if (!minimizer.Maximize())
                {
                    minimizer.LowerBound = guess - 5 * Mathf.Deg2Rad;
                    minimizer.UpperBound = guess + 5 * Mathf.Deg2Rad;
                    if (!minimizer.Maximize())
                    {
                        minimizer.LowerBound = Mathf.Clamp(guess - 10 * Mathf.Deg2Rad, -90 * Mathf.Deg2Rad, -60 * Mathf.Deg2Rad);
                        minimizer.UpperBound = Math.Max(-10 * Mathf.Deg2Rad, guess + 10 * Mathf.Deg2Rad);
                        minimizer.Maximize();
                    }
                }
            }
            return (float)minimizer.Solution;
        }

        public virtual float GetAoA(Conditions conditions, float offsettingForce, bool useThrust = true, bool dryTorque = false, float guess = float.NaN, float pitchInputGuess = float.NaN, bool lockPitchInput = false, float tolerance = 0.0003f)
        {
#if ENABLE_PROFILER
            UnityEngine.Profiling.Profiler.BeginSample("AeroPredictor.GetAoA(Conditions, float, bool, bool, float, float, bool, float");
#endif
            if (lockPitchInput && (float.IsNaN(pitchInputGuess) || float.IsInfinity(pitchInputGuess)))
                pitchInputGuess = 0;
            Accord.Math.Optimization.BrentSearch solver;
            switch (ThrustIsConstantWithAoA)
            {
                case true when lockPitchInput:
                    Vector3 thrustForce = useThrust ? this.GetThrustForce(conditions) : Vector3.zero;
                    solver = new Accord.Math.Optimization.BrentSearch((aoa) => GetLiftForceMagnitude(this.GetLiftForce(conditions, (float)aoa, pitchInputGuess) + thrustForce, (float)aoa) - offsettingForce,
                        -10 * Mathf.Deg2Rad, 35 * Mathf.Deg2Rad, tolerance);
                    break;
                case true when !lockPitchInput:
                    thrustForce = useThrust ? this.GetThrustForce(conditions) : Vector3.zero;
                    solver = new Accord.Math.Optimization.BrentSearch((aoa) => GetLiftForceMagnitude(this.GetLiftForce(conditions, (float)aoa, GetPitchInput(conditions, (float)aoa, dryTorque, pitchInputGuess)) + thrustForce, (float)aoa)
                        - offsettingForce, -10 * Mathf.Deg2Rad, 35 * Mathf.Deg2Rad, tolerance);
                    break;
                case false when lockPitchInput:
                    solver = new Accord.Math.Optimization.BrentSearch((aoa) => GetLiftForceMagnitude(this.GetLiftForce(conditions, (float)aoa, pitchInputGuess) + this.GetThrustForce(conditions, (float)aoa), (float)aoa) - offsettingForce,
                        -10 * Mathf.Deg2Rad, 35 * Mathf.Deg2Rad, tolerance);
                    break;
                case false when !lockPitchInput:
                default:
                    solver = new Accord.Math.Optimization.BrentSearch((aoa) => GetLiftForceMagnitude(this.GetLiftForce(conditions, (float)aoa, GetPitchInput(conditions, (float)aoa, dryTorque, pitchInputGuess)) + this.GetThrustForce(conditions, (float)aoa), (float)aoa)
                        - offsettingForce, -10 * Mathf.Deg2Rad, 35 * Mathf.Deg2Rad, tolerance);
                    break;

            }

            if (float.IsNaN(guess) || float.IsInfinity(guess))
                solver.FindRoot();
            else
            {
                solver.LowerBound = guess - 2 * Mathf.Deg2Rad;
                solver.UpperBound = guess + 2 * Mathf.Deg2Rad;
                if (!solver.FindRoot())
                {
                    solver.LowerBound = guess - 5 * Mathf.Deg2Rad;
                    solver.UpperBound = guess + 5 * Mathf.Deg2Rad;
                    if (!solver.FindRoot())
                    {
                        solver.LowerBound = Math.Min(-10 * Mathf.Deg2Rad, guess - 10 * Mathf.Deg2Rad);
                        solver.UpperBound = Math.Max(35 * Mathf.Deg2Rad, guess + 10 * Mathf.Deg2Rad);
                        solver.FindRoot();
                    }
                }
            }

#if ENABLE_PROFILER
            UnityEngine.Profiling.Profiler.EndSample();
#endif
            return (float)solver.Solution;
        }

        public abstract float GetPitchInput(Conditions conditions, float AoA, bool dryTorque = false, float guess = float.NaN, float tolerance = 0.0003f);
        
        public abstract Vector3 GetAeroForce(Conditions conditions, float AoA, float pitchInput = 0);

        public virtual Vector3 GetLiftForce(Conditions conditions, float AoA, float pitchInput = 0)
        {
            return GetAeroForce(conditions, AoA, pitchInput);
        }

        public abstract Vector3 GetAeroTorque(Conditions conditions, float AoA, float pitchInput = 0, bool dryTorque = false);
        
        public virtual void GetAeroCombined(Conditions conditions, float AoA, float pitchInput, out Vector3 forces, out Vector3 torques, bool dryTorque = false)
        {
            forces = GetAeroForce(conditions, AoA, pitchInput);
            torques = GetAeroTorque(conditions, AoA, pitchInput);
        }

        public virtual float GetLiftForceMagnitude(Conditions conditions, float AoA, float pitchInput = 0)
        {
            return GetLiftForceMagnitude(GetLiftForce(conditions, AoA, pitchInput), AoA);
        }
        public static float GetLiftForceMagnitude(Vector3 force, float AoA)
        {
            return ToFlightFrame(force, AoA).y;
        }

        public virtual float GetDragForceMagnitude(Conditions conditions, float AoA, float pitchInput = 0)
        {
            return GetDragForceMagnitude(GetAeroForce(conditions, AoA, pitchInput), AoA);
        }
        public static float GetDragForceMagnitude(Vector3 force, float AoA)
        {
            return -ToFlightFrame(force, AoA).z;
        }

        public abstract Vector3 GetThrustForce(Conditions conditions, float AoA);
        public virtual Vector3 GetThrustForce(Conditions conditions) => GetThrustForce(conditions, 0);
        public virtual Vector3 GetthrustForceFlightFrame(Conditions conditions, float AoA)
        {
            return ToFlightFrame(GetThrustForce(conditions, AoA), AoA);
        }

        public virtual Vector2 GetThrustForce2D(Conditions conditions) => GetThrustForce2D(conditions, 0);
        public virtual Vector2 GetThrustForce2D(Conditions conditions, float AoA)
        {
            Vector3 thrustForce = GetThrustForce(conditions, AoA);
            return new Vector2(thrustForce.z, thrustForce.y);
        }
        public virtual Vector2 GetThrustForce2DFlightFrame(Conditions conditions, float AoA)
        {
            Vector3 thrustForce = ToFlightFrame(GetThrustForce(conditions, AoA), AoA);
            return new Vector2(thrustForce.z, thrustForce.y);
        }

        public virtual float GetFuelBurnRate(Conditions conditions) => GetFuelBurnRate(conditions, 0);
        public abstract float GetFuelBurnRate(Conditions conditions, float AoA);

        public static Vector3 ToFlightFrame(Vector3 force, float AoA)
        {
            return Quaternion.AngleAxis((AoA * Mathf.Rad2Deg), Vector3.left) * force;
        }
        public static Vector3 ToVesselFrame(Vector3 force, float AoA)
        {
            return Quaternion.AngleAxis((-AoA * Mathf.Rad2Deg), Vector3.left) * force;
        }

        public static float GetUsefulThrustMagnitude(Vector3 thrustVector)
        {
            Vector2 usefulThrust = new Vector2(Math.Max(thrustVector.z, 0), Math.Max(thrustVector.y, 0));
            if (usefulThrust.x == thrustVector.z && usefulThrust.y == thrustVector.y)
                return usefulThrust.magnitude;
            Vector2 antiThrust = new Vector2(Math.Min(thrustVector.z, 0), Math.Min(thrustVector.y, 0));
            return usefulThrust.magnitude - antiThrust.magnitude;
        }

        public static Vector3 InflowVect(float AoA)
        {
            Vector3 vesselForward = Vector3.forward;
            Vector3 vesselUp = Vector3.up;
            return vesselForward * Mathf.Cos(-AoA) + vesselUp * Mathf.Sin(-AoA);
        }

        //public abstract AeroPredictor Clone();

        public readonly struct Conditions
        {
            public readonly CelestialBody body;
            public readonly float speed;
            public readonly float altitude;
            public readonly float mach;
            public readonly float atmDensity;
            public readonly float atmPressure;
            public readonly float pseudoReDragMult;
            public readonly bool oxygenAvailable;
            public readonly float speedOfSound;
            public readonly float Q;

            public Conditions(CelestialBody body, float speed, float altitude)
            {
                this.body = body;
                this.speed = speed;
                this.altitude = altitude;
                
                lock (body)
                {
                    atmPressure = (float)body.GetPressure(altitude);
                    atmDensity = (float)Extensions.KSPClassExtensions.GetDensity(body, altitude);
                    speedOfSound = (float) body.GetSpeedOfSound(atmPressure, atmDensity);
                    oxygenAvailable = body.atmosphereContainsOxygen;
                }
                mach = speed / speedOfSound;
                
                lock (PhysicsGlobals.DragCurvePseudoReynolds)
                    pseudoReDragMult = PhysicsGlobals.DragCurvePseudoReynolds.Evaluate(atmDensity * speed);
                Q = 0.0005f * atmDensity * this.speed * this.speed;
            }
        }
    }

    public interface ILiftAoADerivativePredictor
    {
        float GetLiftForceMagnitudeAoADerivative(AeroPredictor.Conditions conditions, float AoA, float pitchInput = 0);
    }
}
