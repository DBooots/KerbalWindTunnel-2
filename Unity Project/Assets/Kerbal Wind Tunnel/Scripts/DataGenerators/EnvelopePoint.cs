using System;
using UnityEngine;
using KSP.Localization;
using static KerbalWindTunnel.VesselCache.AeroOptimizer;

namespace KerbalWindTunnel.DataGenerators
{
    public readonly struct EnvelopePoint
    {
        private static readonly Unity.Profiling.ProfilerMarker s_ctorMarker = new Unity.Profiling.ProfilerMarker("EnvelopePoint..ctor");
        
        public readonly float AoA_level;
        public readonly float AoA_max;
        public readonly float drag;
        public readonly float lift;
        public readonly float lift_max;
        public readonly float LDRatio;
        public readonly float dLift;

        public readonly float thrust_available;
        public readonly float thrust_required;

        public readonly float pitchInput;
        //public readonly float stabilityRange;
        //public readonly float staticMargin;
        //public readonly float dTorque;

        public readonly float fuelBurnRate;

        public readonly float altitude;
        public readonly float speed;
        public readonly float mach;
        public readonly float invDynamicPressure;

        public readonly float invMass;
        private readonly float invWingArea;

        public float Thrust_Excess { get => thrust_available - thrust_required; }
        public float Accel_Excess { get => Thrust_Excess * invMass * WindTunnelWindow.invGAccel; }
        public float EnergyHeight { get => altitude + speed * speed * 0.5f * WindTunnelWindow.invGAccel; }
        public float Power_Required { get => thrust_required * speed; }
        public float Power_Available { get => thrust_available * speed; }
        public float Power_Excess { get => Thrust_Excess * speed; }

        public float Coefficient(float force) => force * invWingArea * invDynamicPressure;
        public float Specific(float value) => value * invMass * WindTunnelWindow.invGAccel;

        public EnvelopePoint(AeroPredictor vessel, CelestialBody body, float altitude, float speed, float AoA_guess = float.NaN, float maxA_guess = float.NaN, float pitchI_guess = float.NaN)
        {
            s_ctorMarker.Begin();
            this.altitude = altitude;
            invMass = 1 / vessel.Mass;
            invWingArea = 1 / vessel.Area;
            AeroPredictor.Conditions conditions = new AeroPredictor.Conditions(body, speed, altitude);
            float gravParameter, radius;
            gravParameter = (float)body.gravParameter;
            radius = (float)body.Radius;
            float r = radius + altitude;
            mach = conditions.mach;
            invDynamicPressure = 1 / conditions.Q;
            float weight = vessel.Mass * (gravParameter / (r * r) - speed * speed / r);

            AoA_max = vessel.FindMaxAoA(conditions, out lift_max, maxA_guess);

            KeyAoAData keyAoAs = vessel.SolveLevelFlight(conditions, weight, null, new KeyAoAData() { maxLift = AoA_max, levelFlight = AoA_guess });

            AoA_level = keyAoAs.levelFlight;

            Vector2 thrustForce = vessel.GetThrustForce2D(conditions, AoA_level);
            fuelBurnRate = vessel.GetFuelBurnRate(conditions, AoA_level);

            if (AoA_level < AoA_max)
                pitchInput = vessel.FindStablePitchInput(conditions, AoA_level, guess: pitchI_guess);
            else
                pitchInput = 1;

            thrust_available = AeroPredictor.GetUsefulThrustMagnitude(thrustForce);

            //Vector3 force = vessel.GetAeroForce(conditions, AoA_level, pitchInput);
            vessel.GetAeroCombined(conditions, AoA_level, pitchInput, out Vector3 force, out Vector3 torque);
            Vector3 aeroforce = AeroPredictor.ToFlightFrame(force, AoA_level);
            drag = -aeroforce.z;
            Vector2 flightFrameThrust = AeroPredictor.ToFlightFrame(thrustForce, AoA_level);
            thrust_required = drag;
            if (keyAoAs.levelFlightResidual > 0)
            {
                if (thrust_available > 0 && flightFrameThrust.y != 0)
                    thrust_required += keyAoAs.levelFlightResidual / flightFrameThrust.y * thrust_available;
                else
                    thrust_required += keyAoAs.levelFlightResidual;
            }

            lift = aeroforce.y;
            LDRatio = Math.Abs(lift / drag);
            if (vessel is ILiftAoADerivativePredictor derivativePredictor)
                    dLift = derivativePredictor.GetLiftForceMagnitudeAoADerivative(conditions, AoA_level, pitchInput) * Mathf.Deg2Rad; // Deg2Rad = 1/Rad2Deg
                else
                    dLift = (vessel.GetLiftForceMagnitude(conditions, AoA_level + WindTunnelWindow.AoAdelta, pitchInput) - lift) /
                        (WindTunnelWindow.AoAdelta * Mathf.Rad2Deg);
            /*staticMargin = vessel.GetStaticMargin(conditions, AoA_level, pitchInput, dLift: dLift, baselineTorque: torque.x);
            dTorque = Coefficient(staticMargin * vessel.MAC * dLift);*/
            //GetStabilityValues(vessel, conditions, AoA_level, out stabilityRange);
            s_ctorMarker.End();
        }

        private static void GetStabilityValues(AeroPredictor vessel, AeroPredictor.Conditions conditions, float AoA_centre, out float stabilityRange)
        {
            const int step = 5;
            const int range = 90;
            const int alphaSteps = range / step;
            float[] torques = new float[2 * alphaSteps + 1];
            float[] aoas = new float[2 * alphaSteps + 1];
            int start, end;
            for (int i = 0; i <= 2 * alphaSteps; i++)
            {
                aoas[i] = (i - alphaSteps) * step * Mathf.Deg2Rad;
                torques[i] = vessel.GetAeroTorque(conditions, aoas[i], 0).x;
            }
            int eq = 0 + alphaSteps;
            int dir = (int)Math.Sign(torques[eq]);
            if (dir == 0)
            {
                start = eq - 1;
                end = eq + 1;
            }
            else
            {
                while (eq > 0 && eq < 2 * alphaSteps)
                {
                    eq += dir;
                    if (Math.Sign(torques[eq]) != dir)
                        break;
                }
                if (eq == 0 || eq == 2 * alphaSteps)
                {
                    stabilityRange = 0;
                    return;
                }
                if (dir < 0)
                {
                    start = eq;
                    end = eq + 1;
                }
                else
                {
                    start = eq - 1;
                    end = eq;
                }
            }
            while (torques[start] > 0 && start > 0)
                start -= 1;
            while (torques[end] < 0 && end < 2 * alphaSteps - 1)
                end += 1;
            float min = (Mathf.InverseLerp(torques[start], torques[start + 1], 0) + start) * step;
            float max = (-Mathf.InverseLerp(torques[end], torques[end - 1], 0) + end) * step;
            stabilityRange = max - min;
        }

        public override string ToString()
        {
            return string.Format($"{Localizer.Format("#autoLOC_KWT322")}:\t{{0:N0}}{Localizer.Format("#autoLOC_KWT001")}\n{Localizer.Format("#autoLOC_KWT323")}:\t{{1:N0}}{Localizer.Format("#autoLOC_KWT005")}\n{Localizer.Format("#autoLOC_KWT300")}:\t{{9:N2}}\n{Localizer.Format("#autoLOC_KWT326")}:\t{{2:N2}}{Localizer.Format("#autoLOC_KWT000")}\n" +   // "Altitude" "m" "Speed" "m/s" "Mach" "Level Flight AoA" "°"
                    $"{Localizer.Format("#autoLOC_KWT329")}:\t{{3:N0}}{Localizer.Format("#autoLOC_KWT004")}\n{Localizer.Format("#autoLOC_KWT330")}:\t{{4:N2}}{Localizer.Format("#autoLOC_KWT006")}\n{Localizer.Format("#autoLOC_KWT328")}:\t{{5:N0}}{Localizer.Format("#autoLOC_KWT004")}\n" +   // "Excess Thrust" "kN" "Excess Acceleration" "g" "Max Lift Force" "kN"
                    $"{Localizer.Format("#autoLOC_KWT327")}:\t{{6:N2}}{Localizer.Format("#autoLOC_KWT000")}\n{Localizer.Format("#autoLOC_KWT313")}:\t{{8:N2}}\n{Localizer.Format("#autoLOC_KWT331")}:\t{{7:N0}}{Localizer.Format("#autoLOC_KWT004")}",  // "Max Lift AoA" "°" "Lift/Drag Ratio" "Available Thrust" "kN"
                    altitude, speed, AoA_level * Mathf.Rad2Deg,
                    Thrust_Excess, Accel_Excess, lift_max,
                    AoA_max * Mathf.Rad2Deg, thrust_available, LDRatio,
                    mach);
        }
    }
}
