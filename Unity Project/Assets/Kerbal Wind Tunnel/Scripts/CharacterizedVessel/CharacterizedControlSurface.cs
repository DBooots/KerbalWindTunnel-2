using System;
using System.Collections.Generic;
using UnityEngine;
using KerbalWindTunnel.Extensions;

namespace KerbalWindTunnel.VesselCache
{
    public class CharacterizedControlSurface : CharacterizedLiftingSurface
    {
        private const float dragCubeDeltaDeflection = 1;
        private const float surfDeltaDeflection = 0.1f;
        private static readonly FloatCurve2.DiffSettings settings = new FloatCurve2.DiffSettings(1E-6f, CharacterizedVessel.toleranceF, true);
        private readonly SimulatedControlSurface controlSurface;
        public readonly CharacterizedPart basePart;
        private readonly bool isAheadOfCoM;
        public FloatCurve2 DeltaDragCoefficientCurve_Pos { get; private set; }
        public FloatCurve2 DeltaDragCoefficientCurve_Neg { get; private set; }

        public FloatCurve DeltaLiftCoefficientCurve { get; private set; }
        public FloatCurve DeltaDragCoefficientCurve_Induced_Pos { get; private set; }
        public FloatCurve DeltaDragCoefficientCurve_Parasite_Pos { get; private set; }
        public FloatCurve DeltaDragCoefficientCurve_Induced_Neg { get; private set; }
        public FloatCurve DeltaDragCoefficientCurve_Parasite_Neg { get; private set; }

        public FloatCurve DeltaTorqueCurve_Lift { get; private set; }
        public FloatCurve DeltaTorqueCurve_Drag_Pos { get; private set; }
        public FloatCurve DeltaTorqueCurve_Drag_Neg { get; private set; }

        public FloatCurve2 DeltaTorqueCurve_Pos { get; private set; }
        public FloatCurve2 DeltaTorqueCurve_Neg { get; private set; }

        private readonly SortedSet<(float aoa, bool continuousDerivative)> partAoAKeys = new SortedSet<(float aoa, bool continuousDerivative)>(CharacterizedVessel.FloatTupleComparer);
        private readonly SortedSet<(float mach, bool continuousDerivative)> partMachKeys = new SortedSet<(float mach, bool continuousDerivative)>(CharacterizedVessel.FloatTupleComparer);

        public CharacterizedControlSurface(ModuleControlSurface controlSurface, CharacterizedPart part) : base(controlSurface.part)
        {
            this.controlSurface = SimulatedControlSurface.Borrow(controlSurface, part.simulatedPart);
            simulatedLiftingSurface = this.controlSurface;
            needsDisposing = true;
            basePart = part;
            isAheadOfCoM = part.simulatedPart.transformPosition.z > part.simulatedPart.vessel.CoM.z;
        }
        public CharacterizedControlSurface(SimulatedControlSurface liftingSurface, CharacterizedPart part) : base(liftingSurface)
        {
            controlSurface = liftingSurface;
            basePart = part;
            isAheadOfCoM = part.simulatedPart.transformPosition.z > part.simulatedPart.vessel.CoM.z;
        }

        protected override void CharacterizeLift()
        {
            base.CharacterizeLift();

            if (LiftCoefficientCurve == null)
            {
                DeltaLiftCoefficientCurve = null;
                return;
            }

            float machMag, evalPt;
            try
            {
                (machMag, evalPt) = GetSafeEvalPt(LiftMachScalarCurve);
            }
            catch (InvalidOperationException)
            {
                DeltaLiftCoefficientCurve = null;
                return;
            }
            float SurfLiftForce(float aoa)
            {
                Vector3 inflow = AeroPredictor.InflowVect(aoa);
                Vector3 lift = controlSurface.GetLift(inflow, evalPt, surfDeltaDeflection) / machMag;
                return AeroPredictor.GetDragForceComponent(lift, aoa);
            }
            DeltaLiftCoefficientCurve = FloatCurveExtensions.ComputeFloatCurve(dragAoAKeys_Induced, SurfLiftForce, CharacterizedVessel.toleranceF);
            DeltaLiftCoefficientCurve = FloatCurveExtensions.Subtract(LiftCoefficientCurve, DeltaLiftCoefficientCurve);
            DeltaLiftCoefficientCurve.Scale(1 / surfDeltaDeflection);
        }

        protected override void CharacterizeDrag()
        {
            base.CharacterizeDrag();

            partAoAKeys.UnionWith(basePart.AoAKeys);
            partMachKeys.UnionWith(basePart.MachKeys);

            if (!basePart.simulatedPart.noDrag && !basePart.simulatedPart.shieldedFromAirstream)
            {

                DeltaDragCoefficientCurve_Pos = FloatCurve2.ComputeCurve(partMachKeys, partAoAKeys, PartDragForcePos, settings);
                DeltaDragCoefficientCurve_Neg = FloatCurve2.ComputeCurve(partMachKeys, partAoAKeys, PartDragForceNeg, settings);

                DeltaDragCoefficientCurve_Pos = FloatCurve2.Subtract(basePart.DragCoefficientCurve, DeltaDragCoefficientCurve_Pos);
                DeltaDragCoefficientCurve_Neg = FloatCurve2.Subtract(basePart.DragCoefficientCurve, DeltaDragCoefficientCurve_Neg);

                if (dragCubeDeltaDeflection != 1)
                {
#pragma warning disable CS0162 // Unreachable code detected
                    float invDragCubeDeltaDeflection = 1 / dragCubeDeltaDeflection;
                    DeltaDragCoefficientCurve_Pos = DeltaDragCoefficientCurve_Pos.ScaledBy(invDragCubeDeltaDeflection);
                    DeltaDragCoefficientCurve_Neg = DeltaDragCoefficientCurve_Neg.ScaledBy(-invDragCubeDeltaDeflection);
#pragma warning restore CS0162 // Unreachable code detected
                }
            }
            else
            {
                DeltaDragCoefficientCurve_Pos = null;
                DeltaDragCoefficientCurve_Neg = null;
            }

            if (DragCoefficientCurve_Parasite != null)
            {
                DeltaDragCoefficientCurve_Parasite_Pos = FloatCurveExtensions.ComputeFloatCurve(dragAoAKeys_Parasite, SurfDragForce_Parasite_Deflected_Pos, CharacterizedVessel.toleranceF);
                DeltaDragCoefficientCurve_Parasite_Pos = FloatCurveExtensions.Subtract(DragCoefficientCurve_Parasite, DeltaDragCoefficientCurve_Parasite_Pos);
                DeltaDragCoefficientCurve_Parasite_Pos.Scale(1 / surfDeltaDeflection);
                DeltaDragCoefficientCurve_Parasite_Neg = FloatCurveExtensions.ComputeFloatCurve(dragAoAKeys_Parasite, SurfDragForce_Parasite_Deflected_Pos, CharacterizedVessel.toleranceF);
                DeltaDragCoefficientCurve_Parasite_Neg = FloatCurveExtensions.Subtract(DragCoefficientCurve_Parasite, DeltaDragCoefficientCurve_Parasite_Neg);
                DeltaDragCoefficientCurve_Parasite_Neg.Scale(-1 / surfDeltaDeflection);
            }
            else
            {
                DeltaDragCoefficientCurve_Parasite_Pos = null;
                DeltaDragCoefficientCurve_Parasite_Neg = null;
            }

            if (DragCoefficientCurve_Induced != null)
            {
                float machMag, evalPt;
                try
                {
                    (machMag, evalPt) = GetSafeEvalPt(LiftMachScalarCurve);
                }
                catch (InvalidOperationException)
                {
                    DeltaDragCoefficientCurve_Induced_Pos = null;
                    return;
                }
                float SurfDragForce_Induced(float aoa, float pitchInput)
                {
                    Vector3 inflow = AeroPredictor.InflowVect(aoa);
                    Vector3 lift = controlSurface.GetLift(inflow, evalPt, pitchInput) / machMag;
                    return AeroPredictor.GetDragForceComponent(lift, aoa);
                }
                float SurfDragForce_Induced_Pos(float aoa)
                    => SurfDragForce_Induced(aoa, surfDeltaDeflection);
                float SurfDragForce_Induced_Neg(float aoa)
                    => SurfDragForce_Induced(aoa, -surfDeltaDeflection);
                DeltaDragCoefficientCurve_Induced_Pos = FloatCurveExtensions.ComputeFloatCurve(dragAoAKeys_Induced, SurfDragForce_Induced_Pos, CharacterizedVessel.toleranceF);
                DeltaDragCoefficientCurve_Induced_Pos = FloatCurveExtensions.Subtract(DragCoefficientCurve_Induced, DeltaDragCoefficientCurve_Induced_Pos);
                DeltaDragCoefficientCurve_Induced_Pos.Scale(1 / surfDeltaDeflection);
                DeltaDragCoefficientCurve_Induced_Neg = FloatCurveExtensions.ComputeFloatCurve(dragAoAKeys_Induced, SurfDragForce_Induced_Neg, CharacterizedVessel.toleranceF);
                DeltaDragCoefficientCurve_Induced_Neg = FloatCurveExtensions.Subtract(DragCoefficientCurve_Induced, DeltaDragCoefficientCurve_Induced_Neg);
                DeltaDragCoefficientCurve_Induced_Neg.Scale(-1 / surfDeltaDeflection);
            }
            else
            {
                DeltaDragCoefficientCurve_Induced_Pos = null;
                DeltaDragCoefficientCurve_Induced_Neg = null;
            }
        }

        protected override void CharacterizeTorque()
        {
            base.CharacterizeTorque();

            FloatCurve[] liftTorqueCurves =
            {
                FloatCurveExtensions.ScaledBy(DeltaLiftCoefficientCurve, k => AeroPredictor.ToFlightFrame(simulatedLiftingSurface.part.CoL, k.time).z),
                FloatCurveExtensions.ScaledBy(DeltaDragCoefficientCurve_Induced_Pos, k => -AeroPredictor.ToFlightFrame(simulatedLiftingSurface.part.CoL, k.time).y)
            };
            DeltaTorqueCurve_Lift = FloatCurveExtensions.Superposition(liftTorqueCurves);
            DeltaTorqueCurve_Drag_Pos = FloatCurveExtensions.ScaledBy(DeltaDragCoefficientCurve_Parasite_Pos, k => -AeroPredictor.ToFlightFrame(simulatedLiftingSurface.part.CoP, k.time).y);
            DeltaTorqueCurve_Drag_Neg = FloatCurveExtensions.ScaledBy(DeltaDragCoefficientCurve_Parasite_Neg, k => -AeroPredictor.ToFlightFrame(simulatedLiftingSurface.part.CoP, k.time).y);

            DeltaTorqueCurve_Pos = DeltaDragCoefficientCurve_Pos.ScaledBy(k => -AeroPredictor.ToFlightFrame(simulatedLiftingSurface.part.CoP, k.timeY).y);
            DeltaTorqueCurve_Neg = DeltaDragCoefficientCurve_Neg.ScaledBy(k => -AeroPredictor.ToFlightFrame(simulatedLiftingSurface.part.CoP, k.timeY).y);
        }
        protected float SurfDragForce_Parasite_Deflected_Pos(float aoa)
            => SurfDragForce_Parasite_Deflected(aoa, surfDeltaDeflection);
        protected float SurfDragForce_Parasite_Deflected_Neg(float aoa)
            => SurfDragForce_Parasite_Deflected(aoa, -surfDeltaDeflection);
        protected float SurfDragForce_Parasite_Deflected(float aoa, float pitchInput)
        {
            float surfaceInput = controlSurface.GetSurfaceInput(pitchInput, isAheadOfCoM);
            return SurfDragForce_Parasite_Internal(aoa, surfaceInput);
        }
        protected override float SurfDragForce_Parasite(float aoa)
        {
            if (!controlSurface.deployed)
                return base.SurfDragForce_Parasite(aoa);

            float surfaceInput = controlSurface.deployAngle * controlSurface.deploymentDirection;
            return SurfDragForce_Parasite_Internal(aoa, surfaceInput);
        }
        protected float SurfDragForce_Parasite_Internal(float aoa, float surfaceInput)
        {
            Vector3 relLiftVector = Quaternion.AngleAxis(surfaceInput, controlSurface.rotationAxis) * controlSurface.liftVector;

            Vector3 inflow = AeroPredictor.InflowVect(aoa);

            float dot = Vector3.Dot(inflow, relLiftVector);
            float absdot = controlSurface.omnidirectional ? Math.Abs(dot) : Mathf.Clamp01(dot);
            Vector3 drag;
            lock (controlSurface.dragCurve)
                drag = -inflow * controlSurface.dragCurve.Evaluate(absdot) * controlSurface.deflectionLiftCoeff * PhysicsGlobals.LiftDragMultiplier;
            drag *= 1000;

            return AeroPredictor.GetDragForceComponent(drag, aoa);
        }

        private float PartDragForcePos(float mach, float aoa) => PartDragForce(mach, aoa, dragCubeDeltaDeflection);
        private float PartDragForceNeg(float mach, float aoa) => PartDragForce(mach, aoa, -dragCubeDeltaDeflection);
        private float PartDragForce(float mach, float aoa, float deflection)
        {
            Vector3 inflow = AeroPredictor.InflowVect(aoa);
            Vector3 drag = controlSurface.GetPartDrag(inflow, mach, deflection, 1);
            return AeroPredictor.GetDragForceComponent(drag, aoa);
        }

        protected override void Null()
        {
            base.Null();

            DeltaLiftCoefficientCurve = null;
            DeltaDragCoefficientCurve_Induced_Pos = null;
            DeltaDragCoefficientCurve_Parasite_Pos = null;
            DeltaDragCoefficientCurve_Induced_Neg = null;
            DeltaDragCoefficientCurve_Parasite_Neg = null;

            DeltaDragCoefficientCurve_Pos = null;
            DeltaDragCoefficientCurve_Neg = null;

            DeltaTorqueCurve_Lift = null;
            DeltaTorqueCurve_Drag_Pos = null;
            DeltaTorqueCurve_Drag_Neg = null;

            DeltaTorqueCurve_Pos = null;
            DeltaTorqueCurve_Neg = null;
        }
    }
}
