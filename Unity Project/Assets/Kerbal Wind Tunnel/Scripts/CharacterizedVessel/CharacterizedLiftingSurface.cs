﻿using KerbalWindTunnel.Extensions;
using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;

namespace KerbalWindTunnel.VesselCache
{
    public class CharacterizedLiftingSurface : IDisposable
    {
        protected SimulatedLiftingSurface simulatedLiftingSurface;
        public readonly Part part;
        public FloatCurve DragCoefficientCurve_Induced { get; private set; }
        public FloatCurve DragCoefficientCurve_Parasite { get; private set; }
        public FloatCurve LiftCoefficientCurve { get; private set; }
        public FloatCurve LiftMachScalarCurve => simulatedLiftingSurface?.liftMachCurve;
        public FloatCurve DragMachScalarCurve => simulatedLiftingSurface?.dragMachCurve;
        protected readonly SortedSet<(float aoa, bool continuousDerivative)> liftAoAKeys = new SortedSet<(float aoa, bool continuousDerivative)>(CharacterizedVessel.FloatTupleComparer);
        protected readonly SortedSet<(float aoa, bool continuousDerivative)> dragAoAKeys_Induced = new SortedSet<(float aoa, bool continuousDerivative)>(CharacterizedVessel.FloatTupleComparer);
        protected readonly SortedSet<(float aoa, bool continuousDerivative)> dragAoAKeys_Parasite = new SortedSet<(float aoa, bool continuousDerivative)>(CharacterizedVessel.FloatTupleComparer);

        public bool IsValid { get => part != null; }
        private bool characterized = false;
        protected bool needsDisposing;

        protected CharacterizedLiftingSurface(Part part)
        {
            this.part = part;
        }
        public CharacterizedLiftingSurface(ModuleLiftingSurface liftingSurface, SimulatedPart part) : this(liftingSurface.part)
        {
            simulatedLiftingSurface = SimulatedLiftingSurface.Borrow(liftingSurface, part);
            needsDisposing = true;
        }
        public CharacterizedLiftingSurface(SimulatedLiftingSurface liftingSurface) : this(liftingSurface.part.part)
        {
            simulatedLiftingSurface = liftingSurface;
            needsDisposing = false;
        }

        protected virtual void Reset()
        {
            lock (this)
            {
                characterized = false;
                liftAoAKeys.Clear();
                dragAoAKeys_Induced.Clear();
                dragAoAKeys_Parasite.Clear();
            }
        }

        protected virtual void CharacterizeLift()
        {
            if (simulatedLiftingSurface.liftCurve.Curve.keys.Length > 0 || simulatedLiftingSurface.liftCurve.Curve.keys[0].value != 0)
            {
                // Every 15 degrees
                for (int i = -180; i <= 180; i += CharacterizedPart.AoASpacing)
                    liftAoAKeys.Add(CharacterizationExtensions.RoundFloatTuple((Mathf.Deg2Rad * i, true)));
                // Wherever the liftCurve has a key, in vessel AoA
                liftAoAKeys.UnionWith(
                    CharacterizationExtensions.RoundFloatTuple(
                        CharacterizationExtensions.GetWorldKeys(simulatedLiftingSurface.liftVector, simulatedLiftingSurface.liftCurve)));
                // 0 and 90 degrees local AoA, but those are likely already part of the collection.
                liftAoAKeys.UnionWith(CharacterizationExtensions.RoundFloatTuple(
                    CharacterizedPart.AxesToWorldKeys(simulatedLiftingSurface.liftVector)));
                // Apply an And to the continuous derivative component.
                CharacterizationExtensions.AndSortedSet(liftAoAKeys);

                float machMag = LiftMachScalarCurve.EvaluateThreadSafe(0), evalPt = 0;
                if (machMag == 0)
                {
                    LiftMachScalarCurve.EvaluateThreadSafe(1);
                    evalPt = 1;
                }
                float SurfLiftForce(float aoa)
                {
                    Vector3 inflow = AeroPredictor.InflowVect(aoa);
                    Vector3 lift = simulatedLiftingSurface.GetLift(inflow, evalPt) / machMag;
                    return AeroPredictor.GetLiftForceMagnitude(lift, aoa);
                }

                LiftCoefficientCurve = KSPClassExtensions.ComputeFloatCurve(liftAoAKeys, SurfLiftForce, CharacterizedVessel.toleranceF);
            }
            else
                LiftCoefficientCurve = null;
        }
        protected virtual void CharacterizeDrag()
        {
            if (simulatedLiftingSurface.dragCurve.Curve.keys.Length > 0 || simulatedLiftingSurface.dragCurve.Curve.keys[0].value != 0)
            {
                // Every 15 degrees
                for (int i = -180; i <= 180; i += CharacterizedPart.AoASpacing)
                    dragAoAKeys_Induced.Add(CharacterizationExtensions.RoundFloatTuple((Mathf.Deg2Rad * i, true)));
                // 0 and 90 degrees local AoA, but those are likely already part of the collection.
                dragAoAKeys_Induced.UnionWith(CharacterizationExtensions.RoundFloatTuple(
                    CharacterizedPart.AxesToWorldKeys(simulatedLiftingSurface.liftVector)));

                dragAoAKeys_Parasite.UnionWith(dragAoAKeys_Induced);

                // Wherever the liftCurve has a key, in vessel AoA
                dragAoAKeys_Induced.UnionWith(
                    CharacterizationExtensions.RoundFloatTuple(
                        CharacterizationExtensions.GetWorldKeys(simulatedLiftingSurface.liftVector, simulatedLiftingSurface.liftCurve)));

                // Wherever the dragCurve has a key, in vessel AoA
                dragAoAKeys_Parasite.UnionWith(
                    CharacterizationExtensions.RoundFloatTuple(
                        CharacterizationExtensions.GetWorldKeys(simulatedLiftingSurface.liftVector, simulatedLiftingSurface.dragCurve)));

                // Apply an And to the continuous derivative component.
                CharacterizationExtensions.AndSortedSet(dragAoAKeys_Induced);
                CharacterizationExtensions.AndSortedSet(dragAoAKeys_Parasite);

                // Induced Drag
                float machMag_induced = LiftMachScalarCurve.EvaluateThreadSafe(0), evalPt_induced = 0;
                if (machMag_induced == 0)
                {
                    LiftMachScalarCurve.EvaluateThreadSafe(1);
                    evalPt_induced = 1;
                }
                float SurfDragForce_Induced(float aoa)
                {
                    Vector3 inflow = AeroPredictor.InflowVect(aoa);
                    Vector3 lift = simulatedLiftingSurface.GetLift(inflow, evalPt_induced) / machMag_induced;
                    return AeroPredictor.GetDragForceMagnitude(lift, aoa);
                }

                if (!simulatedLiftingSurface.perpendicularOnly)
                    DragCoefficientCurve_Induced = KSPClassExtensions.ComputeFloatCurve(dragAoAKeys_Induced, SurfDragForce_Induced, CharacterizedVessel.toleranceF);
                else
                    DragCoefficientCurve_Induced = null;

                // Parasite Drag
                float SurfDragForce_Parasite(float aoa)
                {
                    Vector3 inflow = AeroPredictor.InflowVect(aoa);

                    float dot = Vector3.Dot(inflow, simulatedLiftingSurface.liftVector);
                    float absdot = simulatedLiftingSurface.omnidirectional ? Math.Abs(dot) : Mathf.Clamp01(dot);
                    Vector3 drag;
                    lock (simulatedLiftingSurface.dragCurve)
                        drag = -inflow * simulatedLiftingSurface.dragCurve.Evaluate(absdot) * simulatedLiftingSurface.deflectionLiftCoeff * PhysicsGlobals.LiftDragMultiplier;
                    drag *= 1000;

                    return AeroPredictor.GetDragForceMagnitude(drag, aoa);
                }

                if (simulatedLiftingSurface.useInternalDragModel)
                    DragCoefficientCurve_Parasite = KSPClassExtensions.ComputeFloatCurve(dragAoAKeys_Parasite, SurfDragForce_Parasite, CharacterizedVessel.toleranceF);
                else
                    DragCoefficientCurve_Parasite = null;
            }
            else
                LiftCoefficientCurve = null;
        }

        protected virtual void Null()
        {
            LiftCoefficientCurve = null;
            DragCoefficientCurve_Induced = null;
            DragCoefficientCurve_Parasite = null;
        }

        public void Characterize(Task _) => Characterize();
        public void Characterize()
        {
            lock (this)
            {
                if (characterized)
                    return;
                characterized = true;

                if (simulatedLiftingSurface.part.shieldedFromAirstream)
                {
                    Null();
                    return;
                }

                // Lift Curve
                CharacterizeLift();

                // Drag Curve
                CharacterizeDrag();
            }
        }

        public virtual void Dispose()
        {
            if (!needsDisposing)
                return;
            simulatedLiftingSurface?.Release();
        }
    }
}
