using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using KerbalWindTunnel.Extensions;

namespace KerbalWindTunnel.VesselCache
{
    public class CharacterizedPart : IDisposable
    {
        public readonly SimulatedPart simulatedPart;
        public readonly Part part;
        public FloatCurve2 DragCoefficientCurve { get; private set; }
        public FloatCurve LiftCoefficientCurve { get; private set; }
        public FloatCurve LiftMachScalarCurve { get => part?.DragCubes.BodyLiftCurve.liftMachCurve; }

        private readonly SortedSet<(float aoa, bool continuousDerivative)> partAoAKeys = new SortedSet<(float aoa, bool continuousDerivative)>(CharacterizedVessel.FloatTupleComparer);
        private readonly SortedSet<(float mach, bool continuousDerivative)> dragMachKeys = new SortedSet<(float mach, bool continuousDerivative)>(CharacterizedVessel.FloatTupleComparer);

        public IReadOnlyCollection<(float aoa, bool continuousDerivative)> AoAKeys => partAoAKeys;
        public IReadOnlyCollection<(float mach, bool continuousDerivative)> MachKeys => dragMachKeys;
        public bool IsValid { get => part != null; }
        public bool NoBodyLift { get => simulatedPart.NoBodyLift; }

        private bool characterized = false;
        private readonly bool needsDisposing;

        public const int AoASpacing = 15;

        private static readonly FloatCurve2.DiffSettings settings = new FloatCurve2.DiffSettings(1E-6f, CharacterizedVessel.toleranceF, true);

        public CharacterizedPart(Part part, SimulatedVessel vessel)
        {
            this.part = part;
            simulatedPart = SimulatedPart.Borrow(part, vessel);
            needsDisposing = true;
        }
        public CharacterizedPart(SimulatedPart part)
        {
            this.part = part.part;
            simulatedPart = part;
            needsDisposing = false;
        }

        private void Reset()
        {
            lock (this)
            {
                characterized = false;
                partAoAKeys.Clear();
                dragMachKeys.Clear();
            }
        }

        public void Characterize()
        {
            lock (this)
            {
                if (characterized)
                    return;
                characterized = true;

                // Every 15 degrees
                for (int i = -180; i <= 180; i += AoASpacing)
                    partAoAKeys.Add(CharacterizationExtensions.RoundFloatTuple((Mathf.Deg2Rad * i, true)));
                // Wherever any of the part axes has a minima or maxima dot product with the primary, unrotated, axis.
                partAoAKeys.UnionWith(
                    CharacterizationExtensions.RoundFloatTuple(GetPartAxes(simulatedPart).SelectMany(AxesToWorldKeys)));
                // Apply an And to the continuous derivative component.
                CharacterizationExtensions.AndSortedSet(partAoAKeys);

                if (!simulatedPart.noDrag && !simulatedPart.shieldedFromAirstream)
                {
                    dragMachKeys.UnionWith(GetDragMachSet(simulatedPart.cubes));

                    DragCoefficientCurve = FloatCurve2.ComputeCurve(dragMachKeys, partAoAKeys, PartDragForce, settings);
                }
                else
                    DragCoefficientCurve = null;

                if (!simulatedPart.NoBodyLift)
                {
                    float machMag = LiftMachScalarCurve.EvaluateThreadSafe(0), evalPt = 0;
                    if (machMag == 0)
                    {
                        LiftMachScalarCurve.EvaluateThreadSafe(1);
                        evalPt = 1;
                    }
                    if (machMag == 0)
                        Debug.LogError($"[KWT][CharacterizedPart.Characterize] For part {part.name}, LiftMachCurve evaluated to zero at both M=0 and M=1, resulting in undefined part lift. Please report this error.");
                    float PartLiftForce(float aoa)
                    {
                        Vector3 lift = Vector3.zero;
                        Vector3 inflow = AeroPredictor.InflowVect(aoa);
                        lift = simulatedPart.GetLift(inflow, evalPt);
                        return AeroPredictor.GetLiftForceComponent(lift, aoa) / machMag;
                    }

                    LiftCoefficientCurve = FloatCurveExtensions.ComputeFloatCurve(partAoAKeys, PartLiftForce, CharacterizedVessel.toleranceF);
                }
                else
                    LiftCoefficientCurve = null;
            }
        }

        private float PartDragForce(float mach, float aoa)
        {
            Vector3 inflow = AeroPredictor.InflowVect(aoa);
            Vector3 drag = simulatedPart.GetAero(inflow, mach, 1);
            return AeroPredictor.GetDragForceComponent(drag, aoa);
        }

        public static IEnumerable<Vector3> GetPartAxes(SimulatedPart part)
        {
            Quaternion partToVessel = part.partToVessel;
            yield return partToVessel * Vector3.forward;
            yield return partToVessel * Vector3.right;
            yield return partToVessel * Vector3.up;
        }
        public static IEnumerable<(float, bool)> AxesToWorldKeys(Vector3 v)
        {
#if OUTSIDE_UNITY
            static
#endif
            IEnumerable<float> ZeroOne()
            {
                yield return 0;
                yield return 1;
            }
            foreach (var v_ in CharacterizationExtensions.GetWorldKeys(v, ZeroOne()))
                yield return (v_, false);
        }

        public static IEnumerable<(float mach, bool continuousDerivative)> GetDragMachSet(DragCubeList cubes)
        {
            var surfCurves = cubes.SurfaceCurves;
            IEnumerable<Keyframe> machs =
                surfCurves.dragCurveTip.Curve.keys.Concat(
                surfCurves.dragCurveSurface.Curve.keys).Concat(
                surfCurves.dragCurveTail.Curve.keys).Concat(
                cubes.DragCurveMultiplier.Curve.keys).Concat(
                cubes.DragCurveCdPower.Curve.keys);

            if (!FloatCurveComparer.Instance.Equals(surfCurves.dragCurveMultiplier, cubes.DragCurveMultiplier))
                machs = machs.Concat(surfCurves.dragCurveMultiplier.Curve.keys);

            return machs.GroupBy(KeyframeTime).Select(TangentsEqual);

            (float, bool) TangentsEqual(IGrouping<float, Keyframe> group)
            {
                foreach (var k in group)
                {
                    if (k.outTangent != k.inTangent)
                        return (group.Key, false);
                }
                return (group.Key, true);
            }
            float KeyframeTime(Keyframe keyframe) => keyframe.time;
        }

        public void Dispose()
        {
            if (!needsDisposing)
                return;
            simulatedPart?.Release();
        }
    }
}
