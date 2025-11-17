using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using UnityEngine;
using KSP.Localization;
using KerbalWindTunnel.Extensions;
using static KerbalWindTunnel.VesselCache.AeroOptimizer;

namespace KerbalWindTunnel.VesselCache
{
    public class CharacterizedVessel : AeroPredictor, ILiftAoADerivativePredictor, IDirectAoAMaxProvider, IDisposable
    {
        static readonly Unity.Profiling.ProfilerMarker s_getLiftMarker = new Unity.Profiling.ProfilerMarker("CharacterizedVessel.GetLiftForce");
        static readonly Unity.Profiling.ProfilerMarker s_getLiftMagMarker = new Unity.Profiling.ProfilerMarker("CharacterizedVessel.GetLiftForceMagnitude");
        static readonly Unity.Profiling.ProfilerMarker s_getDragMagMarker = new Unity.Profiling.ProfilerMarker("CharacterizedVessel.GetDragForceMagnitude");
        static readonly Unity.Profiling.ProfilerMarker s_evalLiftMarker = new Unity.Profiling.ProfilerMarker("CharacterizedVessel.EvaluateLiftCurve");
        static readonly Unity.Profiling.ProfilerMarker s_evalDragMarker = new Unity.Profiling.ProfilerMarker("CharacterizedVessel.EvaluateDragCurve");

        public readonly SimulatedVessel vessel;

        public override AeroPredictor GetThreadSafeObject() => this;

        public override float Mass => vessel.Mass;

        public override bool ThrustIsConstantWithAoA => vessel.ThrustIsConstantWithAoA;

        public override float Area => vessel.Area;
        public override float MAC => vessel.MAC;

        public bool DirectAoAInitialized => maxAoATaskHandle?.IsCompleted ?? false;

        public const int tolerance = 5;
        public const float toleranceF = 2E-5f;

        internal readonly List<(FloatCurve machCurve, FloatCurve liftCurve)> surfaceLift = new List<(FloatCurve machCurve, FloatCurve liftCurve)>();
        internal readonly List<(FloatCurve machCurve, FloatCurve liftCurve)> surfaceDragI = new List<(FloatCurve machCurve, FloatCurve liftCurve)>();
        internal readonly List<(FloatCurve machCurve, FloatCurve liftCurve)> surfaceDragP = new List<(FloatCurve machCurve, FloatCurve liftCurve)>();
        internal readonly List<(FloatCurve machCurve, FloatCurve liftCurve)> bodyLift = new List<(FloatCurve machCurve, FloatCurve liftCurve)>();
        internal FloatCurve2 bodyDrag;
        internal FloatCurve2 ctrlDeltaDragPos;
        internal FloatCurve2 ctrlDeltaDragNeg;

        public FloatCurve AoAMax { get; private set; }

        private readonly List<CharacterizedPart> parts = new List<CharacterizedPart>();
        private readonly List<CharacterizedLiftingSurface> surfaces = new List<CharacterizedLiftingSurface>();
        private readonly List<CharacterizedControlSurface> controls = new List<CharacterizedControlSurface>();
        private readonly List<PartCollection> partCollections = new List<PartCollection>();

        private Task taskHandle = null;
        private Task maxAoATaskHandle = null;

        public static readonly Comparer<(float, bool)> FloatTupleComparer = Comparer<(float, bool)>.Create((x, y) =>
        {
            int result = x.Item1.CompareTo(y.Item1);
            return result != 0 ? result : y.Item2.CompareTo(x.Item2);   // Order of x and y reversed here so that a True value comes before a False value
        });

        public CharacterizedVessel(SimulatedVessel vessel)
        {
            this.vessel = vessel;

            /*  How to characterize a vessel:
             *  Lift: 
             *  - Group parts by their lifting surface LiftMach curve if their lift curve is non-zero
             *      - Map their Lift curve local AoA to vessel AoA according to their lift vector
             *      - Treat parts as three lifting surfaces with lift vectors along their primary axes
             *      
             *      L = f(M) * CL(AoA) * q
             *  
             *  - For each group, make a FloatCurve of lift coefficient using the union of those
             *    AoA keys within each group.
             *    
             *  
             *  Drag:
             *  - Group lifting surfaces by their DragMach curve if their drag curve is non-zero
             *      - Map their Drag curve local AoA to vessel AoA according to their lift vector
             *      
             *      D = f(M) * CD(AoA) * q
             *  
             *  - For each group, make a FloatCurve of drag coefficient using the union of those
             *    AoA keys within each group.
             *  - Repeat using the LiftMachCurve for those surfaces where PerpendicularOnly is not true.
             *  
             *  - Find the union of all Mach keys for every draggy part's Tip, Tail, Surface, Multiplier, and Power.
             *  - Find the vessel AoA equating to 0, 90, 180, and 270 (-90) local AoA using each part's primary axes.
             *      - Round each to the nearest 1E-8. Math.Round(value,8).
             *  - Make a FloatCurve2 of part drag coefficient for those AoAs and -180:15:180, and Machs.
             *  
             *      D = f(M, AoA) * f(q)
             */

            /*  Roadmap:
             *  
             *  Control surfaces can incorporate deflection through a FloatCurve2
             *      L = f(M) * f(AoA, defL) * f(q)
             *      D = f(M) * f(AoA, defL) * f(q)
             * 
             *  Rotor Part Collections cannot be characterized as the velocity and AoA of each part
             *  is a function of the vessel's velocity and the part's local velocity.
             *  They will be treated as non-rotating.
             *
             */


            AddCharacterizedComponents(vessel.partCollection);

            void AddCharacterizedComponents(PartCollection collection)
            {
                parts.AddRange(collection.parts.Select(p => new CharacterizedPart(p)));
                surfaces.AddRange(collection.surfaces.Select(s => new CharacterizedLiftingSurface(s)));
                foreach (SimulatedControlSurface ctrl in collection.ctrls)
                {
                    CharacterizedPart ctrlPart = new CharacterizedPart(ctrl.part);
                    // Controls wrap their part so they can change its drag cubes with deflection.
                    // We're messing with that, so we can include it.
                    parts.Add(ctrlPart);
                    controls.Add(new CharacterizedControlSurface(ctrl, ctrlPart));
                }
                foreach (PartCollection child in collection.partCollections)
                {
                    if (child is RotorPartCollection rotorCollection && rotorCollection.isRotating)
                        // Risky since parentCollection may get released. Currently fine since parentCollection is unused.
                        partCollections.Add(PartCollection.BorrowClone(rotorCollection, rotorCollection.parentCollection));
                    else // If the collection isn't rotating, recursively add it as regular parts.
                        AddCharacterizedComponents(child);
                }
            }

            Characterize();
        }

        public void Reset()
        {
            lock (this)
            {
                taskHandle = null;
                parts.Clear();
                surfaces.Clear();
                controls.Clear();
                surfaceLift.Clear();
                surfaceDragI.Clear();
                surfaceDragP.Clear();
                bodyLift.Clear();
                bodyDrag = null;
            }
        }

        public void WaitUntilCharacterized()
        {
            if (taskHandle == null)
                Characterize();
            taskHandle.Wait(new TimeSpan(0, 1, 0));
        }

        public void Characterize()
        {
            lock (this)
            {
                if (taskHandle != null)
                    return;
                if (maxAoATaskHandle != null)
                    return;
#if DEBUG
                Debug.Log("[KWT] Starting Characterization");
#endif

                List<Task> tasks = new List<Task>();
                foreach (CharacterizedPart part in parts)
                    tasks.Add(Task.Run(part.Characterize));
                Task partTask = Task.WhenAll(tasks);
                Task combinePartTask = partTask.ContinueWith(CombineParts);

                tasks.Clear();
                foreach (CharacterizedLiftingSurface surface in surfaces)
                    tasks.Add(Task.Run(surface.Characterize));
                foreach (CharacterizedControlSurface control in controls)
                    tasks.Add(partTask.ContinueWith(control.Characterize)); // Have to wait for parts to finish first.
                Task surfaceTask = Task.WhenAll(tasks).ContinueWith(CombineSurfaces);

                taskHandle = Task.WhenAll(combinePartTask, surfaceTask);
                maxAoATaskHandle = taskHandle.ContinueWith(CharacterizeMaxAoA);
#if DEBUG
                taskHandle.ContinueWith(t => Debug.Log("[KWT] Completed Characterization"));
#endif
            }
        }

        private static bool FloatCurveIsValid(FloatCurve curve)
        {
            if (curve == null) return false;
            if (curve.IsAlwaysZero()) return false;
            return true;
        }
        private void CombineParts(Task _)
        {
            // Offloading one set of superposition work.
            Task<FloatCurve2> dragSuperposition = Task.Run(() => FloatCurve2.Superposition(parts.Select(p => p.DragCoefficientCurve)));

            foreach (var bodyGroup in parts.Where(p => FloatCurveIsValid(p.LiftMachScalarCurve) && FloatCurveIsValid(p.LiftCoefficientCurve)).GroupBy(p => p.LiftMachScalarCurve, FloatCurveComparer.Instance))
                bodyLift.Add((bodyGroup.Key.Clone(), KSPClassExtensions.Superposition(bodyGroup.Select(s => s.LiftCoefficientCurve))));

            bodyDrag = dragSuperposition.Result;
        }
        private void CombineSurfaces(Task _)
        {
            Task<FloatCurve2> posSuperposition = Task.Run(() => FloatCurve2.Superposition(controls.Select(c => c.DeltaDragCoefficientCurve_Pos)));
            Task<FloatCurve2> negSuperposition = Task.Run(() => FloatCurve2.Superposition(controls.Select(c => c.DeltaDragCoefficientCurve_Neg)));

            IEnumerable <CharacterizedLiftingSurface> allSurfaces = surfaces.Concat(controls);
            foreach (var surfGroup in allSurfaces.Where(s => FloatCurveIsValid(s.LiftMachScalarCurve) && FloatCurveIsValid(s.LiftCoefficientCurve)).GroupBy(s => s.LiftMachScalarCurve, FloatCurveComparer.Instance))
                surfaceLift.Add((surfGroup.Key.Clone(), KSPClassExtensions.Superposition(surfGroup.Select(s => s.LiftCoefficientCurve))));
            foreach (var surfGroup in allSurfaces.Where(s => FloatCurveIsValid(s.LiftMachScalarCurve) && FloatCurveIsValid(s.DragCoefficientCurve_Induced)).GroupBy(s => s.LiftMachScalarCurve, FloatCurveComparer.Instance))
                surfaceDragI.Add((surfGroup.Key.Clone(), KSPClassExtensions.Superposition(surfGroup.Select(s => s.DragCoefficientCurve_Induced))));
            foreach (var surfGroup in allSurfaces.Where(s => FloatCurveIsValid(s.DragMachScalarCurve) && FloatCurveIsValid(s.DragCoefficientCurve_Parasite)).GroupBy(s => s.DragMachScalarCurve, FloatCurveComparer.Instance))
                surfaceDragP.Add((surfGroup.Key.Clone(), KSPClassExtensions.Superposition(surfGroup.Select(s => s.DragCoefficientCurve_Parasite))));

            ctrlDeltaDragPos = posSuperposition.Result;
            ctrlDeltaDragNeg = negSuperposition.Result;
        }

        private void CharacterizeMaxAoA(Task _)
        {
            // Since L = f(M) * f(AoA) * q, the peak with respect to AoA becomes only a function of M.
            // This lets us precompute the max AoA for a given Mach.
            // This should even be a FloatCurve since the liftMachCurves that shift the weighting between parts and surfaces
            // is itself a FloatCurve.
            const float machStep = 0.1f;

            SortedSet<float> machKeys = new SortedSet<float>();
            foreach (var (liftMachCurve, _) in surfaceLift)
                machKeys.UnionWith(liftMachCurve.ExtractTimes());
            foreach (var (liftMachCurve, _) in bodyLift)
                machKeys.UnionWith(liftMachCurve.ExtractTimes());

            Conditions baseConditions = new Conditions(WindTunnelWindow.Instance.CelestialBody ?? Planetarium.fetch.Home, 0, 0);

            float GetAoAMax(float mach)
            {
                Conditions conditions = new Conditions(baseConditions.body, baseConditions.speedOfSound * mach, 0);
                return this.FindMaxAoA(conditions, out float lift, 30 * Mathf.Deg2Rad); // TODO: Use some heuristic based on the keys to have a good guess.
            }

            AoAMax = KSPClassExtensions.ComputeFloatCurve(machKeys, GetAoAMax, machStep);
        }

        public void Dispose()
        {
            foreach (CharacterizedPart part in parts)
                part.Dispose();
            parts.Clear();

            foreach (CharacterizedLiftingSurface surf in surfaces)
                surf.Dispose();
            surfaces.Clear();

            foreach (PartCollection collection in partCollections)
                collection.Release();
        }

        public override Vector3 GetAeroForce(Conditions conditions, float AoA, float pitchInput = 0)
            => ToVesselFrame(-GetDragForceMagnitude(conditions, AoA, pitchInput) * Vector3.forward, AoA) +
            GetLiftForce(conditions, AoA, pitchInput);

        public float EvaluateDragCurve(Conditions conditions, float AoA, float pitchInput)
        {
            WaitUntilCharacterized();

            s_evalDragMarker.Begin();
            float magnitude = 0;

            foreach (var (liftMachCurve, liftCurve) in surfaceDragI)
            {
                float groupMagnitude;
                groupMagnitude = liftCurve.EvaluateThreadSafe(AoA);
                groupMagnitude *= liftMachCurve.EvaluateThreadSafe(conditions.mach);
                magnitude += groupMagnitude;
            }
            // These lists could be combined. But separate is easier for debugging.
            foreach (var (dragMachCurve, dragCurve) in surfaceDragP)
            {
                float groupMagnitude;
                groupMagnitude = dragCurve.EvaluateThreadSafe(AoA);
                groupMagnitude *= dragMachCurve.EvaluateThreadSafe(conditions.mach);
                magnitude += groupMagnitude;
            }

            float bodyDrag = this.bodyDrag.Evaluate(conditions.mach, AoA);

            if (pitchInput < 0)
            {
                bodyDrag += ctrlDeltaDragNeg.Evaluate(conditions.mach, AoA) * -pitchInput;
            }
            else if (pitchInput > 0)
            {
                bodyDrag += ctrlDeltaDragPos.Evaluate(conditions.mach, AoA) * pitchInput;
            }
            magnitude += bodyDrag * conditions.pseudoReDragMult;

            float Q = 0.0005f * conditions.atmDensity * conditions.speed * conditions.speed;
            s_evalDragMarker.End();
            return magnitude * Q;
        }

        public override float GetDragForceMagnitude(Conditions conditions, float AoA, float pitchInput = 0)
        {
            s_getDragMagMarker.Begin();
            float magnitude = EvaluateDragCurve(conditions, AoA, pitchInput);

            foreach (PartCollection collection in partCollections)
                lock (collection)
                    magnitude += GetDragForceComponent(collection.GetAeroForce(InflowVect(AoA) * conditions.speed, conditions, pitchInput, out _, Vector3.zero), AoA);

            s_getDragMagMarker.End();
            return magnitude;
        }

        public float EvaluateLiftCurve(Conditions conditions, float AoA, float pitchInput = 0)
        {
            WaitUntilCharacterized();

            s_evalLiftMarker.Begin();
            float magnitude = 0;

            foreach (var (liftMachCurve, liftCurve) in surfaceLift)
            {
                float groupMagnitude;
                groupMagnitude = liftCurve.EvaluateThreadSafe(AoA);
                groupMagnitude *= liftMachCurve.EvaluateThreadSafe(conditions.mach);
                magnitude += groupMagnitude;
            }
            foreach (var (machCurve, liftCurve) in bodyLift)
            {
                float groupMagnitude;
                groupMagnitude = liftCurve.EvaluateThreadSafe(AoA);
                groupMagnitude *= machCurve.EvaluateThreadSafe(conditions.mach);
                magnitude += groupMagnitude;
            }

            s_evalLiftMarker.End();
            return magnitude * conditions.Q;
        }

        public override float GetLiftForceMagnitude(Conditions conditions, float AoA, float pitchInput = 0)
        {
            s_getLiftMagMarker.Begin();
            float magnitude = EvaluateLiftCurve(conditions, AoA, pitchInput);

            foreach (PartCollection collection in partCollections)
                lock (collection)
                    magnitude += GetLiftForceComponent(collection.GetLiftForce(InflowVect(AoA) * conditions.speed, conditions, pitchInput, out _, Vector3.zero), AoA);

            s_getLiftMagMarker.End();
            return magnitude;
        }

        public override void GetAeroCombined(Conditions conditions, float AoA, float pitchInput, out Vector3 forces, out Vector3 torques, bool dryTorque = false)
        {
            torques = GetAeroTorque(conditions, AoA, pitchInput, dryTorque);
            forces = GetAeroForce(conditions, AoA, pitchInput);
        }

        public override Vector3 GetLiftForce(Conditions conditions, float AoA, float pitchInput = 0)
        {
            s_getLiftMarker.Begin();
            Vector3 result = ToVesselFrame(GetLiftForceMagnitude(conditions, AoA, pitchInput) * Vector3.up, AoA);
            s_getLiftMarker.End();
            return result;
        }

        public override Vector3 GetAeroTorque(Conditions conditions, float AoA, float pitchInput = 0, bool dryTorque = false)
            => vessel.GetAeroTorque(conditions, AoA, pitchInput, dryTorque);

        public override Vector3 GetThrustForce(Conditions conditions, float AoA)
            => vessel.GetThrustForce(conditions, AoA);

        public override float GetFuelBurnRate(Conditions conditions, float AoA)
            => vessel.GetFuelBurnRate(conditions, AoA);

        public override Func<double, double> LevelFlightObjectiveFunc(Conditions conditions, float offsettingForce, float pitchInput = 0)
        {
            if (ThrustIsConstantWithAoA)
            {
                Vector3 thrustForce = GetThrustForce(conditions);
                double LevelFlightObjectiveFuncInternal_ConstantThrust(double aoa) =>
                    GetLiftForceMagnitude(conditions, (float)aoa, pitchInput) + GetLiftForceComponent(thrustForce, (float)aoa) - offsettingForce;
                return LevelFlightObjectiveFuncInternal_ConstantThrust;
            }
            else
            {
                double LevelFlightObjectiveFuncInternal(double aoa) =>
                    GetLiftForceMagnitude(conditions, (float)aoa, pitchInput) + GetLiftForceComponent(GetThrustForce(conditions, (float)aoa), (float)aoa) - offsettingForce;
                return LevelFlightObjectiveFuncInternal;
            }
        }
        public override Func<double, double> LevelFlightObjectiveFunc(Conditions conditions, float offsettingForce, Func<float, float> pitchInput)
        {
            if (ThrustIsConstantWithAoA)
            {
                Vector3 thrustForce = GetThrustForce(conditions);
                double LevelFlightObjectiveFuncInternal_ConstantThrust(double aoa) =>
                    GetLiftForceMagnitude(conditions, (float)aoa, pitchInput((float)aoa)) + GetLiftForceComponent(thrustForce, (float)aoa) - offsettingForce;
                return LevelFlightObjectiveFuncInternal_ConstantThrust;
            }
            else
            {
                double LevelFlightObjectiveFuncInternal(double aoa) =>
                    GetLiftForceMagnitude(conditions, (float)aoa, pitchInput((float)aoa)) + GetLiftForceComponent(GetThrustForce(conditions, (float)aoa), (float)aoa) - offsettingForce;
                return LevelFlightObjectiveFuncInternal;
            }
        }

        public float GetLiftForceMagnitudeAoADerivative(Conditions conditions, float AoA, float pitchInput = 0)
        {
            WaitUntilCharacterized();

            float magnitude = 0;

            foreach (var (machCurve, liftCurve) in surfaceLift)
            {
                float machValue;
                machValue = machCurve.EvaluateThreadSafe(conditions.mach);
                magnitude += liftCurve.EvaluateDerivative(AoA) * machValue;
            }
            foreach (var (machCurve, liftCurve) in bodyLift)
            {
                float machValue;
                machValue = machCurve.EvaluateThreadSafe(conditions.mach);
                magnitude += liftCurve.EvaluateDerivative(AoA) * machValue;
            }

            float Q = 0.0005f * conditions.atmDensity * conditions.speed * conditions.speed;
            magnitude *= Q;

            foreach (PartCollection collection in partCollections)
            {
                const float delta = 1E-8f;
                const float invDelta = 1 / (2 * delta);
                float fplus, fminus;
                lock (collection)
                {
                    fplus = GetLiftForceComponent(collection.GetLiftForce(InflowVect(AoA + delta) * conditions.speed, conditions, pitchInput, out _, Vector3.zero), AoA + delta);
                    fminus = GetLiftForceComponent(collection.GetLiftForce(InflowVect(AoA - delta) * conditions.speed, conditions, pitchInput, out _, Vector3.zero), AoA - delta);
                }
                magnitude += (fplus - fminus) * invDelta;
            }

            return magnitude;
        }

        public void WaitUntilAoACharacterized()
        {
            if (maxAoATaskHandle == null)
                Characterize();
            maxAoATaskHandle.Wait(new TimeSpan(0, 1, 0));
        }

        public float GetAoAMax(Conditions conditions)
        {
            WaitUntilAoACharacterized();
            return AoAMax.EvaluateThreadSafe(conditions.mach);
        }

        protected override System.Data.DataSet WriteToDataSet()
        {
            System.Data.DataSet data = new System.Data.DataSet();

            bodyDrag.WriteToDataSet(data, $"{Localizer.Format("#autoLOC_KWT350")}_");   // "bodyDrag"
            //TODO: ctrlDeltaDragNeg.WriteToDataSet(data, "ctrlNeg_");
            //TODO: ctrlDeltaDragPos.WriteToDataSet(data, "ctrlPos_");

#if OUTSIDE_UNITY
            static
#endif
            void WriteCurveSet(System.Data.DataSet ds, List<(FloatCurve machCurve, FloatCurve forceCurve)> curveSet, string name)
            {
                bool multiple = curveSet.Count > 1;
                int setIndex = 0;
                foreach (var (machCurve, forceCurve) in curveSet)
                {
                    string localName = multiple ? $"{name}{setIndex}" : name;
                    System.Data.DataTable curveTable = machCurve.WriteToDataTable();
                    curveTable.TableName = string.Join("_", localName, Localizer.Format("#autoLOC_KWT349"));    // "MFactor"
                    curveTable.Columns[0].ColumnName = Graphing.Graphable.FormatNameAndUnit(Localizer.Format("#autoLOC_KWT301"), "");   // "Mach Number"
                    ds.Tables.Add(curveTable);
                    curveTable = forceCurve.WriteToDataTable();
                    curveTable.TableName = string.Join("_", localName, Localizer.Format("#autoLOC_KWT305"));    // "Coef"
                    foreach (System.Data.DataRow row in curveTable.Rows)
                        row[0] = (float)row[0] * Mathf.Rad2Deg;
                    curveTable.Columns[0].ColumnName = Graphing.Graphable.FormatNameAndUnit(Localizer.Format("#autoLOC_KWT302"), Localizer.Format("#autoLOC_KWT000"));  // "Angle of Attack" "°"
                    ds.Tables.Add(curveTable);
                }
            }

            WriteCurveSet(data, surfaceDragP, Localizer.Format("#autoLOC_KWT351")); // "surfDrag"
            WriteCurveSet(data, surfaceDragI, Localizer.Format("#autoLOC_KWT352")); // "induDrag"
            WriteCurveSet(data, bodyLift, Localizer.Format("#autoLOC_KWT353"));     // "bodyLift"
            WriteCurveSet(data, surfaceLift, Localizer.Format("#autoLOC_KWT354"));  // "surfLift"
            // TODO: induDrag_MFactor = surfLift_MFactor and one of them should be deleted.


            return data;
        }

        protected override async Task WriteToFileXLS(string path, System.Data.DataSet data)
        {
            for (int i = 0; i < Math.Min(data.Tables.Count, 5); i++)
                await Task.Run(() => Graphing.IO.GraphIO.SpreadsheetWriter.Write(path, data.Tables[i].TableName, data.Tables[i], new Graphing.IO.SpreadsheetOptions(false, false, 1, 1))).ConfigureAwait(false);
            for (int i = 5; i < data.Tables.Count; i++)
                await Task.Run(() => Graphing.IO.GraphIO.SpreadsheetWriter.Write(path, data.Tables[i].TableName, data.Tables[i], new Graphing.IO.SpreadsheetOptions(true, true, 1, 0))).ConfigureAwait(false);
        }
        protected override async Task WriteToFileCSV(string path, System.Data.DataSet data)
        {
            for (int i = 0; i < Math.Min(data.Tables.Count, 5); i++)
                await WindTunnel.MiniExcelWrapper.WriteToCSV(path.Insert(path.Length - 4, $"_{data.Tables[i].TableName}"), data.Tables[i], printHeader: false, sheetName: data.Tables[0].TableName);
            for (int i = 5; i < data.Tables.Count; i++)
                await WindTunnel.MiniExcelWrapper.WriteToCSV(path.Insert(path.Length - 4, $"_{data.Tables[i].TableName}"), data.Tables[i], printHeader: true, sheetName: data.Tables[i].TableName);
        }
    }
}
