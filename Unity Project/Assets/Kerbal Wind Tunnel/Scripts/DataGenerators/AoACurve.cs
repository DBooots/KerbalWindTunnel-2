using System;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;
using KSP.Localization;
using Graphing;
using static KerbalWindTunnel.VesselCache.AeroOptimizer;

namespace KerbalWindTunnel.DataGenerators
{
    using LineGraphDefinition = LineGraphDefinition<AoACurve.AoAPoint>;
    public class AoACurve
    {
        public readonly GraphableCollection graphables = new GraphableCollection() { Name = "AoA" };
        public AoAPoint[] AoAPoints { get; private set; }
        public float AverageLiftSlope { get; private set; }
        private static readonly ConcurrentDictionary<(int altitude, int velocity, float aoa), AoAPoint> cache = new ConcurrentDictionary<(int, int, float), AoAPoint>();

        private static Color defaultColor = Color.green;
        private static Color dryColor = Color.yellow;
        private static Func<AoAPoint, Vector2> ToVector(Func<AoAPoint, float> func) => (pt) => new Vector2(pt.AoA * Mathf.Rad2Deg, func(pt));
        public List<GraphDefinition> graphDefinitions = new List<GraphDefinition>
        {
            new LineGraphDefinition("lift_coefSwap", null) { DisplayName = "#autoLOC_KWT308", Color = defaultColor },   // "Lift"
            new LineGraphDefinition("drag_coefSwap", null) { DisplayName = "#autoLOC_KWT311", Color = defaultColor },   // "Drag"
            new LineGraphDefinition("ldRatio", ToVector(p => p.LDRatio)) { DisplayName = "#autoLOC_KWT313", YUnit = "#autoLOC_KWT015", StringFormat = "F2", Color = defaultColor },   // "Lift/Drag Ratio" "-"
            new LineGraphDefinition("lift_slope_coefSwap", null) { DisplayName = "#autoLOC_KWT314", StringFormat = "F3", Color = defaultColor },    // "Lift Slope"
            new GroupedGraphDefinition<LineGraphDefinition> ("pitchInput",
                new LineGraphDefinition("pitchInput_wet", ToVector(p => p.pitchInput * 100)) { DisplayName = "#autoLOC_KWT316", YName = "#autoLOC_KWT315", YUnit = "#autoLOC_KWT003", StringFormat = "N0", Color = defaultColor },  // "Pitch Input (Wet)" "Pitch Input"  "%"
                new LineGraphDefinition("pitchInput_dry", ToVector(p => p.pitchInput_dry * 100)) { DisplayName = "#autoLOC_KWT317", YName = "#autoLOC_KWT315", YUnit = "#autoLOC_KWT003", StringFormat = "N0", Color = dryColor }   // "Pitch Input (Dry)" "Pitch Input"  "%"
                ) {DisplayName = "#autoLOC_KWT315", YUnit = "#autoLOC_KWT003" },  // "Pitch Input"  "%"
            /*new GroupedGraphDefinition<LineGraphDefinition> ("staticMargin",
                new LineGraphDefinition("staticMargin_wet", ToVector(p => p.staticMargin * 100)) { DisplayName = "#autoLOC_KWT337", YName = "#autoLOC_KWT336", YUnit = "#autoLOC_KWT014", StringFormat = "F2", Color = defaultColor },  // "Static Margin (Wet)" "Static Margin" "% MAC"
                new LineGraphDefinition("staticMargin_wet", ToVector(p => p.staticMargin_dry * 100)) { DisplayName = "#autoLOC_KWT338", YName = "#autoLOC_KWT336", YUnit = "#autoLOC_KWT014", StringFormat = "F2", Color = dryColor }   // "Static Margin (Dry)" "Static Margin" "% MAC"
                ) { DisplayName = "#autoLOC_KWT336", YUnit = "#autoLOC_KWT014" },*/ //  "Static Margin" "% MAC"
            new GroupedGraphDefinition<LineGraphDefinition> ("torque",
                new LineGraphDefinition("torque_wet", ToVector(p => p.torque)) { DisplayName = "#autoLOC_KWT319", YName = "#autoLOC_KWT318", YUnit = "#autoLOC_KWT010", StringFormat = "N0", Color = defaultColor },    // "Torque (Wet)" "Torque" "kNm"
                new LineGraphDefinition("torque_dry", ToVector(p => p.torque_dry)) { DisplayName = "#autoLOC_KWT320", YName = "#autoLOC_KWT318", YUnit = "#autoLOC_KWT010", StringFormat = "N0", Color = dryColor }     // "Torque (Dry" "Torque" "kNm"
                ) { DisplayName = "#autoLOC_KWT318", YUnit = "#autoLOC_KWT010" }    // "Torque" "kNm"
        };

        public void SetCoefficientMode(bool useCoefficients)
        {
            foreach (GraphDefinition graphDef in graphDefinitions.Where(g => g.name.EndsWith("_coefSwap")))
            {
                if (graphDef is LineGraphDefinition lineDef)
                {
                    switch (graphDef.name.Substring(0, graphDef.name.IndexOf("_coefSwap")))
                    {
                        case "lift":
                            lineDef.mappingFunc = useCoefficients ? ToVector(p => p.Coefficient(p.Lift)) : ToVector(p => p.Lift);
                            break;
                        case "drag":
                            lineDef.mappingFunc = useCoefficients ? ToVector(p => p.Coefficient(p.Drag)) : ToVector(p => p.Drag);
                            break;
                        case "lift_slope":
                            lineDef.mappingFunc = useCoefficients ? ToVector(p => p.Coefficient(p.dLift)) : ToVector(p => p.dLift);
                            lineDef.YUnit = useCoefficients ? "#autoLOC_KWT007" : "#autoLOC_KWT008";   // "/°" "kN/°"
                            lineDef.StringFormat = useCoefficients ? "F3" : "F1";
                            continue;
                        default:
                            continue;
                    }
                    lineDef.YName = useCoefficients ? "#autoLOC_KWT307" : "#autoLOC_KWT321";    // "Coefficient" "Force"
                    lineDef.YUnit = useCoefficients ? "#autoLOC_KWT015" : "#autoLOC_KWT004";    // "-" "kN"
                    lineDef.StringFormat = useCoefficients ? "F2" : "N0";
                }
            }
        }

        public AoACurve()
        {
            SetCoefficientMode(WindTunnelSettings.UseCoefficients);
            foreach (GraphDefinition graphDefinition in graphDefinitions)
            {
                graphDefinition.XUnit = "#autoLOC_KWT000";  // "°"
                graphDefinition.XName = "#autoLOC_KWT302";  // "Angle of Attack"
                graphDefinition.Visible = false;
            }
            graphables.AddRange(graphDefinitions.Where(g => g.Enabled).Select(g => g.Graph));
        }

        public async Task Calculate(AeroPredictor aeroPredictorToClone, CancellationToken cancellationToken, TaskProgressTracker tracker, CelestialBody body, float altitude, float velocity, float lowerBound, float upperBound, int segments = 50)
        {
            Task <AoAPoint[]> task = Task.Run(() => CalculateTask(aeroPredictorToClone, cancellationToken, body, altitude, velocity, lowerBound, upperBound, segments, tracker), cancellationToken);
            tracker.Task = task;
            try
            {
                await task;
            }
            catch (OperationCanceledException) { return; }
            PushResults(task);
        }

        private static AoAPoint[] CalculateTask(AeroPredictor aeroPredictorToClone, CancellationToken cancellationToken, CelestialBody body, float altitude, float velocity, float lowerBound, float upperBound, int segments = 50, TaskProgressTracker tracker = null)
        {
            // Apply rounding to facilitate caching.
            const int altitudeRound = 10;
            int altitude_ = Mathf.RoundToInt(altitude / altitudeRound) * altitudeRound;
            int velocity_ = Mathf.RoundToInt(velocity);
            List<float> keysList = new List<float>();
            float delta = (upperBound - lowerBound) / segments;
            for (int i = 0; i <= segments; i++)
                keysList.Add(lowerBound + delta * i);

            // Amend the tracker count in case rounding eliminated any keys.
            tracker?.AmendTotal(keysList.Count);

            float wingArea = aeroPredictorToClone.Area;

            // Fill in the data in a parallel loop.
            AoAPoint[] results = new AoAPoint[keysList.Count];
            try
            {
                Parallel.For<AeroPredictor>(0, keysList.Count, new ParallelOptions() { CancellationToken = cancellationToken },
                    aeroPredictorToClone.GetThreadSafeObject,
                    (index, state, predictor) =>
                    {
                        float aoa = keysList[index];
                        if (!cache.TryGetValue((altitude_, velocity_, aoa), out results[index]))
                        {
                            AoAPoint data = new AoAPoint(predictor, body, altitude_, velocity_, aoa);
                            results[index] = data;
                            cache[(altitude_, velocity_, aoa)] = data;
                        }
                        tracker?.Increment();
                        return predictor;
                    },
                    (predictor) => (predictor as VesselCache.IReleasable)?.Release());
            }
            catch (AggregateException aggregateException)
            {
                foreach (var ex in aggregateException.Flatten().InnerExceptions)
                {
                    Debug.LogException(ex);
                }
                throw aggregateException;
            }

            // Last chance to ditch the results before pushing them to the UI
            cancellationToken.ThrowIfCancellationRequested();

            return results;
        }
        private void PushResults(Task<AoAPoint[]> task)
        {
            lock (this)
            {
                AoAPoints = task.Result;
                UpdateGraphs();
            }
            Debug.Log("[KWT] Graphs updated - AoA");
        }

        public static async Task Clear(Task task = null)
        {
            if (task == null)
                cache.Clear();
            else
                await task.ContinueWith(ClearContinuation);
        }
        private static void ClearContinuation(Task _) => cache.Clear();

        public void UpdateGraphs()
        {
            if (AoAPoints == null)
                return;

            AverageLiftSlope = AoAPoints.Select(pt => pt.dLift * pt.invDynamicPressure).Where(v => !float.IsNaN(v) && !float.IsInfinity(v)).Average();

            foreach (GraphDefinition graphDefinition in graphDefinitions)
            {
                if (graphDefinition is LineGraphDefinition lineGraph)
                    lineGraph.UpdateGraph(AoAPoints);
                else if (graphDefinition is GroupedGraphDefinition<LineGraphDefinition> groupedGraphDefinition)
                    foreach (var lineGraphChild in groupedGraphDefinition)
                        lineGraphChild.UpdateGraph(AoAPoints);
            }
        }

        public readonly struct AoAPoint
        {
            public readonly float Lift;
            public readonly float Drag;
            public readonly float LDRatio;
            public readonly float altitude;
            public readonly float speed;
            public readonly float AoA;
            public readonly float invDynamicPressure;
            public readonly float dLift;
            public readonly float mach;
            public readonly float pitchInput;
            public readonly float pitchInput_dry;
            //public readonly float staticMargin;
            //public readonly float staticMargin_dry;
            public readonly float torque;
            public readonly float torque_dry;
            public readonly bool completed;
            private readonly float wingArea;
            private readonly float invWingArea;

            public float Coefficient(float force) => force * invDynamicPressure * invWingArea;

            public AoAPoint(AeroPredictor vessel, CelestialBody body, float altitude, float speed, float AoA)
            {
                this.altitude = altitude;
                this.speed = speed;
                AeroPredictor.Conditions conditions = new AeroPredictor.Conditions(body, speed, altitude);
                this.AoA = AoA;
                this.mach = conditions.mach;
                this.invDynamicPressure = 1 / conditions.Q;
                this.pitchInput = vessel.FindStablePitchInput(conditions, AoA);
                this.pitchInput_dry = vessel.FindStablePitchInput(conditions, AoA, dryTorque: true);
                Vector3 force = AeroPredictor.ToFlightFrame(vessel.GetAeroForce(conditions, AoA, pitchInput), AoA);
                torque = vessel.GetAeroTorque(conditions, AoA).x;
                torque_dry = vessel.GetAeroTorque(conditions, AoA, 0, true).x;
                Lift = force.y;
                Drag = -force.z;
                LDRatio = Math.Abs(Lift / Drag);
                if (vessel is ILiftAoADerivativePredictor derivativePredictor)
                    dLift = derivativePredictor.GetLiftForceMagnitudeAoADerivative(conditions, AoA, pitchInput) * Mathf.Deg2Rad; // Deg2Rad = 1/Rad2Deg
                else
                    dLift = (vessel.GetLiftForceMagnitude(conditions, AoA + WindTunnelWindow.AoAdelta, pitchInput) - Lift) /
                        (WindTunnelWindow.AoAdelta * Mathf.Rad2Deg);
                /*staticMargin = vessel.GetStaticMargin(conditions, AoA, pitchInput, dLift: dLift, baselineTorque: torque);
                staticMargin_dry = vessel.GetStaticMargin(conditions, AoA, pitchInput, dryTorque: true, dLift: dLift, baselineTorque: torque_dry);*/
                wingArea = vessel.Area;
                invWingArea = 1 / wingArea;
                completed = true;
            }

            public override string ToString()
            {
                if (WindTunnelSettings.UseCoefficients)
                {
                    float coefMod = Coefficient(1);
                    return string.Format($"{Localizer.Format("#autoLOC_KWT322")}:\t{{0:N0}}{Localizer.Format("#autoLOC_KWT001")}\n{Localizer.Format("#autoLOC_KWT323")}:\t{{1:N0}}{Localizer.Format("#autoLOC_KWT005")}\n{Localizer.Format("#autoLOC_KWT300")}:\t{{6:N2}}\n{Localizer.Format("#autoLOC_KWT306")}:\t{{2:N2}}{Localizer.Format("#autoLOC_KWT000")}\n" +   // "Altitude" "m" "Speed" "m/s" "Mach" "AoA" "°"
                        $"{Localizer.Format("#autoLOC_KWT310")}:\t{{3:N2}}\n{Localizer.Format("#autoLOC_KWT312")}:\t{{4:N2}}\n{Localizer.Format("#autoLOC_KWT313")}:\t{{5:N2}}\n{Localizer.Format("#autoLOC_KWT315")}:\t{{7:F3}}\n" +     // "Lift Coefficient" "Drag Coefficient" "Lift/Drag Ratio" "Pitch Input"
                        $"{Localizer.Format("#autoLOC_KWT325")}:\t{{8:F2}}",   // "Wing Area"
                        altitude, speed, AoA * Mathf.Rad2Deg,
                        Lift * coefMod, Drag * coefMod, LDRatio, mach, pitchInput,
                        wingArea);
                }
                else
                    return string.Format($"{Localizer.Format("#autoLOC_KWT322")}:\t{{0:N0}}{Localizer.Format("#autoLOC_KWT001")}\n{Localizer.Format("#autoLOC_KWT323")}:\t{{1:N0}}{Localizer.Format("#autoLOC_KWT005")}\n{Localizer.Format("#autoLOC_KWT300")}:\t{{6:N2}}\n{Localizer.Format("#autoLOC_KWT306")}:\t{{2:N2}}{Localizer.Format("#autoLOC_KWT000")}\n" +   // "Altitude" "m" "Speed" "m/s" "Mach" "AoA" "°"
                            $"{Localizer.Format("#autoLOC_KWT308")}:\t{{3:N0}}{Localizer.Format("#autoLOC_KWT004")}\n{Localizer.Format("#autoLOC_KWT311")}:\t{{4:N0}}{Localizer.Format("#autoLOC_KWT004")}\n{Localizer.Format("#autoLOC_KWT313")}:\t{{5:N2}}\n{Localizer.Format("#autoLOC_KWT315")}:\t{{7:F3}}",  // "Lift" "kN" "Drag" "kN" "Lift/Drag Ratio" "Pitch Input"
                            altitude, speed, AoA * Mathf.Rad2Deg,
                            Lift, Drag, LDRatio, mach, pitchInput);
            }
        }
    }
}
