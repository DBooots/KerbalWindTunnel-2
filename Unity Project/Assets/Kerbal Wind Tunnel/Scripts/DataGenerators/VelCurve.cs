using System;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;
using Graphing;

namespace KerbalWindTunnel.DataGenerators
{
    using LineGraphDefinition = LineGraphDefinition<EnvelopePoint>;
    public class VelCurve
    {
        public readonly GraphableCollection graphables = new GraphableCollection() { Name = "speed" };
        public EnvelopePoint[] VelPoints { get; private set; }
        private static readonly ConcurrentDictionary<(int altitude, int velocity), EnvelopePoint> cache = new ConcurrentDictionary<(int, int), EnvelopePoint>();

        private static Color defaultColor = Color.green;
        private static Func<EnvelopePoint, Vector2> ToVector(Func<EnvelopePoint, float> func) => (pt) => new Vector2(WindTunnelSettings.SpeedIsMach ? pt.mach : pt.speed, func(pt));

        public readonly List<LineGraphDefinition> graphDefinitions = new List<LineGraphDefinition>()
        {
            new LineGraphDefinition("aoa_level", ToVector(p => p.AoA_level * Mathf.Rad2Deg)){ DisplayName = "#autoLOC_KWT326", YName = "#autoLOC_KWT303", YUnit = "#autoLOC_KWT000", StringFormat = "F2", Color = defaultColor },   // "Level AoA" "Angle" "°"
            new LineGraphDefinition("aoa_max", ToVector(p => p.AoA_max * Mathf.Rad2Deg)) { DisplayName = "#autoLOC_KWT327", YName = "#autoLOC_KWT303", YUnit = "#autoLOC_KWT000", StringFormat = "F2", Color = defaultColor },    // "Max Lift AoA" "Angle" "°"
            new LineGraphDefinition("ldRatio", ToVector(p => p.LDRatio)) { DisplayName = "#autoLOC_KWT313", YUnit = "#autoLOC_KWT015", StringFormat = "F2", Color = defaultColor },   // "Lift/Drag Ratio" "-"
            new LineGraphDefinition("lift_slope_coefSwap", null) { DisplayName = "#autoLOC_KWT314", StringFormat = "F3", Color = defaultColor },    // "Lift Slope"
            new LineGraphDefinition("thrust_available", ToVector(p => p.thrust_available)) { DisplayName = "#autoLOC_KWT332", YName = "#autoLOC_KWT321", YUnit = "#autoLOC_KWT004", StringFormat = "N0", Color = defaultColor },   // "Thrust Available" "Force" "kN"
            new LineGraphDefinition("thrust_required", ToVector(p => p.thrust_required)) { DisplayName = "#autoLOC_KWT333", YName = "#autoLOC_KWT321", YUnit = "#autoLOC_KWT004", StringFormat = "N0", Color = defaultColor },   // "Thrust Required" "Force" "kN"
            new LineGraphDefinition("excess_thrust", ToVector(p => p.Thrust_Excess)){ DisplayName = "#autoLOC_KWT329", YName = "#autoLOC_KWT321", YUnit = "#autoLOC_KWT004", StringFormat="N0", Color = defaultColor },  // "Excess Thrust" "Force" "kN"
            new LineGraphDefinition("liftMax_coefSwap", null) { DisplayName = "#autoLOC_KWT309", Color = defaultColor },    // "Max Lift"
            new LineGraphDefinition("drag_coefSwap", null) { DisplayName = "#autoLOC_KWT311", Color = defaultColor },   // "Drag"
            new LineGraphDefinition("fuel_economy", ToVector(p => p.fuelBurnRate / p.speed * 100 * 1000)) { DisplayName = "#autoLOC_KWT340", YUnit = "#autoLOC_KWT011", StringFormat = "F2", Color = defaultColor, Enabled = false }, // "Fuel Economy" "kg/100 km"
            new LineGraphDefinition("fuel_rate", ToVector(p => p.fuelBurnRate)) { DisplayName = "#autoLOC_KWT341", YUnit = "#autoLOC_KWT012", StringFormat = "F3", Color = defaultColor, Enabled = false }, // "Fuel Burn Rate" "kg/s"
            new LineGraphDefinition("pitch_input", ToVector(p => p.pitchInput * 100)) { DisplayName = "#autoLOC_KWT315", YUnit = "#autoLOC_KWT003", StringFormat = "N0", Color = defaultColor },    // "Pitch Input" "%"
            /*new LineGraphDefinition("staticMargin", ToVector(p => p.speed >= 40 ? p.staticMargin * 100 : float.NaN)) { DisplayName = "#autoLOC_KWT336", YUnit = "#autoLOC_KWT014", StringFormat = "F2" },   // "Static Margin" "% MAC"
            new LineGraphDefinition("stabilityDerivative", ToVector(p => p.dTorque)) { DisplayName = "#autoLOC_KWT339", YUnit = "#autoLOC_KWT009", StringFormat = "F2" },*/   // "Stability Derivative" "kNm/°"
            new LineGraphDefinition("accel_excess", ToVector(p => p.Accel_Excess)) { DisplayName = "#autoLOC_KWT330", YUnit = "#autoLOC_KWT006", StringFormat = "N2", Color = defaultColor, Enabled = false },  // "Excess Acceleration" "g"
            new LineGraphDefinition("aoa_glide", ToVector(p => p.AoA_gliding * Mathf.Rad2Deg)) { DisplayName = "#autoLOC_KWT355", YName = "#autoLOC_KWT303", YUnit = "#autoLOC_KWT000", StringFormat = "F2", Enabled = WindTunnelSettings.EnableGlidingCalcs }, // "Glide AoA" "Angle" "°"
            new LineGraphDefinition("fpa_glide", ToVector(p => p.glideAngle * Mathf.Rad2Deg)) { DisplayName = "#autoLOC_KWT355", YName = "#autoLOC_KWT303", YUnit = "#autoLOC_KWT000", StringFormat = "F2", Enabled = WindTunnelSettings.EnableGlidingCalcs }   // "Glide Angle" "Angle" "°"
            //new LineGraphDefinition("ldRatio_glide", ToVector(p => p.LDRatioGliding)) { DisplayName = "#autoLOC_KWT356", YUnit = "#autoLOC_KWT015", StringFormat = "F2", Enabled = WindTunnelSettings.EnableGlidingCalcs }  // "Glide Ratio" "-"
        };

        public void SetCoefficientMode(bool useCoefficients)
        {
            foreach (GraphDefinition graphDef in graphDefinitions.Where(g => g.name.EndsWith("_coefSwap")))
            {
                if (graphDef is LineGraphDefinition lineDef)
                {
                    switch (graphDef.name.Substring(0, graphDef.name.IndexOf("_coefSwap")))
                    {
                        case "liftMax":
                            lineDef.mappingFunc = useCoefficients ? ToVector(p => p.Coefficient(p.lift_max)) : ToVector(p => p.lift_max);
                            break;
                        case "drag":
                            lineDef.mappingFunc = useCoefficients ? ToVector(p => p.Coefficient(p.drag)) : ToVector(p => p.drag);
                            break;
                        case "lift_slope":
                            lineDef.mappingFunc = useCoefficients ? ToVector(p => p.Coefficient(p.dLift)) : ToVector(p => p.dLift);
                            lineDef.YUnit = useCoefficients ? "#autoLOC_KWT007" : "#autoLOC_KWT008";    // "/°" "kN/°"
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

        public void SetMachMode(bool speedIsMach)
        {
            string xName, xUnit;
            if (speedIsMach)
            {
                xName = EnvelopeSurf.xMachName;
                xUnit = EnvelopeSurf.xMachUnit;
            }
            else
            {
                xName = EnvelopeSurf.xName;
                xUnit = EnvelopeSurf.xUnit;
            }

            foreach (GraphDefinition graphDefinition in graphDefinitions)
            {
                graphDefinition.XName = xName;
                graphDefinition.XUnit = xUnit;
            }
        }

        public VelCurve()
        {
            SetCoefficientMode(WindTunnelSettings.UseCoefficients);
            foreach (GraphDefinition graphDefinition in graphDefinitions)
            {
                graphDefinition.XUnit = "#autoLOC_KWT005";  // "m/s"
                graphDefinition.XName = "#autoLOC_KWT324";  // "Airspeed"
                graphDefinition.Visible = false;
            }
            graphables.AddRange(graphDefinitions.Where(g => g.Enabled).Select(g => g.Graph));
        }

        public async Task Calculate(AeroPredictor aeroPredictorToClone, CancellationToken cancellationToken, TaskProgressTracker tracker, CelestialBody body, float altitude, float lowerBound, float upperBound, int segments = 50)
        {
            Task<EnvelopePoint[]> task = Task.Run(() => CalculateTask(aeroPredictorToClone, cancellationToken, body, altitude, lowerBound, upperBound, segments, tracker), cancellationToken);
            tracker.Task = task;
            try
            {
                await task;
            }
            catch (OperationCanceledException) { return; }
            PushResults(task);
        }

        private static EnvelopePoint[] CalculateTask(AeroPredictor aeroPredictorToClone, CancellationToken cancellationToken, CelestialBody body, float altitude, float lowerBound, float upperBound, int segments = 50, TaskProgressTracker tracker = null)
        {
            // Apply rounding to facilitate caching.
            const int altitudeRound = 10;
            int altitude_ = Mathf.RoundToInt(altitude / altitudeRound) * altitudeRound;
            lowerBound = Mathf.Round(lowerBound);
            upperBound = Mathf.Round(upperBound);
            SortedSet<int> keys = new SortedSet<int>();
            float delta = (upperBound - lowerBound) / segments;
            for (int i = 0; i <= segments; i++)
                keys.Add(Mathf.RoundToInt(lowerBound + delta * i));
            List<int> keysList = keys.ToList();

            // Amend the tracker count in case rounding eliminated any keys.
            tracker?.AmendTotal(keysList.Count);

            float wingArea = aeroPredictorToClone.Area;

            // Fill in the data in a parallel loop.
            EnvelopePoint[] results = new EnvelopePoint[keys.Count];
            try
            {
                Parallel.For<AeroPredictor>(0, keys.Count, new ParallelOptions() { CancellationToken = cancellationToken },
                    aeroPredictorToClone.GetThreadSafeObject,
                    (index, state, predictor) =>
                    {
                        int velocity = keysList[index];
                        if (!cache.TryGetValue((altitude_, velocity), out results[index]))
                        {
                            EnvelopePoint data = new EnvelopePoint(predictor, body, altitude_, velocity);
                            results[index] = data;
                            cache[(altitude_, velocity)] = data;
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
        private void PushResults(Task<EnvelopePoint[]> task)
        {
            lock (this)
            {
                VelPoints = task.Result;
                UpdateGraphs();
            }
            Debug.Log("[KWT] Graphs updated - Velocity");
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
            if (VelPoints == null)
                return;
            foreach (LineGraphDefinition lineGraphDefinition in graphDefinitions)
            {
                lineGraphDefinition.UpdateGraph(VelPoints);
            }
        }
    }
}
