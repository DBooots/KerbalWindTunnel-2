using System;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;
using Graphing;
using KerbalWindTunnel.Extensions;
using System.Collections.Concurrent;
using System.Collections.Generic;

namespace KerbalWindTunnel.DataGenerators
{
    using SurfGraphDefinition = SurfGraphDefinition<EnvelopePoint>;
    public class EnvelopeSurf
    {
        private static readonly (float speed, float altitude) ascentOrigin = (30, 0);
        public readonly GraphableCollection graphables = new GraphableCollection3() { Name = "envelope" };
        public EnvelopePoint[,] EnvelopePoints { get; private set; } = null;
        private float left, right, bottom, top;
        private static readonly ConcurrentDictionary<SurfCoords, EnvelopePoint> cache = new ConcurrentDictionary<SurfCoords, EnvelopePoint>();

        public static (int x, int y)[] resolution = { (10, 10), (40, 90), (80, 180), (160, 180) };

        private const string xName = "#autoLOC_KWT323", xUnit = "#autoLOC_KWT005", yName = "#autoLOC_KWT322", yUnit = "#autoLOC_KWT001";    // "Speed" "m/s" "Altitude" "m"
        private const string xMachName = "#autoLOC_KWT334", xMachUnit = "-";    // "Sea Level Mach"
        private static Func<EnvelopeLine.AscentPathPoint, Vector2> ToVector =>
            (pt) => new Vector2(
                WindTunnelSettings.SpeedIsMach ? pt.speed / WindTunnelWindow.GetSpeedOfSound(0) : pt.speed,
                pt.altitude);

        public readonly List<GraphDefinition> graphDefinitions = new List<GraphDefinition>
        {
            new SurfGraphDefinition("thrust_excess", p => p.Thrust_Excess) { DisplayName = "#autoLOC_KWT329", ZUnit = "#autoLOC_KWT004", StringFormat = "N0", CMin = 0 },       // "Excess Thrust" "kN"
            new SurfGraphDefinition("thrust_available", p => p.thrust_available) { DisplayName = "#autoLOC_KWT332", ZUnit = "#autoLOC_KWT004", StringFormat = "N0", CMin = 0 }, // "Thrust Available" "kN"
            new SurfGraphDefinition("power_excess", p => p.Specific(p.Power_Excess)) { DisplayName = "#autoLOC_KWT335", ZUnit = "#autoLOC_KWT005", StringFormat = "N0", CMin = 0 }, // "Specific Excess Power" "m/s"
            new SurfGraphDefinition("climbAngle", p => Mathf.Asin(Mathf.Clamp(p.Thrust_Excess * p.invMass * WindTunnelWindow.invGAccel, -1, 1)) * Mathf.Rad2Deg)
                { DisplayName = "#autoLOC_KWT343", ZUnit = "#autoLOC_KWT000", StringFormat = "F2" },    // "Max Climb Angle" "°"
            new SurfGraphDefinition("aoa_level", p => p.AoA_level * Mathf.Rad2Deg) { DisplayName = "#autoLOC_KWT326", ZUnit = "#autoLOC_KWT000", StringFormat = "F2" }, // "Level Flight AoA" "°"
            new SurfGraphDefinition("ldRatio", p => p.LDRatio) { DisplayName = "#autoLOC_KWT313", ZUnit = "#autoLOC_KWT015", StringFormat = "F2" },   // "Lift/Drag Ratio" "-"
            new SurfGraphDefinition("lift_slope_coefSwap", null) { DisplayName = "#autoLOC_KWT314", StringFormat = "F3" },  // "Lift Slope"
            new SurfGraphDefinition("drag_coefSwap", null) { DisplayName = "#autoLOC_KWT311" }, // "Drag"
            new SurfGraphDefinition("aoa_max", p => p.AoA_max * Mathf.Rad2Deg) { DisplayName = "#autoLOC_KWT327", ZUnit = "#autoLOC_KWT000", StringFormat = "F2" }, // "Max Lift AoA" "°"
            new SurfGraphDefinition("liftMax_coefSwap", null) { DisplayName = "#autoLOC_KWT309" },  // "Max Lift"
            new SurfGraphDefinition("pitch_input", p => p.pitchInput * 100) { DisplayName = "#autoLOC_KWT315", ZUnit = "#autoLOC_KWT003", StringFormat = "N0" },    // "Pitch Input" "%"
            /*new SurfGraphDefinition("staticMargin", p => p.speed >= 40 ? p.staticMargin * 100 : float.NaN) { DisplayName = "#autoLOC_KWT336", ZUnit = "#autoLOC_KWT014", StringFormat = "F2" }, // "Static Margin" "% MAC"
            new SurfGraphDefinition("stabilityDerivative", p => p.dTorque) { DisplayName = "#autoLOC_KWT339", ZUnit = "#autoLOC_KWT009", StringFormat = "F2" },*/ // "Stability Derivative" "kNm/°"
            new SurfGraphDefinition("fuel_economy", p => p.speed >= 40 ? p.fuelBurnRate / p.speed * 100 * 1000 : float.NaN) { DisplayName = "#autoLOC_KWT340", ZUnit = "#autoLOC_KWT011", StringFormat = "F2" },    // "Fuel Economy" "kg/100 km"
            new SurfGraphDefinition("fuel_rate", p => p.fuelBurnRate) { DisplayName = "#autoLOC_KWT341", ZUnit = "#autoLOC_KWT012", StringFormat = "F3" },   // "Fuel Burn Rate" "kg/s"
            new SurfGraphDefinition("accel_excess", p => p.Accel_Excess) { DisplayName = "#autoLOC_KWT330", ZUnit = "#autoLOC_KWT006", StringFormat = "N2", CMin = 0, Enabled = false } // "Excess Acceleration" "g"
        };
        //graphables.Add(new SurfGraph(blank, left, right, bottom, top) { Name = "Stability Range", ZUnit = "deg", StringFormat = "F2", ColorScheme = Graphing.Extensions.GradientExtensions.Jet_Dark });
        public readonly OutlineGraphDefinition<EnvelopePoint> envelope = new OutlineGraphDefinition<EnvelopePoint>("fltEnvelope", p => p.Thrust_Excess) { DisplayName = "#autoLOC_KWT342", ZUnit = "#autoLOC_KWT004", StringFormat = "N0", Color = Color.gray, LineWidth = 2, LineOnly = true, MaskCriteria = (v) => !float.IsNaN(v.z) && !float.IsInfinity(v.z) ? v.z : -1 };  // "Flight Envelope" "kN"
        public readonly MetaLineGraphDefinition<EnvelopeLine.AscentPathPoint> fuelPath = new MetaLineGraphDefinition<EnvelopeLine.AscentPathPoint>("path_fuelOptimal", ToVector,
                new Func<EnvelopeLine.AscentPathPoint, float>[] { p => p.climbAngle * Mathf.Rad2Deg, p => p.climbRate, p => p.cost, p => p.time },
                new string[] { "#autoLOC_KWT344", "#autoLOC_KWT345", "#autoLOC_KWT346", "#autoLOC_KWT304" },    // "Climb Angle" "Climb Rate" "Fuel Used" "Time"
                new string[] { "N1", "N0", "N3", "N1" },
                new string[] { "#autoLOC_KWT000", "#autoLOC_KWT005", "#autoLOC_KWT013", "#autoLOC_KWT002" })    // "°" "m/s" "units" "s"
        { DisplayName = "#autoLOC_KWT347", StringFormat = "N0", Color = Color.black, LineWidth = 3 };   // "Fuel-Optimal Path"
        public readonly MetaLineGraphDefinition<EnvelopeLine.AscentPathPoint> timePath = new MetaLineGraphDefinition<EnvelopeLine.AscentPathPoint>("path_timeOptimal", ToVector,
                new Func<EnvelopeLine.AscentPathPoint, float>[] { p => p.climbAngle * Mathf.Rad2Deg, p => p.climbRate, p => p.cost },
                new string[] { "#autoLOC_KWT344", "#autoLOC_KWT345", "#autoLOC_KWT304" },   // "Climb Angle" "Climb Rate" "Time"
                new string[] { "N1", "N0", "N1" },
                new string[] { "#autoLOC_KWT000", "#autoLOC_KWT005", "#autoLOC_KWT002" })   // "°" "m/s" "s"
        { DisplayName = "#autoLOC_KWT348", StringFormat = "N0", Color = Color.white, LineWidth = 3 };   // "Time-Optimal Path"

        public void SetCoefficientMode(bool useCoefficients)
        {
            foreach (GraphDefinition graphDef in graphDefinitions.Where(g => g.name.EndsWith("_coefSwap")))
            {
                if (graphDef is SurfGraphDefinition surfDef)
                {
                    switch (graphDef.name.Substring(0, graphDef.name.IndexOf("_coefSwap")))
                    {
#if OUTSIDE_UNITY
                        case "liftMax":
                            surfDef.mappingFunc = useCoefficients ? p => p.Coefficient(p.lift_max) : p => p.lift_max;
                            break;
                        case "drag":
                            surfDef.mappingFunc = useCoefficients ? p => p.Coefficient(p.drag) : p => p.drag;
                            break;
                        case "lift_slope":
                            surfDef.mappingFunc = useCoefficients ? p => p.Coefficient(p.dLift) : p => p.dLift;
                            surfDef.ZUnit = useCoefficients ? "#autoLOC_KWT007" : "#autoLOC_KWT008";    // "/°" "kN/°"
                            surfDef.StringFormat = useCoefficients ? "F3" : "F1";
                            continue;
#endif
                        default:
                            continue;
                    }
                    surfDef.ZName = useCoefficients ? "#autoLOC_KWT307" : "#autoLOC_KWT321";    // "Coefficient" "Force"
                    surfDef.ZUnit = useCoefficients ? "#autoLOC_KWT015" : "#autoLOC_KWT004";    // "-" "kN"
                    surfDef.StringFormat = useCoefficients ? "F2" : "N0";
                }
            }
        }

        private bool speedIsMach = WindTunnelSettings.SpeedIsMach;
        public void SetMachMode(bool speedIsMach)
        {
            string xName, xUnit;
            if (speedIsMach)
            {
                xName = xMachName;
                xUnit = xMachUnit;
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
            if (this.speedIsMach == speedIsMach)
                return;
            this.speedIsMach = speedIsMach;
            float asl = WindTunnelWindow.GetSpeedOfSound(0);
            fuelPath.Graph.SetValues(
                fuelPath.Graph.Values.Select(
                    v => new Vector2(
                        speedIsMach ? v.x / asl : v.x * asl,
                        v.y))
                .ToArray());
            timePath.Graph.SetValues(
                timePath.Graph.Values.Select(
                    v => new Vector2(
                        speedIsMach ? v.x / asl : v.x * asl,
                        v.y))
                .ToArray());
        }

        public EnvelopeSurf()
        {
            SetCoefficientMode(WindTunnelSettings.UseCoefficients);
            graphDefinitions.Add(envelope);
            graphDefinitions.Add(fuelPath);
            graphDefinitions.Add(timePath);
            foreach (GraphDefinition graphDefinition in graphDefinitions)
            {
                if (WindTunnelSettings.SpeedIsMach)
                {
                    graphDefinition.XName = xMachName;
                    graphDefinition.XUnit = xMachUnit;
                }
                else
                {
                    graphDefinition.XName = xName;
                    graphDefinition.XUnit = xUnit;
                }
                graphDefinition.YName = yName;
                graphDefinition.YUnit = yUnit;
            }

            graphables.AddRange(graphDefinitions.Where(g => g.Enabled).Select(g => g.Graph));
        }

        public async Task Calculate(AeroPredictor aeroPredictorToClone, CancellationToken cancellationToken, TaskProgressTracker tracker, CelestialBody body, float lowerBoundSpeed, float upperBoundSpeed, float lowerBoundAltitude, float upperBoundAltitude)
        {
            Task lineUpdateTask = null;
            int startLevel = 0;
            if (EnvelopePoints != null)
            {
                int width = EnvelopePoints.GetUpperBound(0) + 1;
                int height = EnvelopePoints.GetUpperBound(1) + 1;
                while (startLevel < resolution.Length && resolution[startLevel].x <= width && resolution[startLevel].y <= height)
                    startLevel++;
            }
            for (int i = startLevel; i < resolution.Length; i++)
            {
                if (i > 0)
                {
                    tracker.FollowOnTaskTracker = new TaskProgressTracker();
                    tracker = tracker.FollowOnTaskTracker;
                }
                int xResolution = resolution[i].x, yResolution = resolution[i].y;
                Task<ResultsType> task = Task.Run(() => CalculateTask(aeroPredictorToClone, cancellationToken, body, lowerBoundSpeed, upperBoundSpeed, lowerBoundAltitude, upperBoundAltitude, xResolution, yResolution, tracker));
                tracker.Task = task;
                try
                {
                    await task;
                }
                catch (OperationCanceledException) { return; }
                if (i > 0)
                {
                    if (lineUpdateTask != null)
                    {
                        try
                        {
                            // Wait on the previous lineCalcTask so there's no risk of races to update the graphs.
                            await lineUpdateTask;
                        }
                        catch (OperationCanceledException) { }
                    }
                    lineUpdateTask = CalculateOptimalLines(task.Result, cancellationToken);
                }
                PushResults(task);
            }
        }

        private static ResultsType CalculateTask(AeroPredictor aeroPredictorToClone, CancellationToken cancellationToken, CelestialBody body, float lowerBoundSpeed, float upperBoundSpeed, float lowerBoundAltitude, float upperBoundAltitude, int speedSegments, int altitudeSegments, TaskProgressTracker progressTracker = null)
        {
#if ENABLE_PROFILER
            UnityEngine.Profiling.Profiler.BeginSample($"EnvelopeSurf.Calculate {resolution.IndexOf((speedSegments, altitudeSegments))}");
#endif
            if (aeroPredictorToClone is VesselCache.SimulatedVessel simVessel && !simVessel.DirectAoAInitialized)
                simVessel.InitMaxAoA();
            EnvelopePoint[] results = new EnvelopePoint[(speedSegments + 1) * (altitudeSegments + 1)];
            float stepSpeed = (upperBoundSpeed - lowerBoundSpeed) / speedSegments;
            float stepAltitude = (upperBoundAltitude - lowerBoundAltitude) / altitudeSegments;

            int cachedCount = 0;

            try
            {
                //OrderablePartitioner<EnvelopePoint> partitioner = Partitioner.Create(primaryProgress, true);
                Parallel.For<AeroPredictor>(0, results.Length, new ParallelOptions() { CancellationToken = cancellationToken },
                    aeroPredictorToClone.GetThreadSafeObject,
                    (index, state, predictor) =>
                    {
                        int x = index % (speedSegments + 1), y = index / (speedSegments + 1);
                        SurfCoords coords = new SurfCoords(x * stepSpeed + lowerBoundSpeed,
                        y * stepAltitude + lowerBoundAltitude);

                        if (!cache.TryGetValue(coords, out EnvelopePoint result))
                        {
                            cancellationToken.ThrowIfCancellationRequested();
                            // Is there an efficient way of searching for the nearest filled cache to have a guess for AoAs?
                            result = new EnvelopePoint(predictor, body, y * stepAltitude + lowerBoundAltitude, x * stepSpeed + lowerBoundSpeed);
                            cancellationToken.ThrowIfCancellationRequested();
                            cache[coords] = result;
                        }
                        else
                            Interlocked.Increment(ref cachedCount);
                        results[index] = result;
                        progressTracker?.Increment();
                        return predictor;
                    }, (predictor) => (predictor as VesselCache.IReleasable)?.Release());
            }
            catch (AggregateException aggregateException)
            {
                foreach (var ex in aggregateException.Flatten().InnerExceptions)
                {
                    Debug.LogException(ex);
                }
                throw aggregateException;
            }
#if ENABLE_PROFILER
            UnityEngine.Profiling.Profiler.EndSample();
#endif                
            cancellationToken.ThrowIfCancellationRequested();

            Debug.LogFormat("[KWT] Data run finished. {0} of {1} ({2:F0}%) retrieved from cache.", cachedCount, results.Length, (float)cachedCount / results.Length * 100);

            return (results.To2Dimension(speedSegments + 1), (lowerBoundSpeed, upperBoundSpeed), (lowerBoundAltitude, upperBoundAltitude));
        }
        private void PushResults(Task<ResultsType> data)
        {
            lock (this)
            {
                ResultsType results = data.Result;
                EnvelopePoints = results.data;
                left = results.speedBounds.left;
                right = results.speedBounds.right;
                bottom = results.altitudeBounds.bottom;
                top = results.altitudeBounds.top;
                UpdateGraphs();
            }
            Debug.Log("[KWT] Graphs updated - Envelope");
        }

        public async Task Clear(Task task = null)
        {
            if (task == null)
            {
                cache.Clear();
                EnvelopePoints = null;
            }
            else
                await task.ContinueWith(ClearContinuation);
        }
        private void ClearContinuation(Task _)
        {
            cache.Clear();
            EnvelopePoints = null;
        }

        public void UpdateGraphs()
        {
            if (EnvelopePoints == null)
                return;
            float scalar = WindTunnelSettings.SpeedIsMach ? 1 / WindTunnelWindow.GetSpeedOfSound(0) : 1;
            foreach (GraphDefinition graph in graphDefinitions.Where(g => g.Enabled))
            {
                if (graph is SurfGraphDefinition surfDefinition)
                    surfDefinition.UpdateGraph(left * scalar, right * scalar, bottom, top, EnvelopePoints);
                else if (graph is OutlineGraphDefinition<EnvelopePoint> outlineDefinition)
                    outlineDefinition.UpdateGraph(left * scalar, right * scalar, bottom, top, EnvelopePoints);
            }
        }

        public async Task CalculateOptimalLines(CancellationToken cancellationToken)
            => await CalculateOptimalLines((EnvelopePoints, (left, right), (bottom, top)), cancellationToken);

        private async Task CalculateOptimalLines(ResultsType results, CancellationToken cancellationToken)
        {
            EnvelopePoint[,] data = results.data;
            
            (float speed, float altitude) initialCoords = ascentOrigin;
            if (initialCoords.altitude < results.altitudeBounds.bottom)
                initialCoords.altitude = results.altitudeBounds.bottom;
            (float speed, float altitude) exitCoords = (WindTunnelWindow.Instance.AscentTargetSpeed, WindTunnelWindow.Instance.AscentTargetAltitude);
            (float lower, float step, float upper) speedBounds = (results.speedBounds.left, (results.speedBounds.right - results.speedBounds.left) / (data.GetUpperBound(0) + 1), results.speedBounds.right);
            (float lower, float step, float upper) altitudeBounds = (results.altitudeBounds.bottom, (results.altitudeBounds.top - results.altitudeBounds.bottom) / (data.GetUpperBound(1) + 1), results.altitudeBounds.top);

            if (WindTunnelWindow.Instance.autoSetAscentTarget || exitCoords.speed < 0 || exitCoords.altitude < 0)
            {
                bool sustainable;
                (exitCoords.speed, exitCoords.altitude, sustainable) = GetMaxSustainableEnergy(data, speedBounds, altitudeBounds);
                if (sustainable)
                    WindTunnelWindow.Instance.ProvideAscentTarget(exitCoords);
            }
            await EnvelopeLine.CalculateOptimalLines(exitCoords, initialCoords, speedBounds, altitudeBounds, data, cancellationToken, fuelPath, timePath);
        }

        public static (float speed, float altitude, bool sustainable) GetMaxSustainableEnergy(EnvelopePoint[,] data, (float lower, float step, float upper) speedBounds, (float lower, float step, float upper) altitudeBounds)
        {
            int speedLength = data.GetUpperBound(0);
            int altLength = data.GetUpperBound(1);
            (float speed, float altitude, bool sustainable) result = (ascentOrigin.speed, ascentOrigin.altitude, false);
            if (result.altitude < altitudeBounds.lower)
                result.altitude = altitudeBounds.lower;
            int altIndex = 0;
            float g = (float)(WindTunnelWindow.Instance.CelestialBody.GeeASL * PhysicsGlobals.GravitationalAcceleration);
            for (int h = 0; h <= altLength; h++)
            {
                // Below this speed, the energy is less than the current result
                float Vmin = Mathf.Sqrt(2 * g * (altitudeBounds.step * (altIndex - h)) + result.speed * result.speed);

                for (int v = speedLength; v >= 0 && v * speedBounds.step + speedBounds.lower >= Vmin; v--)
                {
                    if (data[v, h].Thrust_Excess >= 0)
                    {
                        altIndex = h;
                        result = (speedBounds.step * v + speedBounds.lower, altitudeBounds.step * h + altitudeBounds.lower, true);
                        break;
                    }
                }
            }
            return result;
        }

        private readonly struct SurfCoords : IEquatable<SurfCoords>
        {
            public readonly int speed, altitude;

            public SurfCoords(float speed, float altitude)
            {
                this.speed = Mathf.RoundToInt(speed);
                this.altitude = Mathf.RoundToInt(altitude);
            }
            public SurfCoords(EnvelopePoint point) : this(point.speed, point.altitude) { }

            public override bool Equals(object obj)
            {
                if (obj is SurfCoords c)
                    return Equals(c);
                return false;
            }

            public bool Equals(SurfCoords obj)
            {
                return this.speed == obj.speed && this.altitude == obj.altitude;
            }

            public override int GetHashCode()
            {
                // I'm not expecting altitudes over 131 km or speeds over 8 km/s
                // (or negative values for either) so bit-shifting the values in this way
                // should equally weight the two inputs while returning a hash with the
                // same quality as the default uint/int hash
                // (which may or may not be just that number).
                // This means that there will be zero collisions within the expected range.
                return ((((uint)Mathf.RoundToInt(speed)) << 17) | (uint)Mathf.RoundToInt(altitude)).GetHashCode();
            }
        }

        public readonly struct ResultsType
        {
            public readonly EnvelopePoint[,] data;
            public readonly (float left, float right) speedBounds;
            public readonly (float bottom, float top) altitudeBounds;
            public ResultsType(EnvelopePoint[,] data, (float, float) speedBounds, (float, float) altitudeBounds)
            {
                this.data = data;
                this.speedBounds = speedBounds;
                this.altitudeBounds = altitudeBounds;
            }

            public static implicit operator (EnvelopePoint[,], (float, float), (float, float))(ResultsType obj) =>
                (obj.data, obj.speedBounds, obj.altitudeBounds);
            public static implicit operator ResultsType((EnvelopePoint[,] data, (float, float) speedBounds, (float, float) altitudeBounds) obj) =>
                new ResultsType(obj.data, obj.speedBounds, obj.altitudeBounds);
        }
    }
}
