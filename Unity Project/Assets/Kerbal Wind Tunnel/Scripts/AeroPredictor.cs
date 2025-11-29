using System;
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
        public virtual float MAC { get; protected set; }

        /// <summary>
        /// Generates the lift objective function, which is a function that maps to the lift force given an angle of attack, for defined conditions.
        /// Used for minimization/maximization and root finding.
        /// </summary>
        /// <param name="conditions">The conditions.</param>
        /// <param name="pitchInput">The pitch input.</param>
        /// <param name="scalar">A scalar that inverts the output when set to -1. This is used to enable minimization when the peak lift is desired.</param>
        /// <returns>A function that maps to the lift force given an angle of attack.</returns>
        /// <exception cref="System.ArgumentException">Scalar must be 1 or -1.</exception>
        public virtual Func<double, double> AerodynamicLiftObjectiveFunc(Conditions conditions, float pitchInput, int scalar = 1)
        {
            if (scalar != 1 && scalar != -1)
                throw new ArgumentException("Scalar must be 1 or -1.");
            double AerodynamicObjectiveFuncInternal(double aoa)
            {
                float liftMagnitude = GetLiftForceMagnitude(conditions, (float)aoa, pitchInput);
                if (scalar == -1)
                    return -liftMagnitude;
                return liftMagnitude;
            }
            return AerodynamicObjectiveFuncInternal;
        }
        /// <summary>
        /// Generates the level flight objective function, which is a function that maps to the net upwards force on the vessel given an angle of attack, for defined conditions.
        /// Used for root finding.
        /// </summary>
        /// <param name="conditions">The conditions.</param>
        /// <param name="offsettingForce">The offsetting force (i.e. weight).</param>
        /// <param name="pitchInput">The pitch input.</param>
        /// <returns>A function that maps to the net upwards force on the vessel, given an angle of attack.</returns>
        public virtual Func<double, double> LevelFlightObjectiveFunc(Conditions conditions, float offsettingForce, float pitchInput = 0)
        {
            if (ThrustIsConstantWithAoA)
            {
                Vector3 thrustForce = GetThrustForce(conditions);
                double LevelFlightObjectiveFuncInternal_ConstantThrust(double aoa) =>
                    GetLiftForceComponent(GetLiftForce(conditions, (float)aoa, pitchInput) + thrustForce, (float)aoa) - offsettingForce;
                return LevelFlightObjectiveFuncInternal_ConstantThrust;
            }
            else
            {
                double LevelFlightObjectiveFuncInternal(double aoa) =>
                    GetLiftForceComponent(GetLiftForce(conditions, (float)aoa, pitchInput) + GetThrustForce(conditions, (float)aoa), (float)aoa) - offsettingForce;
                return LevelFlightObjectiveFuncInternal;
            }
        }
        /// <summary>
        /// Generates the level flight objective function, which is a function that maps to the net upwards force on the vessel given an angle of attack, for defined conditions.
        /// Used for root finding.
        /// </summary>
        /// <param name="conditions">The conditions.</param>
        /// <param name="offsettingForce">The offsetting force (i.e. weight).</param>
        /// <param name="pitchInput">A function returning pitch input for the angle of attack to be tested.</param>
        /// <returns>A function that maps to the net upwards force on the vessel, given an angle of attack.</returns>
        public virtual Func<double, double> LevelFlightObjectiveFunc(Conditions conditions, float offsettingForce, Func<float, float> pitchInput)
        {
            if (ThrustIsConstantWithAoA)
            {
                Vector3 thrustForce = GetThrustForce(conditions);
                double LevelFlightObjectiveFuncInternal_ConstantThrust(double aoa) =>
                    GetLiftForceComponent(GetLiftForce(conditions, (float)aoa, pitchInput((float)aoa)) + thrustForce, (float)aoa) - offsettingForce;
                return LevelFlightObjectiveFuncInternal_ConstantThrust;
            }
            else
            {
                double LevelFlightObjectiveFuncInternal(double aoa) =>
                    GetLiftForceComponent(GetLiftForce(conditions, (float)aoa, pitchInput((float)aoa)) + GetThrustForce(conditions, (float)aoa), (float)aoa) - offsettingForce;
                return LevelFlightObjectiveFuncInternal;
            }
        }
        /// <summary>
        /// Generates the pitch input objective function, which is a function that maps to the net torque (or a proxy thereof) given a pitch input, for defined conditions and angle of attack.
        /// Used for root finding.
        /// </summary>
        /// <param name="conditions">The conditions.</param>
        /// <param name="aoa">The angle of attack.</param>
        /// <param name="dryTorque">If set to <c>true</c>, uses the dry torque.</param>
        /// <returns>A function that maps to the net torque, given a pitch input.</returns>
        public virtual Func<double, double> PitchInputObjectiveFunc(Conditions conditions, float aoa, bool dryTorque = false)
        {
            double PitchInputObjectiveFuncInternal(double input) =>
                GetAeroTorque(conditions, aoa, (float)input, dryTorque).x;
            return PitchInputObjectiveFuncInternal;
        }

        public abstract Vector3 GetAeroForce(Conditions conditions, float AoA, float pitchInput = 0);

        public virtual Vector3 GetLiftForce(Conditions conditions, float AoA, float pitchInput = 0)
        {
            return GetAeroForce(conditions, AoA, pitchInput);
        }

        public abstract Vector3 GetAeroTorque(Conditions conditions, float AoA, float pitchInput = 0, bool dryTorque = false);

        public virtual float GetStaticMargin(Conditions conditions, float AoA, float pitchInput = 0, bool dryTorque = false, float dLift = float.NaN, float baselineLift = float.NaN, float baselineTorque = float.NaN)
        {
            const float aoaDelta = WindTunnelWindow.AoAdelta;
            if (float.IsNaN(dLift))
            {
                if (this is VesselCache.AeroOptimizer.ILiftAoADerivativePredictor liftDerivativePredictor)
                    dLift = liftDerivativePredictor.GetLiftForceMagnitudeAoADerivative(conditions, AoA, pitchInput);
                else
                {
                    dLift = GetLiftForceMagnitude(conditions, AoA + aoaDelta, pitchInput);
                    if (float.IsNaN(baselineLift))
                        dLift -= GetLiftForceMagnitude(conditions, AoA, pitchInput);
                    else
                        dLift -= baselineLift;
                }
            }
            float dTorque = GetAeroTorque(conditions, AoA + aoaDelta, pitchInput, dryTorque).x;
            if (float.IsNaN(baselineTorque))
                dTorque -= GetAeroTorque(conditions, AoA, pitchInput, dryTorque).x;
            else
                dTorque -= baselineTorque;
            return (dTorque / dLift) / MAC;
        }
        
        public virtual void GetAeroCombined(Conditions conditions, float AoA, float pitchInput, out Vector3 forces, out Vector3 torques, bool dryTorque = false)
        {
            forces = GetAeroForce(conditions, AoA, pitchInput);
            torques = GetAeroTorque(conditions, AoA, pitchInput);
        }

        public virtual float GetLiftForceMagnitude(Conditions conditions, float AoA, float pitchInput = 0)
        {
            return GetLiftForceComponent(GetLiftForce(conditions, AoA, pitchInput), AoA);
        }
        public static float GetLiftForceComponent(Vector3 force, float AoA)
        {
            return ToFlightFrame(force, AoA).y;
        }

        public virtual float GetDragForceMagnitude(Conditions conditions, float AoA, float pitchInput = 0)
        {
            return GetDragForceComponent(GetAeroForce(conditions, AoA, pitchInput), AoA);
        }
        public static float GetDragForceComponent(Vector3 force, float AoA)
        {
            return -ToFlightFrame(force, AoA).z;
        }

        public abstract Vector3 GetThrustForce(Conditions conditions, float AoA);
        public virtual Vector3 GetThrustForce(Conditions conditions) => GetThrustForce(conditions, 0);
        public virtual Vector3 GetThrustForceFlightFrame(Conditions conditions, float AoA)
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
            => Quaternion.AngleAxis((AoA * Mathf.Rad2Deg), Vector3.left) * force;
        public static Vector2 ToFlightFrame(Vector2 force, float AoA)
            => Quaternion.AngleAxis(AoA * Mathf.Rad2Deg, Vector3.forward) * force;
        public static Vector3 ToVesselFrame(Vector3 force, float AoA)
            => Quaternion.AngleAxis((-AoA * Mathf.Rad2Deg), Vector3.left) * force;
        public static Vector2 ToVesselFrame(Vector2 force, float AoA)
            => Quaternion.AngleAxis(-AoA * Mathf.Rad2Deg, Vector3.forward) * force;

        public static float GetUsefulThrustMagnitude(Vector3 thrustVector)
        {
            if (thrustVector.z > 0)
                return thrustVector.magnitude;
            return thrustVector.magnitude - thrustVector.z;
        }
        public static float GetUsefulThrustMagnitude(Vector2 thrustVector)
        {
            if (thrustVector.x > 0)
                return thrustVector.magnitude;
            return thrustVector.magnitude - thrustVector.x;
        }

        public static Vector3 InflowVect(float AoA)
        {
            Vector3 vesselForward = Vector3.forward;
            Vector3 vesselUp = Vector3.up;
            return vesselForward * Mathf.Cos(-AoA) + vesselUp * Mathf.Sin(-AoA);
        }

        public async void WriteToFile(string directory, string filename, Graphing.IO.GraphIO.FileFormat format)
        {
            if ((format & Graphing.IO.GraphIO.FileFormat.Image) > 0)
                throw new ArgumentException($"Format is not supported. {format}");

            if (!System.IO.Directory.Exists(directory))
                System.IO.Directory.CreateDirectory(directory);

            string path = Graphing.IO.GraphIO.ValidateFilePath(directory, filename, format);

            try
            {
                if (System.IO.File.Exists(path))
                    System.IO.File.Delete(path);
            }
            catch (Exception ex) { Debug.Log($"[KWT] Unable to delete file:{ex.Message}"); }

            System.Data.DataSet output = WriteToDataSet();

            if (output == null)
            {
                Debug.LogError($"[KWT] This AeroPredictor does not implement an output method. {this.GetType()}");
                return;
            }

            if ((format & Graphing.IO.GraphIO.FileFormat.Excel) > 0)
                await WriteToFileXLS(path, output);
            else if (format == Graphing.IO.GraphIO.FileFormat.CSV)
                await WriteToFileCSV(path, output);
            else
            {
                output.Dispose();
                throw new NotImplementedException($"The selected format is not supported: {format}");
            }
            output.Dispose();
        }
        protected virtual System.Data.DataSet WriteToDataSet() => null;
        protected virtual async System.Threading.Tasks.Task WriteToFileXLS(string path, System.Data.DataSet data)
        {
            await System.Threading.Tasks.Task.Run(() => Graphing.IO.GraphIO.SpreadsheetWriter.Write(path, "", data, new Graphing.IO.SpreadsheetOptions(false, false, 1, 1)));
        }
        protected virtual async System.Threading.Tasks.Task WriteToFileCSV(string path, System.Data.DataSet data)
        {
            foreach (System.Data.DataTable table in data.Tables)
                await WindTunnel.MiniExcelWrapper.WriteToCSV(path.Insert(path.Length - 4, $"_{table.TableName}"), table, false);
        }

        public readonly struct Conditions
        {
            public const float minSpeed = 0.0001f;
            public const float minSpeedPseudoReynolds = 0.1f;

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

            public Conditions(CelestialBody body, float speed, float altitude, bool restrictPseudoReMult = false) : this(body, speed, altitude, false, restrictPseudoReMult) { }
            private Conditions(CelestialBody body, float speed, float altitude, bool speedIsMach, bool restrictPseudoReMult = false)
            {
                this.body = body;
                this.altitude = altitude;

                lock (body)
                {
                    atmPressure = (float)body.GetPressure(altitude);
                    atmDensity = (float)Extensions.KSPClassExtensions.GetDensity(body, altitude);
                    speedOfSound = (float)body.GetSpeedOfSound(atmPressure, atmDensity);
                    oxygenAvailable = body.atmosphereContainsOxygen;
                }
                if (speedIsMach)
                {
                    mach = speed;
                    speed *= speedOfSound;
                }
                else
                {
                    mach = speed / speedOfSound;
                    if (float.IsInfinity(mach))
                        mach = float.MaxValue;
                }
                this.speed = Mathf.Max(speed, minSpeed);

                lock (PhysicsGlobals.DragCurvePseudoReynolds)
                    pseudoReDragMult = PhysicsGlobals.DragCurvePseudoReynolds.Evaluate(atmDensity * (restrictPseudoReMult ? Mathf.Max(speed, minSpeedPseudoReynolds) : speed));
                Q = 0.0005f * atmDensity * this.speed * this.speed;
            }
            public static Conditions ConditionsByMach(CelestialBody body, float mach, float altitude, bool restrictPseudoReMult = false)
                => new Conditions(body, mach, altitude, true, restrictPseudoReMult);
        }
    }
}
