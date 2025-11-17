using System;
using System.Collections.Generic;
using UnityEngine;
using Smooth.Pools;
using KerbalWindTunnel.Extensions;
using static KerbalWindTunnel.VesselCache.AeroOptimizer;

namespace KerbalWindTunnel.VesselCache
{
    public class SimulatedVessel : AeroPredictor, IReleasable, IDirectAoAMaxProvider
    {
        static readonly Unity.Profiling.ProfilerMarker s_getLiftMarker = new Unity.Profiling.ProfilerMarker("SimulatedVessel.GetLiftForce");
        static readonly Unity.Profiling.ProfilerMarker s_getAeroMarker = new Unity.Profiling.ProfilerMarker("SimulatedVessel.GetAeroForce");

        /*RootSolverSettings pitchInputSolverSettings = new RootSolverSettings(
            RootSolver.LeftBound(-1),
            RootSolver.RightBound(1),
            RootSolver.LeftGuessBound(-0.25f),
            RootSolver.RightGuessBound(0.25f),
            RootSolver.ShiftWithGuess(true),
            RootSolver.Tolerance(0.01f));

        RootSolverSettings coarseAoASolverSettings = new RootSolverSettings(
            WindTunnelWindow.Instance.solverSettings,
            RootSolver.Tolerance(1 * Mathf.PI / 180));

        RootSolverSettings fineAoASolverSettings = new RootSolverSettings(
            WindTunnelWindow.Instance.solverSettings,
            RootSolver.LeftGuessBound(-2 * Mathf.PI / 180),
            RootSolver.RightGuessBound(2 * Mathf.PI / 180),
            RootSolver.ShiftWithGuess(true));*/

        public PartCollection partCollection;

        private int count;
        public float totalMass = 0;
        public float dryMass = 0;
        public float relativeWingArea = 0;
        public int stage = 0;
        
        public FloatCurve DragCurvePseudoReynolds;
        public FloatCurve maxAoA = null;
        public static SortedSet<float> AoAMachs = null;

        public override AeroPredictor GetThreadSafeObject() => BorrowClone(this);

        public override float Mass { get => totalMass; }

        public override bool ThrustIsConstantWithAoA => partCollection.partCollections.Count == 0;

        public override float Area => relativeWingArea;

        public Vector3 GetAeroForce(Conditions conditions, float AoA, float pitchInput, out Vector3 torque, Vector3 torquePoint)
        {
            s_getAeroMarker.Begin();
            Vector3 result = partCollection.GetAeroForce(InflowVect(AoA) * conditions.speed, conditions, pitchInput, out torque, torquePoint);
            s_getAeroMarker.End();
            return result;
        }
        public override Vector3 GetAeroForce(Conditions conditions, float AoA, float pitchInput = 0)
        => GetAeroForce(conditions, AoA, pitchInput, out _, Vector3.zero);

        public Vector3 GetLiftForce(Conditions conditions, float AoA, float pitchInput, out Vector3 torque, Vector3 torquePoint)
        {
            s_getLiftMarker.Begin();
            Vector3 result = partCollection.GetLiftForce(InflowVect(AoA) * conditions.speed, conditions, pitchInput, out torque, torquePoint);
            s_getLiftMarker.End();
            return result;
        }
        public override Vector3 GetLiftForce(Conditions conditions, float AoA, float pitchInput = 0)
            => GetLiftForce(conditions, AoA, pitchInput, out _, Vector3.zero);

        public override Func<double, double> PitchInputObjectiveFunc(Conditions conditions, float aoa, bool dryTorque = false)
        {
            Vector3 inflow = InflowVect(aoa) * conditions.speed;
            partCollection.GetAeroForceStatic(inflow, conditions, out Vector3 staticTorque, dryTorque ? CoM_dry : CoM);
            float staticPitchTorque = staticTorque.x;
            return (input) =>
            {
                partCollection.GetAeroForceDynamic(inflow, conditions, (float)input, out Vector3 torque, dryTorque ? CoM_dry : CoM);
                return torque.x + staticPitchTorque;
            };
        }

        public override Vector3 GetAeroTorque(Conditions conditions, float AoA, float pitchInput = 0, bool dryTorque = false)
        {
            GetAeroForce(conditions, AoA, pitchInput, out Vector3 torque, dryTorque ? CoM_dry : CoM);
            return torque;
        }
        
        public override void GetAeroCombined(Conditions conditions, float AoA, float pitchInput, out Vector3 forces, out Vector3 torques, bool dryTorque = false)
        {
            forces = GetAeroForce(conditions, AoA, pitchInput, out torques, dryTorque ? CoM_dry : CoM);
        }

        public override Vector3 GetThrustForce(Conditions conditions, float AoA)
        {
            Vector3 inflow = InflowVect(AoA) * conditions.speed;
            return partCollection.GetThrustForce(inflow, conditions);
        }
        
        public override float GetFuelBurnRate(Conditions conditions, float AoA)
        {
            Vector3 inflow = InflowVect(AoA) * conditions.speed;
            return partCollection.GetFuelBurnRate(inflow, conditions);
        }

        private static readonly Pool<SimulatedVessel> pool = new Pool<SimulatedVessel>(Create, Reset);

        private static SimulatedVessel Create()
        {
            return new SimulatedVessel();
        }

        public void Release()
        {
            lock (pool)
                pool.Release(this);
        }

        private static void Reset(SimulatedVessel obj)
        {
            obj.partCollection.Release();
            obj.partCollection = null;
            obj.maxAoA = null;
        }

        public static SimulatedVessel Borrow(IShipconstruct v)
        {
            SimulatedVessel vessel;
            // This lock is more expansive than it needs to be.
            // There is still a race condition within Init that causes
            // extra drag in the simulation, but this section is not a
            // performance bottleneck and so further refinement is #TODO.
            lock (pool)
            {
                vessel = pool.Borrow();
                vessel.Init(v);
            }
            return vessel;
        }

        public static SimulatedVessel BorrowClone(SimulatedVessel vessel)
        {
            SimulatedVessel clone;
            lock (pool)
                clone = pool.Borrow();
            clone.InitClone(vessel);
            return clone;
        }

        private void Init(IShipconstruct v)
        {
            if (DragCurvePseudoReynolds == null)
                DragCurvePseudoReynolds = PhysicsGlobals.DragCurvePseudoReynolds.Clone();
            totalMass = 0;
            dryMass = 0;
            CoM = Vector3.zero;
            CoM_dry = Vector3.zero;
            relativeWingArea = 0;
            stage = 0;

            List<Part> oParts = v.Parts;
            List<SimulatedPart> variableDragParts_ctrls = new List<SimulatedPart>();
            count = oParts.Count;

            if (HighLogic.LoadedSceneIsEditor)
            {
                for (int i = 0; i < v.Parts.Count; i++)
                {
                    Part p = v.Parts[i];
                    if (p.dragModel == Part.DragModel.CUBE && !p.DragCubes.None)
                    {
                        DragCubeList cubes = p.DragCubes;
                        lock (cubes)
                        {
                            DragCubeList.CubeData p_drag_data = new DragCubeList.CubeData();

                            try
                            {
                                cubes.SetDragWeights();
                                cubes.SetPartOcclusion();
                                cubes.AddSurfaceDragDirection(-Vector3.forward, 0, ref p_drag_data);
                            }
                            catch (NullReferenceException nre)
                            {
                                cubes.SetDrag(Vector3.forward, 0);
                                cubes.ForceUpdate(true, true);
                                cubes.SetDragWeights();
                                cubes.SetPartOcclusion();
                                cubes.AddSurfaceDragDirection(-Vector3.forward, 0, ref p_drag_data);
                                Debug.LogError(string.Format("[KWT] Caught NRE on Drag Initialization.  Should be fixed now.  {0}", nre));
                            }
                        }
                    }
                }
            }

            bool lgWarning = false;
            for (int i = 0; i < count; i++)
            {
                if (!lgWarning)
                {
                    ModuleWheels.ModuleWheelDeployment gear = oParts[i].FindModuleImplementing<ModuleWheels.ModuleWheelDeployment>();
                    bool forcedRetract = !oParts[i].ShieldedFromAirstream && gear != null && gear.Position > 0;

                    if (forcedRetract)
                        lgWarning = true;
                }
            }

            // Recursively add all parts to collections
            // What a crazy change to make just to accomodate rotating parts!
            partCollection = PartCollection.BorrowWithoutAdding(this);
            Part root = oParts[0];
            while (root.parent != null)
                root = root.parent;
            partCollection.AddPartRecursive(root);

            CoM /= totalMass;
            CoM_dry /= dryMass;

            partCollection.origin = CoM;

            if (relativeWingArea == 0)
            {
                ScreenMessages.PostScreenMessage("#autoLOC_KWT209", 5, ScreenMessageStyle.UPPER_CENTER);   // "No wings found, using a reference area of 1."
                relativeWingArea = 1;
            }
            if (v is ShipConstruct construct)
                MAC = construct.shipSize.x;
            else if (v is Vessel vessel)
                MAC = vessel.vesselSize.x;
            else
                MAC = 1;
            MAC = relativeWingArea / MAC;

            //if (lgWarning)
                //ScreenMessages.PostScreenMessage("#autoLOC_KWT210", 5, ScreenMessageStyle.UPPER_CENTER);  // "Landing gear deployed, results may not be accurate."
        }

        private void InitClone(SimulatedVessel vessel)
        {
            if (DragCurvePseudoReynolds == null)
                DragCurvePseudoReynolds = PhysicsGlobals.DragCurvePseudoReynolds.Clone();
            totalMass = vessel.totalMass;
            dryMass = vessel.dryMass;
            CoM = vessel.CoM;
            CoM_dry = vessel.CoM_dry;
            relativeWingArea = vessel.relativeWingArea;
            stage = vessel.stage;
            count = vessel.count;
            maxAoA = vessel.maxAoA?.Clone();

            partCollection = PartCollection.BorrowClone(this, vessel);
        }

        public void InitMaxAoA()
        {
            // If there are rotating parts, this won't ever come in handy so it's not worth the time.
            // Except, it turns out that mach number isn't calculated per-part, but is a vessel-wide number.
            //if (partCollection.partCollections.Count > 0)
                //return;
            const float machStep = 0.1f;
            DirectAoAInitialized = false;
#if ENABLE_PROFILER
            UnityEngine.Profiling.Profiler.BeginSample("SimulatedVessel.InitMaxAoA");
#endif
            FindAoAMachs();

            Conditions baseConditions = new Conditions(WindTunnelWindow.Instance.CelestialBody ?? Planetarium.fetch.Home, 0, 0);

            float GetAoAMax(float mach)
            {
                Conditions conditions = new Conditions(baseConditions.body, baseConditions.speedOfSound * mach, 0);
                return this.FindMaxAoA(conditions, out float lift, 30 * Mathf.Deg2Rad);
            }

            maxAoA = KSPClassExtensions.ComputeFloatCurve(AoAMachs, GetAoAMax, machStep);

#if ENABLE_PROFILER
            UnityEngine.Profiling.Profiler.EndSample();
#endif
            DirectAoAInitialized = true;
        }

        private void FindAoAMachs()
        {
            if (AoAMachs != null)
                return;

            AoAMachs = new SortedSet<float>();
            foreach (var curve in PhysicsGlobals.LiftingSurfaceCurves.Values)
                AoAMachs.UnionWith(curve.liftMachCurve.ExtractTimes());
        }

        public float GetAoAMax(Conditions conditions)
            => maxAoA.Evaluate(conditions.mach);

        public bool DirectAoAInitialized { get; protected set; } = false;

        protected override System.Data.DataSet WriteToDataSet()
        {
            System.Data.DataSet data = new System.Data.DataSet();
            System.Data.DataTable drag = data.Tables.Add("Drag");
            System.Data.DataTable lift = data.Tables.Add("Lift");

            drag.Columns.Add("AoA", typeof(float));
            lift.Columns.Add("AoA", typeof(float));

            foreach (float _ in AoAMachs)
            {
                drag.Columns.Add().DataType = typeof(float);
                lift.Columns.Add().DataType = typeof(float);
            }
            drag.Rows.Add();
            lift.Rows.Add();
            int m = 1;
            foreach (float mach in AoAMachs)
            {
                drag.Rows[0][m] = lift.Rows[0][m] = mach;
                m++;
            }

            Conditions baseConditions = new Conditions(WindTunnelWindow.Instance.CelestialBody ?? Planetarium.fetch.Home, 0, 0);

            for (int i = -180; i <= 180; i += 5)
            {
                System.Data.DataRow dragRow = drag.Rows.Add();
                System.Data.DataRow liftRow = lift.Rows.Add();
                float aoa = i * Mathf.Deg2Rad;
                dragRow[0] = liftRow[0] = aoa;
                m = 1;
                foreach (float mach in AoAMachs)
                {
                    float speed = baseConditions.speedOfSound * mach;
                    if (speed == 0)
                        speed = 0.001f;
                    Conditions conditions = new Conditions(baseConditions.body, speed, 0);
                    Vector3 force = GetAeroForce(conditions, aoa);
                    dragRow[m] = GetDragForceComponent(force, aoa) / conditions.Q;
                    liftRow[m] = GetLiftForceComponent(force, aoa) / conditions.Q;
                    m++;
                }
            }
            data.AcceptChanges();

            return data;
        }
    }
}
