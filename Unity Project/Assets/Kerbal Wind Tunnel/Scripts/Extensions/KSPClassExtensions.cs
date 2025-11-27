using System;
using System.Collections.Generic;
using UnityEngine;

namespace KerbalWindTunnel.Extensions
{
    public static class KSPClassExtensions
    {
        /*
        From Trajectories
        Copyright 2014, Youen Toupin
        This method is part of Trajectories, under MIT license.
        StockAeroUtil by atomicfury
        */
        /// <summary>
        /// Gets the air density (rho) for the specified altitude on the specified body.
        /// This is an approximation, because actual calculations, taking sun exposure into account to compute air temperature, require to know the actual point on the body where the density is to be computed (knowing the altitude is not enough).
        /// However, the difference is small for high altitudes, so it makes very little difference for trajectory prediction.
        /// From StockAeroUtil.cs from Trajectories
        /// </summary>
        /// <param name="body"></param>
        /// <param name="altitude">Altitude above sea level (in meters)</param>
        /// <returns></returns>
        public static double GetDensity(this CelestialBody body, double altitude)
        {
            if (!body.atmosphere)
                return 0;

            if (altitude > body.atmosphereDepth)
                return 0;

            double pressure = body.GetPressure(altitude);

            // get an average day/night temperature at the equator
            double sunDot = 0.5;
            float sunAxialDot = 0;
            double atmosphereTemperatureOffset = (double)body.latitudeTemperatureBiasCurve.Evaluate(0)
                + (double)body.latitudeTemperatureSunMultCurve.Evaluate(0) * sunDot
                + (double)body.axialTemperatureSunMultCurve.Evaluate(sunAxialDot);
            double temperature = // body.GetFullTemperature(altitude, atmosphereTemperatureOffset);
                body.GetTemperature(altitude)
                + (double)body.atmosphereTemperatureSunMultCurve.Evaluate((float)altitude) * atmosphereTemperatureOffset;


            return body.GetDensity(pressure, temperature);
        }

        public static Vector3 ProjectOnPlaneSafe(Vector3 vector, Vector3 planeNormal)
        {
            return vector - Vector3.Dot(vector, planeNormal) / planeNormal.sqrMagnitude * planeNormal;
        }
    }

    public class PartDragCurveComparer : IEqualityComparer<DragCubeList>
    {
        public static PartDragCurveComparer Instance = new PartDragCurveComparer();

        public bool Equals(DragCubeList cubes1, DragCubeList cubes2)
        {
            FloatCurveComparer curveComparer = FloatCurveComparer.Instance;

            PhysicsGlobals.SurfaceCurvesList surfCurvesList1 = cubes1.SurfaceCurves, surfCurvesList2 = cubes2.SurfaceCurves;
            if (!curveComparer.Equals(surfCurvesList1.dragCurveTip, surfCurvesList2.dragCurveTip))
                return false;
            if (!curveComparer.Equals(surfCurvesList1.dragCurveSurface, surfCurvesList2.dragCurveSurface))
                return false;
            if (!curveComparer.Equals(surfCurvesList1.dragCurveTail, surfCurvesList2.dragCurveTail))
                return false;
            if (!curveComparer.Equals(surfCurvesList1.dragCurveMultiplier, surfCurvesList2.dragCurveMultiplier))
                return false;

            FloatCurve dragMult1 = cubes1.DragCurveMultiplier, dragMult2 = cubes2.DragCurveMultiplier;
            if (!curveComparer.Equals(dragMult1, dragMult2))
                return false;

            FloatCurve cdPower1 = cubes1.DragCurveCdPower, cdPower2 = cubes2.DragCurveCdPower;
            if (!curveComparer.Equals(cdPower1, cdPower2))
                return false;

            return true;
        }

        public int GetHashCode(DragCubeList cubes)
        {
            HashCode hashCode = new HashCode();
            FloatCurveComparer curveComparer = FloatCurveComparer.Instance;
            PhysicsGlobals.SurfaceCurvesList surfCurvesList = cubes.SurfaceCurves;

            hashCode.Add(surfCurvesList.dragCurveTip, curveComparer);
            hashCode.Add(surfCurvesList.dragCurveSurface, curveComparer);
            hashCode.Add(surfCurvesList.dragCurveTail, curveComparer);
            hashCode.Add(surfCurvesList.dragCurveMultiplier, curveComparer);
            if (!curveComparer.Equals(surfCurvesList.dragCurveMultiplier, cubes.DragCurveMultiplier))
                hashCode.Add(cubes.DragCurveMultiplier);
            hashCode.Add(cubes.DragCurveCdPower);

            return hashCode.ToHashCode();
        }
    }
}
