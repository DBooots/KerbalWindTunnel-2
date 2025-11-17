using System;
using UnityEngine;
using Accord.Math.Optimization;
using static KerbalWindTunnel.AeroPredictor;

namespace KerbalWindTunnel.VesselCache
{
    public static class AeroOptimizer
    {
        public const float defaultAoATolerance = 0.05f * Mathf.Deg2Rad;
        public const float defaultAoAOptTolerance = 1E-5f;
        public const float defaultInputTolerance = 0.01f;
        public const float levelGuessRange = 5 * Mathf.Deg2Rad;

        private static readonly int[] levelFlightStepdownValues = new int[] { -5, -10, -20, -40, -90 };

        static readonly Unity.Profiling.ProfilerMarker s_findMax = new Unity.Profiling.ProfilerMarker("AeroOptimizer.FindMaxAoA");
        static readonly Unity.Profiling.ProfilerMarker s_findMin = new Unity.Profiling.ProfilerMarker("AeroOptimizer.FindMinAoA");
        static readonly Unity.Profiling.ProfilerMarker s_findLevel = new Unity.Profiling.ProfilerMarker("AeroOptimizer.FindLevelAoA");
        static readonly Unity.Profiling.ProfilerMarker s_findInput = new Unity.Profiling.ProfilerMarker("AeroOptimizer.FindStablePitchInput");

        public static float FindMaxAoA(this AeroPredictor predictor, Conditions conditions, out float lift, float guess = float.NaN, float tolerance = defaultAoAOptTolerance)
        {
            const float pitchInput = 0;
            float result;
            s_findMax.Begin();
            if (predictor is IDirectAoAMaxProvider aoaProvider && aoaProvider.DirectAoAInitialized)
            {
                result = aoaProvider.GetAoAMax(conditions);
                lift = (float)predictor.AerodynamicObjectiveFunc(conditions, pitchInput)(result);
            }
            else
            {
                IOptimizationMethod<double, double> optimizationMethod;
                if (predictor is ILiftAoADerivativePredictor derivativePredictor)
                    optimizationMethod = FindMaxAoA_NoDerivative(predictor.AerodynamicObjectiveFunc(conditions, pitchInput, -1), guess, tolerance);
                else
                    optimizationMethod = FindMaxAoA_NoDerivative(predictor.AerodynamicObjectiveFunc(conditions, pitchInput, -1), guess, tolerance);
                lift = -(float)optimizationMethod.Value;
                result = (float)optimizationMethod.Solution;
            }
            s_findMax.End();
            return result;
        }
        private static IOptimizationMethod<double, double> FindMaxAoA_NoDerivative(Func<double, double> objectiveFunc, float guess, float tolerance)
        {
            const float lowerBound = 10 * Mathf.Deg2Rad;
            const float midUpperBound = 60 * Mathf.Deg2Rad;
            const float upperBound = 80 * Mathf.Deg2Rad;
            const float stage1 = 5 * Mathf.Deg2Rad;
            const float stage2 = 10 * Mathf.Deg2Rad;
            const float stage3 = 20 * Mathf.Deg2Rad;

            BrentSearch optimizer = new BrentSearch(objectiveFunc, lowerBound, upperBound, tolerance);
            if (float.IsNaN(guess) || float.IsInfinity(guess))
                optimizer.Minimize();
            else
            {
                guess = Mathf.Clamp(guess, -90 * Mathf.Deg2Rad, 90 * Mathf.Deg2Rad);
                (float, float, float) lowers = (guess - stage1, guess - stage2, Mathf.Min(lowerBound, guess - stage3));
                (float, float, float) uppers = (guess + stage1, guess + stage2, Mathf.Clamp(guess + stage3, midUpperBound, upperBound));
                SequentialBrentMinimize(optimizer, lowers, uppers);
            }
            return optimizer;
        }
        private static void SequentialBrentMinimize(BrentSearch optimizer, (float, float, float) lowers, (float, float, float) uppers)
        {
            optimizer.LowerBound = lowers.Item1;
            optimizer.UpperBound = uppers.Item1;
            if (optimizer.Minimize())
                return;

            optimizer.LowerBound = lowers.Item2;
            optimizer.UpperBound = uppers.Item2;
            if (optimizer.Minimize())
                return;

            optimizer.LowerBound = lowers.Item3;
            optimizer.UpperBound = lowers.Item3;
            optimizer.Minimize();

            return;
        }

        public static float FindMinAoA(this AeroPredictor predictor, Conditions conditions, float guess = float.NaN, float tolerance = defaultAoAOptTolerance)
        {
            const float pitchInput = 0;
            s_findMin.Begin();
            IOptimizationMethod<double, double> optimizationMethod;
            if (predictor is ILiftAoADerivativePredictor derivativePredictor)
                optimizationMethod = FindMinAoA_NoDerivative(predictor.AerodynamicObjectiveFunc(conditions, pitchInput), guess, tolerance);
            else
                optimizationMethod = FindMinAoA_NoDerivative(predictor.AerodynamicObjectiveFunc(conditions, pitchInput), guess, tolerance);
            s_findMin.End();
            return (float)optimizationMethod.Solution;
        }
        private static IOptimizationMethod<double, double> FindMinAoA_NoDerivative(Func<double, double> objectiveFunc, float guess, float tolerance)
        {
            const float lowerBound = -80 * Mathf.Deg2Rad;
            const float midLowerBound = -60 * Mathf.Deg2Rad;
            const float upperBound = -10 * Mathf.Deg2Rad;
            const float stage1 = 5 * Mathf.Deg2Rad;
            const float stage2 = 10 * Mathf.Deg2Rad;
            const float stage3 = 20 * Mathf.Deg2Rad;

            BrentSearch optimizer = new BrentSearch(objectiveFunc, lowerBound, upperBound, tolerance);
            if (float.IsNaN(guess) || float.IsInfinity(guess))
                optimizer.Minimize();
            else
            {
                guess = Mathf.Clamp(guess, -90 * Mathf.Deg2Rad, 90 * Mathf.Deg2Rad);
                (float, float, float) lowers = (guess - stage1, guess - stage2, Mathf.Clamp(guess - stage3, midLowerBound, lowerBound));
                (float, float, float) uppers = (guess + stage1, guess + stage2, Mathf.Max(guess + stage3, upperBound));
                SequentialBrentMinimize(optimizer, lowers, uppers);
            }
            return optimizer;
        }

        public static float FindLevelAoA(this AeroPredictor predictor, Conditions conditions, float offsettingForce, float guess = 0, float pitchInput = 0, float tolerance = defaultAoATolerance)
        {
            s_findLevel.Begin();
            if (float.IsNaN(guess) || float.IsInfinity(guess))
                guess = 0;
            else
                guess = Mathf.Clamp(guess, -90 * Mathf.Deg2Rad, 90 * Mathf.Deg2Rad);
            IOptimizationMethod<double, double> optimizationMethod;
            if (predictor is ILiftAoADerivativePredictor derivativePredictor)
                optimizationMethod = FindLevelAoA_NoDerivative(predictor.LevelFlightObjectiveFunc(conditions, offsettingForce, pitchInput), guess, tolerance);
            else
                optimizationMethod = FindLevelAoA_NoDerivative(predictor.LevelFlightObjectiveFunc(conditions, offsettingForce, pitchInput), guess, tolerance);
            s_findLevel.End();
            return (float)optimizationMethod.Solution;
        }
        private static IOptimizationMethod<double, double> FindLevelAoA_NoDerivative(Func<double, double> objectiveFunc, float guess, float tolerance)
        {
            const float lowerBound = -10 * Mathf.Deg2Rad;
            const float upperBound = 35 * Mathf.Deg2Rad;
            const float step = 5 * Mathf.Deg2Rad;
            guess = Mathf.Clamp(guess, -90 * Mathf.Deg2Rad + step, 90 * Mathf.Deg2Rad - step);

            BrentSearch solver = new BrentSearch(objectiveFunc, lowerBound, upperBound, tolerance);
            SequentialBrentSearch(solver, (guess - step, lowerBound), (guess + step, upperBound));
            return solver;
        }
        private static void SequentialBrentSearch(BrentSearch solver, (float, float) lowers, (float, float) uppers)
        {
            solver.LowerBound = lowers.Item1;
            solver.UpperBound = uppers.Item1;
            if (solver.FindRoot())
                return;

            solver.LowerBound = lowers.Item2;
            solver.UpperBound = uppers.Item2;
            solver.FindRoot();

            return;
        }

        public static float FindLevelAoAWithControl(this AeroPredictor predictor, Conditions conditions, float offsettingForce, float aoaGuess = 0, float inputGuess = 0, bool dryTorque = false, float aoaTolerance = defaultAoATolerance, float inputTolerance = defaultInputTolerance)
        {
            if (float.IsNaN(aoaGuess) || float.IsInfinity(aoaGuess))
                aoaGuess = 0;
            else
                aoaGuess = Mathf.Clamp(aoaGuess, -90 * Mathf.Deg2Rad, 90 * Mathf.Deg2Rad);
            if (float.IsNaN(inputGuess) || float.IsInfinity(inputGuess))
                inputGuess = 0;
            else
                inputGuess = Mathf.Clamp01(inputGuess);

            float approxAoA = FindLevelAoA(predictor, conditions, offsettingForce, aoaGuess, aoaTolerance);
            float approxInput = FindStablePitchInput(predictor, conditions, approxAoA, inputGuess, dryTorque, inputTolerance);

            float pitchFun(float aoa) => FindStablePitchInput(predictor, conditions, aoa, approxInput, dryTorque, inputTolerance);
            Func<double, double> objectiveFunc = predictor.LevelFlightObjectiveFunc(conditions, offsettingForce, pitchFun);

            IOptimizationMethod<double, double> optimizationMethod;
            if (predictor is ILiftAoADerivativePredictor derivativePredictor)
                optimizationMethod = FindLevelAoA_NoDerivative(objectiveFunc, approxAoA, aoaTolerance);
            else
                optimizationMethod = FindLevelAoA_NoDerivative(objectiveFunc, approxAoA, aoaTolerance);
            return (float)optimizationMethod.Solution;
        }

        public static float FindStablePitchInput(this AeroPredictor predictor, Conditions conditions, float aoa, float guess = 0, bool dryTorque = false, float tolerance = defaultAoATolerance)
        {
            s_findInput.Begin();
            IOptimizationMethod<double, double> optimizationMethod;
            if (float.IsNaN(guess) || float.IsInfinity(guess))
                guess = 0;
            else
                guess = Mathf.Clamp01(guess);
            optimizationMethod = FindStablePitchInput_NoDerivative(predictor.PitchInputObjectiveFunc(conditions, aoa, dryTorque), guess, tolerance, out bool succeeded);
            s_findInput.End();
            if (succeeded)
                return (float)optimizationMethod.Solution;
            else if (predictor.GetAeroTorque(conditions, aoa, 0, dryTorque).x > 0)
                return -1;
            else
                return 1;
        }
        private static IOptimizationMethod<double, double> FindStablePitchInput_NoDerivative(Func<double, double> objectiveFunc, float guess, float tolerance, out bool succeeded)
        {
            BrentSearch solver = new BrentSearch(objectiveFunc, -1, 1, tolerance);
            SequentialBrentSearch(solver, (Mathf.Clamp01(guess - 0.3f), -1), (Mathf.Clamp01(guess + 0.3f), 1));
            succeeded = solver.Status == BrentSearchStatus.Success;
            return solver;
        }

        public struct KeyAoAData
        {
#if OUTSIDE_UNITY
            public float zeroLift = float.NaN;
            public float maxLift = float.NaN;
            public float maxLiftForce = float.NaN;
            public float levelFlight = float.NaN;
            public float levelFlightResidual = 0;
            public KeyAoAData() { }
#else
            public float zeroLift;
            public float maxLift;
            public float maxLiftForce;
            public float levelFlight;
            public float levelFlightResidual;
#endif
        }

        public static KeyAoAData SolveLevelFlight(this AeroPredictor predictor, Conditions conditions, float weight, KeyAoAData? knowns, KeyAoAData? guess = null)
        {
            // 1. If we have a guess, it's probably within +/- 5 degrees
            // If it's not, the guess is bad and we'll search from scratch

            // 2. Zero degrees is probably a good starting lower bound, and maxLift is a probably good upper bound.
            // This requires knowing maxLift.

            // 3. If that does not bracket the root, there are two scenarios:
            // A. There is a positive incidence on either wings or engines and the root is below zero.
            // B. There is a negative incidence on the engines and/or the root is above maxLift.
            // We can know the difference based on the sign of the objective function at any point in the bracket:
            // + and it's A, - and it's B.

            // In the case of A. The zero lift angle can't be far, so step down in increments.
            // In the case of B:
            //      If thrust is constant with AoA and T > W, we know there's probably a root at or below the angle at which Ty = W.
            //      Otherwise, find the maxima of the level flight objective function on (maxLift, 90)
            //      I refuse to condone anything that needs greater than 90 AoA for level flight. If that's you, fix your design.
            //      If this is positive, this is the upper bound.
            //      If this is negative, level flight is not possible and this is the value.
            const float defaultLowerBound = -5 * Mathf.Deg2Rad;
            KeyAoAData knowns_ = knowns ?? new KeyAoAData();

            Func<double, double> levelObjFunc = predictor.LevelFlightObjectiveFunc(conditions, weight);
            if (!float.IsNaN(knowns_.levelFlight))
            {
                knowns_.levelFlightResidual = -(float)levelObjFunc(knowns_.levelFlight);
                return knowns_;
            }
            BrentSearch rootFinder = new BrentSearch(levelObjFunc, 0, 35, defaultAoATolerance);

            // 1. Use a good guess, if we have it.
            float levelGuess = guess?.levelFlight ?? float.NaN;
            if (!float.IsNaN(levelGuess))
            {
                rootFinder.LowerBound = levelGuess - levelGuessRange;
                rootFinder.UpperBound = levelGuess + levelGuessRange;
                if (rootFinder.FindRoot())
                {
                    knowns_.levelFlight = (float)rootFinder.Solution;
                    return knowns_;
                }
            }

            // 2. Use a reasonable bracket.
            if (float.IsNaN(knowns_.maxLift))
                knowns_.maxLift = predictor.FindMaxAoA(conditions, out knowns_.maxLiftForce, guess?.maxLift ?? float.NaN);
            rootFinder.LowerBound = defaultLowerBound;
            rootFinder.UpperBound = knowns_.maxLift;
            if (rootFinder.FindRoot())
            {
                knowns_.levelFlight = (float)rootFinder.Solution;
                return knowns_;
            }

            // 3. The root was not bracketed.
            if (Math.Sign(rootFinder.Value) > 0)
            {
                // A. The root must exist and is below zero degrees. Step down by reasonable increments.
                for (int i = 0; i < levelFlightStepdownValues.Length; i++)
                {
                    rootFinder.UpperBound = rootFinder.LowerBound;
                    rootFinder.LowerBound = levelFlightStepdownValues[i] * Mathf.Deg2Rad;
                    if (rootFinder.FindRoot())
                        break;
                }
                knowns_.levelFlight = (float)rootFinder.Solution;
                return knowns_;
            }
            // B. The root is above AoA max or does not exist.
            rootFinder.LowerBound = knowns_.maxLift;
            float thrustIncidence = 0;
            // Check if there's a root within (aoaMax, aoa => Ty >= W) when T >= W
            if (predictor.ThrustIsConstantWithAoA)
            {
                Vector3 thrust = predictor.GetThrustForce2D(conditions);
                float thrustMag = thrust.magnitude;
                thrustIncidence = Mathf.Atan2(thrust.y, thrust.x);
                if (thrustMag > weight)
                {
                    rootFinder.UpperBound = Math.Sign(thrust.x) * Math.Asin(weight / thrustMag) - thrustIncidence;
                    if (rootFinder.UpperBound > rootFinder.LowerBound && rootFinder.FindRoot())
                    {
                        knowns_.levelFlight = (float)rootFinder.Solution;
                        return knowns_;
                    }
                    rootFinder.LowerBound = Math.Max(rootFinder.LowerBound, rootFinder.UpperBound);
                }
            }
            // Checking if the root exists by finding the maxima.
            // Unless the lift curve shape is exceptionally weird the lift function should be monotonic, so maximizing will converge.
            rootFinder.UpperBound = 90 * Mathf.Deg2Rad;
            if (Math.Sign(thrustIncidence) < 0)
                rootFinder.UpperBound -= thrustIncidence;   // Only if the engines have a negative thrust angle can the upper bound be greater than 90.
            rootFinder.Tolerance = defaultAoAOptTolerance;
            rootFinder.Maximize();

            if (rootFinder.Value < 0)
            {
                // The root does not exist.
                knowns_.levelFlight = (float)rootFinder.Solution;
                knowns_.levelFlightResidual = -(float)rootFinder.Value;
                return knowns_;
            }
            // The root does exist.
            rootFinder.UpperBound = rootFinder.Solution;
            rootFinder.FindRoot();
            knowns_.levelFlight = (float)rootFinder.Solution;
            return knowns_;
        }

        public interface ILiftAoADerivativePredictor
        {
            float GetLiftForceMagnitudeAoADerivative(Conditions conditions, float AoA, float pitchInput = 0);
        }

        public interface IDirectAoAMaxProvider
        {
            bool DirectAoAInitialized { get; }
            float GetAoAMax(Conditions conditions);
        }
    }
}
