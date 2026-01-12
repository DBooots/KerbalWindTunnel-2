using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace KerbalWindTunnel.Extensions
{
    public static class FloatCurveExtensions
    {
        public static FloatCurve ComputeFloatCurve(IEnumerable<float> keys, Func<float, float> func, float delta = 0.000001f)
            => ComputeFloatCurve(keys.Select(k => (k, false)), func, delta);
        public static FloatCurve ComputeFloatCurve(IEnumerable<(float value, bool continuousDerivative)> keys, Func<float, float> func, float delta = 0.000001f)
        {
            float invDelta = 1 / delta;

            FloatCurve curve = new FloatCurve();

            foreach (var (key, continuousDerivative) in keys)
            {
                float value = func(key);
                float inTangent, outTangent;
                inTangent = -(func(key - delta) - value) * invDelta;
                outTangent = continuousDerivative ? inTangent : (func(key + delta) - value) * invDelta;
                curve.Add(key, value, inTangent, outTangent);
            }
            curve.Curve.keys[0].inTangent = 0;
            curve.Curve.keys[curve.Curve.keys.Length - 1].outTangent = 0;
            return curve;
        }

        public static FloatCurve Clone(this FloatCurve inCurve)
            => new FloatCurve(inCurve.Curve.keys);

        public static IEnumerable<float> ExtractTimes(this FloatCurve curve)
            => curve.Curve.keys.Select(k => k.time);

        public static float EvaluateDerivative(this FloatCurve curve, float time)
        {
            if (curve.Curve.keys.Length <= 1)
                return 0;
            if (time < curve.minTime || time > curve.maxTime)
                return 0;

            int k0Index = FindIndex(curve.Curve.keys, time, out bool exact);
            Keyframe keyframe0 = curve.Curve.keys[k0Index];

            if (time == curve.maxTime)
                return keyframe0.inTangent / 2;

            if (time == curve.minTime)
                return keyframe0.outTangent / 2;

            if (exact)
                return (keyframe0.inTangent + keyframe0.outTangent) / 2;

            Keyframe keyframe1 = curve.Curve.keys[k0Index + 1];
            return EvaluateFloatCurveDerivativeKeyframe(time, keyframe0, keyframe1);
        }
        private static float EvaluateFloatCurveDerivativeKeyframe(float time, Keyframe keyframe0, Keyframe keyframe1)
            => EvaluateBicubicDerivative(time, keyframe0.time, keyframe1.time, keyframe0.value, keyframe0.outTangent, keyframe1.inTangent, keyframe1.value);
        public static float EvaluateBicubicDerivative(float time, float t0, float t1, float value0, float outTangent, float inTangent, float value1)
        {
            float dt = t1 - t0;

            float invDt = 1 / dt;
            float t = (time - t0) * invDt;

            float m0 = outTangent;
            float m1 = inTangent;

            float t2 = t * t;

            float a = 6 * t2 - 6 * t;
            float b = 3 * t2 - 4 * t + 1;
            float c = 3 * t2 - 2 * t;
            float d = -6 * t2 + 6 * t;

            return b * m0 + c * m1 + (a * value0 + d * value1) * invDt;
        }

        private static int FindIndex(Keyframe[] keys, float value, out bool exact)
        {
            Keyframe valueKey = new Keyframe(value, 0);
            int result = Array.BinarySearch(keys, valueKey, keyframeTimeComparer);
            if (result >= 0)
            {
                exact = true;
                return result;
            }
            exact = false;
            return (~result) - 1;
        }

        public static readonly Comparer<Keyframe> keyframeTimeComparer = Comparer<Keyframe>.Create((k1, k2) => k1.time.CompareTo(k2.time));

        public static bool IsAlwaysZero(this FloatCurve curve)
        {
            foreach (Keyframe keyframe in curve.Curve.keys)
                if (keyframe.value != 0 || keyframe.outTangent != 0 || keyframe.inTangent != 0)
                    return false;
            return true;
        }

        public static float EvaluateThreadSafe(this FloatCurve curve, float time)
        {
            lock (curve)
                return curve.Evaluate(time);
#pragma warning disable CS0162 // Unreachable code detected
            if (time <= curve.minTime)
                return curve.Curve.keys[0].value;
            if (time >= curve.maxTime)
                return curve.Curve.keys[curve.Curve.length - 1].value;

            int index0 = FindIndex(curve.Curve.keys, time, out _);
            Keyframe keyframe0 = curve.Curve.keys[index0];
            Keyframe keyframe1 = curve.Curve.keys[index0 + 1];
            return EvaluateFloatCurveKeyframe(time, keyframe0, keyframe1);
#pragma warning restore CS0162 // Unreachable code detected
        }
        private static float EvaluateFloatCurveKeyframe(float time, Keyframe keyframe0, Keyframe keyframe1)
            => EvaluateBicubic(time, keyframe0.time, keyframe1.time, keyframe0.value, keyframe0.outTangent, keyframe1.inTangent, keyframe1.value);
        public static float EvaluateBicubic(float time, float t0, float t1, float value0, float outTangent, float inTangent, float value1)
        {
            float dt = t0 - t1;
            float t = (time - t1) / dt;

            float m0 = outTangent * dt;
            float m1 = inTangent * dt;

            float t2 = t * t;
            float t3 = t2 * t;

            float a = 2 * t3 - 3 * t2 + 1;
            float b = t3 - 2 * t2 + t;
            float c = t3 - t2;
            float d = -2 * t3 + 3 * t2;

            return a * value0 + b * m0 + c * m1 + d * value1;
        }

        public static FloatCurve Superposition(IEnumerable<FloatCurve> curves)
        {
            SortedSet<float> keys_ = new SortedSet<float>();
            foreach (FloatCurve curve in curves)
            {
                if (curve == null)
                    continue;
                keys_.UnionWith(curve.ExtractTimes());
            }
            return Superposition(curves, keys_.ToArray());
        }
        public static FloatCurve Superposition(IEnumerable<FloatCurve> curves, IEnumerable<float> keys)
        {
            float[] keys_ = keys.Distinct().ToArray();
            Array.Sort(keys_);
            return Superposition(curves, keys_);
        }

        public static FloatCurve Superposition(IEnumerable<FloatCurve> curves, IList<float> sortedUniqueKeys)
        {
            FloatCurve result = new FloatCurve();
            int length = sortedUniqueKeys.Count;
            float[] values = new float[length];
            float[] inTangents = new float[length];
            float[] outTangents = new float[length];

            foreach (FloatCurve curve in curves)
            {
                if (curve == null)
                    continue;
                SortedSet<float> curveKeys = new SortedSet<float>(curve.ExtractTimes());

                for (int i = length - 1; i >= 0; i--)
                {
                    float f = sortedUniqueKeys[i];
                    // This curve has this keyframe
                    if (curveKeys.Contains(f))
                    {
                        Keyframe keyframe = curve.Curve.keys.First(k => k.time.Equals(f));
                        inTangents[i] += keyframe.inTangent;
                        outTangents[i] += keyframe.outTangent;
                        values[i] += keyframe.value;
                    }
                    else // Evaluate the curve and use it
                    {
                        float derivative = curve.EvaluateDerivative(f);
                        inTangents[i] += derivative;
                        outTangents[i] += derivative;
                        values[i] += curve.EvaluateThreadSafe(f);
                    }
                }
            }
            for (int i = 0; i < length; i++)
                result.Add(sortedUniqueKeys[i], values[i], inTangents[i], outTangents[i]);
            return result;
        }

        public static FloatCurve Subtract(FloatCurve minuend, FloatCurve subtrahend)
            => Subtract(minuend, Enumerable.Repeat(subtrahend, 1));
        public static FloatCurve Subtract(FloatCurve minuend, IEnumerable<FloatCurve> subtrahends)
        {
            SortedSet<float> keys_ = new SortedSet<float>(minuend.ExtractTimes());
            foreach (FloatCurve curve in subtrahends)
            {
                if (curve == null)
                    continue;
                keys_.UnionWith(curve.ExtractTimes());
            }
            return Subtract(minuend, subtrahends, keys_.ToArray());
        }
        public static FloatCurve Subtract(FloatCurve minuend, IEnumerable<FloatCurve> subtrahends, IEnumerable<float> keys)
        {
            float[] keys_ = keys.Distinct().ToArray();
            Array.Sort(keys_);
            return Subtract(minuend, subtrahends, keys_);
        }

        public static FloatCurve Subtract(FloatCurve minuend, IEnumerable<FloatCurve> subtrahends, IList<float> sortedUniqueKeys)
        {
            if (minuend == null)
                throw new ArgumentNullException(nameof(minuend));

            FloatCurve result = new FloatCurve();
            int length = sortedUniqueKeys.Count;
            float[] values = new float[length];
            float[] inTangents = new float[length];
            float[] outTangents = new float[length];
            
            bool negate = false;

            foreach (FloatCurve curve in Enumerable.Repeat(minuend, 1).Concat(subtrahends))
            {
                int multiplier = negate ? -1 : 1;
                if (curve == null)
                    continue;
                SortedSet<float> curveKeys = new SortedSet<float>(curve.ExtractTimes());

                for (int i = length - 1; i >= 0; i--)
                {
                    float f = sortedUniqueKeys[i];
                    // This curve has this keyframe
                    if (curveKeys.Contains(f))
                    {
                        Keyframe keyframe = curve.Curve.keys.First(k => k.time.Equals(f));
                        inTangents[i] += keyframe.inTangent * multiplier;
                        outTangents[i] += keyframe.outTangent * multiplier;
                        values[i] += keyframe.value * multiplier;
                    }
                    else // Evaluate the curve and use it
                    {
                        float derivative = curve.EvaluateDerivative(f) * multiplier;
                        inTangents[i] += derivative;
                        outTangents[i] += derivative;
                        values[i] += curve.EvaluateThreadSafe(f) * multiplier;
                    }
                }
                negate = true;
            }
            for (int i = 0; i < length; i++)
                result.Add(sortedUniqueKeys[i], values[i], inTangents[i], outTangents[i]);
            return result;
        }

        public static void Scale(this FloatCurve curve, float scalar)
        {
            Keyframe[] keys = curve.Curve.keys;
            for (int i = keys.Length - 1; i >= 0; --i)
            {
                ref Keyframe key = ref keys[i];
                key.value *= scalar;
                key.inTangent *= scalar;
                key.outTangent *= scalar;
            }
            curve.Curve.keys = keys;
        }
        public static void Scale(this FloatCurve curve, Func<Keyframe, float> scalarFunc)
        {
            Keyframe[] keys = curve.Curve.keys;
            for (int i = keys.Length - 1; i >= 0; --i)
            {
                ref Keyframe key = ref keys[i];
                float scalar = scalarFunc(key);
                key.value *= scalar;
                key.inTangent *= scalar;
                key.outTangent *= scalar;
            }
            curve.Curve.keys = keys;
        }
        public static void ScaleTimes(this FloatCurve curve, float scalar)
        {
            Keyframe[] keys = curve.Curve.keys;
            float invScalar = 1 / scalar;
            for (int i = keys.Length - 1; i >= 0; --i)
            {
                ref Keyframe key = ref keys[i];
                key.time *= scalar;
                key.inTangent *= invScalar;
                key.outTangent *= invScalar;
            }
            curve.Curve.keys = keys;
        }
        public static void ScaleTimes(this FloatCurve curve, Func<Keyframe, float> scalarFunc)
        {
            Keyframe[] keys = curve.Curve.keys;
            for (int i = keys.Length - 1; i >= 0; --i)
            {
                ref Keyframe key = ref keys[i];
                float scalar = scalarFunc(key);
                float invScalar = 1 / scalar;
                key.time *= scalar;
                key.inTangent *= invScalar;
                key.outTangent *= invScalar;
            }
            curve.Curve.keys = keys;
        }
        public static FloatCurve ScaledBy(FloatCurve curve, float scalar)
        {
            if (curve == null)
                return null;
            FloatCurve result = curve.Clone();
            result.Scale(scalar);
            return result;
        }
        public static FloatCurve ScaledBy(FloatCurve curve, Func<Keyframe, float> scalarFunc)
        {
            if (curve == null)
                return null;
            FloatCurve result = curve.Clone();
            result.Scale(scalarFunc);
            return result;
        }
        public static FloatCurve TimesScaledBy(FloatCurve curve, float scalar)
        {
            if (curve == null)
                return null;
            FloatCurve result = curve.Clone();
            result.ScaleTimes(scalar);
            return result;
        }
        public static FloatCurve TimeScaledBy(FloatCurve curve, Func<Keyframe, float> scalarFunc)
        {
            if (curve == null)
                return null;
            FloatCurve result = curve.Clone();
            result.ScaleTimes(scalarFunc);
            return result;
        }

        public delegate bool KeyframeMargins(float valueError, float valueAbsoluteError, float tangentDifference, float tangentError);
        internal static bool DefaultMargins(float valueError, float _, float tangentDifference, float tangentError)
        {
            const float stdValueError = 0.005f / 100;
            const float stdTangentDifference = 0.5f / 100;
            const float stdTangentError = 1f / 100;
#if DEBUG
            bool valueValid = Mathf.Abs(valueError) < stdValueError;
            bool tangentValid = Mathf.Abs(tangentDifference) < stdTangentDifference;
            bool tanErrValid = Mathf.Abs(tangentError) < stdTangentError;
#endif
            if (float.IsNaN(valueError) || float.IsNaN(tangentDifference) || float.IsNaN(tangentError))
                return false;
            return Mathf.Abs(valueError) < stdValueError &&
                Mathf.Abs(tangentDifference) < stdTangentDifference &&
                Mathf.Abs(tangentError) < stdTangentError;
        }
        private class KeyframeTimeEqualityComparer : IEqualityComparer<Keyframe>
        {
            public bool Equals(Keyframe x, Keyframe y)
                => x.time.Equals(y.time);
            public int GetHashCode(Keyframe obj)
                => obj.time.GetHashCode();
        }
        private static readonly KeyframeTimeEqualityComparer keyframeTimeEqualityComparer = new KeyframeTimeEqualityComparer();
        public static void Simplify(this FloatCurve curve, KeyframeMargins removalPredicate = null)
        {
            if (removalPredicate == null)
                removalPredicate = DefaultMargins;
            List<Keyframe> keyframes = new List<Keyframe>(curve.Curve.keys);
            for (int i = 1; i < keyframes.Count - 1; i++)
            {
                float computedValue = EvaluateFloatCurveKeyframe(keyframes[i].time, keyframes[i - 1], keyframes[i + 1]);
                float computedTangent = EvaluateFloatCurveDerivativeKeyframe(keyframes[i].time, keyframes[i - 1], keyframes[i + 1]);//(EvaluateFloatCurveKeyframe(keyframes[i].time + delta, keyframes[i - 1], keyframes[i + 1]) - computedValue) / delta;
                float averageTangent = (keyframes[i].inTangent + keyframes[i].outTangent) * 0.5f;
                if (removalPredicate(
                    (computedValue - keyframes[i].value) / keyframes[i].value,
                    computedValue - keyframes[i].value,
                    (keyframes[i].outTangent - keyframes[i].inTangent) / averageTangent,
                    (computedTangent - averageTangent) / averageTangent))
                {
                    keyframes.RemoveAt(i);
                    i--;
                }
            }
            for (int i = curve.Curve.keys.Length - 2; i >= 1; i--)
            {
                if (!keyframes.Contains(curve.Curve.keys[i], keyframeTimeEqualityComparer))
                    curve.Curve.RemoveKey(i);
            }
        }

        public static System.Data.DataTable WriteToDataTable(this FloatCurve curve)
        {
            System.Data.DataTable result = new System.Data.DataTable();
            result.Columns.Add("time", typeof(float));
            result.Columns.Add("value", typeof(float));
            result.Columns.Add("inTangent", typeof(float));
            result.Columns.Add("outTangent", typeof(float));
            foreach (Keyframe keyframe in curve.Curve.keys)
            {
                System.Data.DataRow row = result.Rows.Add();
                row[0] = keyframe.time;
                row[1] = keyframe.value;
                row[2] = keyframe.inTangent;
                row[3] = keyframe.outTangent;
            }
            return result;
        }

        public static System.Data.DataSet WriteToDataSet(this FloatCurve2 curve, System.Data.DataSet dataSet = null, string prefix = "", bool ignoreCross = true)
        {
            if (dataSet == null)
                dataSet = new System.Data.DataSet();

            WriteToDataTable(curve, k => k.value, dataSet.Tables.Add($"{prefix}values"));
            WriteToDataTable(curve, k => k.dDx_in, dataSet.Tables.Add($"{prefix}dDx_in"));
            WriteToDataTable(curve, k => k.dDx_out, dataSet.Tables.Add($"{prefix}dDx_out"));
            WriteToDataTable(curve, k => k.dDy_in, dataSet.Tables.Add($"{prefix}dDy_in"));
            WriteToDataTable(curve, k => k.dDy_out, dataSet.Tables.Add($"{prefix}dDy_out"));
            if (ignoreCross)
                return dataSet;

            WriteToDataTable(curve, k => k.ddDx_in_Dy_in, dataSet.Tables.Add($"{prefix}ddDx_in_Dy_in"));
            WriteToDataTable(curve, k => k.ddDx_out_Dy_in, dataSet.Tables.Add($"{prefix}ddDx_out_Dy_in"));
            WriteToDataTable(curve, k => k.ddDx_in_Dy_out, dataSet.Tables.Add($"{prefix}ddDx_in_Dy_out"));
            WriteToDataTable(curve, k => k.ddDx_out_Dy_out, dataSet.Tables.Add($"{prefix}ddDx_out_Dy_out"));

            return dataSet;

#if OUTSIDE_UNITY
            static
#endif
            void WriteToDataTable(FloatCurve2 fc2, Func<FloatCurve2.Keyframe2, float> selector, System.Data.DataTable table)
            {
                table.Columns.Add("x", typeof(float));
                foreach (float _ in fc2.xKeys)
                    table.Columns.Add().DataType = typeof(float);
                System.Data.DataRow row = table.Rows.Add();

                int upperBoundX = fc2.GetUpperBound(0);
                int upperBoundY = fc2.GetUpperBound(1);

                for (int x = 0; x <= upperBoundX; x++)
                    row[x + 1] = fc2.xKeys[x];

                for (int y = 0; y <= upperBoundY; y++)
                {
                    row = table.Rows.Add();
                    row[0] = fc2.yKeys[y];
                    for (int x = 0; x <= upperBoundX; x++)
                        row[x + 1] = selector(fc2.values[x, y]);
                }
            }
        }
    }
    public class FloatCurveComparer : IEqualityComparer<FloatCurve>
    {
        public static readonly FloatCurveComparer Instance = new FloatCurveComparer();

        public int GetHashCode(FloatCurve curve)
        {
            if (curve == null)
                return HashCode.Combine(curve);
            AnimationCurve ac = curve.Curve;
            HashCode h = new HashCode();
            h.Add(ac.length);
            foreach (Keyframe k in ac.keys)
            {
                h.Add(k.time);
                h.Add(k.value);
                h.Add(k.inTangent);
                h.Add(k.outTangent);
            }
            return h.ToHashCode();
        }

        public bool Equals(FloatCurve curve1, FloatCurve curve2)
        {
            if (curve1 == null && curve2 == null)
                return true;
            if (curve1 == null || curve2 == null)
                return false;
            if (ReferenceEquals(curve1, curve2))
                return true;
            if (curve1.Curve.length != curve2.Curve.length)
                return false;
            AnimationCurve ac1 = curve1.Curve, ac2 = curve2.Curve;
            for (int i = ac1.length - 1; i >= 0; i--)
            {
                Keyframe k1 = ac1.keys[i], k2 = ac2.keys[i];
                if (k1.time != k2.time ||
                    k1.value != k2.value ||
                    k1.inTangent != k2.inTangent ||
                    k1.outTangent != k2.outTangent)
                    return false;
            }
            return true;
        }
    }
}
