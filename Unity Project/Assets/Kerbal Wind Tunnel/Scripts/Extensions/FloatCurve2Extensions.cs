using System;
using System.Collections.Generic;
using UnityEngine;
using static KerbalWindTunnel.FloatCurve2;

namespace KerbalWindTunnel.Extensions
{
    public static class FloatCurve2Extensions
    {
        public static FloatCurve2 Simplify(this FloatCurve2 curve, int axis, FloatCurveExtensions.KeyframeMargins removalPredicate = null)
        {
            if (removalPredicate == null)
                removalPredicate = DefaultMargins;
            switch (axis)
            {
                case 0:
                    return SimplifyHorizontal(curve, removalPredicate);
                case 1:
                    return SimplifyVertical(curve, removalPredicate);
                default:
                    throw new ArgumentException("Axis must be zero or one.");
            }
        }
        internal static bool DefaultMargins(float valueError, float _, float tangentDifference, float tangentError)
        {
            const float stdValueError = 0.1f / 100;
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
        private static FloatCurve2 SimplifyHorizontal(FloatCurve2 curve, FloatCurveExtensions.KeyframeMargins removalPredicate)
        {
            List<float> xKeys = new List<float>(curve.xKeys);
            int yCount = curve.yKeys.Length;
            List<int> keyIndices = new List<int>(xKeys.Count) { 0 };
            for (int i = 1; i < xKeys.Count - 1; i++)
            {
                bool remove = true;
                int i0 = curve.xKeys.IndexOf(xKeys[i - 1]);
                int i1 = curve.xKeys.IndexOf(xKeys[i + 1]);
                int i_ = curve.xKeys.IndexOf(xKeys[i]);
                for (int j = 0; j < yCount; j++)
                {
                    ref Keyframe2 k0 = ref curve.values[i0, j];
                    ref Keyframe2 k1 = ref curve.values[i1, j];
                    ref Keyframe2 k_ = ref curve.values[i_, j];
                    float computedValue = FloatCurveExtensions.EvaluateBicubic(xKeys[i], k0.timeX, k1.timeX, k0.value, k0.dDx_out, k1.dDx_in, k1.value);
                    float computedTangent = FloatCurveExtensions.EvaluateBicubicDerivative(xKeys[i], k0.timeX, k1.timeX, k0.value, k0.dDx_out, k1.dDx_in, k1.value);
                    float averageTangent = (k_.dDx_in + k_.dDx_out) * 0.5f;
                    if (!removalPredicate(
                        (computedValue - k_.value) / k_.value,
                        computedValue - k_.value,
                        (k_.dDx_out - k_.dDx_in) / averageTangent,
                        (computedTangent - averageTangent) / averageTangent))
                    {
                        remove = false;
                        break;
                    }
                }
                if (remove)
                {
                    xKeys.RemoveAt(i);
                    i--;
                }
                else
                    keyIndices.Add(i_);
            }
            keyIndices.Add(curve.xKeys.Length - 1);
            if (xKeys.Count == curve.xKeys.Length)
                return curve;
            Keyframe2[,] keyframes = new Keyframe2[xKeys.Count - 1, yCount];
            for (int i = 0; i < xKeys.Count; i++)
            {
                for (int j = 0; j < yCount; j++)
                {
                    keyframes[i, j] = curve.values[keyIndices[i], j];
                }
            }
            return new FloatCurve2(xKeys, curve.yKeys, keyframes);
        }
        private static FloatCurve2 SimplifyVertical(FloatCurve2 curve, FloatCurveExtensions.KeyframeMargins removalPredicate)
        {
            List<float> yKeys = new List<float>(curve.yKeys);
            int xCount = curve.xKeys.Length;
            List<int> keyIndices = new List<int>(yKeys.Count) { 0 };
            for (int j = 1; j < yKeys.Count - 1; j++)
            {
                bool remove = true;
                int j0 = curve.yKeys.IndexOf(yKeys[j - 1]);
                int j1 = curve.yKeys.IndexOf(yKeys[j + 1]);
                int j_ = curve.yKeys.IndexOf(yKeys[j]);
                for (int i = 0; i < xCount; i++)
                {
                    ref Keyframe2 k0 = ref curve.values[i, j0];
                    ref Keyframe2 k1 = ref curve.values[i, j1];
                    ref Keyframe2 k_ = ref curve.values[i, j_];
                    float computedValue = FloatCurveExtensions.EvaluateBicubic(yKeys[j], k0.timeY, k1.timeY, k0.value, k0.dDy_out, k1.dDy_in, k1.value);
                    float computedTangent = FloatCurveExtensions.EvaluateBicubicDerivative(yKeys[j], k0.timeY, k1.timeY, k0.value, k0.dDy_out, k1.dDy_in, k1.value);
                    float averageTangent = (k_.dDy_in + k_.dDy_out) * 0.5f;
                    if (!removalPredicate(
                        (computedValue - k_.value) / k_.value,
                        computedValue - k_.value,
                        (k_.dDy_out - k_.dDy_in) / averageTangent,
                        (computedTangent - averageTangent) / averageTangent))
                    {
                        remove = false;
                        break;
                    }
                }
                if (remove)
                {
                    yKeys.RemoveAt(j);
                    j--;
                }
                else
                    keyIndices.Add(j_);
            }
            keyIndices.Add(curve.yKeys.Length - 1);
            if (yKeys.Count == curve.yKeys.Length)
                return curve;
            Keyframe2[,] keyframes = new Keyframe2[xCount, yKeys.Count];
            for (int i = 0; i < xCount; i++)
            {
                for (int j = 0; j < yKeys.Count; j++)
                {
                    keyframes[i, j] = curve.values[i, keyIndices[j]];
                }
            }
            return new FloatCurve2(curve.xKeys, yKeys, keyframes);
        }
    }
}
