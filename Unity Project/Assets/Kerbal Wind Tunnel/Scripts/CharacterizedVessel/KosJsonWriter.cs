using Graphing.IO;
using KerbalWindTunnel.Extensions;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace KerbalWindTunnel.VesselCache
{
    public class KosJsonWriter
    {
        private readonly StringBuilder stringBuilder = new StringBuilder();
        public void WriteToJson(CharacterizedVessel vessel, string filename)
        {
            string path = GraphIO.ValidateFilePath(WindTunnel.graphPath, filename, ".json");
#if OUTSIDE_UNITY
            static
#endif
            FloatCurve ScaleFloatCurve(FloatCurve curve, float scalar)
            {
                return new FloatCurve(curve.Curve.keys.Select(
                    k => new Keyframe(k.time, k.value * scalar, k.inTangent * scalar, k.outTangent * scalar)
                    ).ToArray());
            }
#if OUTSIDE_UNITY
            static
#endif
            FloatCurve ScaleFloatCurveTime(FloatCurve curve, float scalar)
            {
                float invScalar = 1 / scalar;
                return new FloatCurve(curve.Curve.keys.Select(
                    k => new Keyframe(k.time * scalar, k.value, k.inTangent * invScalar, k.outTangent * invScalar)
                    ).ToArray());
            }
            lock (stringBuilder)
            {
                stringBuilder.Clear();
                string indentString = GenerateIndentString(1);
                stringBuilder.AppendLine("{");
                stringBuilder.AppendLine(indentString + "\"entries\": [");
                WriteValue("numCurves", 2, true);
                WriteValue(vessel.bodyLift.Count + vessel.surfaceLift.Count, 2, vessel.bodyLift.Count + vessel.surfaceLift.Count > 0);
                if (vessel.bodyLift.Count + vessel.surfaceLift.Count > 0)
                {
                    // Treat the body lift curves and the surface lift curves the same.
                    IEnumerable<(FloatCurve machCurve, FloatCurve liftCurve)> liftCurveSets = vessel.bodyLift.Union(vessel.surfaceLift);
                    // Write the mach scalar curves (a List of FloatCurves)
                    WriteValue("machScalar", 2, true);
                    WriteList(liftCurveSets.Select(curveSet => curveSet.machCurve), 2, true);
                    // Write the lift coefficient curves (a List of FloatCurves)
                    WriteValue("liftCurve", 2, true);
                    WriteList(liftCurveSets.Select(curveSet => ScaleFloatCurveTime(curveSet.liftCurve, Mathf.Rad2Deg)), 2, true);
                    // Write the AoA at which max lift is obtained (a FloatCurve with respect to Mach number)
                    WriteValue("maxLiftAoA", 2, true);
                    WriteFloatCurve(ScaleFloatCurve(GetMaxLiftAoA(vessel), Mathf.Rad2Deg), 2, true);
                    // Write the AoA at which max L/D is obtained (a FloatCurve with respect to Mach number (with some assumption about altitude))
                    WriteValue("maxLDAoA", 2, true);
                    WriteFloatCurve(ScaleFloatCurve(GetMaxLDAoA(vessel), Mathf.Rad2Deg), 2, false);
                }
                stringBuilder.AppendLine();
                stringBuilder.AppendLine(indentString + "],");
                stringBuilder.AppendLine(indentString + "\"$type\": \"kOS.Safe.Encapsulation.Lexicon\"");
                stringBuilder.Append("}");

                System.IO.File.WriteAllText(path, stringBuilder.ToString());
            }
        }
        private void WriteList<T>(IEnumerable<T> values, int indentLevel = 0, bool appendComma = false)
        {
            string indentString = GenerateIndentString(indentLevel);
            string innerIndentString = GenerateIndentString(indentLevel + 1);
            stringBuilder.AppendLine(indentString + "{");
            stringBuilder.AppendLine(innerIndentString + "\"items\": [");
            var enumerator = values.GetEnumerator();
            bool moveNext = enumerator.MoveNext();
            while (moveNext)
            {
                T value = enumerator.Current;
                moveNext = enumerator.MoveNext();
                switch (value)
                {
                    case FloatCurve floatCurve:
                        WriteFloatCurve(floatCurve, indentLevel + 2, moveNext);
                        break;
                    case Keyframe keyframe:
                        WriteKeyframe(keyframe, indentLevel + 2, moveNext);
                        break;
                    case string s:
                        WriteValue(s, indentLevel + 2, moveNext);
                        break;
                    case double d:
                        WriteValue(d, indentLevel + 2, moveNext);
                        break;
                    case float f:
                        WriteValue(f, indentLevel + 2, moveNext);
                        break;
                    case int i:
                        WriteValue(i, indentLevel + 2, moveNext);
                        break;
                    default:
                        throw new NotImplementedException();
                }
            }
            stringBuilder.AppendLine();
            stringBuilder.AppendLine(innerIndentString + "],");
            stringBuilder.AppendLine(innerIndentString + "\"$type\": \"kOS.Safe.Encapsulation.ListValue\"");
            if (appendComma)
                stringBuilder.AppendLine(indentString + "},");
            else
                stringBuilder.Append(indentString + "}");
        }
        private void WriteFloatCurve(FloatCurve curve, int indentLevel = 0, bool appendComma = false)
            => WriteList(curve.Curve.keys, indentLevel, appendComma);
        /*private void WriteFloatCurve(FloatCurve curve, int indentLevel = 0, bool appendComma = false)
        {
            string indentString = GenerateIndentString(indentLevel);
            string innerIndentString = GenerateIndentString(indentLevel + 1);
            stringBuilder.AppendLine(indentString + "{");
            stringBuilder.AppendLine(innerIndentString + "\"items\": [");
            foreach (Keyframe keyframe in curve.Curve.keys.Take(curve.Curve.keys.Length - 1))
            {
                WriteKeyframe(keyframe, indentLevel + 2);
                stringBuilder.AppendLine(",");
            }
            WriteKeyframe(curve.Curve.keys[curve.Curve.keys.Length - 1], indentLevel + 2);
            stringBuilder.AppendLine();
            stringBuilder.AppendLine(innerIndentString + "],");
            stringBuilder.AppendLine(innerIndentString + "\"$type\": \"kOS.Safe.Encapsulation.ListValue\"");
            if (appendComma)
                stringBuilder.AppendLine(indentString + "},");
            else
                stringBuilder.Append(indentString + "}");
        }*/
        private static IEnumerable<float> KeyframeAsEnumerable(Keyframe keyframe)
        {
            yield return keyframe.time;
            yield return keyframe.value;
            yield return keyframe.inTangent;
            yield return keyframe.outTangent;
        }
        private void WriteKeyframe(Keyframe keyframe, int indentLevel = 0, bool appendComma = false)
            => WriteList(KeyframeAsEnumerable(keyframe), indentLevel, appendComma);
        /*private void WriteKeyframe(Keyframe keyframe, int indentLevel = 0, bool appendComma = false)
        {
            string indentString = GenerateIndentString(indentLevel);
            string innerIndentString = GenerateIndentString(indentLevel + 1);
            stringBuilder.AppendLine(indentString + "{");
            stringBuilder.AppendLine(innerIndentString + "\"items\": [");
            WriteValue(keyframe.time, indentLevel + 2);
            stringBuilder.AppendLine(",");
            WriteValue(keyframe.value, indentLevel + 2);
            stringBuilder.AppendLine(",");
            WriteValue(keyframe.inTangent, indentLevel + 2);
            stringBuilder.AppendLine(",");
            WriteValue(keyframe.outTangent, indentLevel + 2);
            stringBuilder.AppendLine();
            stringBuilder.AppendLine(innerIndentString + "],");
            stringBuilder.AppendLine(innerIndentString + "\"$type\": \"kOS.Safe.Encapsulation.ListValue\"");
            if (appendComma)
                stringBuilder.AppendLine(indentString + "},");
            else
                stringBuilder.Append(indentString + "}");
        }*/
        private void WriteValue(double value, int indentLevel = 0, bool appendComma = false)
        {
            string indentString = GenerateIndentString(indentLevel);
            stringBuilder.AppendLine(indentString + "{");
            stringBuilder.AppendLine(indentString + "    \"value\": " + value.ToString() + ",");
            stringBuilder.AppendLine(indentString + "    \"$type\": \"kOS.Safe.Encapsulation.ScalarDoubleValue\"");
            if (appendComma)
                stringBuilder.AppendLine(indentString + "},");
            else
                stringBuilder.Append(indentString + "}");
        }
        private void WriteValue(float value, int indentLevel = 0, bool appendComma = false)
        {
            string indentString = GenerateIndentString(indentLevel);
            stringBuilder.AppendLine(indentString + "{");
            stringBuilder.AppendLine(indentString + "    \"value\": " + value.ToString() + ",");
            stringBuilder.AppendLine(indentString + "    \"$type\": \"kOS.Safe.Encapsulation.ScalarDoubleValue\"");
            if (appendComma)
                stringBuilder.AppendLine(indentString + "},");
            else
                stringBuilder.Append(indentString + "}");
        }

        private void WriteValue(int value, int indentLevel = 0, bool appendComma = false)
        {
            string indentString = GenerateIndentString(indentLevel);
            stringBuilder.AppendLine(indentString + "{");
            stringBuilder.AppendLine(indentString + "    \"value\": " + value.ToString() + ",");
            stringBuilder.AppendLine(indentString + "    \"$type\": \"kOS.Safe.Encapsulation.ScalarIntValue\"");
            if (appendComma)
                stringBuilder.AppendLine(indentString + "},");
            else
                stringBuilder.Append(indentString + "}");
        }
        private void WriteValue(string value, int indentLevel = 0, bool appendComma = false)
        {
            string indentString = GenerateIndentString(indentLevel);
            stringBuilder.AppendLine(indentString + "{");
            stringBuilder.AppendLine(indentString + "    \"value\": \"" + value + "\",");
            stringBuilder.AppendLine(indentString + "    \"$type\": \"kOS.Safe.Encapsulation.StringValue\"");
            if (appendComma)
                stringBuilder.AppendLine(indentString + "},");
            else
                stringBuilder.Append(indentString + "}");
        }
        private string GenerateIndentString(int indentLevel)
        {
            if (indentLevel <= 0)
                return string.Empty;
            return new string(' ', indentLevel * 4);
        }
        private static TSource MaxBy<TSource, TKey>(IEnumerable<TSource> source, Func<TSource, TKey> keySelector, IComparer<TKey> comparer = null)
        {
            if (source is null)
                throw new ArgumentNullException(nameof(source));
            if (keySelector is null)
                throw new ArgumentNullException(nameof(keySelector));

            if (comparer == null)
                comparer = Comparer<TKey>.Default;
            IEnumerator<TSource> enumerator = source.GetEnumerator();
            if (!enumerator.MoveNext())
            {
                if (typeof(TSource).IsValueType)
                    throw new InvalidOperationException();
                return default;
            }
            TSource value = enumerator.Current;
            TKey key = keySelector(value);
            while (enumerator.MoveNext() && enumerator.Current != null)
            {
                if ((value == null && enumerator.Current != null) ||
                    (comparer.Compare(keySelector(enumerator.Current), key) > 0))
                    value = enumerator.Current;
            }
            return value;
        }
        public override string ToString()
            => stringBuilder.ToString();

        private static readonly FloatCurve machAltitude = new FloatCurve(new Keyframe[] {
            new Keyframe(0, 0, 0, 0),
            new Keyframe(1, 0, 0, 5000),
            new Keyframe(2, 5000, 4167, 4167),
            new Keyframe(5, 15000, 1917, 1917),
            new Keyframe(25, 25000, 500, 0) });
        public static FloatCurve GetMaxLiftAoA(CharacterizedVessel vessel)  // with respect to Mach #
         => vessel.AoAMax;
        public static FloatCurve GetMaxLDAoA(CharacterizedVessel vessel)    // with respect to Mach #
        {
            SortedSet<float> machKeys = new SortedSet<float>();
            foreach (FloatCurve curve in vessel.bodyLift.Select(curveSet => curveSet.machCurve))
                machKeys.UnionWith(curve.Curve.keys.Select(k => k.time));
            foreach (FloatCurve curve in vessel.surfaceLift.Select(curveSet => curveSet.machCurve))
                machKeys.UnionWith(curve.Curve.keys.Select(k => k.time));

            CelestialBody body = WindTunnelWindow.Instance.CelestialBody;

            float FindMaxLDAoAForMach(float mach)
            {
                float altitude = machAltitude.EvaluateThreadSafe(mach);
                AeroPredictor.Conditions conditions = AeroPredictor.Conditions.ConditionsByMach(body, mach, altitude, true);
                double CalculateLD(double aoa)
                {
                    float aoa_ = (float)aoa;
                    return vessel.EvaluateLiftCurve(conditions, aoa_, 0) / vessel.EvaluateDragCurve(conditions, aoa_, 0);
                }
                return (float)Accord.Math.Optimization.BrentSearch.Maximize(CalculateLD, -5 * Mathf.Deg2Rad, vessel.AoAMax.Evaluate(mach), AeroOptimizer.defaultAoAOptTolerance);
            }
            return FloatCurveExtensions.ComputeFloatCurve(machKeys, FindMaxLDAoAForMach, 0.15f);
        }
    }
}
