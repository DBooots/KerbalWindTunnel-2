using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;
using Graphing.IO;
using KerbalWindTunnel.Extensions;

namespace KerbalWindTunnel.VesselCache
{
    public class KosJsonWriter
    {
        private readonly StringBuilder stringBuilder = new StringBuilder();
        public void WriteToJson(CharacterizedVessel vessel, string filename)
        {
            string path = GraphIO.ValidateFilePath(WindTunnel.graphPath, filename, ".json");
            lock (stringBuilder)
            {
                stringBuilder.Clear();

                List<(FloatCurve machCurve, FloatCurve coefCurve)>[] curveSets = vessel.CurveSets;
                Dictionary<FloatCurve, (string name, List<(FloatCurve coefCurve, string name)> curveList)> curveSetDict =
                    CharacterizedVessel.CompileCurveSetsForExport(curveSets);
                HashSet<FloatCurve> outstandingKeys = new HashSet<FloatCurve>(curveSetDict.Keys, FloatCurveComparer.Instance);
                HashSet<FloatCurve> lift = new HashSet<FloatCurve>(CharacterizedVessel._liftIndices.
                    SelectMany(i => curveSets[i]).Select(cs => cs.coefCurve),
                    FloatCurveComparer.Instance);
                HashSet<FloatCurve> controls = new HashSet<FloatCurve>(CharacterizedVessel._ctrlIndices.Intersect(CharacterizedVessel._liftIndices).
                    SelectMany(i => curveSets[i]).Select(cs => cs.coefCurve),
                    FloatCurveComparer.Instance);

                IEnumerable<List<(string, object)>> WriteCurveSetIndex(int index)
                {
                    foreach ((FloatCurve machCurve, _) in curveSets[index])
                    {
                        if (outstandingKeys.Contains(machCurve))
                        {
                            // Write the mach scalar curve
                            List<(string, object)> result = new List<(string, object)> { ("machScalar", machCurve) };
                            List<FloatCurve> liftCurves = new List<FloatCurve>();
                            List<FloatCurve> ctrlCurves = new List<FloatCurve>();
                            foreach ((FloatCurve coefCurve, _) in curveSetDict[machCurve].curveList)
                            {
                                if (!lift.Contains(coefCurve))
                                    continue;
                                if (controls.Contains(coefCurve))
                                    ctrlCurves.Add(coefCurve);
                                else
                                    liftCurves.Add(coefCurve);
                            }
                            // Write the lift coefficient curve and control delta lift coefficient curve
                            if (liftCurves.Count > 0)
                                result.Add(("liftCurve", FloatCurveExtensions.TimesScaledBy(FloatCurveExtensions.Superposition(liftCurves), Mathf.Rad2Deg)));
                            if (ctrlCurves.Count > 0)
                                result.Add(("ctrlCurve", FloatCurveExtensions.TimesScaledBy(FloatCurveExtensions.Superposition(ctrlCurves), Mathf.Rad2Deg)));
                            outstandingKeys.Remove(machCurve);
                            yield return result;
                        }
                        else
                            yield return null;
                    }
                }
                List<List<(string, object)>> liftData = CharacterizedVessel._liftIndices.SelectMany(WriteCurveSetIndex).Where(l => l != null).ToList();

                FloatCurve maxLiftAoA = FloatCurveExtensions.ScaledBy(GetMaxLiftAoA(vessel), Mathf.Rad2Deg);
                FloatCurve maxStableAoA = FloatCurveExtensions.ScaledBy(GetMaxStableAoA(vessel), Mathf.Rad2Deg);
                List<(string, object)> vesselData = new List<(string, object)>
                {
                    ("numCurves", liftData.Count),
                    ("liftData", liftData),
                    // Write the AoA at which max lift is obtained (a FloatCurve with respect to Mach number)
                    ("maxLiftAoA", maxLiftAoA),
                    // Write the max AoA which is attainable using control surfaces only (a FloatCurve with respect to Mach number)
                    ("maxStableAoA", maxStableAoA),
                    // Write the max targetable AoA, which is the min of maxLiftAoA and maxStableAoA (a FloatCurve with respect to Mach number)
                    ("maxAoA", FloatCurveExtensions.Min(maxLiftAoA, maxStableAoA)),
                    // Write the AoA at which max L/D is obtained (a FloatCurve with respect to Mach number (with some assumption about altitude))
                    ("maxLDAoA", FloatCurveExtensions.ScaledBy(GetMaxLDAoA(vessel), Mathf.Rad2Deg))
                };

                WriteDictionary(vesselData);

                System.IO.File.WriteAllText(path, stringBuilder.ToString());
            }
        }

        private void WriteList(System.Collections.IEnumerable values, int indentLevel = 0, bool appendComma = false)
        {
            string indentString = GenerateIndentString(indentLevel);
            string innerIndentString = GenerateIndentString(indentLevel + 1);
            stringBuilder.AppendLine(indentString + "{");
            stringBuilder.AppendLine(innerIndentString + "\"items\": [");
            var enumerator = values.GetEnumerator();
            bool moveNext = enumerator.MoveNext();
            while (moveNext)
            {
                object value = enumerator.Current;
                moveNext = enumerator.MoveNext();
                WriteObject(value, indentLevel + 2, moveNext);
            }
            stringBuilder.AppendLine();
            stringBuilder.AppendLine(innerIndentString + "],");
            stringBuilder.AppendLine(innerIndentString + "\"$type\": \"kOS.Safe.Encapsulation.ListValue\"");
            if (appendComma)
                stringBuilder.AppendLine(indentString + "},");
            else
                stringBuilder.Append(indentString + "}");
        }

        private void WriteDictionary(Dictionary<string, object> dict, int indentLevel = 0, bool appendComma = false)
            => WriteDictionary(dict.Select(kvp => (kvp.Key, kvp.Value)), indentLevel, appendComma);
        private void WriteDictionary(IEnumerable<(string, object)> dictList, int indentLevel = 0, bool appendComma = false)
        {
            string indentString = GenerateIndentString(indentLevel);
            string innerIndentString = GenerateIndentString(indentLevel + 1);
            stringBuilder.AppendLine(indentString + "{");
            stringBuilder.AppendLine(innerIndentString + "\"entries\": [");

            var enumerator = dictList.GetEnumerator();
            bool moveNext = enumerator.MoveNext();
            while (moveNext)
            {
                (string key, object value) = enumerator.Current;
                moveNext = enumerator.MoveNext();
                WriteValue(key, indentLevel + 2, true);
                WriteObject(value, indentLevel + 2, moveNext);
            }

            stringBuilder.AppendLine();
            stringBuilder.AppendLine(innerIndentString + "],");
            stringBuilder.AppendLine(innerIndentString + "\"$type\": \"kOS.Safe.Encapsulation.Lexicon\"");
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

        private void WriteObject(object value, int indentLevel = 0, bool appendComma = false)
        {
            switch (value)
            {
                case FloatCurve floatCurve:
                    WriteFloatCurve(floatCurve, indentLevel, appendComma);
                    break;
                case Keyframe keyframe:
                    WriteKeyframe(keyframe, indentLevel, appendComma);
                    break;
                case string s:
                    WriteValue(s, indentLevel, appendComma);
                    break;
                case double d:
                    WriteValue(d, indentLevel, appendComma);
                    break;
                case float f:
                    WriteValue(f, indentLevel, appendComma);
                    break;
                case int i:
                    WriteValue(i, indentLevel, appendComma);
                    break;
                default:
                    Type type = value.GetType();
                    if (value is System.Collections.IDictionary && type.GenericTypeArguments.Length == 2 && type.GenericTypeArguments[0] == typeof(string))
                    {
                        WriteDictionary((Dictionary<string, object>)value, indentLevel, appendComma);
                        break;
                    }
                    else if (value is System.Collections.IEnumerable enumerable)
                    {
                        if (type.IsGenericType)
                        {
                            Type elementType = type.GenericTypeArguments[0];
                            if (elementType.IsGenericType && elementType.GetGenericTypeDefinition() == typeof(ValueTuple<,>) && elementType.GenericTypeArguments[0] == typeof(string))
                                WriteDictionary((IEnumerable<(string, object)>)value, indentLevel, appendComma);
                            else
                                WriteList(enumerable, indentLevel, appendComma);
                        }
                        else
                            WriteList(enumerable, indentLevel, appendComma);
                        break;
                    }
                    else
                        throw new NotImplementedException();
            }
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
        public static FloatCurve GetMaxStableAoA(CharacterizedVessel vessel)    // with respect to Mach #
        {
            SortedSet<float> machKeys = new SortedSet<float>();
            foreach (FloatCurve curve in vessel.bodyLift.Select(curveSet => curveSet.machCurve))
                machKeys.UnionWith(curve.Curve.keys.Select(k => k.time));
            foreach (FloatCurve curve in vessel.surfaceLift.Select(curveSet => curveSet.machCurve))
                machKeys.UnionWith(curve.Curve.keys.Select(k => k.time));

            CelestialBody body = WindTunnelWindow.Instance.CelestialBody;
            float guess = float.NaN;

            float FindMaxStableAoAForMach(float mach)
            {
                float altitude = machAltitude.EvaluateThreadSafe(mach);
                AeroPredictor.Conditions conditions = AeroPredictor.Conditions.ConditionsByMach(body, mach, altitude, true);
                float result = AeroOptimizer.FindStableAoA(vessel, conditions, 1, guess);
                guess = result;
                return result;
            }
            return FloatCurveExtensions.ComputeFloatCurve(machKeys, FindMaxStableAoAForMach, 0.15f);
        }
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
                return (float)Accord.Math.Optimization.BrentSearch.Maximize(CalculateLD, -5 * Mathf.Deg2Rad, vessel.AoAMax.EvaluateThreadSafe(mach), AeroOptimizer.defaultAoAOptTolerance);
            }
            return FloatCurveExtensions.ComputeFloatCurve(machKeys, FindMaxLDAoAForMach, 0.15f);
        }
    }
}
