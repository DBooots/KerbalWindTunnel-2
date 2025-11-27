using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using KerbalWindTunnel.Extensions;

namespace KerbalWindTunnel
{
    public class FloatCurve2 : IDisposable
    {
        public System.Threading.ReaderWriterLockSlim ReadWriteLock { get; } = new System.Threading.ReaderWriterLockSlim();
        public readonly float[] xKeys;
        private readonly HashSet<float> xKeysSet = new HashSet<float>();
        public readonly float[] yKeys;
        private readonly HashSet<float> yKeysSet = new HashSet<float>();
        public readonly Keyframe2[,] values;
        private readonly Coefficients[,] coefficientsCache;

        public (int, int) Size { get => (xKeys.Length, yKeys.Length); }
        public int Length { get { return values.Length; } }

        public int GetUpperBound(int dimension) { return (dimension == 0 ? xKeys.Length : yKeys.Length) - 1; }

        public FloatCurve2(IEnumerable<float> xKeys, IEnumerable<float> yKeys)
        {
            this.xKeys = xKeys.ToArray();
            this.yKeys = yKeys.ToArray();
            xKeysSet.UnionWith(this.xKeys);
            yKeysSet.UnionWith(this.yKeys);

            values = new Keyframe2[this.xKeys.Length, this.yKeys.Length];
            Array.Sort(this.xKeys);
            Array.Sort(this.yKeys);
            if (this.xKeys.Length == 0 || this.yKeys.Length == 0)
                return;
            else
                coefficientsCache = new Coefficients[this.xKeys.Length - 1, this.yKeys.Length - 1];
        }

        public FloatCurve2(IEnumerable<float> xKeys, IEnumerable<float> yKeys, float[,] values) : this(xKeys, yKeys)
        {
            int xLength = this.xKeys.Length;
            int yLength = this.yKeys.Length;

            if ((values.GetUpperBound(0) + 1 != xLength) || (values.GetUpperBound(1) + 1 != yLength))
                throw new ArgumentException("Array dimensions do not match provided keys.", nameof(values));

            for (int i = xLength - 2; i >= 1; i--)
            {
                float xDiff = this.xKeys[i + 1] - this.xKeys[i - 1];
                for (int j = yLength - 2; j >= 1; j--)
                {
                    this.values[i, j] = new Keyframe2(this.xKeys[i], this.yKeys[j], values[i, j],
                        (values[i + 1, j] - values[i - 1, j]) / xDiff,
                        (values[i, j + 1] - values[i, j - 1]) / (this.yKeys[j + 1] - this.yKeys[j - 1]),
                        (values[i + 1, j + 1] - values[i + 1, j - 1] - values[i - 1, j + 1] + values[i - 1, j - 1]) / (xDiff * (this.yKeys[j + 1] - this.yKeys[j - 1])));
                }
            }
            for (int i = xLength - 2; i >= 1; i--)
            {
                float xDiff = this.xKeys[i + 1] - this.xKeys[i - 1];
                float yDiff0 = this.yKeys[1] - this.yKeys[0];
                float yDiff1 = this.yKeys[yLength - 1] - this.yKeys[yLength - 2];

                int j = 0;
                this.values[i, j] = new Keyframe2(this.xKeys[i], this.yKeys[j], values[i, j],
                    (values[i + 1, j] - values[i - 1, j]) / (this.xKeys[i + 1] - this.xKeys[i - 1]),
                    (-3 * values[i, j] + 3 * values[i, j + 1] - this.values[i, j + 1].dDy_in) / 2,
                    (values[i + 1, j + 1] - values[i + 1, j] - values[i - 1, j + 1] + values[i - 1, j]) / (xDiff * yDiff0));

                j = yLength - 1;
                this.values[i, j] = new Keyframe2(this.xKeys[i], this.yKeys[j], values[i, j],
                    (values[i + 1, j] - values[i - 1, j]) / (this.xKeys[i + 1] - this.xKeys[i - 1]),
                    (-3 * values[i, j] - 3 * values[i, j - 1] + this.values[i, j - 1].dDy_in) / 2,
                    (values[i + 1, j] - values[i + 1, j - 1] - values[i - 1, j] + values[i - 1, j - 1]) / (xDiff * yDiff1));
            }
            for (int j = yLength - 2; j >= 1; j--)
            {
                float yDiff = this.yKeys[j + 1] - this.yKeys[j - 1];
                float xDiff0 = this.xKeys[1] - this.xKeys[0];
                float xDiff1 = this.xKeys[xLength - 1] - this.xKeys[xLength - 2];

                int i = 0;
                this.values[i, j] = new Keyframe2(this.xKeys[i], this.yKeys[j], values[i, j],
                    (-3 * values[i, j] + 3 * values[i + 1, j] - this.values[i + 1, j].dDx_in) / 2,
                    (values[i, j + 1] - values[i, j - 1]) / yDiff,
                    (values[i + 1, j + 1] - values[i + 1, j - 1] - values[i, j + 1] + values[i, j - 1]) / (xDiff0 * yDiff));

                i = xLength - 1;
                this.values[i, j] = new Keyframe2(this.xKeys[i], this.yKeys[j], values[i, j],
                    (-3 * values[i, j] - 3 * values[i - 1, j] + this.values[i - 1, j].dDx_in) / 2,
                    (values[i, j + 1] - values[i, j - 1]) / yDiff,
                    (values[i, j + 1] - values[i, j - 1] - values[i - 1, j + 1] + values[i - 1, j - 1]) / (xDiff1 * yDiff));
            }
            {   // Braces to keep this int i and j confined to allow consisten variable nomenclature.
                int i, j;
                i = j = 0;
                this.values[i, j] = new Keyframe2(this.xKeys[i], this.yKeys[j], values[i, j],
                    (-3 * values[i, j] + 3 * values[i + 1, j] - this.values[i + 1, j].dDx_in) / 2,
                    (-3 * values[i, j] + 3 * values[i, j + 1] - this.values[i, j + 1].dDy_in) / 2,
                    (values[i + 1, j + 1] - values[i + 1, j] - values[i, j + 1] + values[i, j]) / ((this.xKeys[i + 1] - this.xKeys[i]) * (this.yKeys[j + 1] - this.yKeys[j])));

                j = yLength - 1;
                this.values[i, j] = new Keyframe2(this.xKeys[i], this.yKeys[j], values[i, j],
                    (-3 * values[i, j] + 3 * values[i + 1, j] - this.values[i + 1, j].dDx_in) / 2,
                    (-3 * values[i, j] - 3 * values[i, j - 1] + this.values[i, j - 1].dDy_in) / 2,
                    (values[i + 1, j] - values[i + 1, j - 1] - values[i, j] + values[i, j - 1]) / ((this.xKeys[i + 1] - this.xKeys[i]) * (this.yKeys[j] - this.yKeys[j - 1])));

                i = xLength - 1;
                this.values[i, j] = new Keyframe2(this.xKeys[i], this.yKeys[j], values[i, j],
                    (-3 * values[i, j] - 3 * values[i - 1, j] + this.values[i - 1, j].dDx_in) / 2,
                    (-3 * values[i, j] - 3 * values[i, j - 1] + this.values[i, j - 1].dDy_in) / 2,
                    (values[i, j] - values[i, j - 1] - values[i - 1, j] + values[i - 1, j - 1]) / ((this.xKeys[i] - this.xKeys[i - 1]) * (this.yKeys[j] - this.yKeys[j - 1])));


                j = 0;
                this.values[i, j] = new Keyframe2(this.xKeys[i], this.yKeys[j], values[i, j],
                    (-3 * values[i, j] - 3 * values[i - 1, j] + this.values[i - 1, j].dDx_in) / 2,
                    (-3 * values[i, j] + 3 * values[i, j + 1] - this.values[i, j + 1].dDy_in) / 2,
                    (values[i, j + 1] - values[i, j] - values[i - 1, j + 1] + values[i - 1, j]) / ((this.xKeys[i] - this.xKeys[i - 1]) * (this.yKeys[j + 1] - this.yKeys[j])));
            }
        }

        public FloatCurve2(IEnumerable<float> xKeys, IEnumerable<float> yKeys, float[,] values, float[,] xPartialTangents, float[,] yPartialTangents, float[,] mixedTangents) : this(xKeys, yKeys)
        {
            int xLength = this.xKeys.Length - 1;
            int yLength = this.yKeys.Length - 1;

            if ((values.GetUpperBound(0) != xLength) || (values.GetUpperBound(1) != yLength))
                throw new ArgumentException("Array dimensions do not match provided keys.", nameof(values));
            if ((xPartialTangents.GetUpperBound(0) != xLength) || (xPartialTangents.GetUpperBound(1) != yLength))
                throw new ArgumentException("Array dimensions do not match provided keys.", nameof(xPartialTangents));
            if ((yPartialTangents.GetUpperBound(0) != xLength) || (yPartialTangents.GetUpperBound(1) != yLength))
                throw new ArgumentException("Array dimensions do not match provided keys.", nameof(yPartialTangents));
            if ((mixedTangents.GetUpperBound(0) != xLength) || (mixedTangents.GetUpperBound(1) != yLength))
                throw new ArgumentException("Array dimensions do not match provided keys.", nameof(mixedTangents));

            for (int i = xLength; i >= 0; i--)
                for (int j = yLength; j >= 0; j--)
                    this.values[i, j] = new Keyframe2(this.xKeys[i], this.yKeys[j], values[i, j], xPartialTangents[i, j], yPartialTangents[i, j], mixedTangents[i, j]);
        }

        public FloatCurve2(IEnumerable<float> xKeys, IEnumerable<float> yKeys, float[,] values, float[,] xinTangents, float[,] xoutTangents, float[,] yinTangents, float[,] youtTangents, float[,] xinyinTangents, float[,] xinyoutTangents, float[,] xoutyinTangents, float[,] xoutyoutTangents) : this(xKeys, yKeys)
        {
            int xLength = this.xKeys.Length - 1;
            int yLength = this.yKeys.Length - 1;

            if ((values.GetUpperBound(0) != xLength) || (values.GetUpperBound(1) != yLength))
                throw new ArgumentException("Array dimensions do not match provided keys.", nameof(values));

            if ((xinTangents.GetUpperBound(0) != xLength) || (xinTangents.GetUpperBound(1) != yLength))
                throw new ArgumentException("Array dimensions do not match provided keys.", nameof(xinTangents));
            if ((xoutTangents.GetUpperBound(0) != xLength) || (xoutTangents.GetUpperBound(1) != yLength))
                throw new ArgumentException("Array dimensions do not match provided keys.", nameof(xoutTangents));
            if ((yinTangents.GetUpperBound(0) != xLength) || (yinTangents.GetUpperBound(1) != yLength))
                throw new ArgumentException("Array dimensions do not match provided keys.", nameof(yinTangents));
            if ((youtTangents.GetUpperBound(0) != xLength) || (youtTangents.GetUpperBound(1) != yLength))
                throw new ArgumentException("Array dimensions do not match provided keys.", nameof(youtTangents));

            if ((xinyinTangents.GetUpperBound(0) != xLength) || (xinyinTangents.GetUpperBound(1) != yLength))
                throw new ArgumentException("Array dimensions do not match provided keys.", nameof(xinyinTangents));
            if ((xinyoutTangents.GetUpperBound(0) != xLength) || (xinyoutTangents.GetUpperBound(1) != yLength))
                throw new ArgumentException("Array dimensions do not match provided keys.", nameof(xinyoutTangents));
            if ((xoutyinTangents.GetUpperBound(0) != xLength) || (xoutyinTangents.GetUpperBound(1) != yLength))
                throw new ArgumentException("Array dimensions do not match provided keys.", nameof(xoutyinTangents));
            if ((xoutyoutTangents.GetUpperBound(0) != xLength) || (xoutyoutTangents.GetUpperBound(1) != yLength))
                throw new ArgumentException("Array dimensions do not match provided keys.", nameof(xoutyoutTangents));


            for (int i = xLength; i >= 0; i--)
                for (int j = yLength; j >= 0; j--)
                    this.values[i, j] = new Keyframe2(this.xKeys[i], this.yKeys[j], values[i, j], xinTangents[i, j], xoutTangents[i, j], yinTangents[i, j], youtTangents[i, j], xinyinTangents[i, j], xinyoutTangents[i, j], xoutyinTangents[i, j], xoutyoutTangents[i, j]);
        }
        public FloatCurve2(IEnumerable<float> xKeys, IEnumerable<float> yKeys, Keyframe2[,] values) : this(xKeys, yKeys)
        {
            int xLength = this.xKeys.Length - 1;
            int yLength = this.yKeys.Length - 1;

            if ((values.GetUpperBound(0) != xLength) || (values.GetUpperBound(1) != yLength))
                throw new ArgumentException("Array dimensions do not match provided keys.", nameof(values));
            for (int i = xLength; i >= 0; i--)
                for (int j = yLength; j >= 0; j--)
                    this.values[i, j] = values[i, j];
        }

        public static FloatCurve2 ComputeCurve(IEnumerable<float> xKeys, IEnumerable<float> yKeys, Func<float, float, float> func, DiffSettings settings = new DiffSettings())
            => ComputeCurve(xKeys.Select(k => (k, false)), yKeys.Select(k => (k, false)), func, settings);

        private ref struct EvalPts
        {
            public float v0, v1, v2, v3, v4, v5, v6, v7, v8;
        }

        public static FloatCurve2 ComputeCurve(IEnumerable<(float value, bool continuousDerivative)> xKeys, IEnumerable<(float value, bool continuousDerivative)> yKeys, Func<float, float, float> func, DiffSettings settings = new DiffSettings())
        {
            IList<(float value, bool continuousDerivative)> xKeys_ = xKeys as IList<(float, bool)> ?? xKeys.ToArray();
            IList<(float value, bool continuousDerivative)> yKeys_ = yKeys as IList<(float, bool)> ?? yKeys.ToArray();

            FloatCurve2 curve = new FloatCurve2(xKeys_.Select(k => k.value), yKeys_.Select(k => k.value));
            int lx = xKeys_.Count - 1, ly = yKeys_.Count - 1;

            for (int i = lx; i >= 0; i--)
            {
                float deltaX = xKeys_[i].continuousDerivative ? settings.dx_continuous : settings.dx;
                float invDeltaX = 1 / deltaX;
                for (int j = ly; j >= 0; j--)
                {
                    float deltaY = yKeys_[j].continuousDerivative ? settings.dy_continuous : settings.dy;
                    float invDeltaY = 1 / deltaY;
                    float invDelta2 = invDeltaX * invDeltaY;

                    // Mapping is as follows:
                    //  6   2   5
                    //  3   0   1
                    //  8   4   7

                    EvalPts values = new EvalPts();

                    // Calculate base value
                    values.v0 = func(xKeys_[i].value, yKeys_[j].value);
                    // Calculate X differential values (1 and 3)
                    if (i < lx)
                    {
                        values.v1 = func(xKeys_[i].value + deltaX, yKeys_[j].value);
                        if (i > 0)
                        {
                            if (xKeys_[i].continuousDerivative)
                                values.v3 = 2 * values.v0 - values.v1;
                            else
                                values.v3 = func(xKeys_[i].value - deltaX, yKeys_[j].value);
                        }
                        else
                            values.v3 = values.v0;
                    }
                    else if (i > 0)
                    {
                        values.v3 = func(xKeys_[i].value - deltaX, yKeys_[j].value);
                        values.v1 = values.v0;
                    }
                    else
                    {
                        values.v1 = values.v0;
                        values.v3 = values.v0;
                    }
                    // Calculate Y differential values (2 and 4)
                    if (j < ly)
                    {
                        values.v2 = func(xKeys_[i].value, yKeys_[j].value + deltaY);
                        if (j > 0)
                        {
                            if (yKeys_[j].continuousDerivative)
                                values.v4 = 2 * values.v0 - values.v2;
                            else
                                values.v4 = func(xKeys_[i].value, yKeys_[j].value - deltaY);
                        }
                        else
                            values.v4 = values.v0;
                    }
                    else if (j > 0)
                    {
                        values.v4 = func(xKeys_[i].value, yKeys_[j].value - deltaY);
                        values.v2 = values.v0;
                    }
                    else
                    {
                        values.v2 = values.v0;
                        values.v4 = values.v0;
                    }
                    // Calculate combined differential values (5, 6, 7, and 8)
                    if (settings.zeroCrossDiff)
                    {
                        values.v5 = values.v0;
                        values.v6 = values.v0;
                        values.v7 = values.v0;
                        values.v8 = values.v0;
                    }
                    else if (i < lx && j < ly)
                    {
                        values.v5 = func(xKeys_[i].value + deltaX, yKeys_[j].value + deltaY);
                        if (i > 0)
                        {
                            if (xKeys_[i].continuousDerivative)
                                values.v6 = 2 * values.v2 - values.v5;
                            else
                                values.v6 = func(xKeys_[i].value - deltaX, yKeys_[j].value + deltaY);
                        }
                        else
                            values.v6 = values.v0;
                        if (j > 0)
                        {
                            if (yKeys_[j].continuousDerivative)
                                values.v7 = 2 * values.v1 - values.v5;
                            else
                                values.v7 = func(xKeys_[i].value + deltaX, yKeys_[j].value - deltaY);
                        }
                        else
                            values.v7 = values.v0;
                        if (i > 0 && j > 0)
                        {
                            if (xKeys_[i].continuousDerivative && yKeys_[j].continuousDerivative)
                                values.v8 = 2 * values.v0 - values.v5;
                            else if (xKeys_[i].continuousDerivative)
                                values.v8 = 2 * values.v4 - values.v7;
                            else if (yKeys_[j].continuousDerivative)
                                values.v8 = 2 * values.v2 - values.v6;
                            else
                                values.v8 = func(xKeys_[i].value - deltaX, yKeys_[j].value - deltaY);
                        }
                        else
                            values.v8 = values.v0;
                    }
                    else if (i > 0 && j < ly)   // i >= lx  5 and 7 are not used
                    {
                        values.v5 = values.v0;
                        values.v7 = values.v0;
                        values.v6 = func(xKeys_[i].value - deltaX, yKeys_[j].value + deltaY);
                        if (j > 0)
                        {
                            if (yKeys_[j].continuousDerivative)
                                values.v8 = 2 * values.v2 - values.v6;
                            else
                                values.v8 = func(xKeys_[i].value - deltaX, yKeys_[j].value - deltaY);
                        }
                        else
                            values.v8 = values.v0;
                    }
                    else if (i < lx && j > 0)   // j >= ly  5 and 6 are not used
                    {
                        values.v5 = values.v0;
                        values.v6 = values.v0;
                        values.v7 = func(xKeys_[i].value + deltaX, yKeys_[j].value - deltaY);
                        if (i > 0)
                        {
                            if (xKeys_[i].continuousDerivative)
                                values.v8 = 2 * values.v4 - values.v7;
                            else
                                values.v8 = func(xKeys_[i].value - deltaX, yKeys_[j].value - deltaY);
                        }
                        else
                            values.v8 = values.v0;
                    }
                    else if (i > 0 && j > 0)    // i >= lx & j >= ly    5, 6, and 7 are not used
                    {
                        values.v5 = values.v0;
                        values.v6 = values.v0;
                        values.v7 = values.v0;
                        values.v8 = func(xKeys_[i].value - deltaX, yKeys_[j].value - deltaY);
                    }
                    else
                    {
                        values.v5 = values.v0;
                        values.v6 = values.v0;
                        values.v7 = values.v0;
                        values.v8 = values.v0;
                    }

                    // Mapping is as follows:
                    //  6   2   5
                    //  3   0   1
                    //  8   4   7

                    // 'Values' becomes differentials here to save allocations
                    float values2 = values.v2;
                    float values4 = values.v4;
                    values.v1 = (values.v1 - values.v0) * invDeltaX;
                    values.v3 = (values.v0 - values.v3) * invDeltaX;
                    values.v2 = (values.v2 - values.v0) * invDeltaY;
                    values.v4 = (values.v0 - values.v4) * invDeltaY;
                    if (settings.zeroCrossDiff)
                    {
                        values.v5 = 0;
                        values.v6 = 0;
                        values.v7 = 0;
                        values.v8 = 0;
                    }
                    else
                    {
                        values.v5 = ((values.v5 - values2) * invDeltaX - values.v1) * invDeltaY;
                        values.v6 = ((values2 - values.v6) * invDeltaX - values.v3) * invDeltaY;
                        values.v7 = (values.v1 - (values.v7 - values4) * invDeltaX) * invDeltaY;
                        values.v8 = (values.v3 - (values4 - values.v8) * invDeltaX) * invDeltaY;
                    }

                    curve.values[i, j] = new Keyframe2(
                        xKeys_[i].value, yKeys_[j].value,
                        values.v0,
                        values.v3, values.v1,
                        values.v4, values.v2,
                        values.v8, values.v6, values.v7, values.v5);
                }
            }
            return curve;
        }

        protected static int FindIndex(float[] keys, float value, out bool exact)
        {
            int result = Array.BinarySearch(keys, value);
            if (result >= 0)
            {
                exact = true;
                return result;
            }
            exact = false;
            return (~result) - 1;
        }

        private readonly struct Knowns
        {
            public readonly float a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p;
            public Knowns(Keyframe2[,] values, int xSquare, int ySquare, float dx, float dy)
            {
                a = values[xSquare, ySquare].value;
                b = values[xSquare + 1, ySquare].value;
                c = values[xSquare, ySquare + 1].value;
                d = values[xSquare + 1, ySquare + 1].value;
                e = values[xSquare, ySquare].dDx_out * dx;
                f = values[xSquare + 1, ySquare].dDx_in * dx;
                g = values[xSquare, ySquare + 1].dDx_out * dx;
                h = values[xSquare + 1, ySquare + 1].dDx_in * dx;
                i = values[xSquare, ySquare].dDy_out * dy;
                j = values[xSquare + 1, ySquare].dDy_out * dy;
                k = values[xSquare, ySquare + 1].dDy_in * dy;
                l = values[xSquare + 1, ySquare + 1].dDy_in * dy;
                m = values[xSquare, ySquare].ddDx_out_Dy_out * dx * dy;
                n = values[xSquare + 1, ySquare].ddDx_in_Dy_out * dx * dy;
                o = values[xSquare, ySquare + 1].ddDx_out_Dy_in * dx * dy;
                p = values[xSquare + 1, ySquare + 1].ddDx_in_Dy_in * dx * dy;
            }
            public override int GetHashCode()
            {
                HashCode hashCode = new HashCode();
                hashCode.Add(a);    // 0
                hashCode.Add(b);    // 1
                hashCode.Add(c);    // 2
                hashCode.Add(d);    // 3
                hashCode.Add(e);    // 4
                hashCode.Add(f);    // 5
                hashCode.Add(g);    // 6
                hashCode.Add(h);    // 7
                hashCode.Add(i);    // 8
                hashCode.Add(j);    // 9
                hashCode.Add(k);    // 10
                hashCode.Add(l);    // 11
                hashCode.Add(m);    // 12
                hashCode.Add(n);    // 13
                hashCode.Add(o);    // 14
                hashCode.Add(p);    // 15
                return hashCode.ToHashCode();
            }
        }

        private readonly struct Coefficients
        {
            public readonly float a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p;
            public readonly int knownsHash;
#if OUTSIDE_UNITY
            public readonly bool isValid = false;
#else
            public readonly bool isValid;
#endif
            public Coefficients(in Knowns knowns)
            {
                knownsHash = knowns.GetHashCode();
                a = 1 * knowns.a;
                b = 1 * knowns.e;
                c = -3 * knowns.a + 3 * knowns.b - 2 * knowns.e - 1 * knowns.f;
                d = 2 * knowns.a - 2 * knowns.b + 1 * knowns.e + 1 * knowns.f;
                e = 1 * knowns.i;
                f = 1 * knowns.m;
                g = -3 * knowns.i + 3 * knowns.j - 2 * knowns.m - 1 * knowns.n;
                h = 2 * knowns.i - 2 * knowns.j + 1 * knowns.m + 1 * knowns.n;
                i = -3 * knowns.a + 3 * knowns.c - 2 * knowns.i - 1 * knowns.k;
                j = -3 * knowns.e + 3 * knowns.g - 2 * knowns.m - 1 * knowns.o;
                k = 9 * knowns.a - 9 * knowns.b - 9 * knowns.c + 9 * knowns.d + 6 * knowns.e + 3 * knowns.f - 6 * knowns.g - 3 * knowns.h + 6 * knowns.i - 6 * knowns.j + 3 * knowns.k - 3 * knowns.l + 4 * knowns.m + 2 * knowns.n + 2 * knowns.o + 1 * knowns.p;
                l = -6 * knowns.a + 6 * knowns.b + 6 * knowns.c - 6 * knowns.d - 3 * knowns.e - 3 * knowns.f + 3 * knowns.g + 3 * knowns.h - 4 * knowns.i + 4 * knowns.j - 2 * knowns.k + 2 * knowns.l - 2 * knowns.m - 2 * knowns.n - 1 * knowns.o - 1 * knowns.p;
                m = 2 * knowns.a - 2 * knowns.c + 1 * knowns.i + 1 * knowns.k;
                n = 2 * knowns.e - 2 * knowns.g + 1 * knowns.m + 1 * knowns.o;
                o = -6 * knowns.a + 6 * knowns.b + 6 * knowns.c - 6 * knowns.d - 4 * knowns.e - 2 * knowns.f + 4 * knowns.g + 2 * knowns.h - 3 * knowns.i + 3 * knowns.j - 3 * knowns.k + 3 * knowns.l - 2 * knowns.m - 1 * knowns.n - 2 * knowns.o - 1 * knowns.p;
                p = 4 * knowns.a - 4 * knowns.b - 4 * knowns.c + 4 * knowns.d + 2 * knowns.e + 2 * knowns.f - 2 * knowns.g - 2 * knowns.h + 2 * knowns.i - 2 * knowns.j + 2 * knowns.k - 2 * knowns.l + 1 * knowns.m + 1 * knowns.n + 1 * knowns.o + 1 * knowns.p;
                isValid = true;
            }
        }

        private ref Coefficients GetCoeffs(float timeX, float timeY, out float normalizedX, out float normalizedY, out int xSquare, out int ySquare, bool prefer1 = false)
        {
            if (timeX <= xKeys[0])
            {
                xSquare = 0;
                timeX = xKeys[0];
            }
            else if (timeX >= xKeys[xKeys.Length - 1])
            {
                xSquare = xKeys.Length - 2;
                timeX = xKeys[xKeys.Length - 1];
            }
            else
            {
                xSquare = FindIndex(xKeys, timeX, out bool exactX);
                if (prefer1 && exactX)
                    xSquare -= 1;
            }

            if (timeY <= yKeys[0])
            {
                ySquare = 0;
                timeY = yKeys[0];
            }
            else if (timeY >= yKeys[yKeys.Length - 1])
            {
                ySquare = yKeys.Length - 2;
                timeY = yKeys[yKeys.Length - 1];
            }
            else
            {
                ySquare = FindIndex(yKeys, timeY, out bool exactY);
                if (prefer1 && exactY)
                    ySquare -= 1;
            }

            float dx = (xKeys[xSquare + 1] - xKeys[xSquare]);
            float dy = (yKeys[ySquare + 1] - yKeys[ySquare]);
            normalizedX = Mathf.Clamp01((timeX - xKeys[xSquare]) / dx);
            normalizedY = Mathf.Clamp01((timeY - yKeys[ySquare]) / dy);

            Knowns knowns = new Knowns(values, xSquare, ySquare, dx, dy);
            int knownsHash = knowns.GetHashCode();

            ref Coefficients thisCoeff = ref coefficientsCache[xSquare, ySquare];

            if (!thisCoeff.isValid || thisCoeff.knownsHash != knownsHash)
                coefficientsCache[xSquare, ySquare] = new Coefficients(knowns);

            return ref thisCoeff;
        }

        public float Evaluate(float timeX, float timeY)
        {
            if (xKeys.Length == 0 || yKeys.Length == 0)
                return 0;
            ReadWriteLock.EnterReadLock();
            try
            {
                ref Coefficients coeffs = ref GetCoeffs(timeX, timeY, out float x, out float y, out _, out _);

                float x2 = x * x;
                float x3 = x2 * x;
                float y2 = y * y;
                float y3 = y2 * y;

                return (coeffs.a + coeffs.b * x + coeffs.c * x2 + coeffs.d * x3) +
                    (coeffs.e + coeffs.f * x + coeffs.g * x2 + coeffs.h * x3) * y +
                    (coeffs.i + coeffs.j * x + coeffs.k * x2 + coeffs.l * x3) * y2 +
                    (coeffs.m + coeffs.n * x + coeffs.o * x2 + coeffs.p * x3) * y3;
            }
            finally
            {
                ReadWriteLock.ExitReadLock();
            }
        }

        public float EvaluateDerivative(float timeX, float timeY, (int, int) dimension, bool prefer1 = false)
        {
            if (xKeys.Length == 0 || yKeys.Length == 0)
                return 0;
            ReadWriteLock.EnterReadLock();
            try
            {
                ref Coefficients coeffs = ref GetCoeffs(timeX, timeY, out float x, out float y, out int xSquare, out int ySquare, prefer1);

                float x2 = x * x;
                float y2 = y * y;

                float dX = xKeys[xSquare + 1] - xKeys[xSquare];
                float dY = yKeys[ySquare + 1] - yKeys[ySquare];

                if (dimension.Equals((1, 0)))
                {
                    if (timeX < xKeys[0] || timeX > xKeys[xKeys.Length - 1])
                        return 0;
                    else
                    {
                        float y3 = y2 * y;
                        return ((coeffs.b + 2 * coeffs.c * x + 3 * coeffs.d * x2) +
                            (coeffs.f + 2 * coeffs.g * x + 3 * coeffs.h * x2) * y +
                            (coeffs.j + 2 * coeffs.k * x + 3 * coeffs.l * x2) * y2 +
                            (coeffs.n + 2 * coeffs.o * x + 3 * coeffs.p * x2) * y3) / dX;
                    }
                }
                else if (dimension.Equals((0, 1)))
                {
                    if (timeY < yKeys[0] || timeY > yKeys[yKeys.Length - 1])
                        return 0;
                    else
                    {
                        float x3 = x2 * x;
                        return ((coeffs.e + coeffs.f * x + coeffs.g * x2 + coeffs.h * x3) +
                            2 * (coeffs.i + coeffs.j * x + coeffs.k * x2 + coeffs.l * x3) * y +
                            3 * (coeffs.m + coeffs.n * x + coeffs.o * x2 + coeffs.p * x3) * y2) / dY;
                    }
                }
                else if (dimension.Equals((1, 1)))
                {
                    if ((timeX < xKeys[0] || timeX > xKeys[xKeys.Length - 1]) && (timeY < yKeys[0] || timeY > yKeys[yKeys.Length - 1]))
                        return 0;
                    else
                        return ((coeffs.f + 2 * coeffs.g * x + 3 * coeffs.h * x2) +
                            2 * (coeffs.j + 2 * coeffs.k * x + 3 * coeffs.l * x2) * y +
                            3 * (coeffs.n + 2 * coeffs.o * x + 3 * coeffs.p * x2) * y2) / (dX * dY);
                }
                else
                    throw new ArgumentOutOfRangeException(nameof(dimension));
            }
            finally
            {
                ReadWriteLock.ExitReadLock();
            }
        }

        public static FloatCurve2 Superposition(IEnumerable<FloatCurve2> curves)
        {
            SortedSet<float> xKeys = new SortedSet<float>();
            SortedSet<float> yKeys = new SortedSet<float>();
            foreach (FloatCurve2 curve in curves)
            {
                if (curve == null) continue;
                xKeys.UnionWith(curve.xKeys);
                yKeys.UnionWith(curve.yKeys);
            }
            return Superposition(curves, xKeys.ToArray(), yKeys.ToArray());
        }
        public static FloatCurve2 Superposition(IEnumerable<FloatCurve2> curves, IEnumerable<float> xKeys, IEnumerable<float> yKeys)
        {
            float[] xKeys_ = xKeys.Distinct().ToArray();
            float[] yKeys_ = yKeys.Distinct().ToArray();
            Array.Sort(xKeys_);
            Array.Sort(yKeys_);
            return Superposition(curves, xKeys_, yKeys_);
        }
        public static FloatCurve2 Superposition(IEnumerable<FloatCurve2> curves, IList<float> xKeys, IList<float> yKeys)
        {
            int xLength = xKeys.Count;
            int yLength = yKeys.Count;
            float[,] values = new float[xLength, yLength];
            float[,] dDx_in = new float[xLength, yLength];
            float[,] dDx_out = new float[xLength, yLength];
            float[,] dDy_in = new float[xLength, yLength];
            float[,] dDy_out = new float[xLength, yLength];
            float[,] ddDx_in_Dy_in = new float[xLength, yLength];
            float[,] ddDx_in_Dy_out = new float[xLength, yLength];
            float[,] ddDx_out_Dy_in = new float[xLength, yLength];
            float[,] ddDx_out_Dy_out = new float[xLength, yLength];


            foreach (FloatCurve2 curve in curves)
            {
                if (curve == null) continue;
                for (int i = xLength - 1; i >= 0; i--)
                {
                    float xTime = xKeys[i];
                    for (int j = yLength - 1; j >= 0; j--)
                    {
                        float yTime = yKeys[j];
                        if (curve.xKeysSet.Contains(xTime) && curve.yKeysSet.Contains(yTime))
                        {
                            Keyframe2 value = curve.values[FindIndex(curve.xKeys, xTime, out _), FindIndex(curve.yKeys, yTime, out _)];
                            values[i, j] += value.value;
                            dDx_in[i, j] += value.dDx_in;
                            dDx_out[i, j] += value.dDx_out;
                            dDy_in[i, j] += value.dDy_in;
                            dDy_out[i, j] += value.dDy_out;
                            ddDx_in_Dy_in[i, j] += value.ddDx_in_Dy_in;
                            ddDx_in_Dy_out[i, j] += value.ddDx_in_Dy_out;
                            ddDx_out_Dy_in[i, j] += value.ddDx_out_Dy_in;
                            ddDx_out_Dy_out[i, j] += value.ddDx_out_Dy_out;
                            continue;
                        }
                        values[i, j] += curve.Evaluate(xTime, yTime);
                        float ddx = curve.EvaluateDerivative(xTime, yTime, (1, 0));
                        float ddy = curve.EvaluateDerivative(xTime, yTime, (0, 1));
                        float dddxdy = curve.EvaluateDerivative(xTime, yTime, (1, 1));
                        dDx_out[i, j] += ddx;
                        dDy_out[i, j] += ddy;
                        if (curve.xKeysSet.Contains(xTime))
                        {
                            dDx_in[i, j] += curve.EvaluateDerivative(xTime, yTime, (1, 0), true);
                            dDy_in[i, j] += ddy;
                            float cross = curve.EvaluateDerivative(xTime, yTime, (1, 1), true);
                            ddDx_in_Dy_in[i, j] += cross;
                            ddDx_in_Dy_out[i, j] += cross;
                            ddDx_out_Dy_in[i, j] += dddxdy;
                            ddDx_out_Dy_out[i, j] += dddxdy;
                        }
                        else if (curve.yKeysSet.Contains(yTime))
                        {
                            dDx_in[i, j] += ddx;
                            dDy_in[i, j] += curve.EvaluateDerivative(xTime, yTime, (0, 1), true);
                            float cross = curve.EvaluateDerivative(xTime, yTime, (1, 1), true);
                            ddDx_in_Dy_in[i, j] += cross;
                            ddDx_in_Dy_out[i, j] += dddxdy;
                            ddDx_out_Dy_in[i, j] += cross;
                            ddDx_out_Dy_out[i, j] += dddxdy;
                        }
                        else
                        {
                            dDx_in[i, j] += ddx;
                            dDy_in[i, j] += ddy;
                            ddDx_in_Dy_in[i, j] += dddxdy;
                            ddDx_in_Dy_out[i, j] += dddxdy;
                            ddDx_out_Dy_in[i, j] += dddxdy;
                            ddDx_out_Dy_out[i, j] += dddxdy;
                        }
                    }
                }
            }
            return new FloatCurve2(xKeys, yKeys, values, dDx_in, dDx_out, dDy_in, dDy_out, ddDx_in_Dy_in, ddDx_in_Dy_out, ddDx_out_Dy_in, ddDx_out_Dy_out);
        }

        public static FloatCurve2 Subtract(FloatCurve2 minuend, IEnumerable<FloatCurve2> subtrahends)
        {
            SortedSet<float> xKeys = new SortedSet<float>(minuend.xKeys);
            SortedSet<float> yKeys = new SortedSet<float>(minuend.yKeys);
            foreach (FloatCurve2 curve in subtrahends)
            {
                if (curve == null) continue;
                xKeys.UnionWith(curve.xKeys);
                yKeys.UnionWith(curve.yKeys);
            }
            return Subtract(minuend, subtrahends, xKeys.ToArray(), yKeys.ToArray());
        }
        public static FloatCurve2 Subtract(FloatCurve2 minuend, IEnumerable<FloatCurve2> subtrahends, IEnumerable<float> xKeys, IEnumerable<float> yKeys)
        {
            float[] xKeys_ = xKeys.Distinct().ToArray();
            float[] yKeys_ = yKeys.Distinct().ToArray();
            Array.Sort(xKeys_);
            Array.Sort(yKeys_);
            return Subtract(minuend,subtrahends, xKeys_, yKeys_);
        }
        public static FloatCurve2 Subtract(FloatCurve2 minuend, FloatCurve2 subtrahend) => Subtract(minuend, AsEnumerable(subtrahend));
        private static IEnumerable<T> AsEnumerable<T>(T item) { yield return item; }
        public static FloatCurve2 Subtract(FloatCurve2 minuend, IEnumerable<FloatCurve2> subtrahends, IList<float> xKeys, IList<float> yKeys)
        {
            int xLength = xKeys.Count;
            int yLength = yKeys.Count;
            float[,] values = new float[xLength, yLength];
            float[,] dDx_in = new float[xLength, yLength];
            float[,] dDx_out = new float[xLength, yLength];
            float[,] dDy_in = new float[xLength, yLength];
            float[,] dDy_out = new float[xLength, yLength];
            float[,] ddDx_in_Dy_in = new float[xLength, yLength];
            float[,] ddDx_in_Dy_out = new float[xLength, yLength];
            float[,] ddDx_out_Dy_in = new float[xLength, yLength];
            float[,] ddDx_out_Dy_out = new float[xLength, yLength];

            bool negate = false;

            foreach (FloatCurve2 curve in AsEnumerable(minuend).Concat(subtrahends))
            {
                int multiplier = negate ? -1 : 1;
                if (curve == null) continue;
                for (int i = xLength - 1; i >= 0; i--)
                {
                    float xTime = xKeys[i];
                    for (int j = yLength - 1; j >= 0; j--)
                    {
                        float yTime = yKeys[j];
                        if (curve.xKeysSet.Contains(xTime) && curve.yKeysSet.Contains(yTime))
                        {
                            Keyframe2 value = curve.values[FindIndex(curve.xKeys, xTime, out _), FindIndex(curve.yKeys, yTime, out _)];
                            values[i, j] += value.value * multiplier;
                            dDx_in[i, j] += value.dDx_in * multiplier;
                            dDx_out[i, j] += value.dDx_out * multiplier;
                            dDy_in[i, j] += value.dDy_in * multiplier;
                            dDy_out[i, j] += value.dDy_out * multiplier;
                            ddDx_in_Dy_in[i, j] += value.ddDx_in_Dy_in * multiplier;
                            ddDx_in_Dy_out[i, j] += value.ddDx_in_Dy_out * multiplier;
                            ddDx_out_Dy_in[i, j] += value.ddDx_out_Dy_in * multiplier;
                            ddDx_out_Dy_out[i, j] += value.ddDx_out_Dy_out * multiplier;
                            continue;
                        }
                        values[i, j] += curve.Evaluate(xTime, yTime) * multiplier;
                        float ddx = curve.EvaluateDerivative(xTime, yTime, (0, 1)) * multiplier;
                        float ddy = curve.EvaluateDerivative(xTime, yTime, (1, 0)) * multiplier;
                        float dddxdy = curve.EvaluateDerivative(xTime, yTime, (1, 1)) * multiplier;
                        dDx_out[i, j] += ddx;
                        dDy_out[i, j] += ddy;
                        if (curve.xKeysSet.Contains(xTime))
                        {
                            dDx_in[i, j] += curve.EvaluateDerivative(xTime, yTime, (0, 1), true) * multiplier;
                            float cross = curve.EvaluateDerivative(xTime, yTime, (1, 1), true) * multiplier;
                            ddDx_in_Dy_in[i, j] += cross;
                            ddDx_in_Dy_out[i, j] += cross;
                            ddDx_out_Dy_in[i, j] += dddxdy;
                            ddDx_out_Dy_out[i, j] += dddxdy;
                        }
                        else if (curve.yKeysSet.Contains(yTime))
                        {
                            dDy_in[i, j] += curve.EvaluateDerivative(xTime, yTime, (0, 1), true) * multiplier;
                            float cross = curve.EvaluateDerivative(xTime, yTime, (1, 1), true) * multiplier;
                            ddDx_in_Dy_in[i, j] += cross;
                            ddDx_out_Dy_in[i, j] += cross;
                            ddDx_in_Dy_out[i, j] += dddxdy;
                            ddDx_out_Dy_out[i, j] += dddxdy;
                        }
                        else
                        {
                            dDx_in[i, j] += ddx;
                            dDy_in[i, j] += ddy;
                            ddDx_in_Dy_in[i, j] += dddxdy;
                            ddDx_in_Dy_out[i, j] += dddxdy;
                            ddDx_out_Dy_in[i, j] += dddxdy;
                            ddDx_out_Dy_out[i, j] += dddxdy;
                        }
                    }
                }
                negate = true;
            }
            return new FloatCurve2(xKeys, yKeys, values, dDx_in, dDx_out, dDy_in, dDy_out, ddDx_in_Dy_in, ddDx_in_Dy_out, ddDx_out_Dy_in, ddDx_out_Dy_out);
        }

        public static FloatCurve2 Simplify(FloatCurve2 curve, int axis, KSPClassExtensions.KeyframeMargins removalPredicate = null)
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
        private static FloatCurve2 SimplifyHorizontal(FloatCurve2 curve, KSPClassExtensions.KeyframeMargins removalPredicate)
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
                    float computedValue = KSPClassExtensions.EvaluateBicubic(xKeys[i], k0.timeX, k1.timeX, k0.value, k0.dDx_out, k1.dDx_in, k1.value);
                    float computedTangent = KSPClassExtensions.EvaluateBicubicDerivative(xKeys[i], k0.timeX, k1.timeX, k0.value, k0.dDx_out, k1.dDx_in, k1.value);
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
        private static FloatCurve2 SimplifyVertical(FloatCurve2 curve, KSPClassExtensions.KeyframeMargins removalPredicate)
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
                    float computedValue = KSPClassExtensions.EvaluateBicubic(yKeys[j], k0.timeY, k1.timeY, k0.value, k0.dDy_out, k1.dDy_in, k1.value);
                    float computedTangent = KSPClassExtensions.EvaluateBicubicDerivative(yKeys[j], k0.timeY, k1.timeY, k0.value, k0.dDy_out, k1.dDy_in, k1.value);
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

        public void Dispose()
            => ReadWriteLock.Dispose();

        public struct Keyframe2
        {
            public float timeX;
            public float timeY;
            public float value;
            public float dDx_in;
            public float dDx_out;
            public float dDy_in;
            public float dDy_out;
            public float ddDx_in_Dy_in;
            public float ddDx_in_Dy_out;
            public float ddDx_out_Dy_in;
            public float ddDx_out_Dy_out;

            public Keyframe2(float timeX, float timeY, float value, float dDx_in, float dDx_out, float dDy_in, float dDy_out, float ddDx_in_Dy_in, float ddDx_in_Dy_out, float ddDx_out_Dy_in, float ddDx_out_Dy_out)
            {
                this.timeX = timeX;
                this.timeY = timeY;
                this.value = value;
                this.dDx_in = dDx_in;
                this.dDx_out = dDx_out;
                this.dDy_in = dDy_in;
                this.dDy_out = dDy_out;
                this.ddDx_in_Dy_in = ddDx_in_Dy_in;
                this.ddDx_in_Dy_out = ddDx_in_Dy_out;
                this.ddDx_out_Dy_in = ddDx_out_Dy_in;
                this.ddDx_out_Dy_out = ddDx_out_Dy_out;
            }

            public Keyframe2(float timeX, float timeY, float value, float dDx, float dDy, float ddDyDx) : this(timeX, timeY, value, dDx, dDx, dDy, dDy, ddDyDx, ddDyDx, ddDyDx, ddDyDx) { }

            public static Keyframe2 operator +(Keyframe2 key, float value)
            {
                key.value += value;
                return key;
            }
            /*public static Keyframe2 operator + (Keyframe2 key1, Keyframe2 key2)
            {
                if (key1.timeX != key2.timeX || key1.timeY != key2.timeY)
                    throw new ArgumentException("The given keys did not match coordinates.");
                return new Keyframe2(key1.timeX, key2.timeX, key1.value + key2.value, key1.dDx + key2.dDx, key1.dDy + key2.dDy, key1.ddDxDy + key2.ddDxDy);
            }*/
        }

        public readonly struct DiffSettings
        {
            public readonly float dx;
            public readonly float dy;
            public readonly float dx_continuous;
            public readonly float dy_continuous;
            public readonly bool zeroCrossDiff;
            public DiffSettings(float dx, float dy, bool zeroCrossDiff = false) : this(dx, dy, dx, dy, zeroCrossDiff) { }
            public DiffSettings(float dx, float dy, float dx_continuous, float dy_continuous, bool zeroCrossDiff = false)
            {
                this.dx = dx;
                this.dx_continuous = dx_continuous;
                this.dy = dy;
                this.dy_continuous = dy_continuous;
                this.zeroCrossDiff = zeroCrossDiff;
            }
        }
    }
}
