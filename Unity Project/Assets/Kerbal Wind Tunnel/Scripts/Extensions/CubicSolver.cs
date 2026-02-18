using System;
using System.Collections.Generic;

namespace KerbalWindTunnel.Extensions
{
    public sealed class CubicSolver
    {
        private const double eps_default = 1e-8;
        private double eps = eps_default;
        private const double oneThird = 1.0 / 3.0;
        private ApproxEqualityComparer equalityComparer = new ApproxEqualityComparer((float)eps_default);
        public ApproxEqualityComparer EqualityComparer { get => equalityComparer; }

        private double EPS
        {
            get => eps;
            set
            {
                eps = value;
                equalityComparer = new ApproxEqualityComparer((float)value);
            }
        }

        // Form is a * x^3 + b * x^2 + c * x + d = 0
        public HashSet<float> SolveCubic(float a, float b, float c, float d)
        {
            HashSet<float> roots = new HashSet<float>(equalityComparer);

            // Degenerate cases
            if (Math.Abs(a) < eps)
            {
                return SolveQuadratic(b, c, d);
            }

            // Normalize coefficients
            double inv_a = 1 / a;
            double A = b * inv_a;
            double B = c * inv_a;
            double C = d * inv_a;

            // Depressed cubic: x = y - A/3
            double sqA = A * A;
            double p = oneThird * (-sqA * oneThird + B);
            double q = 0.5 * (2.0 / 27.0 * A * sqA - A * B * oneThird + C);

            double discriminant = q * q + p * p * p;

            if (discriminant > EPS)
            {
                // One real root
                double sqrtD = Math.Sqrt(discriminant);
                double u = CubeRoot(-q + sqrtD);
                double v = CubeRoot(-q - sqrtD);

                double x = u + v - A * oneThird;
                roots.Add((float)x);
            }
            else if (Math.Abs(discriminant) <= EPS)
            {
                // Multiple real roots (at least two equal)
                double u = CubeRoot(-q);

                double x1 = 2 * u - A * oneThird;
                double x2 = -u - A * oneThird;

                roots.Add((float)x1);
                roots.Add((float)x2);
            }
            else
            {
                // Three distinct real roots
                double phi = Math.Acos(-q / Math.Sqrt(-(p * p * p)));
                double t = 2 * Math.Sqrt(-p);

                for (int k = 0; k < 3; k++)
                {
                    double x = t * Math.Cos((phi + 2 * Math.PI * k) * oneThird) - A * oneThird;
                    roots.Add((float)x);
                }
            }

            return roots;
        }

        private HashSet<float> SolveQuadratic(float a, float b, float c)
        {
            HashSet<float> roots = new HashSet<float>();

            if (Math.Abs(a) < eps)
            {
                if (Math.Abs(b) < eps)
                    return roots;

                roots.Add((float)(-c / (double)b));
                return roots;
            }

            double discriminant = (double)b * b - 4 * (double)a * c;

            if (discriminant > EPS)
            {
                double sqrtD = Math.Sqrt(discriminant);
                roots.Add((float)((-b + sqrtD) / (2 * (double)a)));
                roots.Add((float)((-b - sqrtD) / (2 * (double)a)));
            }
            else if (Math.Abs(discriminant) <= EPS)
            {
                roots.Add((float)(-b / (2 * (double)a)));
            }

            return roots;
        }

        private static double CubeRoot(double x)
            => x >= 0 ? Math.Pow(x, oneThird) : -Math.Pow(-x, oneThird);

        public sealed class ApproxEqualityComparer : IEqualityComparer<float> 
        {
            public float EPS { get; private set; }
            public ApproxEqualityComparer(float eps)
            {
                EPS = eps;
            }
            public bool Equals(float x, float y)
                => Math.Abs(x - y) <= EPS;

            public int GetHashCode(float obj)
                => obj.GetHashCode();
        }
    }
}
