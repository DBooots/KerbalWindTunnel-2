using System;

namespace KerbalWindTunnel.Extensions.Optimization
{
    public static class PeakFinding
    {
        public static float StepMinimize(Func<float, float> func, float start, float stepSize, out float value, int subdivisions = 0, int maxIterations = 500)
        {
            value = func(start);
            if (subdivisions < 0)
                return start;
            float pt = start;
            float nextValue = func(pt + stepSize);
            if (nextValue > value)
            {
                stepSize = -stepSize;
                nextValue = func(pt + stepSize);
            }
            int i = 0;
            float nextPt = pt + stepSize;
            while (nextValue < value)
            {
                pt = nextPt;
                value = nextValue;
                i++;
                if (i >= maxIterations)
                    break;
                nextPt += stepSize;
                nextValue = func(nextPt);
            }
            return StepMinimize(func, pt, stepSize / 2, out value, i < maxIterations ? subdivisions - 1 : -1, maxIterations);
        }
        public static float StepMaximize(Func<float, float> func, float start, float stepSize, out float value, int subdivisions = 0, int maxIterations = 500)
            => StepMinimize(i => -func(i), start, stepSize, out value, subdivisions, maxIterations);
    }
}
