﻿using Graphing.Extensions;
using System;
using System.Linq;
using UnityEngine;

namespace Graphing
{
    /// <summary>
    /// A class representing a line graph with a z component.
    /// </summary>
    public class Line3Graph : Graphable3, ILineGraph
    {
        public override float XMin { get => base.XMin; set => Debug.LogError("Cannot set bounds for this type of object."); }
        public override float XMax { get => base.XMax; set => Debug.LogError("Cannot set bounds for this type of object."); }
        public override float YMin { get => base.YMin; set => Debug.LogError("Cannot set bounds for this type of object."); }
        public override float YMax { get => base.YMax; set => Debug.LogError("Cannot set bounds for this type of object."); }
        /// <summary>
        /// The width of the line in pixels.
        /// </summary>
        public float LineWidth { get => lineWidth; set { lineWidth = value; OnDisplayChanged(new LineWidthChangedEventArgs(LineWidth)); } }
        protected float lineWidth = 1;
        protected Vector3[] _values;
        /// <summary>
        /// Gets or sets the points in the line.
        /// </summary>
        public Vector3[] Values
        {
            get => _values;
            set => SetValues(value);
        }

        protected override float DefaultColorFunc(Vector3 value) => 0;

        private Line3Graph() { UseSingleColor = true; }

        /// <summary>
        /// Constructs a new <see cref="Line3Graph"/> with the provided points.
        /// </summary>
        /// <param name="values"></param>
        public Line3Graph(Vector3[] values) : this()
        {
            SetValuesInternal(values);
        }

        /// <summary>
        /// Draws the object on the specified <see cref="Texture2D"/>.
        /// </summary>
        /// <param name="texture">The texture on which to draw the object.</param>
        /// <param name="xLeft">The X axis lower bound.</param>
        /// <param name="xRight">The X axis upper bound.</param>
        /// <param name="yBottom">The Y axis lower bound.</param>
        /// <param name="yTop">The Y axis upper bound.</param>
        public override void Draw(ref Texture2D texture, float xLeft, float xRight, float yBottom, float yTop)
        {
            if (!Visible) return;
            float xRange = xRight - xLeft;
            float yRange = yTop - yBottom;
            int width = texture.width;
            int height = texture.height;
            int[] xPix, yPix;
            // TODO: Add robustness for NaNs and Infinities.
            if (!Transpose)
            {
                xPix = _values.Select(vect => Mathf.RoundToInt((vect.x - xLeft) / xRange * width)).ToArray();
                yPix = _values.Select(vect => Mathf.RoundToInt((vect.y - yBottom) / yRange * height)).ToArray();
            }
            else
            {
                xPix = _values.Select(vect => Mathf.RoundToInt((vect.y - yBottom) / yRange * width)).ToArray();
                yPix = _values.Select(vect => Mathf.RoundToInt((vect.x - xLeft) / xRange * height)).ToArray();
            }

            for (int i = _values.Length - 2; i >= 0; i--)
            {
                DrawingHelper.DrawLine(ref texture, xPix[i], yPix[i], xPix[i + 1], yPix[i + 1], EvaluateColor(new Vector3(xPix[i], yPix[i], _values[i].z)), EvaluateColor(new Vector3(xPix[i + 1], yPix[i + 1], _values[i + 1].z)));
                for (int w = 2; w <= LineWidth; w++)
                {
                    int l = w % 2 == 0 ? (-w) >> 1 : (w - 1) >> 1;
                    DrawingHelper.DrawLine(ref texture, xPix[i] + l, yPix[i], xPix[i + 1] + l, yPix[i + 1], EvaluateColor(new Vector3(xPix[i], yPix[i], _values[i].z)), EvaluateColor(new Vector3(xPix[i + 1], yPix[i + 1], _values[i + 1].z)));
                    DrawingHelper.DrawLine(ref texture, xPix[i], yPix[i] + l, xPix[i + 1], yPix[i + 1] + l, EvaluateColor(new Vector3(xPix[i], yPix[i], _values[i].z)), EvaluateColor(new Vector3(xPix[i + 1], yPix[i + 1], _values[i + 1].z)));
                }
            }

            texture.Apply();
        }

        /// <summary>
        /// Gets a value from the object given a selected coordinate.
        /// </summary>
        /// <param name="x">The x value of the selected coordinate.</param>
        /// <param name="y">The y value of the selected coordinate.</param>
        /// <returns></returns>
        public override float ValueAt(float x, float y)
            => ValueAt(x, y, 1, 1);
        /// <summary>
        /// Gets a value from the object given a selected coordinate.
        /// </summary>
        /// <param name="x">The x value of the selected coordinate.</param>
        /// <param name="y">The y value of the selected coordinate.</param>
        /// <param name="width">The x domain of the graph.</param>
        /// <param name="height">The y range of the graph.</param>
        /// <returns></returns>
        public virtual float ValueAt(float x, float y, float width, float height)
        {
            if (_values.Length <= 0)
                return 0;

            if (Transpose) x = y;

            Vector2 point = new Vector2(x, y);
            Vector2 closestPoint = new Vector2(_values[0].x, _values[0].y);
            float currentDistance = float.PositiveInfinity;
            int length = _values.Length;
            for (int i = 0; i < length - 1 - 1; i++)
            {
                Vector2 pt1 = new Vector2(_values[i].x, _values[i].y);
                Vector2 pt2 = new Vector2(_values[i + 1].x, _values[i + 1].y);
                Vector2 lineDir = (pt2 - pt1).normalized;
                Vector2 closestPt = pt1 + Vector2.Dot(point - pt1, lineDir) * lineDir;
                if (Vector2.Dot(closestPt - pt1, lineDir) <= 0)
                {
                    closestPt = pt1;
                }
                else if ((closestPt - pt1).sqrMagnitude >= (pt2 - pt1).sqrMagnitude)
                {
                    closestPt = pt2;
                }
                Vector2 LocalTransform(Vector2 vector) => new Vector2(vector.x / width, vector.y / height);
                float distance = (LocalTransform(point) - LocalTransform(closestPoint)).sqrMagnitude;
                if (distance < currentDistance)
                {
                    currentDistance = distance;
                    closestPoint = closestPt;
                }
            }
            return closestPoint.y;
        }

        /// <summary>
        /// Sets the values in the line.
        /// </summary>
        /// <param name="values"></param>
        public void SetValues(Vector3[] values)
        {
            SetValuesInternal(values);
            OnValuesChanged(new ValuesChangedEventArgs(Values, new (float, float)[] { (XMin, XMax), (YMin, YMax), (ZMin, ZMax) }));
        }

        private void SetValuesInternal(Vector3[] values)
        {
            _values = values;
            if (_values.Length <= 0)
            {
                this.yMin = this.yMax = this.xMin = this.xMax = this.zMin = this.zMax = 0;
                return;
            }
            float xLeft = float.MaxValue;
            float xRight = float.MinValue;
            float yMin = float.MaxValue;
            float yMax = float.MinValue;
            float zMin = float.MaxValue;
            float zMax = float.MinValue;
            for (int i = values.Length - 1; i >= 0; i--)
            {
                if (!float.IsInfinity(values[i].x) && !float.IsNaN(values[i].x))
                {
                    xLeft = Math.Min(xLeft, values[i].x);
                    xRight = Math.Max(xRight, values[i].x);
                }
                if (!float.IsInfinity(values[i].y) && !float.IsNaN(values[i].y))
                {
                    yMin = Math.Min(yMin, values[i].y);
                    yMax = Math.Max(yMax, values[i].y);
                }
                if (!float.IsInfinity(values[i].z) && !float.IsNaN(values[i].z))
                {
                    zMin = Math.Min(zMin, values[i].z);
                    zMax = Math.Max(zMax, values[i].z);
                }
            }
            this.xMax = xRight;
            this.xMin = xLeft;
            this.yMax = yMax;
            this.yMin = yMin;
            this.zMax = zMax;
            this.zMin = zMin;
        }

        /// <summary>
        /// Gets a formatted value from the object given a selected coordinate.
        /// </summary>
        /// <param name="x">The x value of the selected coordinate.</param>
        /// <param name="y">The y value of the selected coordinate.</param>
        /// <param name="withName">When true, requests the object include its name.</param>
        /// <returns></returns>
        public override string GetFormattedValueAt(float x, float y, bool withName = false)
            => GetFormattedValueAt(x, y, 1, 1, withName);
        /// <summary>
        /// Gets a formatted value from the object given a selected coordinate.
        /// </summary>
        /// <param name="x">The x value of the selected coordinate.</param>
        /// <param name="y">The y value of the selected coordinate.</param>
        /// <param name="width">The x domain of the graph.</param>
        /// <param name="height">The y range of the graph.</param>
        /// <param name="withName">When true, requests the object include its name.</param>
        /// <returns></returns>
        public virtual string GetFormattedValueAt(float x, float y, float width, float height, bool withName = false)
        {
            if (_values.Length <= 0) return "";
            return String.Format("{2}{0:" + StringFormat + "}{1}", ValueAt(x, y, width, height), ZUnit, withName && !String.IsNullOrEmpty(DisplayName) ? DisplayName + ": " : "");
        }

        /// <summary>
        /// Outputs the object's values to file.
        /// </summary>
        /// <param name="directory">The directory in which to place the file.</param>
        /// <param name="filename">The filename for the file.</param>
        /// <param name="sheetName">An optional sheet name for within the file.</param>
        public override void WriteToFile(string directory, string filename, string sheetName = "")
        {
            if (_values.Length <= 0)
                return;

            if (!System.IO.Directory.Exists(directory))
                System.IO.Directory.CreateDirectory(directory);

            if (sheetName == "")
                sheetName = this.Name.Replace("/", "-").Replace("\\", "-");

            string fullFilePath = string.Format("{0}/{1}{2}.csv", directory, filename, sheetName != "" ? "_" + sheetName : "");

            try
            {
                if (System.IO.File.Exists(fullFilePath))
                    System.IO.File.Delete(fullFilePath);
            }
            catch (Exception ex) { Debug.LogErrorFormat("Unable to delete file:{0}", ex.Message); }

            string strCsv = "";
            if (XName != "")
                strCsv += string.Format("{0} [{1}]", XName, XUnit != "" ? XUnit : "-");
            else
                strCsv += string.Format("{0}", XUnit != "" ? XUnit : "-");

            if (YName != "")
                strCsv += String.Format(",{0} [{1}]", YName, YUnit != "" ? YUnit : "-");
            else
                strCsv += String.Format(",{0}", YUnit != "" ? YUnit : "-");

            if (ZName != "")
                strCsv += String.Format(",{0} [{1}]", ZName, ZUnit != "" ? ZUnit : "-");
            else
                strCsv += String.Format(",{0}", ZUnit != "" ? ZUnit : "-");

            try
            {
                System.IO.File.AppendAllText(fullFilePath, strCsv + "\r\n");
            }
            catch (Exception ex) { Debug.LogException(ex); }

            for (int i = 0; i < _values.Length; i++)
            {
                strCsv = String.Format("{0}, {1}, {2:" + StringFormat.Replace("N", "F") + "}", _values[i].x, _values[i].y, _values[i].z);

                try
                {
                    System.IO.File.AppendAllText(fullFilePath, strCsv + "\r\n");
                }
                catch (Exception) { }
            }
        }
    }
}