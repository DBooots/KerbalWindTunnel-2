﻿using System;
using UnityEngine;

namespace Graphing
{
    /// <summary>
    /// An abstract class for directly graphable objects.
    /// </summary>
    public abstract class Graphable : IGraphable, IGraph
    {
        /// <summary>
        /// The name of the object.
        /// </summary>
        public string Name { get; set; } = "";

        private string _displayName = "";
        /// <summary>
        /// The display name of the object. Can be different than the <see cref="Name"/> of the object.
        /// </summary>
        public string DisplayName
        {
            get => string.IsNullOrEmpty(_displayName) ? Name : _displayName;
            set 
            {
                bool changed = _displayName != value;
                _displayName = value;
                if (changed) OnDisplayChanged(new DisplayNameChangedEventArgs(_displayName));
            }
        }
        /// <summary>
        /// The visibility status of the object.
        /// </summary>
        public bool Visible
        {
            get => _visible;
            set
            {
                bool changed = _visible != value;
                _visible = value;
                if (changed) OnDisplayChanged(new VisibilityChangedEventArgs(_visible));
            }
        }
        private bool _visible = true;
        /// <summary>
        /// Should the value be displayed on mouseover.
        /// </summary>
        public bool DisplayValue { get; set; } = true;
        /// <summary>
        /// The lower X bound of the object.
        /// </summary>
        public virtual float XMin { get => xMin; set { xMin = value; OnDisplayChanged(new BoundsChangedEventArgs(AxisUI.AxisDirection.Horizontal, XMin, XMax)); } }
        protected float xMin;
        /// <summary>
        /// The upper X bound of the object.
        /// </summary>
        public virtual float XMax { get => xMax; set { xMax = value; OnDisplayChanged(new BoundsChangedEventArgs(AxisUI.AxisDirection.Horizontal, XMin, XMax)); } }
        protected float xMax;
        /// <summary>
        /// The lower Y bound of the object.
        /// </summary>
        public virtual float YMin { get => yMin; set { yMin = value; OnDisplayChanged(new BoundsChangedEventArgs(AxisUI.AxisDirection.Vertical, YMin, YMax)); } }
        protected float yMin;
        /// <summary>
        /// The upper Y bound of the object.
        /// </summary>
        public virtual float YMax { get => yMax; set { yMax = value; OnDisplayChanged(new BoundsChangedEventArgs(AxisUI.AxisDirection.Vertical, YMin, YMax)); } }
        protected float yMax;
        /// <summary>
        /// A flag indicating if this object should be transposed before drawing.
        /// </summary>
        public virtual bool Transpose
        {
            get => transpose;
            set
            {
                if (transpose == value) return;
                transpose = value;
                OnDisplayChanged(new TransposeChangedEventArgs(Transpose));
            }
        }
        protected bool transpose = false;
        /// <summary>
        /// The name of the X axis.
        /// </summary>
        public string XName { get => _xName; set { _xName = value; OnDisplayChanged(new AxisNameChangedEventArgs(AxisUI.AxisDirection.Horizontal, XName)); } }
        private string _xName = "";
        protected internal string yName = null;
        /// <summary>
        /// The name of the Y axis.
        /// </summary>
        public virtual string YName { get => string.IsNullOrEmpty(yName) ? DisplayName : yName; set { yName = value; OnDisplayChanged(new AxisNameChangedEventArgs(AxisUI.AxisDirection.Vertical, YName)); } }
        /// <summary>
        /// The unit for the X axis.
        /// </summary>
        public string XUnit { get => _xUnit; set { _xUnit = value; OnDisplayChanged(new AxisUnitChangedEventArgs(AxisUI.AxisDirection.Horizontal, XUnit)); } }
        private string _xUnit = "";
        /// <summary>
        /// The unit for the Y axis.
        /// </summary>
        public string YUnit { get => _yUnit; set { _yUnit = value; OnDisplayChanged(new AxisUnitChangedEventArgs(AxisUI.AxisDirection.Vertical, YUnit)); } }
        private string _yUnit = "";
        /// <summary>
        /// A standard Format String for use in <see cref="float.ToString(string)"/>.
        /// </summary>
        public string StringFormat { get; set; } = "G";
        /// <summary>
        /// Defines the color scheme for the graph.
        /// </summary>
        public virtual Gradient ColorScheme
        {
            get => colorScheme;
            set
            {
                colorScheme = value;
                _useSingleColor = false;
                OnDisplayChanged(new ColorChangedEventArgs(ColorScheme, ColorFunc, UseSingleColor, color));
            }
        }
        protected Gradient colorScheme;

        public bool UseSingleColor
        {
            get => _useSingleColor;
            set
            {
                bool changed = _useSingleColor != value;
                _useSingleColor = value;
                if (changed)
                    OnDisplayChanged(new ColorChangedEventArgs(ColorScheme, ColorFunc, UseSingleColor, color));
            }
        }
        private bool _useSingleColor;

        public Color color
        {
            get => _color;
            set
            {
                bool changed = _color != value;
                _color = value;
                changed |= _useSingleColor == false;
                _useSingleColor = true;
                if (changed)
                    OnDisplayChanged(new ColorChangedEventArgs(ColorScheme, ColorFunc, UseSingleColor, color));
            }
        }
        protected Color _color = Color.white;

        public static bool ValidFloat(float value) => !(float.IsNaN(value) || float.IsInfinity(value));

        public virtual Color EvaluateColor(Vector3 value)
        {
            if (_useSingleColor)
                return _color;
            if (ColorScheme == null)
                return Color.clear;
            if (!ValidFloat(ValueAt(value.x, value.y)))
                return Color.clear;
            float mappedValue;
            if (ColorFunc == null)
                mappedValue = DefaultColorFunc(value);
            else
                mappedValue = ColorFunc(value);
            mappedValue = NormalizeColorFuncOutput(mappedValue);
            return ColorScheme.Evaluate(mappedValue);
        }
        protected abstract float DefaultColorFunc(Vector3 value);
        protected virtual float NormalizeColorFuncOutput(float value) => value;

        /// <summary>
        /// Provides the mapping function as input to the <see cref="ColorScheme"/>.
        /// </summary>
        public virtual Func<Vector3, float> ColorFunc
        {
            get => colorFunc ?? DefaultColorFunc;
            set
            {
                colorFunc = value;
                OnDisplayChanged(new ColorChangedEventArgs(ColorScheme, ColorFunc, UseSingleColor, color));
            }
        }
        protected Func<Vector3, float> colorFunc;

        /// <summary>
        /// Draws the object on the specified <see cref="Texture2D"/>.
        /// </summary>
        /// <param name="texture">The texture on which to draw the object.</param>
        /// <param name="xLeft">The X axis lower bound.</param>
        /// <param name="xRight">The X axis upper bound.</param>
        /// <param name="yBottom">The Y axis lower bound.</param>
        /// <param name="yTop">The Y axis upper bound.</param>
        public abstract void Draw(ref Texture2D texture, float xLeft, float xRight, float yBottom, float yTop);
        /// <summary>
        /// Gets a value from the object given a selected coordinate.
        /// </summary>
        /// <param name="x">The x value of the selected coordinate.</param>
        /// <param name="y">The y value of the selected coordinate.</param>
        /// <returns></returns>
        public abstract float ValueAt(float x, float y);
        /// <summary>
        /// An event to be triggered when an object's values change.
        /// </summary>
        public event EventHandler<IValueEventArgs> ValuesChanged;
        /// <summary>
        /// An event to be triggered when an object's display formatting changes.
        /// </summary>
        public event EventHandler<IDisplayEventArgs> DisplayChanged;

        protected virtual IO.SpreadsheetOptions SpreadsheetOptions => IO.GraphIO.defaultOptions;

        public abstract void WriteToFileCSV(string path);

        public virtual void WriteToFileXLS(string path, string worksheet)
        {
            System.Data.DataTable data = new System.Data.DataTable(worksheet);
            WriteToDataTable(data);
            data.AcceptChanges();
            IO.GraphIO.SpreadsheetWriter.Write(path, worksheet, data, SpreadsheetOptions);
            data.Dispose();
        }

        /// <summary>
        /// Outputs the object's values to a <see cref="System.Data.DataTable"/> object.
        /// The incoming DataTable object can be expected to be empty.
        /// </summary>
        /// <param name="dataTable">The <see cref="System.Data.DataTable"/> object to write to.</param>
        public abstract void WriteToDataTable(System.Data.DataTable dataTable);

        /// <summary>
        /// Invokes the <see cref="ValuesChanged"/> event for this object.
        /// </summary>
        /// <param name="eventArgs">Any relevant <see cref="EventArgs"/>.</param>
        public virtual void OnValuesChanged(IValueEventArgs eventArgs)
        {
            ValuesChanged?.Invoke(this, eventArgs);
        }
        /// <summary>
        /// Invokes the <see cref="DisplayChanged"/> event for this object.
        /// </summary>
        /// <param name="eventArgs">Any relevant <see cref="EventArgs"/>.</param>
        public virtual void OnDisplayChanged(IDisplayEventArgs eventArgs)
        {
            DisplayChanged?.Invoke(this, eventArgs);
        }

        /// <summary>
        /// Gets a formatted value from the object given a selected coordinate.
        /// </summary>
        /// <param name="x">The x value of the selected coordinate.</param>
        /// <param name="y">The y value of the selected coordinate.</param>
        /// <param name="withName">When true, requests the object include its name.</param>
        /// <returns></returns>
        public virtual string GetFormattedValueAt(float x, float y, bool withName = false)
        {
            string effectiveUnit = YUnit;
            if (effectiveUnit == "-")
                effectiveUnit = "";
            return string.Format("{2}{0:" + StringFormat + "} {1}", ValueAt(x, y), effectiveUnit, withName && !string.IsNullOrEmpty(DisplayName) ? DisplayName + ": " : "");
        }

        public static string FormatNameAndUnit(string axisName, string unit, string fallback = "")
        {
            if (string.IsNullOrWhiteSpace(axisName))
                return string.IsNullOrWhiteSpace(unit) ? fallback : $"{unit}";
            else
                return $"{axisName} [{(unit != "" ? unit : "-")}]";
        }

        public static bool VisiblePredicate(IGraphable graph) => graph.Visible;
    }

    /// <summary>
    /// An abstract class for directly graphable objects that have a Z component.
    /// </summary>
    public abstract class Graphable3 : Graphable, IGraphable3
    {
        /// <summary>
        /// The lower Z bound of the object.
        /// </summary>
        public virtual float ZMin { get => zMin; set { zMin = value; OnDisplayChanged(new BoundsChangedEventArgs(AxisUI.AxisDirection.Depth, ZMin, ZMax)); } }
        protected float zMin;
        /// <summary>
        /// The upper Z bound of the object.
        /// </summary>
        public virtual float ZMax { get => zMax; set { zMax = value; OnDisplayChanged(new BoundsChangedEventArgs(AxisUI.AxisDirection.Depth, ZMin, ZMax)); } }
        protected float zMax;
        /// <summary>
        /// The unit for the Z axis.
        /// </summary>
        public string ZUnit { get => _zUnit ?? ""; set { _zUnit = value; OnDisplayChanged(new AxisUnitChangedEventArgs(AxisUI.AxisDirection.Depth, ZUnit)); } }
        private string _zUnit;
        /// <summary>
        /// The name of the Y axis.
        /// </summary>
        public override string YName { get => yName ?? ""; set { yName = value; OnDisplayChanged(new AxisNameChangedEventArgs(AxisUI.AxisDirection.Vertical, YName)); } }
        /// <summary>
        /// The name of the Z axis.
        /// </summary>
        public string ZName { get { return string.IsNullOrEmpty(zName) ? DisplayName : zName; } set { zName = value; OnDisplayChanged(new AxisNameChangedEventArgs(AxisUI.AxisDirection.Depth, ZName)); } }
        protected string zName = null;

        /// <summary>
        /// Gets a formatted value from the object given a selected coordinate.
        /// </summary>
        /// <param name="x">The x value of the selected coordinate.</param>
        /// <param name="y">The y value of the selected coordinate.</param>
        /// <param name="withName">When true, requests the object include its name.</param>
        /// <returns></returns>
        public override string GetFormattedValueAt(float x, float y, bool withName = false)
        {
            string effectiveUnit = ZUnit;
            if (effectiveUnit == "-")
                effectiveUnit = "";
            return string.Format("{2}{0:" + StringFormat + "} {1}", ValueAt(x, y), effectiveUnit, withName && !string.IsNullOrEmpty(DisplayName) ? DisplayName + ": " : "");
        }
    }
}
