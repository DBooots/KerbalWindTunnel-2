using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Graphing;
using Graphing.Extensions;
using UnityEngine;
using KSP.Localization;

namespace KerbalWindTunnel.DataGenerators
{
    public abstract class GraphDefinition
    {
        private const string autoLocStr = "#autoLOC";
        public readonly string name;
        private bool enabled = true;
        protected IGraphable graph;

        protected static string LocalizeIfNeeded(string value)
        {
            if (value.StartsWith(autoLocStr))
                return Localizer.Format(value);
            return value;
        }

        public IGraphable Graph { get => graph; }

        public string DisplayName { get => graph.DisplayName; set => graph.DisplayName = LocalizeIfNeeded(value); }
        public string XName { get => graph.XName; set => graph.XName = LocalizeIfNeeded(value); }
        public string YName { get => graph.YName; set => graph.YName = LocalizeIfNeeded(value); }
        public string XUnit { get => graph.XUnit; set => graph.XUnit = LocalizeIfNeeded(value); }
        public string YUnit { get => graph.YUnit; set => graph.YUnit = LocalizeIfNeeded(value); }
        public bool Enabled { get => enabled; set { enabled = value; graph.Visible = visible && enabled; } }
        private bool visible = true;
        public bool Visible { get => graph.Visible; set { visible = value; graph.Visible = visible && enabled; } }

        public GraphDefinition(string name)
            => this.name = name;
    }

    public abstract class GraphDefinition<TData, TPoint, TGraph> : GraphDefinition where TGraph : Graphable
    {
        public string StringFormat { get => ((Graphable)graph).StringFormat; set => ((Graphable)graph).StringFormat = value; }
        public Gradient ColorScheme { get => ((Graphable)graph).ColorScheme; set => ((Graphable)graph).ColorScheme = value; }
        public Color Color { get => ((Graphable)graph).color; set => ((Graphable)graph).color = value; }

        public Func<TData, TPoint> mappingFunc;
        public new TGraph Graph { get => (TGraph)graph; }
        public GraphDefinition(string name, Func<TData, TPoint> mappingFunc)
            : base(name)
            => this.mappingFunc = mappingFunc;
    }

    public class SurfGraphDefinition<T> : GraphDefinition<T, float, SurfGraph>
    {
        private readonly static float[,] blank = new float[0, 0];

        public string ZName { get => ((SurfGraph)graph).ZName; set => ((SurfGraph)graph).ZName = LocalizeIfNeeded(value); }
        public string ZUnit { get => ((SurfGraph)graph).ZUnit; set => ((SurfGraph)graph).ZUnit = LocalizeIfNeeded(value); }
        public float CMin { get => ((SurfGraph)graph).CMin; set => ((SurfGraph)graph).CMin = value; }
        public float CMax { get => ((SurfGraph)graph).CMax; set => ((SurfGraph)graph).CMax = value; }

        public SurfGraphDefinition(string name, Func<T, float> mappingFunc, Gradient ColorScheme = null) : base(name, mappingFunc)
        {
            graph = new SurfGraph(blank, 0, 0, 0, 0) { Name = name, ColorScheme = ColorScheme ?? GradientExtensions.Jet_Dark };
        }

        public void UpdateGraph(in float left, in float right, in float bottom, in float top, in T[,] values)
            => Graph.SetValues(values.SelectToArray(mappingFunc), left, right, bottom, top);
    }

    public class LineGraphDefinition<T> : GraphDefinition<T, Vector2, LineGraph>
    {
        private readonly static Vector2[] blank = new Vector2[0];
        public float LineWidth { get => ((LineGraph)graph).LineWidth; set => ((LineGraph)graph).LineWidth = value; }
        public LineGraphDefinition(string name, Func<T, Vector2> mappingFunc)
            : base(name, mappingFunc)
            => graph = new LineGraph(blank) { Name = name };

        public virtual void UpdateGraph(in T[] values)
            => Graph.SetValues(values.Select(mappingFunc).ToArray());
    }

    public class MetaLineGraphDefinition<T> : GraphDefinition<T, Vector2, MetaLineGraph>
    {
        private readonly static Vector2[] blank = new Vector2[0];

        public float LineWidth { get => ((LineGraph)graph).LineWidth; set => ((LineGraph)graph).LineWidth = value; }
        public Func<T, float>[] MetaFuncs { get; set; }
        public MetaLineGraphDefinition(string name, Func<T, Vector2> mappingFunc, Func<T, float>[] metaFuncs, string[] metaFields, string[] metaStringFormats, string[] metaUnits)
            : base(name, mappingFunc)
        {
            MetaFuncs = metaFuncs;
            graph = new MetaLineGraph(blank, metaFields.Select(LocalizeIfNeeded).ToArray(), new float[metaFields.Length][])
            {
                Name = name,
                MetaUnits = metaUnits.Select(LocalizeIfNeeded).ToArray(),
                MetaStringFormats = metaStringFormats
            };
        }
        public void UpdateGraph(IList<T> values)
        {
            float[] MetaMap(Func<T, float> metaFunc) => values.Select(metaFunc).ToArray();
            Graph.SetValues(values.Select(mappingFunc).ToArray(), MetaFuncs.Select(MetaMap).ToArray());
        }
    }

    public class OutlineGraphDefinition<T> : GraphDefinition<T, float, OutlineMask>
    {
        private readonly static float[,] blank = new float[0, 0];

        public string ZName { get => ((OutlineMask)graph).ZName; set => ((OutlineMask)graph).ZName = value; }
        public string ZUnit { get => ((OutlineMask)graph).ZUnit; set => ((OutlineMask)graph).ZUnit = value; }
        public float LineWidth { get => ((OutlineMask)graph).LineWidth; set => ((OutlineMask)graph).LineWidth = value; }
        public bool LineOnly { get => ((OutlineMask)graph).LineOnly; set => ((OutlineMask)graph).LineOnly = value; }
        public Func<Vector3, float> MaskCriteria { get => ((OutlineMask)graph).MaskCriteria; set => ((OutlineMask)graph).MaskCriteria = value; }

        public OutlineGraphDefinition(string name, Func<T, float> mappingFunc)
            : base(name, mappingFunc)
            => graph = new OutlineMask(blank, 0, 0, 0, 0) { Name = name };

        public void UpdateGraph(in float left, in float right, in float bottom, in float top, in T[,] values)
            => Graph.SetValues(values.SelectToArray(mappingFunc), left, right, bottom, top);
    }

    public class GroupedGraphDefinition<TGraphDefinition> : GraphDefinition, IEnumerable<TGraphDefinition> where TGraphDefinition : GraphDefinition
    {
        public readonly TGraphDefinition[] children;
        public GroupedGraphDefinition(string name, params TGraphDefinition[] children) : base(name)
        {
            this.children = children;
            graph = new GraphableCollection(children.Select(g => g.Graph));
        }

        public IEnumerator<TGraphDefinition> GetEnumerator()
            => ((IEnumerable<TGraphDefinition>)children).GetEnumerator();

        IEnumerator IEnumerable.GetEnumerator()
            => children.GetEnumerator();
    }
}
