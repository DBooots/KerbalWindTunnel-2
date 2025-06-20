﻿using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Graphing.UI;

namespace Graphing
{
    public class Grapher : MonoBehaviour
    {
        [SerializeField]
        protected Transform leftAxisGroup;
        [SerializeField]
        protected Transform bottomAxisGroup;
        [SerializeField]
        protected Transform rightAxisGroup;
        [SerializeField]
        protected Transform topAxisGroup;
        [SerializeField]
        protected GameObject _axisPrefab;
        [SerializeField]
        protected Transform graphingSystem;

        [SerializeField]
        public Material surfGraphMaterial;
        [SerializeField]
        public Material outlineGraphMaterial;
        [SerializeField]
        public Material lineVertexMaterial;

        public CrosshairController CrosshairController { get => GetComponentInChildren<CrosshairController>(true); }

        public int ZOffset2D { get; set; } = 10;

        public AxisStyle DefaultStyle { get; set; } = new AxisStyle();
        
        protected readonly List<IGraphable> graphs = new List<IGraphable>();

        public AxisUI PrimaryVerticalAxis { get => leftAxisGroup.GetComponentsInChildren<AxisUI>().LastOrDefault(); }
        public AxisUI PrimaryHorizontalAxis { get => bottomAxisGroup.GetComponentInChildren<AxisUI>(); }
        public AxisUI SecondaryVerticalAxis { get => rightAxisGroup.GetComponentInChildren<AxisUI>(); }
        public AxisUI SecondaryHorizontalAxis { get => topAxisGroup.GetComponentsInChildren<AxisUI>().LastOrDefault(); }
        public AxisUI PrimaryColorAxis
        {
            get
            {
                var axes = GetComponentsInChildren<AxisUI>();
                return axes.FirstOrDefault(a => (a.Use & AxisUI.AxisDirection.Color) > 0);
            }
        }
        public AxisUI PrimaryDepthAxis
        {
            get
            {
                var axes = GetComponentsInChildren<AxisUI>();
                return axes.FirstOrDefault(a => (a.Use & AxisUI.AxisDirection.Depth) > 0);
            }
        }

        public readonly Dictionary<Type, Type> typeLookup = new Dictionary<Type, Type>
        {
            { typeof(SurfGraph), typeof(SurfGraphDrawer) },
            { typeof(OutlineMask), typeof(OutlineGraphDrawer) },
            { typeof(Line3Graph), typeof(LineGraphDrawer) },
            { typeof(LineGraph), typeof(LineGraphDrawer) },
            { typeof(MetaLineGraph), typeof(LineGraphDrawer) },
            { typeof(GraphableCollection), typeof(GraphDrawerCollection) },
            { typeof(GraphableCollection3), typeof(GraphDrawerCollection) }
        };

        private readonly System.Collections.Concurrent.ConcurrentQueue<(GraphDrawer drawer, bool active)> activatorQueue = new System.Collections.Concurrent.ConcurrentQueue<(GraphDrawer drawer, bool visible)>();

        /// <summary>Provides the primary vertical axis, creating one if one does not exist.</summary>
        /// <returns>The vertical axis designated as primary.</returns>
        public AxisUI ProvidePrimaryVerticalAxis()
        {
            AxisUI axis = PrimaryVerticalAxis;
            return axis ?? AddAxis(AxisSide.Left);
        }
        /// <summary>Provides the primary horizontal axis, creating one if one does not exist.</summary>
        /// <returns>The horizontal axis designated as primary.</returns>
        public AxisUI ProvidePrimaryHorizontalAxis()
        {
            AxisUI axis = PrimaryHorizontalAxis;
            return axis ?? AddAxis(AxisSide.Bottom);
        }
        /// <summary>Provides the secondary vertical axis, creating one if one does not exist.</summary>
        /// <returns>The vertical axis designated as secondary.</returns>
        public AxisUI ProvideSecondaryVerticalAxis()
        {
            AxisUI axis = SecondaryVerticalAxis;
            return axis ?? AddAxis(AxisSide.Right);
        }
        /// <summary>Provides the secondary horizontal axis, creating one if one does not exist.</summary>
        /// <returns>The horizontal axis designated as secondary.</returns>
        public AxisUI ProvideSecondaryHorizontalAxis()
        {
            AxisUI axis = SecondaryHorizontalAxis;
            return axis ?? AddAxis(AxisSide.Top);
        }
        /// <summary>Provides a color axis, creating one if one does not exist.</summary>
        /// <param name="side">The side on which to create an axis if one is to be created.</param>
        /// <returns>The color axis.</returns>
        public AxisUI ProvideColorAxis(AxisSide side = AxisSide.Bottom, bool returnNullInsteadOfCreate = false, bool requiresDepth = false)
        {
            AxisUI axis = GetComponentsInChildren<AxisUI>().FirstOrDefault(a => requiresDepth ? (a.Use & AxisUI.AxisDirection.ColorWithDepth) > 0 : (a.Use & AxisUI.AxisDirection.Color) > 0);
            return axis ?? (returnNullInsteadOfCreate ? null : AddAxis(side, requiresDepth ? AxisUI.AxisDirection.ColorWithDepth : AxisUI.AxisDirection.Color));
        }

        /// <summary>Adds a graph to this <see cref="Grapher"/>.</summary>
        /// <param name="graph">The graph to be added.</param>
        /// <returns>The <see cref="GraphDrawer"/> object associated with this graph.</returns>
        public GraphDrawer AddGraph(IGraphable graph)
        {
            if (graphs.Contains(graph))
            {
                Debug.LogWarning($"This graph ({graph.Name}) is already attached to the grapher.");
                return GetComponentsInChildren<GraphDrawer>().First(d => d.Graph == graph);
            }
            graphs.Add(graph);
            //GraphDrawer drawer = Instantiate(_graphDrawerPrefab, graphingSystem).GetComponent<GraphDrawer>();
            GraphDrawer drawer = InstantiateGraphDrawer(graph, graphingSystem, (surfGraphMaterial, outlineGraphMaterial, lineVertexMaterial));
            if (graph is IGraphable3 graphable3 && !GetComponentsInChildren<AxisUI>().Any(a => (a.Use & AxisUI.AxisDirection.Depth) > 0))
            {
                drawer.SetOriginAndScale(AxisUI.AxisDirection.Depth, 0, 1 / (graphable3.ZMax - graphable3.ZMin));
            }
            return drawer;
        }

        /// <summary>Adds a graph to this <see cref="Grapher"/>, linking with with the specified horizontal and vertical axes.</summary>
        /// <param name="graph">The graph to be added.</param>
        /// <param name="axes">The axes to be associated with the graph.</param>
        /// <returns>The <see cref="GraphDrawer"/> object associated with this graph.</returns>
        public GraphDrawer AddGraph(IGraphable graph, params AxisUI[] axes)
        {
            GraphDrawer graphDrawer = AddGraph(graph);
            foreach (AxisUI axis in axes)
                axis?.AssignGraphToAxis(graphDrawer);
            return graphDrawer;
        }

        /// <summary>Adds a graph to this <see cref="Grapher"/>, associating it with the default axes.</summary>
        /// <param name="graph">The graph to be added.</param>
        /// <returns>The <see cref="GraphDrawer"/> object associated with this graph.</returns>
        public GraphDrawer AddGraphToDefaultAxes(IGraphable graph, bool createColorAxisForCollections = false, bool requireDepthForCollections = false)
            => AddGraph(graph,
                ProvidePrimaryHorizontalAxis(),
                ProvidePrimaryVerticalAxis(),
                ProvideColorAxis(returnNullInsteadOfCreate: !(graph is IColorGraph || (createColorAxisForCollections && graph is GraphableCollection)),
                    requiresDepth: graph is IGraphable3 || (requireDepthForCollections && graph is GraphableCollection))
                );
        public bool RemoveGraph(IGraphable graph)
        {
            if (graph == null)
                return false;
            if (!graphs.Contains(graph))
                return false;
            GraphDrawer graphDrawer = GetComponentsInChildren<GraphDrawer>().FirstOrDefault(d => d.Graph == graph);
            return RemoveGraph(graphDrawer);
        }
        /// <summary>Removes a graph from the Grapher.</summary>
        /// <param name="graphDrawer">The <see cref="GraphDrawer"/> associated with the graph to be removed.</param>
        /// <returns><c>true</c> if successful in removing the graph, otherwise <c>false</c>.</returns>
        public bool RemoveGraph(GraphDrawer graphDrawer)
        {
            if (graphDrawer == null)
                return false;
            if (!graphs.Contains(graphDrawer.Graph))
                return false;
            UnregisterGraphDrawer(graphDrawer);
            graphs.Remove(graphDrawer.Graph);
            Destroy(graphDrawer);
            return true;
        }
        /// <summary>Gets the <see cref="GraphDrawer"/> object for a given graph.</summary>
        /// <param name="graph">The graph of the GraphDrawer to find.</param>
        /// <param name="searchSubChildren">Will recursively search sub-children if set to <c>true</c>.</param>
        /// <returns>The <see cref="GraphDrawer"/> associated with the graph.</returns>
        public GraphDrawer GetGraphDrawer(IGraphable graph, bool searchSubChildren = false)
        {
            if (searchSubChildren)
                return GetComponentsInChildren<GraphDrawer>().FirstOrDefault(d => d.Graph == graph);
            if (!graphs.Contains(graph))
                return null;
            for (int i = 0; i < transform.childCount; i++)
            {
                GraphDrawer drawer = transform.GetChild(i).GetComponent<GraphDrawer>();
                if (drawer != null && drawer.Graph == graph)
                    return drawer;
            }
            return null;
        }

        /// <summary>Gets the <see cref="AxisUI"/> object on the given side at the given index.</summary>
        /// <param name="side">The side to select.</param>
        /// <param name="index">The index of the axis to be returned.</param>
        /// <returns>The <see cref="AxisUI"/> object referenced, if one exists. Otherwise <c>null</c>.</returns>
        /// <exception cref="System.ArgumentOutOfRangeException">index - index cannot be negative.</exception>
        public AxisUI GetAxis(AxisSide side, int index = 0)
        {
            if (index < 0)
                throw new ArgumentOutOfRangeException("index", "index cannot be negative.");
            AxisUI[] sideAxes = GetAxes(side);
            if (sideAxes.Length > index)
                return sideAxes[index];
            return null;
        }
        /// <summary>Gets the <see cref="AxisUI"/> object of the given direction at the given index.</summary>
        /// <param name="use">The direction of the desired axis.</param>
        /// <param name="index">The index of the axis within those of the selected direction.</param>
        /// <returns>The <see cref="AxisUI"/> object referenced, if one exists. Otherwise <c>null</c>.</returns>
        public AxisUI GetAxis(AxisUI.AxisDirection use, int index = 0)
        {
            int i = -1;
            foreach (AxisUI axis in GetAxes(use))
            {
                i++;
                if (i == index)
                    return axis;
            }
            return null;
        }
        /// <summary>Gets all <see cref="AxisUI"/> objects of a given direction.</summary>
        /// <param name="use">The direction of the axes objects to return.</param>
        /// <returns>A collection of all child axes that have the given direction.</returns>
        public IEnumerable<AxisUI> GetAxes(AxisUI.AxisDirection use) => GetComponentsInChildren<AxisUI>().Where(a => (a.Use & use) == use);
        public AxisUI[] GetAxes(AxisSide side)
        {
            Transform axisGroup = GetAxisTransform(side);
            if (side == AxisSide.Left || side == AxisSide.Top)
                return axisGroup.GetComponentsInChildren<AxisUI>().Reverse().ToArray();
            return axisGroup.GetComponentsInChildren<AxisUI>();
        }
        /*
        public void AddAxisTest()
        {
            OutlineMask outline = new OutlineMask(new float[,] { { 3, 2, 1 }, { 2, 1, 0 }, { 1, 2, 1 }, { 0, 2, 4 } }, 0, 1, 0, 1) { MaskCriteria = (v) => v.z - 1.5f };

            SurfGraph surfGraph = new SurfGraph(new float[,] { { 3, 2, 1 }, { 2, 1, 0 }, { 1, 2, 1 }, { 0, 2, 4 } }, 0, 1, 0, 1);

            GraphableCollection3 collection = new GraphableCollection3
            {
                outline,
                surfGraph
            };
            GraphDrawer drawer = AddGraphToDefaultAxes(collection);
            rightAxisGroup.GetComponent<UnityEngine.UI.LayoutGroup>().padding = new RectOffset(5, 1, 0, 0);
            Axis colorAxis = AddAxis(AxisSide.Right, Axis.AxisDirection.Color);
            //colorAxis.AxisBackground = Extensions.GradientExtensions.Jet_Tex;
            AssignGraphToAxis(drawer, colorAxis);
            Material axisMaterial = GetAxisGraphMaterial(colorAxis);
            //if (axisMaterial != null)
              //  GraphDrawer.SetMaterialGradient(axisMaterial, Extensions.GradientExtensions.Jet_Tex);
            //else
              //  Debug.Log("Material is null. :(");
        }//*/

        /// <summary>
        /// Handler method for when the bounds of an <see cref="Axis"/> object have changed.
        /// </summary>
        /// <param name="axis">The axis.</param>
        /// <param name="min">The new minimum.</param>
        /// <param name="max">The new maximum.</param>
        /// <param name="oldMin">The old minimum.</param>
        /// <param name="oldMax">The old maximum.</param>
        protected void AxisBoundsChangedHandler(object axis, Axis.AxisBoundsEventArgs eventArgs)
        {
            if (!CrosshairController.IsHeld)
                return;

            float oldMax = eventArgs.oldMax;
            float oldMin = eventArgs.oldMin;
            float max = eventArgs.max;
            float min = eventArgs.min;

            // Only one axis with the selected use, let's adjust the crosshairs.
            Vector2 normalizedPosition = CrosshairController.NormalizedPosition;
            AxisUI axisObj = (AxisUI)axis;
            if (axisObj == PrimaryHorizontalAxis)
                normalizedPosition.x = Mathf.Clamp01((normalizedPosition.x * (oldMax - oldMin) + oldMin - min) / (max - min));
            else if (axisObj == PrimaryVerticalAxis)
                normalizedPosition.y = Mathf.Clamp01((normalizedPosition.y * (oldMax - oldMin) + oldMin - min) / (max - min));
            else
                return;
            CrosshairController.SetCrosshairPosition(normalizedPosition);
            OnGraphClicked(this, normalizedPosition);
        }

        protected bool wasClicked;

        /// <summary>
        /// Adds an axis to this <see cref="Grapher"/>.
        /// </summary>
        /// <param name="side">The side on which to place the axis.</param>
        /// <param name="style">The style of the axis.</param>
        /// <param name="use">The direction/use of the axis.</param>
        /// <returns>The <see cref="AxisUI"/> object created</returns>
        public AxisUI AddAxis(AxisSide side, AxisStyle style, AxisUI.AxisDirection use = AxisUI.AxisDirection.Undefined)
        {
            if (use == AxisUI.AxisDirection.Vertical && (side == AxisSide.Bottom || side == AxisSide.Top))
                Debug.LogWarning("Top or Bottom graph assigned with a Y-Axis use. This is not recommended behaviour.");
            if (use == AxisUI.AxisDirection.Horizontal && (side == AxisSide.Left || side == AxisSide.Right))
                Debug.LogWarning("Left or Right graph assigned with a X-Axis use. This is not recommended behaviour.");
            Transform axisGroupTransform = GetAxisTransform(side);
            AxisUI axis = Instantiate(_axisPrefab, axisGroupTransform).GetComponent<AxisUI>();
            if (side == AxisSide.Left || side == AxisSide.Top)
                axis.transform.SetAsFirstSibling();
            axis.SetAxisStyle(style);
            axis.Side = side;
            if (use == AxisUI.AxisDirection.Undefined)
                axis.Use = (side == AxisSide.Left || side == AxisSide.Right) ? AxisUI.AxisDirection.Vertical : AxisUI.AxisDirection.Horizontal;
            else
                axis.Use = use;
            axis.AxisBoundsChangedEvent += AxisBoundsChangedHandler;
            return axis;
        }
        /// <summary>
        /// Adds an axis to this <see cref = "Grapher" /> using the default style.
        /// </summary>
        /// <param name="style">The style of the axis.</param>
        /// <param name="use">The direction/use of the axis.</param>
        /// <returns>The <see cref="AxisUI"/> object created</returns>
        public AxisUI AddAxis(AxisSide side, AxisUI.AxisDirection use = AxisUI.AxisDirection.Undefined) => AddAxis(side, DefaultStyle, use);

        /// <summary>
        /// Removes the specified axis.
        /// </summary>
        /// <param name="axis">The axis to be removed.</param>
        public void RemoveAxis(AxisUI axis)
        {
            axis.AxisBoundsChangedEvent -= AxisBoundsChangedHandler;
            Destroy(axis.gameObject);
        }

        /// <summary>
        /// Gets the axis group transform on a given side.
        /// </summary>
        /// <param name="side">The side of the transform to return.</param>
        /// <returns>The <see cref="Transform"/> of the axis group container.</returns>
        protected Transform GetAxisTransform(AxisSide side)
        {
            switch (side)
            {
                default:
                case AxisSide.Left:
                    return leftAxisGroup;
                case AxisSide.Bottom:
                    return bottomAxisGroup;
                case AxisSide.Right:
                    return rightAxisGroup;
                case AxisSide.Top:
                    return topAxisGroup;
            }
        }

        /// <summary>
        /// Unregisters a graph drawer from all axes under this Grapher.
        /// </summary>
        /// <param name="graph">The graph drawer to unregister.</param>
        public void UnregisterGraphDrawer(GraphDrawer graph)
        {
            foreach (AxisUI axis in GetComponentsInChildren<AxisUI>())
                axis.DetachGraphFromAxis(graph);
        }

        public GraphDrawer InstantiateGraphDrawer(IGraphable graph, Transform transform, (Material surfMaterial, Material outlineMaterial, Material lineVertexMaterial) materialSet)
        {
            if (!typeLookup.TryGetValue(graph.GetType(), out Type graphDrawerType))
                graphDrawerType = TypeFallback(graph.GetType());
            string objName = graph.Name;
            if (string.IsNullOrEmpty(objName))
                objName = graph.GetType().Name;
            GraphDrawer drawer = (GraphDrawer)new GameObject(objName).AddComponent(graphDrawerType);
            drawer.Initialize();
            if (drawer is GraphDrawer.ISingleMaterialUser singleMatUser)
            {
                switch (drawer.MaterialSet)
                {
                    case 1:
                        singleMatUser.InitializeMaterial(materialSet.surfMaterial);
                        break;
                    case 2:
                        singleMatUser.InitializeMaterial(materialSet.outlineMaterial);
                        break;
                    case 3:
                        singleMatUser.InitializeMaterial(materialSet.lineVertexMaterial);
                        break;
                }
            }
            else if (drawer is GraphDrawerCollection collection)
                collection.InitializeMaterials(materialSet.surfMaterial, materialSet.outlineMaterial, materialSet.lineVertexMaterial);
            drawer.transform.SetParent(transform);
            drawer.transform.localRotation = Quaternion.Euler(0, 180, 0);
            drawer.SetGraph(graph, this);
            return drawer;
        }
        public virtual Type TypeFallback(Type graphType)
        {
            if (typeof(ILineGraph).IsAssignableFrom(graphType))
                return typeof(LineGraphDrawer);
            if (typeof(SurfGraph).IsAssignableFrom(graphType))
                return typeof(SurfGraphDrawer);
            if (typeof(OutlineMask).IsAssignableFrom(graphType))
                return typeof(OutlineGraphDrawer);
            if (typeof(GraphableCollection).IsAssignableFrom(graphType))
                return typeof(GraphDrawerCollection);
            return null;
        }

        /// <summary>
        /// Gets the graph coordinates of a point on the graph.
        /// </summary>
        /// <param name="graph">The graph to query.</param>
        /// <param name="relativePosition">The normalized position relative to the origin.</param>
        /// <returns>A <see cref="Vector2"/> of the x and y coordinates based on the axes origin and scale.</returns>
        public Vector2 GetGraphCoordinate(IGraphable graph, Vector2 relativePosition)
        {
            AxisUI horizontal = null, vertical = null;
            foreach (AxisUI axis in GetComponentsInChildren<AxisUI>())
            {
                if (!axis.Contains(graph))
                    continue;
                if (horizontal == null && axis.Use == AxisUI.AxisDirection.Horizontal)
                {
                    horizontal = axis;
                    if (vertical != null)
                        break;
                }
                if (vertical == null && axis.Use == AxisUI.AxisDirection.Vertical)
                {
                    vertical = axis;
                    if (horizontal != null)
                        break;
                }
            }

            float xMin = 0, xRange = 0;
            float yMin = 0, yRange = 0;
            if (horizontal != null)
            {
                xMin = horizontal.Min;
                xRange = horizontal.Max - xMin;
            }
            if (vertical != null)
            {
                yMin = vertical.Min;
                yRange = vertical.Max - yMin;
            }
            return new Vector2(xMin + relativePosition.x * xRange, yMin + relativePosition.y * yRange);
        }
        public Vector2 GetGraphCoordinate(Vector2 relativePosition)
        {
            return new Vector2(PrimaryHorizontalAxis?.GetValue(relativePosition.x) ?? 0, PrimaryVerticalAxis?.GetValue(relativePosition.y) ?? 0);
        }

        /// <summary>
        /// Gets a formatted string of the display value for a point on the graph.
        /// </summary>
        /// <param name="relativePosition">The normalized position relative to the origin.</param>
        /// <returns>A formatted string all visible graphs' display values.</returns>
        public virtual string GetDisplayValue(Vector2 relativePosition)
        {
            string returnString = null;
            bool multipleGraphs = false;
            bool oneGraph = false;
            foreach (IGraphable graph in graphs.Where(g => g.Visible))
            {
                multipleGraphs = oneGraph;
                if (multipleGraphs)
                    break;
                oneGraph = true;
            }
            foreach (IGraphable graph in graphs.Where(g => g.Visible))
            {
                Vector2 absolutePosition = GetGraphCoordinate(graph, relativePosition);

                string graphString = graph.GetFormattedValueAt(absolutePosition.x, absolutePosition.y, multipleGraphs);

                if (string.IsNullOrEmpty(returnString))
                    returnString = graphString;
                else
                    returnString = string.Format("{0}\n{1}", returnString, graphString);
            }
            if (string.IsNullOrEmpty(returnString))
                return "";
            return returnString;
        }
        /// <summary>
        /// Handler method for any changes affecting the display of the graphs.
        /// </summary>
        /// <param name="sender">The sender graph.</param>
        /// <param name="eventArgs">The <see cref="EventArgs"/> instance containing the event data.</param>
        public virtual void DisplayNameChangedHandler(IGraphable sender, DisplayNameChangedEventArgs displayNameChangedEventArgs)
        {
            // COULDDO: Set up a legend or other things requiring the name of this graph.
        }

        public void QueueActivation(GraphDrawer graphDrawer, bool visible) => activatorQueue.Enqueue((graphDrawer, visible));

        /// <summary>
        /// Unity Awake method.
        /// </summary>
        protected virtual void Awake()
        {
            foreach (AxisUI axis in GetComponentsInChildren<AxisUI>())
                axis.AxisBoundsChangedEvent += AxisBoundsChangedHandler;
            CrosshairController.OnClick += OnGraphClicked;
        }

        // TODO: Refactor to get rid of this.
        protected virtual void Update()
        {
            while (activatorQueue.TryDequeue(out var activator))
            {
                activator.drawer.gameObject.SetActive(activator.active);
            }
        }

        /// <summary>
        /// Event method for when the graph is clicked on. Passes on the click to any methods under the <see cref="GraphClicked"/> event.
        /// </summary>
        /// <param name="relativePosition">The normalized, relative position of the mouse.</param>
        protected virtual void OnGraphClicked(object _, Vector2 relativePosition)
            => GraphClicked?.Invoke(this, relativePosition);

        /// <summary>
        /// Triggered when the graph is clicked on.
        /// </summary>
        public event EventHandler<Vector2> GraphClicked;

        protected virtual void OnDestroy()
        {
            foreach (AxisUI axis in GetComponentsInChildren<AxisUI>())
            {
                axis.AxisBoundsChangedEvent -= AxisBoundsChangedHandler;
            }
        }
    }
}