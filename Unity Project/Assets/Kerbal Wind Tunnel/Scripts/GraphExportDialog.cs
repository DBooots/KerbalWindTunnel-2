using System;
using System.Collections.Generic;
using System.Linq;
using Graphing;
using Graphing.IO;

namespace KerbalWindTunnel
{
    public class GraphExportDialog
    {
        public const string popupWindowName = "KWTExport";
        public const string popupConfirmName = "KWTOverwrite";
        public enum OutputMode
        {
            Visible = 0,
            All = 1,
            Vessel = 2
        }

        private string filename;
        public static GraphIO.FileFormat Format
        {
            get => format;
            set
            {
                if (value != GraphIO.FileFormat.Image)
                    lastNonJSONFormat = value;
                format = value;
            }
        }
        private static GraphIO.FileFormat format = GraphIO.FileFormat.XLSX;
        private static GraphIO.FileFormat lastNonJSONFormat = GraphIO.FileFormat.XLSX;
        private GraphableCollection collection;
        private OutputMode Mode
        {
            get => outputMode;
            set
            {
                if (value != OutputMode.Vessel)
                    Format = lastNonJSONFormat;
                outputMode = value;
            }
        }
        private OutputMode outputMode;
        public readonly PopupDialog dialog;

        public GraphExportDialog(GraphableCollection collection)
        {
            filename = EditorLogic.fetch.ship.shipName;
            this.collection = collection;
            Mode = WindTunnelWindow.Instance.GraphMode == 0 ? OutputMode.Visible : OutputMode.All;
            DialogGUIToggleGroup formatGroup = new DialogGUIToggleGroup(
                new DialogGUIToggle(() => Format == GraphIO.FileFormat.XLSX, "#autoLOC_KWT202", _ => Format = GraphIO.FileFormat.XLSX), // "Spreadsheet"
                new DialogGUIToggle(() => Format == GraphIO.FileFormat.CSV, "#autoLOC_KWT203", _ => Format = GraphIO.FileFormat.CSV)    // "CSV"
                );
            DialogGUIHorizontalLayout formatLayout = new DialogGUIHorizontalLayout(UnityEngine.TextAnchor.MiddleLeft,
                new DialogGUILabel("#autoLOC_KWT201", false, true),    // "Format: "
                formatGroup
                );
            if (AppDomain.CurrentDomain.GetAssemblies().Any(a => a.FullName.StartsWith("kOS,")))
                formatGroup.AddChild(
                    new DialogGUIToggle(() => Format == GraphIO.FileFormat.Image, "#autoLOC_KWT212", _ => Format = GraphIO.FileFormat.Image)    // "JSON for kOS"  Using Image as a stand-in, since no format entry can be exactly this value.
                    {
                        OptionInteractableCondition = () => Mode == OutputMode.Vessel && WindTunnelWindow.Instance.Vessel is VesselCache.CharacterizedVessel
                    });
            else
                formatLayout.AddChild(new DialogGUIFlexibleSpace());

            List<DialogGUIBase> dialogItems = new List<DialogGUIBase>()
            {
                new DialogGUIContentSizer(UnityEngine.UI.ContentSizeFitter.FitMode.PreferredSize, UnityEngine.UI.ContentSizeFitter.FitMode.MinSize),
                new DialogGUIHorizontalLayout(UnityEngine.TextAnchor.MiddleLeft,
                    new DialogGUILabel("#autoLOC_KWT200", false, true),   // "Save As: "
                    new DialogGUITextInput(filename, "#autoLOC_KWT211", false, 60, value => filename = value, 300, 30)  // "Enter filename"
                    ),
                new DialogGUISpace(5),
                formatLayout,
                new DialogGUIHorizontalLayout(UnityEngine.TextAnchor.MiddleLeft,
                    new DialogGUIToggleGroup(
                        new DialogGUIToggleButton(() => Mode == OutputMode.Visible, "#autoLOC_KWT204", _ => Mode = OutputMode.Visible, h: 25),  // "Visible graph(s)"
                        new DialogGUIToggleButton(() => Mode == OutputMode.All, "#autoLOC_KWT205", _ => Mode = OutputMode.All, h: 25),  // "All graphs"
                        new DialogGUIToggleButton(() => Mode == OutputMode.Vessel, "#autoLOC_KWT206", _ => Mode = OutputMode.Vessel, h: 25) // "Vessel"
                        )
                ),
                new DialogGUIHorizontalLayout(
                    new DialogGUIFlexibleSpace(),
                    new DialogGUIButton("#autoLOC_174778", Export, false),  // "Save"
                    new DialogGUIButton("#autoLOC_174783", Dismiss, false), // "Cancel"
                    new DialogGUIFlexibleSpace()
                    )
            };

            dialog = PopupDialog.SpawnPopupDialog(
                new MultiOptionDialog(popupWindowName, "", "#autoLOC_KWT207", UISkinManager.defaultSkin, dialogItems.ToArray()),    // "Export"
                false, UISkinManager.defaultSkin, isModal: true);
            dialog.GetComponentInChildren<TMPro.TMP_InputField>(true).gameObject.AddComponent<Extensions.InputLockSelectHandler>().Setup("WindTunnelExport", ControlTypes.KEYBOARDINPUT);
        }

        public void Dismiss()
        {
            dialog?.Dismiss();
        }

        private void Export()
        {
            string path;
            if (Format != GraphIO.FileFormat.Image) // Again, using Image as a stand-in for JSON.
                path = GraphIO.ValidateFilePath(WindTunnel.graphPath, filename, Format);
            else
                path = GraphIO.ValidateFilePath(WindTunnel.graphPath, filename, ".json");
            if (System.IO.File.Exists(path))
            {
                PopupDialog.SpawnPopupDialog(new UnityEngine.Vector2(0.5f, 0.5f), new UnityEngine.Vector2(0.5f, 0.5f),
                    new MultiOptionDialog(popupConfirmName, "#autoLOC_KWT208", "", UISkinManager.defaultSkin,   // "The specified file already exists. Would you like to replace it?"
                        new DialogGUIHorizontalLayout(
                            new DialogGUIFlexibleSpace(),
                            new DialogGUIButton("#autoLOC_174798", () => { DeleteFile(path); ContinueExport(filename); Dismiss(); }, true), // "Yes (overwrite)"
                            new DialogGUIButton("#autoLOC_174804", () => { }, true),    // "No (cancel)"
                            new DialogGUIFlexibleSpace()
                            )
                        ), false, UISkinManager.defaultSkin, true);
                return;
            }

            Dismiss();
            ContinueExport(filename);
        }
        private void DeleteFile(string path)
        {
            System.IO.File.Delete(path);
        }
        private void ContinueExport(string filename)
        {
            if (Mode <= OutputMode.All)
                System.Threading.Tasks.Task.Run(() => collection.WriteToFile(WindTunnel.graphPath, filename, Mode == OutputMode.Visible, Format));
            else
            {
                if (Format == GraphIO.FileFormat.Image)
                {
                    if (WindTunnelWindow.Instance.Vessel is VesselCache.CharacterizedVessel characterizedVessel)
                    {
                        VesselCache.KosJsonWriter jsonWriter = new VesselCache.KosJsonWriter();
                        jsonWriter.WriteToJson(characterizedVessel, filename);
                    }
                    else
                        throw new System.InvalidOperationException("Cannot export a non-characterized vessel to JSON.");
                }
                else
                    WindTunnelWindow.Instance.Vessel.WriteToFile(WindTunnel.graphPath, filename, Format);
            }
        }
    }
}
