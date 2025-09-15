using System.Collections.Generic;
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
        public static GraphIO.FileFormat format = GraphIO.FileFormat.XLSX;
        private GraphableCollection collection;
        private OutputMode outputMode;
        public readonly PopupDialog dialog;

        public GraphExportDialog(GraphableCollection collection)
        {
            filename = EditorLogic.fetch.ship.shipName;
            this.collection = collection;
            outputMode = WindTunnelWindow.Instance.GraphMode == 0 ? OutputMode.Visible : OutputMode.All;
            List<DialogGUIBase> dialogItems = new List<DialogGUIBase>()
            {
                new DialogGUIContentSizer(UnityEngine.UI.ContentSizeFitter.FitMode.PreferredSize, UnityEngine.UI.ContentSizeFitter.FitMode.MinSize),
                new DialogGUIHorizontalLayout(UnityEngine.TextAnchor.MiddleLeft,
                    new DialogGUILabel("#autoLOC_KWT200", false, true),   // "Save As: "
                    new DialogGUITextInput(filename, "#autoLOC_KWT211", false, 60, value => filename = value, 300, 30)  // "Enter filename"
                    ),
                new DialogGUISpace(5),
                new DialogGUIHorizontalLayout(UnityEngine.TextAnchor.MiddleLeft,
                    new DialogGUILabel("#autoLOC_KWT201", false, true),    // "Format: "
                    new DialogGUIToggleGroup(
                        new DialogGUIToggle(() => format == GraphIO.FileFormat.XLSX, "#autoLOC_KWT202", _ => format = GraphIO.FileFormat.XLSX), // "Spreadsheet"
                        new DialogGUIToggle(() => format == GraphIO.FileFormat.CSV, "#autoLOC_KWT203", _ => format = GraphIO.FileFormat.CSV)    // "CSV"
                        ),
                    new DialogGUIFlexibleSpace()
                    ),
                new DialogGUIHorizontalLayout(UnityEngine.TextAnchor.MiddleLeft,
                    new DialogGUIToggleGroup(
                        new DialogGUIToggleButton(() => outputMode == OutputMode.Visible, "#autoLOC_KWT204", _ => outputMode = OutputMode.Visible, h: 25),  // "Visible graph(s)"
                        new DialogGUIToggleButton(() => outputMode == OutputMode.All, "#autoLOC_KWT205", _ => outputMode = OutputMode.All, h: 25),  // "All graphs"
                        new DialogGUIToggleButton(() => outputMode == OutputMode.Vessel, "#autoLOC_KWT206", _ => outputMode = OutputMode.Vessel, h: 25) // "Vessel"
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
            string path = GraphIO.ValidateFilePath(WindTunnel.graphPath, filename, format);
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
            if (outputMode <= OutputMode.All)
                // Acceptable to not await an Async method since this is the main thread calling an IO operation.
                // Discard is used suppress the warning.
                System.Threading.Tasks.Task.Run(() => collection.WriteToFile(WindTunnel.graphPath, filename, outputMode == OutputMode.Visible, format));
            else
                WindTunnelWindow.Instance.Vessel.WriteToFile(WindTunnel.graphPath, filename, format);
        }
    }
}
