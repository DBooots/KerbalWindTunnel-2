using System.Collections.Generic;
using UnityEngine;
using KSP.Localization;

namespace KerbalWindTunnel
{
    public class WindTunnelSettings
    {
        const string popupWindowName = "KWTSettings";

        public static bool UseCharacterized
        {
            get { return Instance.useCharacterized; }
            set
            {
                Instance.useCharacterized = value;
                settingsChanged = true;
            }
        }
        [Persistent]
        private bool useCharacterized = true;

        public static bool UseCoefficients
        {
            get { return Instance.useCoefficients; }
            set
            {
                Instance.useCoefficients = value;
                settingsChanged = true;
            }
        }
        [Persistent]
        private bool useCoefficients = true;

        public static bool SpeedIsMach
        {
            get { return Instance.speedIsMach; }
            set
            {
                Instance.speedIsMach = value;
                settingsChanged = true;
            }
        }
        [Persistent]
        private bool speedIsMach;

        public static bool StartMinimized
        {
            get { return Instance.startMinimized; }
            set
            {
                Instance.startMinimized = value;
                settingsChanged = true;
            }
        }
        [Persistent]
        private bool startMinimized;

        public static bool UseSingleColorHighlighting
        {
            get { return Instance.useSingleColorHighlighting; }
            set
            {
                Instance.useSingleColorHighlighting = value;
                settingsChanged = true;
            }
        }

        [Persistent]
        private bool useSingleColorHighlighting = true;

        public static bool HighlightIgnoresLiftingSurfaces
        {
            get { return Instance.highlightIgnoresLiftingSurfaces; }
            set
            {
                Instance.highlightIgnoresLiftingSurfaces = value;
                settingsChanged = true;
            }
        }

        [Persistent]
        private bool highlightIgnoresLiftingSurfaces = false;

        public static bool ShowEnvelopeMask
        {
            get { return Instance.showEnvelopeMask; }
            set
            {
                Instance.showEnvelopeMask = value;
                settingsChanged = true;
            }
        }
        [Persistent]
        private bool showEnvelopeMask = true;

        public static bool ShowEnvelopeMaskAlways
        {
            get { return Instance.showEnvelopeMaskAlways; }
            set
            {
                Instance.showEnvelopeMaskAlways = value;
                settingsChanged = true;
            }
        }
        [Persistent]
        private bool showEnvelopeMaskAlways = false;

        public static bool UseBlizzy
        {
            get { return Instance.useBlizzy; }
            set
            {
                Instance.useBlizzy = value;
                settingsChanged = true;
            }
        }
        [Persistent]
        private bool useBlizzy = false;

        public static bool AutoFitAxes
        {
            get { return Instance.autoFitAxes; }
            set
            {
                Instance.autoFitAxes = value;
                settingsChanged = true;
            }
        }
        [Persistent]
        private bool autoFitAxes = true;

        public static int RotationCount
        {
            get => Instance.rotationCount;
            set
            {
                Instance.rotationCount = value;
                settingsChanged = true;
            }
        }
        [Persistent]
        private int rotationCount = 1;

        public static bool EnableGlidingCalcs
        {
            get => Instance.enableGlidingCalcs;
            set
            {
                Instance.enableGlidingCalcs = value;
                settingsChanged = true;
            }
        }
        [Persistent]
        private bool enableGlidingCalcs = false;

        private static bool settingsChanged = false;
        private static bool settingsLoaded = false;

        public static readonly WindTunnelSettings Instance = new WindTunnelSettings();

        public static void InitializeSettings()
        {
            if (settingsLoaded)
                return;

            Instance.LoadSettingsFromFile();

            settingsLoaded = true;
        }
        private void LoadSettingsFromFile()
        {
            ConfigNode[] settingsNode = GameDatabase.Instance.GetConfigNodes("KerbalWindTunnelSettings");
            if (settingsNode.Length < 1)
            {
                Debug.Log("[KWT] Settings file not found.");
                // To trigger creating a settings file.
                settingsChanged = true;
                return;
            }
            ConfigNode.LoadObjectFromConfig(this, settingsNode[0]);
        }

        public static void SaveSettings()
        {
            Instance.SaveSettingsToFile();
        }
        private void SaveSettingsToFile()
        {
            if (!settingsChanged)
                return;

            ConfigNode data = ConfigNode.CreateConfigFromObject(this, 0, new ConfigNode("KerbalWindTunnelSettings"));

            ConfigNode save = new ConfigNode();
            save.AddNode(data);
            save.Save("GameData/WindTunnel/KerbalWindTunnelSettings.cfg");
        }

        public static PopupDialog SpawnDialog(System.Action acceptAction = null, bool invokeOnlyOnChange = false)
        {
            List<DialogGUIBase> dialog = new List<DialogGUIBase>
            {
                new DialogGUIToggle(UseCoefficients, "#autoLOC_KWT100", b => UseCoefficients = b ),     // "Lift, Drag as coefficients"
                new DialogGUIToggle(SpeedIsMach, "#autoLOC_KWT101", (b) => SpeedIsMach = b),            // "Speed as Mach"
                new DialogGUIToggle(UseCharacterized, "#autoLOC_KWT102",  b => UseCharacterized = b),   // "Use faster vessel characterization"
                new DialogGUIToggle(StartMinimized, "#autoLOC_KWT103", b => StartMinimized = b),        // "Start minimized"
                new DialogGUIToggle(UseSingleColorHighlighting, "#autoLOC_KWT104", b => UseSingleColorHighlighting = b),    // "Use simple part highlighting"
                new DialogGUIToggle(ShowEnvelopeMask, "#autoLOC_KWT105", b => ShowEnvelopeMask = b),    // "Show flight envelope outline on graphs"
                new DialogGUIToggle(ShowEnvelopeMaskAlways && ShowEnvelopeMask, "#autoLOC_KWT106", b => { ShowEnvelopeMaskAlways = b; ShowEnvelopeMask |= b; }),    // "Show flight envelope outline even on flight envelope"
            };

            dialog.Add(new DialogGUIHorizontalLayout(TextAnchor.MiddleLeft,
                new DialogGUILabel(() => Localizer.Format("#autoLOC_KWT107", RotationCount), UISkinManager.defaultSkin.toggle, true),   // "Propeller rotation evaluations: "
                new DialogGUISlider(() => Mathf.Log(RotationCount, 2), 0, 4, true, 100, 20, value => RotationCount = (int)Mathf.Pow(2, value))
                ));

            if (ToolbarManager.ToolbarAvailable)
                dialog.Add(new DialogGUIToggle(UseBlizzy, "#autoLOC_KWT108", b => UseBlizzy = b));      // "Use Blizzy's Toolbar"

            dialog.Add(new DialogGUIToggle(EnableGlidingCalcs, "Enable gliding calculations", b => EnableGlidingCalcs = b));

            dialog.Add(new DialogGUIButton("#autoLOC_6001205", () =>         // "Accept"
            {
                if (!invokeOnlyOnChange || settingsChanged)
                    acceptAction?.Invoke();
            }, true));

            return PopupDialog.SpawnPopupDialog(new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f),
                new MultiOptionDialog(popupWindowName, "", "#autoLOC_KWT109", UISkinManager.defaultSkin, dialog.ToArray()), // "Kerbal Wind Tunnel Settings"
                false, UISkinManager.defaultSkin, true);
        }
    }
}
