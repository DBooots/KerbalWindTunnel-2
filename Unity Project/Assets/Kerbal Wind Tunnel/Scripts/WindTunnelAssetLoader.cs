using System.Collections.Generic;
using System.Linq;
using UI_Tools.Universal_Text;
using UnityEngine;
using UnityEngine.UI;

namespace KerbalWindTunnel.AssetLoader
{
    [KSPAddon(KSPAddon.Startup.Instantly, true)]
    public class WindTunnelAssetLoader : MonoBehaviour
    {
        public const string inputLockID = "WindTunnel";
        internal static bool useTMP = true;
        private const string assetBundlePath = "GameData/WindTunnel/windtunnelassetbundle.dat";
        private const string windowPrefabName = "Wind Tunnel Window";

        private const string shaderBundlePath = "GameData/WindTunnel/graphingShaders.assetbundle";
        private static Dictionary<string, Shader> shaders = null;
        private static GameObject[] loadedPrefabs = null;

        private const string kspediaResourcesPath = "GameData/WindTunnel/Textures/KSPedia Localization/";
        public static GameObject WindowPrefab {
            get { ProcessPrefabs(); return _windowPrefab; }
            private set => _windowPrefab = value;
        }
        private static GameObject _windowPrefab;
        private static bool processedPrefabs = false;
        private static readonly Color inactiveTextColor = new Color(0.65f, 0.65f, 0.65f);
        public static void ProcessPrefabs()
        {
            if (processedPrefabs || loadedPrefabs == null)
                return;
            //UT_Base.CreateUTComponents(loadedPrefabs);
            foreach (GameObject go in loadedPrefabs)
                SetSkinBasedOnComponents(go, SkinSettingStrictness.Relaxed);
            if (useTMP)
                UT_Base.SetMode(loadedPrefabs, UT_Mode.TMPro);
            processedPrefabs = true;
        }

        public static void ProcessKSPediaResources()
        {
            List<System.Threading.Tasks.Task> tasks = new List<System.Threading.Tasks.Task>();
            string currentLanguage;
#if UNITY_EDITOR
            currentLanguage = "en-us";
#else
            currentLanguage = KSP.Localization.Localizer.CurrentLanguage;
#endif
            string path = KSPUtil.ApplicationRootPath + kspediaResourcesPath;

            Debug.Log("[KWT] Processing KSPedia localized image files.");
            foreach (string file in System.IO.Directory.EnumerateFiles(path, "*.*", System.IO.SearchOption.AllDirectories))
            {
                if (!System.IO.Path.HasExtension(file))
                    continue;

                // only load those for the current language
                int localizationStart = file.LastIndexOf("_");
                int localizationEnd = file.LastIndexOf(".") - 1;

                if (localizationStart > localizationEnd)
                    continue;
                if (localizationStart < 0)
                    continue;

                if (!string.Equals(file.Substring(localizationStart + 1, currentLanguage.Length), currentLanguage, System.StringComparison.CurrentCultureIgnoreCase))
                    continue;

                Debug.Log(file);

                // strip out the localization suffix
                string file_ = file.Remove(localizationStart, currentLanguage.Length + 1);
                string filename = System.IO.Path.GetFileName(file_);
                tasks.Add(KSPediaLocalizer.LoadTexture(file_, filename));
            }
            System.Threading.Tasks.Task.WhenAll(tasks).Wait(5000);
        }

        [System.Diagnostics.CodeAnalysis.SuppressMessage("CodeQuality", "IDE0051:Remove unused private members", Justification = "Unity Method")]
        private void Start()
        {
            ProcessKSPediaResources();

            string path = KSPUtil.ApplicationRootPath + assetBundlePath;

            AssetBundle prefabs = AssetBundle.LoadFromFile(path);
            if (prefabs == null)
                Debug.LogError("[KWT] Failed to find UI asset bundle.");

            if (prefabs != null)
                loadedPrefabs = prefabs.LoadAllAssets<GameObject>();

            Debug.Log("[KWT] Loaded the following prefabs:");
            foreach (var prefab in loadedPrefabs) Debug.Log(prefab.name);

            foreach (GameObject go in loadedPrefabs)
            {
                if (go.name == windowPrefabName)
                    _windowPrefab = go;
                if (go.name == "AxisBoundPanel")
                    go.AddComponent<CanvasLayerSetter>();
                if (useTMP)
                    foreach (Dropdown dropdown in go.GetComponentsInChildren<Dropdown>(true))
                        dropdown.template?.gameObject.AddComponent<CanvasLayerSetter>();
                foreach (UT_InputField inputField in go.GetComponentsInChildren<UT_InputField>(true))
                {
                    Extensions.InputLockSelectHandler inputLockHandler = inputField.gameObject.AddComponent<Extensions.InputLockSelectHandler>();
                    inputLockHandler.Setup(inputLockID, ControlTypes.KEYBOARDINPUT);
                }
            }
            if (_windowPrefab == null)
                Debug.LogError("[KWT] Failed to load UI prefab.");

            if (shaders == null)
            {
                AssetBundle bundle = AssetBundle.LoadFromFile(shaderBundlePath);
                if (bundle == null)
                    Debug.LogError("[KWT] Failed to find shader assetbundle.");

                shaders = new Dictionary<string, Shader>();
                foreach (var shader in bundle.LoadAllAssets<Shader>())
                {
                    if (!shaders.ContainsKey(shader.name))
                    {
                        shaders.Add(shader.name, shader);
                    }
                }
            }
        }

        private enum SkinSettingStrictness
        {
            Loose = 0,
            Relaxed = 1,
            Strict = 2
        }

        private static void SetSkinBasedOnComponents(GameObject rootObject, SkinSettingStrictness strictness = SkinSettingStrictness.Loose)
        {
            UISkinDef skin = UISkinManager.defaultSkin;//HighLogic.UISkin;
            //GUISkin GUISkin = HighLogic.Skin;

            SetWindowSkin(rootObject, skin.window, strictness);
            foreach (Selectable selectable in rootObject.GetComponentsInChildren<Selectable>(true))
            {
                if (selectable is InputField textField)
                {
                    UIStyle textFieldStyle = textField.multiLine ? skin.textArea : skin.textField;
                    SetSelectableSkin(selectable, textField.multiLine ? skin.textArea : skin.textField);
                    Text textComponent = textField.textComponent;
                    textComponent.color = textFieldStyle.normal.textColor;

                    if (textField.placeholder is Text placeholder)
                        placeholder.color = new Color(0.624f, 0.624f, 0.624f);
                    // Do something about the label color.
                }
                else if (selectable is UnityEngine.UI.Button)
                {
                    if (strictness > SkinSettingStrictness.Loose && selectable.transform.childCount == 0)
                        continue;
                    SetSelectableSkin(selectable, skin.button);
                    Text buttonText = selectable.GetComponentInChildren<Text>(true);
                    if (buttonText != null && (strictness < SkinSettingStrictness.Relaxed || buttonText.color == new Color(50 / 255f, 50 / 255f, 50 / 255f)))
                    {
                        buttonText.color = skin.button.normal.textColor;
                        selectable.gameObject.AddComponent<SelectableTextColorChanger>().SetupColors(skin.button.normal.textColor, skin.button.disabled.textColor.a != 0 ? skin.button.disabled.textColor : inactiveTextColor, skin.button.active.textColor, skin.button.highlight.textColor);
                    }
                }
                else if (selectable is Toggle toggle)
                {
                    if (strictness > SkinSettingStrictness.Loose && selectable.transform.childCount == 0)
                        continue;
                    Text toggleText = selectable.GetComponentInChildren<Text>(true);
                    // The items in a Dropdown template are Toggles, so we want to ignore them.
                    // This is the exact number of .parent?. calls to get to where the Dropdown component would be.
                    // Don't worry, it short-circuits if the gameObject doesn't have that many parents.
                    if (selectable.transform.parent?.parent?.parent?.parent?.GetComponent<Dropdown>() != null)
                    {
                        //SetSelectableSkin(selectable, skin.button);
                        toggleText.color = skin.toggle.normal.textColor;
                        continue;
                    }
                    SetSelectableSkin(selectable, toggle.graphic == null ? skin.button : skin.toggle);
                    if (toggleText != null)
                    {
                        if (toggle.graphic == null)
                        {
                            // This looks like a button
                            selectable.gameObject.AddComponent<SelectableTextColorChanger>().SetupColors(
                                skin.button.normal.textColor, skin.button.disabled.textColor.a != 0 ? skin.button.disabled.textColor : inactiveTextColor, skin.button.active.textColor, skin.button.highlight.textColor);
                            selectable.gameObject.AddComponent<ToggleSpriteSwapper>().Setup(
                                skin.button.normal.background, skin.button.active.background, skin.button.normal.textColor, skin.button.active.textColor, true);
                            toggleText.color = skin.button.normal.textColor;
                        }
                        else
                        {
                            // This looks like a checkbox
                            selectable.gameObject.AddComponent<ToggleSpriteSwapper>().Setup(
                                skin.toggle.normal.background, skin.toggle.active.background, skin.toggle.normal.textColor, skin.toggle.active.textColor, false);
                            toggleText.color = skin.toggle.normal.textColor;

                            // Fix the size of the graphic for checkboxes.
                            if (toggle.image?.rectTransform != toggle.transform)
                                toggle.image.rectTransform.sizeDelta = new Vector2(30, 30);
                            // Do something about the old check:
                            if (toggle.graphic != null)
                                toggle.graphic.enabled = false;
                            toggle.graphic = null;
                        }
                    }
                }
                else if (selectable is Dropdown || selectable is TMPro.TMP_Dropdown)
                {
                    SetSelectableSkin(selectable, skin.button);
                    Text buttonText = selectable.GetComponentInChildren<Text>(true);
                    if (buttonText != null && (strictness < SkinSettingStrictness.Relaxed || buttonText.color == new Color(50 / 255f, 50 / 255f, 50 / 255f)))
                    {
                        buttonText.color = skin.button.normal.textColor;
                    }

                    selectable.gameObject.AddComponent<SelectableTextColorChanger>().SetupColors(
                        skin.button.normal.textColor, skin.button.disabled.textColor.a != 0 ? skin.button.disabled.textColor : inactiveTextColor, skin.button.active.textColor, skin.button.highlight.textColor);
                    // Figure out what changes to make to the Template.
                    Debug.Log(selectable.gameObject.name);
                    Image background = selectable.GetComponentInChildren<ScrollRect>(true)?.GetComponent<Image>();
                    if (background != null)
                        background.sprite = skin.window.normal.background;
                }
                else if (selectable is Scrollbar scrollbar)
                {
                    bool horizontal = scrollbar.direction == Scrollbar.Direction.LeftToRight || scrollbar.direction == Scrollbar.Direction.RightToLeft;
                    SetSelectableSkin(selectable, horizontal ? skin.horizontalScrollbarThumb : skin.verticalScrollbarThumb);
                    if (scrollbar.TryGetComponent<Image>(out Image image))
                        image.sprite = horizontal ? skin.horizontalScrollbar.normal.background : skin.verticalScrollbar.normal.background;
                }
                else if (selectable is Slider slider)
                {
                    bool horizontal = slider.direction == Slider.Direction.LeftToRight || slider.direction == Slider.Direction.RightToLeft;
                    SetSelectableSkin(selectable, horizontal ? skin.horizontalSliderThumb : skin.verticalSliderThumb);
                    Image image = slider.fillRect?.GetComponent<Image>();
                    if (image != null)
                        image.sprite = horizontal ? skin.horizontalSlider.normal.background : skin.verticalSlider.normal.background;
                    for (int i = 0; i < slider.transform.childCount; i++)
                    {
                        if (slider.transform.GetChild(i).TryGetComponent<Image>(out image))
                        {
                            image.sprite = horizontal ? skin.horizontalSlider.normal.background : skin.verticalSlider.normal.background;
                            break;
                        }
                    }
                }
            }
            foreach (ScrollRect scrollView in rootObject.GetComponentsInChildren<ScrollRect>(true))
            {
                // If this is the scroll window of a dropdown, it will be handled in the dropdown section.
                if (scrollView.transform.parent?.GetComponent<Dropdown>() != null)
                    continue;
                if (scrollView.TryGetComponent<Image>(out Image image))
                    image.sprite = skin.scrollView.normal.background ?? skin.window.normal.background;
            }
            if (strictness < SkinSettingStrictness.Relaxed)
                foreach (Image image in rootObject.GetComponentsInChildren<Image>(true).Where(i => i.sprite == null && i.mainTexture == null && i.transform.childCount > 0))
                    image.sprite = skin.box.normal.background;
        }

        private static void SetSelectableSkin(Selectable selectable, UIStyle style)
        {
            SpriteState spriteState = selectable.spriteState;
            spriteState.highlightedSprite = style.highlight.background;
            spriteState.selectedSprite = style.normal.background;   // Either normal or highlight
            spriteState.pressedSprite = style.active.background;
            spriteState.disabledSprite = style.disabled.background;
            selectable.spriteState = spriteState;
            selectable.image.sprite = style.normal.background;
            selectable.transition = Selectable.Transition.SpriteSwap;
        }
        private static void SetWindowSkin(GameObject rootObject, UIStyle style, SkinSettingStrictness strictness = SkinSettingStrictness.Loose)
        {
            foreach (Image image in rootObject.GetComponentsInChildren<Image>(true))
            {
                for (Transform transform = image.transform; transform != null; transform = transform.parent)
                {
                    if (transform.parent == null || transform.GetComponent<Canvas>() != null)
                    {
                        if (strictness < SkinSettingStrictness.Strict || image.sprite.name == "Background")
                        {
                            image.sprite = style.normal.background;
                            image.color = new Color(1, 1, 1, 1);
                        }
                        break;
                    }
                    if (transform.GetComponent<Image>() != null)
                        break;
                }
            }
        }

        public static Shader GetShader(string name)
        {
            if (shaders == null)
                return null;
            if (!shaders.ContainsKey(name))
                return null;
            return shaders[name];
        }
    }
}