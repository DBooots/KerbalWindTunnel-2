using System;
using System.Collections.Generic;
using System.IO;
using System.Threading.Tasks;
using UnityEngine;
using KSP.Localization;

// Localization Guide:
// 1. Take screenshots that exactly match the dimensions and layout shown.
// 2. Name them according to the identifier_language.png convention.
// 3. This and the ImageLocalizer script will handle the rest.

namespace KerbalWindTunnel.AssetLoader
{
    [ExecuteInEditMode]
    public class KSPediaLocalizer : MonoBehaviour
    {
        private static readonly Dictionary<string, Texture2D> textures = new Dictionary<string, Texture2D>();
        private static KSPediaLocalizer instance;

        private void Awake()
        {
            if (instance == null)
            {
                instance = this;
                Debug.Log("[KWT] KSPediaLocalizer created.");
            }
            else
#if UNITY_EDITOR
                DestroyImmediate(this);
#else
                Destroy(this);
#endif
        }

        private void OnDestroy()
        {
            if (this != instance)
                return;
            instance = null;
            Debug.Log("[KWT] Clearing KSPedia texture cache.");
            foreach (KeyValuePair<string, Texture2D> texEntry in textures)
#if UNITY_EDITOR
                DestroyImmediate(texEntry.Value);
#else
                Destroy(texEntry.Value);
#endif
            textures.Clear();
        }

        public static bool ContainsKey(string filename) => textures.ContainsKey(filename);
        public static void AddTexture(string filename, Texture2D texture)
        {
            if (string.IsNullOrEmpty(filename))
                throw new ArgumentNullException("filename");
            textures.Add(filename, texture);
        }
        public static async Task LoadTexture(string path, string identifier)
        {
            if (string.IsNullOrEmpty(path))
                throw new ArgumentNullException("path");
            if (string.IsNullOrEmpty(identifier))
                throw new ArgumentNullException("identifier");

            if (textures.ContainsKey(identifier))
                return;
            byte[] bytes;
            string localPath = path;
            if (!Path.HasExtension(localPath))
                localPath = Path.Combine(localPath, identifier);
            localPath = PrepareFilename(localPath);
            using (FileStream stream = File.Open(localPath, FileMode.Open, FileAccess.Read))
            {
                bytes = new byte[stream.Length];
                await stream.ReadAsync(bytes, 0, (int)stream.Length);
            }
            Texture2D texture = new Texture2D(2, 2);
            texture.LoadImage(bytes);
            lock (textures)
            {
                if (textures.ContainsKey(identifier))
                    return;
                textures.Add(identifier, texture);
            }
        }

        public static Texture2D Fetch(string identifier)
        {
            if (string.IsNullOrEmpty(identifier))
                throw new ArgumentNullException("identifier");
            if (textures.ContainsKey(identifier))
                return textures[identifier];
            Debug.LogException(new KeyNotFoundException(identifier));
            return null;
        }

        public static Texture2D FetchOrCreate(string filename)
           => FetchOrCreate(filename, filename);
        public static Texture2D FetchOrCreate(string path, string identifier)
        {
            if (string.IsNullOrEmpty(path))
                throw new ArgumentNullException("path");
            if (string.IsNullOrEmpty(identifier))
                throw new ArgumentNullException("identifier");

            if (textures.ContainsKey(identifier))
                return textures[identifier];
            Texture2D texture = new Texture2D(2, 2);
            string localPath = path;
            if (!Path.HasExtension(localPath))
                localPath = Path.Combine(localPath, identifier);
            localPath = PrepareFilename(localPath);
            texture.LoadImage(File.ReadAllBytes(localPath));
            lock (textures)
            {
                if (textures.ContainsKey(identifier))
                    return textures[identifier];
                textures.Add(identifier, texture);
                return texture;
            }
        }

        public static void AliasTexture(string original, string alias)
        {
            if (!textures.ContainsKey(original))
                throw new KeyNotFoundException(original);
            lock (textures)
                textures.Add(alias, textures[original]);
        }

        private static string PrepareFilename(string path)
        {
            if (string.IsNullOrEmpty(path))
                throw new ArgumentNullException("path");

            if (!(
                path.EndsWith(".png", StringComparison.CurrentCultureIgnoreCase) ||
                path.EndsWith(".jpg", StringComparison.CurrentCultureIgnoreCase) ||
                path.EndsWith(".jpeg", StringComparison.CurrentCultureIgnoreCase)))
                throw new FileLoadException($"[KWT-Localizer] Path must end in a supported file format. \"{path}\"");

            if (!Path.IsPathRooted(path))
                path = KSPUtil.ApplicationRootPath + path;

            int extensionIndex = path.LastIndexOf(".");

            string language;
#if UNITY_EDITOR
            language = "en-us";
#else
            language = Localizer.CurrentLanguage;
#endif

            if (TryLoad(path, $"_{language}", extensionIndex, out string localizedPath))
                return localizedPath;

            // If using en-us, try without suffix
            if (string.Equals(language, "en-us", StringComparison.InvariantCultureIgnoreCase))
            {
                if (TryLoad(path, "", 0, out localizedPath))
                {
#if DEBUG
                    Debug.LogError($"[KWT-Localizer] The path {path} is not localized.");
#endif
                    return localizedPath;
                }
            }

            // Try uppercase
            if (TryLoad(path, $"_{language.ToUpper()}", extensionIndex, out localizedPath))
            {
                Debug.LogWarning($"[KWT-Localizer] The path {path} in language {language} is in upper case when it shouldn't be.");
                return localizedPath;
            }

            // Try the default language
            if (!string.Equals(language, "en-us", StringComparison.InvariantCultureIgnoreCase))
            {
                if (TryLoad(path, "_en-us", extensionIndex, out localizedPath))
                {
                    string preferredPath = path.Insert(extensionIndex, $"_{language}");
                    Debug.LogError($"[KWT-Localizer] Could not find {path} in language {language}: {preferredPath}");
                    return localizedPath;
                }
                if (TryLoad(path, "_EN-US", extensionIndex, out localizedPath))
                {
                    string preferredPath = path.Insert(extensionIndex, $"_{language}");
                    Debug.LogError($"[KWT-Localizer] Could not find {path} in language {language}: {preferredPath}");
                    return localizedPath;
                }
                
                localizedPath = path.Insert(extensionIndex, "_en-us");

                // Try just the filename
                if (TryLoad(path, "", 0, out localizedPath))
                {
                    Debug.LogWarning($"[KWT-Localizer] The path {path} is not localized.");
                    return localizedPath;
                }
            }

            throw new FileNotFoundException($"[KWT-Localizer] Could not find {path}.");
        }

        private static bool TryLoad(string path, string localizedSuffix, int extensionIndex, out string localizedPath)
        {
            localizedPath = path.Insert(extensionIndex, localizedSuffix);
            return File.Exists(localizedPath);
        }
    }
}
