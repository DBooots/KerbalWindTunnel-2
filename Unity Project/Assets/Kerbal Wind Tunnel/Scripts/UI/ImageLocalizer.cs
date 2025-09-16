using System;
using UnityEngine;
using UnityEngine.UI;

namespace KerbalWindTunnel.AssetLoader
{
    [ExecuteInEditMode]
    [RequireComponent(typeof(Image))]
    public class ImageLocalizer : MonoBehaviour
    {
        [SerializeField]
        public string identifier;
        [SerializeField]
        public string path;
        [SerializeField]
        public Rect spriteRect;
        [SerializeField]
        public bool useFullImage = true;

        [NonSerialized]
        public Sprite sprite;

        private Image image;

        private void Awake()
        {
            image = GetComponent<Image>();

            GameObject canvasObj = GetComponentInParent<Canvas>()?.gameObject;
            if (canvasObj?.GetComponent<KSPediaLocalizer>() == null)
                return;
            canvasObj.AddComponent<KSPediaLocalizer>();
        }

        private void Start()
        {
            if (string.IsNullOrEmpty(identifier))
                return;
            CreateSprite();
        }
#if UNITY_EDITOR
        private void OnValidate()
        {
            try
            {
                CreateSprite();
            }
            catch (Exception ex)
            {

            }
        }
#endif

        private void CreateSprite()
        {
            string localPath = path;
#if !OUTSIDE_UNITY
            localPath = Application.dataPath + "/Kerbal Wind Tunnel/KSPedia/Textures/" + identifier;
#endif
            DestroySprite();

            Texture2D texture;
            if (KSPediaLocalizer.ContainsKey(identifier))
                texture = KSPediaLocalizer.Fetch(identifier);
            else if (string.IsNullOrEmpty(localPath))
                texture = KSPediaLocalizer.FetchOrCreate(identifier);
            else
                texture = KSPediaLocalizer.FetchOrCreate(localPath, identifier);

            if (texture == null)
            {
                Debug.LogError("[KWT] ImageLocalizer received null texture.");
                return;
            }

            if (useFullImage)
                sprite = Sprite.Create(texture, new Rect(0, 0, texture.width, texture.height), new Vector2(0.5f, 0.5f));
            else
                sprite = Sprite.Create(texture, spriteRect, new Vector2(0.5f, 0.5f));

            image.sprite = sprite;
            //image.overrideSprite = texture;
        }

        private void OnDestroy()
            => DestroySprite();

        private void DestroySprite()
        {
            image.sprite = null;

            if (sprite != null)
#if UNITY_EDITOR
                DestroyImmediate(sprite);
#else
                Destroy(sprite);
#endif
        }
    }
}
