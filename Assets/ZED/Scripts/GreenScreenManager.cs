
using UnityEngine;
using System.IO;



[RequireComponent(typeof(TextureOverlay))]
public class GreenScreenManager : MonoBehaviour
{
    /// <summary>
    /// The plane used for rendering
    /// </summary>
    private GameObject screen = null;
    /// <summary>
    /// The screen manager script
    /// </summary>
    private TextureOverlay screenManager = null;

    /// <summary>
    /// Chroma key settings
    /// </summary>
    [System.Serializable]
    public struct ChromaKey
    {
        public Color color;
        public float smoothness;
        public float range;
    }
    /// <summary>
    /// Chroma key data
    /// </summary>
    [System.Serializable]
    public struct ChromaKeyData
    {
        public ChromaKey chromaKeys;
        public int numberColors;
        public int erosion;
        public int numberBlurIterations;
        public float blurPower;
        public float whiteClip;
        public float blackClip;
        public float spill;
    }
    /// <summary>
    /// Max number of colors available
    /// </summary>
    private const uint numberColorMax = 1;
    /// <summary>
    /// Array of available chroma key settings
    /// </summary>
    public ChromaKey keys;
    /// <summary>
    /// Array of available key colors
    /// </summary>
    [SerializeField]
    public Color keyColors = new Color(0.0f, 1.0f, 0.0f, 1);
    /// <summary>
    /// Array of available similarity
    /// </summary>
    /// 
     [SerializeField]
    public float smoothness;
    /// <summary>
    /// Array of available blend
    /// </summary>
    /// 
     [SerializeField]
    public float range;
    /// <summary>
    /// Array of available blend
    /// </summary>
    /// 
     [SerializeField]
    public float spill = 0.2f;
    /// <summary>
    /// Default color for chroma key
    /// </summary>
    private Color defaultColor = new Color(0.0f, 1.0f, 0.0f, 1);
    /// <summary>
    /// DEfault similarity
    /// </summary>
    /// 

    private const float defaultSmoothness = 0.08f;
    /// <summary>
    /// Default blend
    /// </summary>
    private const float defaultRange = 0.4f;
    /// <summary>
    /// Default blend
    /// </summary>
    private const float defaultSpill = 0.1f;
    /// <summary>
    /// Default erosion
    /// </summary>
    private const int defaultErosion = 0;
    /// <summary>
    /// Default white clip
    /// </summary>
    private const float defaultWhiteClip = 1.0f;
    /// <summary>
    /// Default black clip
    /// </summary>
    private const float defaultBlackClip = 0.0f;
    /// <summary>
    /// Default sigma
    /// </summary>
    private const float defaultSigma = 0.1f;

    /// <summary>
    /// Current number of colors used
    /// </summary>
    [SerializeField]
    [HideInInspector]
    public int current_numberColors = 1;

    /// <summary>
    /// Final rendering material
    /// </summary>
    public Material finalMat;
    /// <summary>
    /// Green screen effect material
    /// </summary>
    private Material greenScreenMat;

    private Material preprocessMat;

    /// <summary>
    /// Alpha texture for blending
    /// </summary>
    private RenderTexture finalTexture;

    /// <summary>
    /// Available canals for display chroma key effect
    /// </summary>
    public enum CANAL
    {
        FOREGROUND,
        BACKGROUND,
        ALPHA,
        KEY,
        FINAL
    };
    /// <summary>
    /// Current canal used
    /// </summary>
    [HideInInspector]
    [SerializeField]
    public CANAL canal = CANAL.FINAL;

    /// <summary>
    /// Erosion value
    /// </summary>
    [HideInInspector]
    [SerializeField]
    public int erosion = 0;

    /// <summary>
    /// CutOff value
    /// </summary>
    [SerializeField]
    public float whiteClip = 1.0f;

    [SerializeField]
    public float blackClip = 0.0f;

    /// <summary>
    /// Green screen shader name
    /// </summary>
    [SerializeField]
    public string pathFileShader = "Config_greenscreen.json";


    /// <summary>
    /// Blur material
    /// </summary>
    private Material blurMaterial;
    /// <summary>
    /// Blur iteration number. A larger value increase blur effect.
    /// </summary>
    public int numberBlurIterations = 5;
    /// <summary>
    /// Sigma value. A larger value increase blur effect.
    /// </summary>
    public float sigma_ = 0.1f;
    /// <summary>
    /// Current sigma value
    /// </summary>
    private float currentSigma = -1;
    /// <summary>
    /// Weights for blur
    /// </summary>
    private float[] weights_ = new float[5];
    /// <summary>
    /// Offsets for blur
    /// </summary>
    private float[] offsets_ = { 0.0f, 1.0f, 2.0f, 3.0f, 4.0f };


    public Material matYUV;


    public void SetDefaultValues()
    {
        keyColors = defaultColor;
        smoothness = defaultSmoothness;
        range = defaultRange;
        spill = defaultSpill;
        erosion = defaultErosion;
        whiteClip = defaultWhiteClip;
        blackClip = defaultBlackClip;
        sigma_ = defaultSigma;
    }



    private void Awake()
    {
        if (screen == null)
        {
            screen = gameObject.transform.GetChild(0).gameObject;
            finalMat = screen.GetComponent<Renderer>().material;
            screenManager = GetComponent<TextureOverlay>();
        }
#if !UNITY_EDITOR
        Debug.Log("Load Chroma keys");
        LoadChromaKeys();
        UpdateShader();
#endif
    }

    private void Start()
    {
        finalTexture = new RenderTexture(sl.ZEDCamera.GetInstance().ImageWidth, sl.ZEDCamera.GetInstance().ImageHeight, 0, RenderTextureFormat.ARGB32, RenderTextureReadWrite.Linear);

        //alphaTexture.wrapMode = TextureWrapMode.Clamp;

        finalMat.SetTexture("_MaskTex", finalTexture);
        greenScreenMat = Resources.Load("Materials/Mat_ZED_Compute_GreenScreen") as Material;
        blurMaterial = Resources.Load("Materials/Mat_ZED_Blur") as Material;
        matYUV = Resources.Load("Materials/Mat_ZED_YUV") as Material;
        matYUV.SetInt("_isLinear", System.Convert.ToInt32(QualitySettings.activeColorSpace));

        preprocessMat = Resources.Load("Materials/Mat_ZED_Preprocess") as Material;
        preprocessMat.SetTexture("_CameraTex", screenManager.camZedLeft);
        ComputeWeights(1);

        //Send the values to the current shader
        blurMaterial.SetFloatArray("weights2", weights_);
        blurMaterial.SetFloatArray("offset2", offsets_);
        greenScreenMat.SetTexture("_CameraTex", screenManager.camZedLeft);

        UpdateNumberColors();
        UpdateShader();
        UpdateCanal();
    }



    /// <summary>
    /// Update the current canal (VIEW)
    /// </summary>
    public void UpdateCanal()
    {
        foreach (CANAL c in System.Enum.GetValues(typeof(CANAL)))
        {
            manageKeyWord(false, c.ToString());
        }
        manageKeyWord(true, canal.ToString());
    }

    /// <summary>
    /// Enable or disable a keyword
    /// </summary>
    /// <param name="value"></param>
    /// <param name="name"></param>
    void manageKeyWord(bool value, string name)
    {
        if (finalMat != null)
        {
            if (value)
            {
                finalMat.EnableKeyword(name);
            }
            else
            {
                finalMat.DisableKeyword(name);
            }
        }
    }

#if UNITY_EDITOR
    private void OnValidate()
    {
        UpdateNumberColors();
        UpdateShader();
    }
#endif

    private void OnApplicationQuit()
    {
        if (finalTexture != null && finalTexture.IsCreated()) finalTexture.Release();
    }

    public void Reset()
    {
            ZEDManager zedManager = null;
            zedManager = transform.parent.gameObject.GetComponent<ZEDManager>();
            if (zedManager != null)
                zedManager.videoMode = sl.RESOLUTION.HD1080;
            SetDefaultValues();
    }

    private void OnEnable()
    {
    }

    /// <summary>
    /// Update all the data to the shader
    /// The weights and offsets will be set when sigma change
    /// </summary>
    public void UpdateShader()
    {
        if (greenScreenMat != null)
        {
            greenScreenMat.SetColor("_keyColor", keyColors);
            greenScreenMat.SetFloat("_range", range);

            preprocessMat.SetFloat("_erosion", erosion + offsets_[2]);

            preprocessMat.SetFloat("_smoothness", smoothness);
            preprocessMat.SetFloat("_whiteClip", whiteClip);
            preprocessMat.SetFloat("_blackClip", blackClip);
            preprocessMat.SetFloat("_spill", spill);
            preprocessMat.SetColor("_keyColor", keyColors);
        }
    }

    /// <summary>
    /// Load the data from a file and fill a structure
    /// </summary>
    /// <returns></returns>
    private ChromaKeyData LoadData()
    {
        if (File.Exists(pathFileShader))
        {
            string dataAsJson = File.ReadAllText(pathFileShader);
            return JsonUtility.FromJson<ChromaKeyData>(dataAsJson);
        }
        else
        {
            ChromaKeyData chromaKeyData = new ChromaKeyData();

            chromaKeyData.numberColors = -1;
            chromaKeyData.spill = defaultSpill;

            //Load default values
            chromaKeyData.chromaKeys.color = defaultColor;
            chromaKeyData.chromaKeys.smoothness = defaultSmoothness;
            chromaKeyData.chromaKeys.range = defaultRange;

            return chromaKeyData;
        }
    }

    /// <summary>
    /// Save the chroma keys used in a file (JSON format)
    /// </summary>
    public void SaveChromaKeys()
    {
        ChromaKeyData chromaKeyData = new ChromaKeyData();
        chromaKeyData.chromaKeys = new ChromaKey();
        chromaKeyData.numberColors = current_numberColors;
        chromaKeyData.erosion = erosion;
        chromaKeyData.blurPower = sigma_;
        chromaKeyData.numberBlurIterations = numberBlurIterations;
        chromaKeyData.whiteClip = whiteClip;
        chromaKeyData.spill = spill;
        chromaKeyData.blackClip = blackClip;


        chromaKeyData.chromaKeys.color = keyColors;
        chromaKeyData.chromaKeys.smoothness = smoothness;
        chromaKeyData.chromaKeys.range = range;

        SaveData(chromaKeyData);
    }

    /// <summary>
    /// Return a string from a pointer to char
    /// </summary>
    private void SaveData(ChromaKeyData chromaKeyData)
    {
        string dataAsJson = JsonUtility.ToJson(chromaKeyData);

        File.WriteAllText(pathFileShader, dataAsJson);
    }

    /// <summary>
    /// Fill the current chroma keys with the data from a file
    /// </summary>
    public void LoadChromaKeys()
    {
        ChromaKeyData chromaKeyData = LoadData();
        if (chromaKeyData.numberColors != -1)
        {
            current_numberColors = chromaKeyData.numberColors;
            erosion = chromaKeyData.erosion;
            sigma_ = chromaKeyData.blurPower;
            numberBlurIterations = chromaKeyData.numberBlurIterations;
            whiteClip = chromaKeyData.whiteClip;
            blackClip = chromaKeyData.blackClip;
            spill = chromaKeyData.spill;

            keyColors = chromaKeyData.chromaKeys.color;
            smoothness = chromaKeyData.chromaKeys.smoothness;
            range = chromaKeyData.chromaKeys.range;
        }
        UpdateShader();

    }

    // Update the current number of colors used in the shader 
    public void UpdateNumberColors()
    {
        if (greenScreenMat != null)
        {
            greenScreenMat.SetInt("_numberColors", current_numberColors);
        }
    }

    private void ComputeWeights(float sigma)
    {
        float[] weights = new float[5];
        float[] offsets = { 0.0f, 1.0f, 2.0f, 3.0f, 4.0f };
        // Calculate the weights 
        weights[0] = Gaussian(0, sigma);
        float sum = weights[0];
        for (int i = 1; i < 5; ++i)
        {
            weights[i] = Gaussian(i, sigma);
            sum += 2.0f * weights[i];
        }

        for (int i = 0; i < 5; ++i)
        {
            weights[i] /= sum;
        }

        // fix for just 3 fetches 
        weights_[0] = weights[0];
        weights_[1] = weights[1] + weights[2];
        weights_[2] = weights[3] + weights[4];

        offsets_[0] = 0.0f;
        offsets_[1] = ((weights[1] * offsets[1]) + (weights[2] * offsets[2])) / weights_[1];
        offsets_[2] = ((weights[3] * offsets[3]) + (weights[4] * offsets[4])) / weights_[2];
    }

    private void Blur(RenderTexture source, RenderTexture dest, Material mat, int pass, int numberIterations = -1, int downscale = 2)
    {
        bool oddEven = false;
        if (numberIterations == -1) numberIterations = numberBlurIterations;
        //Create two buffers to make a multi-pass blur
        RenderTexture buffer = RenderTexture.GetTemporary(source.width / downscale, source.height / downscale, source.depth, source.format, RenderTextureReadWrite.Linear);
        RenderTexture buffer2 = RenderTexture.GetTemporary(source.width / downscale, source.height / downscale, source.depth, source.format, RenderTextureReadWrite.Linear);

        Graphics.Blit(source, buffer);
        //To each pass alternate the buffer, and set the blur direction
        for (int i = 0; i < numberIterations; i++)
        {
            blurMaterial.SetInt("horizontal", System.Convert.ToInt32(oddEven));

            if (!oddEven)
            {
                Graphics.Blit(buffer, buffer2, mat, pass);
            }
            else
            {
                Graphics.Blit(buffer2, buffer, mat, pass);
            }
            oddEven = !oddEven;
        }

        //Copy the buffer to the final texture
        if (oddEven)
        {
            Graphics.Blit(buffer2, dest);
        }
        else
        {
            Graphics.Blit(buffer, dest);
        }

        //Destroy all the temporary buffers
        RenderTexture.ReleaseTemporary(buffer);
        RenderTexture.ReleaseTemporary(buffer2);
    }



    /// <summary>
    /// Return a string from a pointer to char
    /// </summary>
    private void OnPreRender()
    {
        if (screenManager.camZedLeft == null || screenManager.camZedLeft.width == 0) return;
        if (canal.Equals(CANAL.FOREGROUND)) return;
        
		RenderTexture tempYUV = RenderTexture.GetTemporary(screenManager.camZedLeft.width, screenManager.camZedLeft.height, 0, RenderTextureFormat.Default, RenderTextureReadWrite.Linear);
		RenderTexture tempYUVBlur = RenderTexture.GetTemporary(screenManager.camZedLeft.width, screenManager.camZedLeft.height, 0, RenderTextureFormat.Default, RenderTextureReadWrite.Linear);

		RenderTexture tempAlpha = RenderTexture.GetTemporary(finalTexture.width, finalTexture.height, 0, RenderTextureFormat.RFloat, RenderTextureReadWrite.Linear);
        RenderTexture tempFinalAlpha = RenderTexture.GetTemporary(finalTexture.width, finalTexture.height, 0, RenderTextureFormat.ARGB32, RenderTextureReadWrite.Linear);

        Graphics.Blit(screenManager.camZedLeft, tempYUV, matYUV);
        Blur(tempYUV, tempYUVBlur, blurMaterial, 1, 2, 1);

        Graphics.Blit(tempYUVBlur, tempAlpha, greenScreenMat);
        preprocessMat.SetTexture("_MaskTex", tempAlpha);
        Graphics.Blit(screenManager.camZedLeft, tempFinalAlpha, preprocessMat);

        //If the sigma has changed recompute the weights and offsets used by the blur
        if (sigma_ != 0)
        {
            if (sigma_ != currentSigma)
            {
                currentSigma = sigma_;

                ComputeWeights(currentSigma);

                //Send the values to the current shader
                blurMaterial.SetFloatArray("weights", weights_);
                blurMaterial.SetFloatArray("offset", offsets_);
            }
            Blur(tempFinalAlpha, finalTexture, blurMaterial, 0, 2, 1);

        }
        else
        {
            Graphics.Blit(tempFinalAlpha, finalTexture);
        }
        

        //Destroy all the temporary buffers
        RenderTexture.ReleaseTemporary(tempYUVBlur);
        RenderTexture.ReleaseTemporary(tempAlpha);
        RenderTexture.ReleaseTemporary(tempYUV);
        RenderTexture.ReleaseTemporary(tempFinalAlpha);
    }

    float Gaussian(float x, float sigma)
    {
        return (1.0f / (2.0f * Mathf.PI * sigma)) * Mathf.Exp(-((x * x) / (2.0f * sigma)));
    }
}



