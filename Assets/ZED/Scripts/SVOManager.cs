using UnityEngine;

/// <summary>
/// Manages the SVO, only used as an interface for Unity
/// </summary>
public class SVOManager : MonoBehaviour
{
    [SerializeField]
    public bool record = false;
    [SerializeField]
    public bool read = false;
    [SerializeField]
    [HideInInspector]
    public string videoFile = "";
    [SerializeField]
    [Tooltip("Loop the SVO")]
    public bool loop = false;

    [HideInInspector]
    [SerializeField]
    private int currentFrame = 0;
    [HideInInspector]
    [SerializeField]
    private int numberFrameMax = 0;

    [HideInInspector]
    [SerializeField]
    public bool pause = false;

    [HideInInspector]
    [SerializeField]
    public bool needUpdateFrame = false;

    [HideInInspector]
    public int NumberFrameMax
    {
        set
        {
            numberFrameMax = value;
        }
        get
        {
            return numberFrameMax;
        }
    }

    public int CurrentFrame
    {
        get
        {
            return currentFrame;
        }
        set
        {
            currentFrame = value;
        }
    }
}
