using UnityEngine;
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
}
