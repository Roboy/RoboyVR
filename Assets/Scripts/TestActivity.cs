using UnityEngine;

/// <summary>
/// class to determine call order of respective unity methods.
/// Test for enabled/disabled scripts/gameobjects
/// </summary>
public class TestActivity : MonoBehaviour
{
    #region PRIVATE_MEMBER_VARIABLES
    private int count = 0;
    #endregion

    #region UNITY_MONOBEHAVIOUR_METHODS

    void Awake()
    {
        Debug.Log(gameObject.name + " Awake");
    }
    /// <summary>
    /// Called once by Unity during startup
    /// </summary>
    void Start()
    {
        Debug.Log(gameObject.name + " Start");
    }

    void OnEnable()
    {
        Debug.Log(gameObject.name + " OnEnable");
    }
    /// <summary>
    /// Called once every frame
    /// </summary>
    void Update()
    {
        if (count < 2)
        {
            Debug.Log(gameObject.name + " Update");
            count++;
        }
    }
    #endregion
}
