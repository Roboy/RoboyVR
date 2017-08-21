using System.Collections;
using UnityEngine;

/// <summary>
/// Provides all functionalities and manages Middleware mode
/// </summary>
public class MiddlewareManager : MonoBehaviour
{
    #region PUBLIC_MEMBER_VARIABLES
    #endregion

    #region PRIVATE_MEMBER_VARIABLES
    /// <summary>
    /// For testing purposes
    /// </summary>
    private int tendonID;
    #endregion

    #region UNITY_MONOBEHAVIOUR_METHODS
    /// <summary>
    /// Called once by Unity during startup
    /// </summary>
    void Start()
    {
        Vector3[] points = { new Vector3(0, -1, 0), new Vector3(0.2f, 0, 0.2f) };
        string[] names = { "hip", "thigh_right"};
        tendonID = 1;
        VRUILogic.Instance.AddTendon(tendonID, points, names, 1f);
    }

    /// <summary>
    /// Called once every frame
    /// </summary>
    void Update()
    {
        float x = Mathf.Cos(Time.realtimeSinceStartup) * 0.5f + 0.5f;
        VRUILogic.Instance.UpdateTendon(tendonID, x);
    }
    #endregion

    #region PUBLIC_METHODS
    #endregion

    #region PRIVATE_METHODS

    #endregion
}