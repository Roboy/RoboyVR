using System.Collections;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Wrapper class around GraphRenderer and ExtensionMethod.
/// Creates a Graph with a list containing the displayed data. Provides Scaling, setting of boundaries, 
/// operations on and alteration of data. 
/// </summary>
public class GraphObject : MonoBehaviour
{
    #region PUBLIC_MEMBER_VARIABLES
    private GraphRenderer graph;
    #endregion

    #region PRIVATE_MEMBER_VARIABLES
    #endregion

    #region UNITY_MONOBEHAVIOUR_METHODS
    /// <summary>
    /// Called once by Unity during startup / next frame when game obj created
    /// </summary>
    void Awake()
    {
        graph = new GraphRenderer();
    }

    /// <summary>
    /// Called once every frame
    /// </summary>
    void Update()
    {
    }
    #endregion

    #region PUBLIC_METHODS
    #endregion

    #region PRIVATE_METHODS
    #endregion
}

