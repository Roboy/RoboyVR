using UnityEngine;

/// <summary>
/// This class describes the general behaviour of a mode, containing its screens, UI layouts and so on. 
/// </summary>
public class UIModeManager : MonoBehaviour
{
    #region PUBLIC_MEMBER_VARIABLES
    #endregion

    #region PRIVATE_MEMBER_VARIABLES
    /// <summary>
    /// Specifies, if the screen should remain at its position or rotate with the view along the cave wall (enter is (0,0,0)
    /// </summary>
    [SerializeField]
    private bool fixedScreen = true;

    /// <summary>
    /// The screen belonging to that mode
    /// </summary>
    [SerializeField]
    private Canvas screen;
    #endregion

    #region UNITY_MONOBEHAVIOUR_METHODS
    /// <summary>
    /// Called once by Unity during startup
    /// </summary>
    void Start()
    {
        
    }

    /// <summary>
    /// rotates screen and updates values
    /// </summary>
    public virtual void AdjustScreen()
    {
        if (!fixedScreen)
        {
            //TODO: BUGGY!!!!!
            float angle = VRUILogic.Instance.GetCameraRotation().eulerAngles.y;
            screen.transform.RotateAround(Vector3.zero, Vector3.up,  angle);
        }
    }
    #endregion

    #region PUBLIC_METHODS
    #endregion

    #region PRIVATE_METHODS
    #endregion
}

