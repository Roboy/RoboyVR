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
    /// Saves position of object when initialized
    /// </summary>
    private Vector3 initPos; 
    /// <summary>
    /// The screen belonging to that mode
    /// </summary>
    [SerializeField]
    private Canvas screen;
    /// <summary>
    /// How much the screen should be rotated further to the headset (no hiding screens behind Roboy)
    /// </summary>
    [SerializeField]
    private float rotationOffset = 20;
    #endregion
    #region UNITY_MONOBEHAVIOUR_METHODS
    /// <summary>
    /// Called once by Unity during startup
    /// </summary>
    void OnEnable()
    {
        initPos = screen.transform.position;
    }

    /// <summary>
    /// rotates screen and updates values
    /// </summary>
    public virtual void AdjustScreen()
    {
        if (!fixedScreen)
        {
            //only use angle around y axis to rotate around
            float angle = VRUILogic.Instance.GetCameraRotation().eulerAngles.y + rotationOffset;
            //Vector 0 as pivot point. can be arbitary point
            screen.transform.position  = Quaternion.Euler (0,angle,0) * (initPos    - Vector3.zero) + Vector3.zero ; 
            //make canvas face towards pivot point (zero vector)
            screen.transform.localRotation = Quaternion.Euler(0, angle, 0);
        }
    }
    #endregion

    #region PUBLIC_METHODS
    #endregion

    #region PRIVATE_METHODS
    #endregion
}

