using UnityEngine;

/// <summary>
/// Turns gameobject along with view (rotation of headset).
/// This script turns the gameobject it is attached to according to the y axis of the view frustrum. Can be freezed / enabled again
/// </summary>
public class RotateWithView : MonoBehaviour
{
    #region PRIVATE_MEMBER_VARIABLES
    /// <summary>
    /// Specifies, if the screen should remain at its position or rotate with the view along the cave wall (enter is (0,0,0)
    /// </summary>
    [SerializeField]
    private bool m_ScreenFixed = false;

    /// <summary>
    /// How much the screen should be rotated further to the headset (no hiding screens behind Roboy)
    /// </summary>
    [SerializeField]
    private float m_RotationOffset = 35;

    /// <summary>
    /// pivot point around which the game object will be rotated
    /// </summary>
    [SerializeField]
    private Vector3 m_Pivot = Vector3.zero;

    /// <summary>
    /// Saves position of object when initialized
    /// </summary>
    private Vector3 m_InitialPosition;
    #endregion

    #region UNITY_MONOBEHAVIOUR_METHODS

    /// <summary>
    /// initialize standard position
    /// </summary>
    void Awake()
    {
        m_InitialPosition = gameObject.transform.position;
    }

    /// <summary>
    /// Set position soon as enabled (needed for no jumping behavior)
    /// </summary>
    void OnEnable()
    {
        AdjustScreen();
    }

    /// <summary>
    /// Continuously update position
    /// </summary>
    void Update()
    {
        AdjustScreen();
    }
    #endregion

    #region PUBLIC_METHODS

    /// <summary>
    /// Screen will turn with y angle of view frustrum.
    /// </summary>
    public void EnableTurning()
    {
        m_ScreenFixed = false;
    }

    /// <summary>
    /// If function called, screen will not move further with camera position
    /// </summary>
    public void FreezeScreenPosition()
    {
        m_ScreenFixed = true;
    }
    #endregion

    #region PRIVATE_METHODS
    /// <summary>
    /// rotates screen around specified pivot point and updates values
    /// </summary>
    private void AdjustScreen()
    {
        if (!m_ScreenFixed)
        {
            //only use angle around y axis to rotate around
            float angle = VRUILogic.Instance.GetCameraRotation().eulerAngles.y + m_RotationOffset;
            //Vector 0 as pivot point. can be arbitary point
            gameObject.transform.position = Quaternion.Euler(0, angle, 0) * (m_InitialPosition - m_Pivot) + m_Pivot;
            //make canvas face towards pivot point (zero vector)
            gameObject.transform.localRotation = Quaternion.Euler(0, angle, 0);
        }
    }
    #endregion
}

