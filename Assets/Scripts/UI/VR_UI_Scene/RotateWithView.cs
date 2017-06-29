using UnityEngine;
/// <summary>
/// Turns gameobject along with view.
/// This script turns the gameobject it is attached to according to the y axis of the view frustrum. Can be freezed / enabled again
/// </summary>
public class RotateWithView : MonoBehaviour {
#region PRIVATE_MEMBER_VARIABLES
    /// <summary>
    /// Specifies, if the screen should remain at its position or rotate with the view along the cave wall (enter is (0,0,0)
    /// </summary>
    [SerializeField]
    private bool fixedScreen = false;

    /// <summary>
    /// Saves position of object when initialized
    /// </summary>
    private Vector3 initPos;

    /// <summary>
    /// How much the screen should be rotated further to the headset (no hiding screens behind Roboy)
    /// </summary>
    [SerializeField]
    private float rotationOffset = 35;

    /// <summary>
    /// pivot point around which the game object will be rotated
    /// </summary>
    [SerializeField]
    private Vector3 pivot = Vector3.zero;
    #endregion
#region UNITY_MONOBEHAVIOUR_METHODS

    /// <summary>
    /// initialize standard position
    /// </summary>
    void Awake()
    {
        initPos = gameObject.transform.position;
    }

    /// <summary>
    /// Called as soon as enabled
    /// </summary>
    void OnEnable()
    {
        AdjustScreen();
    }

    /// <summary>
    /// Called once every frame
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
    public void enableTurning()
    {
        fixedScreen = false;
    }

    /// <summary>
    /// If function called, screen will not move further with camera position
    /// </summary>
    public void freezeScreenPosition()
    {
        fixedScreen = true;
    }
#endregion

#region PRIVATE_METHODS
    /// <summary>
    /// rotates screen and updates values
    /// </summary>
    private void AdjustScreen()
    {
        if (!fixedScreen)
        {
            //only use angle around y axis to rotate around
            float angle = VRUILogic.Instance.GetCameraRotation().eulerAngles.y + rotationOffset;
            //Vector 0 as pivot point. can be arbitary point
            gameObject.transform.position = Quaternion.Euler(0, angle, 0) * (initPos - pivot) + pivot;
            //make canvas face towards pivot point (zero vector)
            gameObject.transform.localRotation = Quaternion.Euler(0, angle, 0);
        }
    }
#endregion
}

