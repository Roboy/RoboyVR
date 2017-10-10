using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// ControllerTool is a base class for all tools which are attached to a controller. It provides access to steamVR functions to track the input of the controllers.
/// On top of that it provides a function to vibrate the controller for a defined time.
/// </summary>
public class ControllerTool : MonoBehaviour {

    public SteamVR_TrackedObject ControllerObject { get { return m_SteamVRController; } }

    /// <summary>
    /// Returns the controller identity for verification purposes for outside classes.
    /// </summary>
    public SteamVR_Controller.Device Controller { get { return SteamVR_Controller.Input((int)m_SteamVRController.index); } }
    
    /// <summary>
    /// Returns a component which listens to controller events like OnTouchpad.
    /// </summary>
    public SteamVR_TrackedController ControllerEventListener { get { return m_SteamVRTrackedController; } }

    /// <summary>
    /// Protected controller for input tracking in classes deriving from this class like SelectorTool or ShootingTool.
    /// </summary>
    protected SteamVR_Controller.Device m_SteamVRDevice;

    /// <summary>
    /// Returns the controller identity for verification purposes for deriving classes.
    /// </summary>
    protected SteamVR_TrackedObject m_SteamVRController;
    
    /// <summary>
    /// Protected controller for event tracking in classes deriving from this class like SelectorTool or ShootingTool.
    /// </summary>
    protected SteamVR_TrackedController m_SteamVRTrackedController;

    /// <summary>
    /// As we use a Coroutine to initialize we need this so we can check whether all steam vr controller scripts are found and initialized.
    /// </summary>
    protected bool m_Initialized = false;

    /// <summary>
    /// Calls initialize for all controller members.
    /// </summary>
    void Awake()
    {
        Initialize();
    }

    /// <summary>
    /// Starts a coroutine to vibrate the controller for a fixed time.
    /// </summary>
    public void Vibrate()
    {
        StartCoroutine(vibrateController());
    }

    /// <summary>
    /// Initiliazes the controller in a coroutine. Intermediate function for outside classes.
    /// </summary>
    public void Initialize()
    {
        StartCoroutine(initializeCoroutine());
    }

    /// <summary>
    /// Coroutine to vibrate the controller for a fixed time.
    /// </summary>
    /// <returns></returns>
    private IEnumerator vibrateController()
    {
        float duration = 0.25f;
        float currDuration = 0f;
        float vibrationStrength = 250f;

        while (currDuration < duration)
        {
            float sinValue = Mathf.Sin(currDuration / duration * Mathf.PI) * vibrationStrength;
            SteamVR_Controller.Input((int)m_SteamVRController.index).TriggerHapticPulse((ushort)sinValue);
            currDuration += Time.fixedDeltaTime;
            yield return Time.fixedDeltaTime;
        }
    }

    /// <summary>
    /// Coroutine to initialize all controller members.
    /// </summary>
    /// <returns></returns>
    private IEnumerator initializeCoroutine()
    {
        m_SteamVRController = GetComponentInParent<SteamVR_TrackedObject>();

        while (!m_SteamVRController.isValid)
            yield return null;

        m_SteamVRDevice = SteamVR_Controller.Input((int)m_SteamVRController.index);
        m_SteamVRTrackedController = GetComponentInParent<SteamVR_TrackedController>();
        m_Initialized = true;
    }
}
