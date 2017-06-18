using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// ViewSelectionManager handles the transition between various view scenarios.
/// </summary>
public class ViewSelectionManager : MonoBehaviour {


    #region PUBLIC_MEMBER_VARIABLES

    /// <summary>
    /// Reference to the Canvas that is placed on the Camera plane(HMD).
    /// </summary>
    public Canvas InstructionCanvas;
    /// <summary>
    /// Reference to the image where intructive text can be displayed.
    /// </summary>
    public Image BackgroundImage;
    /// <summary>
    /// Reference to the image where the simulation feed can be displayed.
    /// </summary>
    public RawImage GazeboImage;
    /// <summary>
    /// Reference to the image where the htc feed can be displayed.
    /// </summary>
    public Image HtcImage;
    /// <summary>
    /// Reference to the image where the zed feed can be displayed.
    /// </summary>
    public RawImage ZedImage;

    #endregion PUBLIC_MEMBER_VARIABLES

    #region PRIVATE_MEMBER_VARIABLES
    #endregion PRIVATE_MEMBER_VARIABLES

    #region MONOBEHAVIOR_METHODS
    #endregion MONOBEHAVIOR_METHODS

    #region PUBLIC_METHODS

    /// <summary>
    /// Turn head tracking for BeRoboy on.
    /// </summary>
    public void TurnTrackingOn()
    {
        BeRoboyManager.Instance.TrackingEnabled = true;
    }


    /// <summary>
    /// Turn head tracking for BeRoboy off.
    /// </summary>
    public void TurnTrackingOff()
    {
        BeRoboyManager.Instance.TrackingEnabled = false;
    }
    
    /// <summary>
    /// Switches the view to the simulation view.
    /// </summary>
    public void SwitchToSimulationView()
    {
        //Turn tracking scripts On/Off
        BeRoboyManager.Instance.enabled = true;
        TurnTrackingOn();

        //Turn Canvas On/Off and enable the image that should be seen.
        InstructionCanvas.gameObject.SetActive(true);
        BackgroundImage.gameObject.SetActive(false);
        GazeboImage.gameObject.SetActive(true);
        HtcImage.gameObject.SetActive(false);
        ZedImage.gameObject.SetActive(false);
        
    }
    
    /// <summary>
    /// Switches the view to the ZED(real roboy camera in the head) view.
    /// </summary>
    public void SwitchToZEDView()
    {
        //Turn tracking scripts On/Off
        BeRoboyManager.Instance.enabled = true;
        TurnTrackingOff();

        //Turn Canvas On/Off and enable the image that should be seen.
        InstructionCanvas.gameObject.SetActive(true);
        BackgroundImage.gameObject.SetActive(false);
        GazeboImage.gameObject.SetActive(false);
        HtcImage.gameObject.SetActive(false);
        ZedImage.gameObject.SetActive(true);
        
    }
    
    /// <summary>
    /// Switches the view to the observer view.
    /// </summary>
    public void SwitchToObserverView()
    {

        //Turn tracking scripts On/Off
        BeRoboyManager.Instance.enabled = false;
        TurnTrackingOff();

        //Turn Canvas On/Off and enable the image that should be seen.
        InstructionCanvas.gameObject.SetActive(false);
        
    }
    
    /// <summary>
    /// Switches the view to the beroboy view.
    /// </summary>
    public void SwitchToBeRoboyView()
    {

        //Turn tracking scripts On/Off
        BeRoboyManager.Instance.enabled = true;
        TurnTrackingOn();

        //Turn Canvas On/Off and enable the image that should be seen.
        InstructionCanvas.gameObject.SetActive(false);

    }

    #endregion PUBLIC_METHODS

    #region PRIVATE_METHODS
    #endregion PRIVATE_METHODS

}
