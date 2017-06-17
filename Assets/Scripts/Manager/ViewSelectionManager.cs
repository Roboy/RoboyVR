using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

// Change name to ViewSelectionManager or so
public class ViewSelectionManager : MonoBehaviour {

    public Canvas InstructionCanvas;
    public Image BackgroundImage;
    public RawImage GazeboImage;
    public Image HtcImage;
    public RawImage ZedImage;


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
    
    
}
