using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

// Change name to ViewSelectionManager or so
public class ViewSelectionManager : MonoBehaviour {

    public Canvas InstructionCanvas;
    public Image BackgroundImage;
    public Image GazeboImage;
    public Image HtcImage;
    public Image ZedImage;


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
        InstructionCanvas.enabled = true;
        BackgroundImage.enabled = false;
        GazeboImage.enabled = true;
        HtcImage.enabled = false;
        ZedImage.enabled = false;
        


    }
    
    /// <summary>
    /// Switches the view to the ZED(real roboy camera in the head) view.
    /// </summary>
    public void SwitchToZEDView()
    {
        BeRoboyManager.Instance.enabled = true;
        TurnTrackingOff();
        InstructionCanvas.enabled = true;
        BackgroundImage.enabled = false;
        GazeboImage.enabled = false;
        HtcImage.enabled = false;
        ZedImage.enabled = true;

    }
    
    /// <summary>
    /// Switches the view to the observer view.
    /// </summary>
    public void SwitchToObserverView()
    {
        BeRoboyManager.Instance.enabled = false;
        TurnTrackingOff();
        InstructionCanvas.enabled = false;

    }
    
    /// <summary>
    /// Switches the view to the beroboy view.
    /// </summary>
    public void SwitchToBeRoboyView()
    {
        BeRoboyManager.Instance.enabled = true;
        TurnTrackingOn();
        InstructionCanvas.enabled = false;
    }
    
    
}
