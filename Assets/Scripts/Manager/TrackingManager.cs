using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// Change name to ViewSelectionManager or so
public class TrackingManager : MonoBehaviour {

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
        TurnTrackingOn();
    }
    
    /// <summary>
    /// Switches the view to the ZED(real roboy camera in the head) view.
    /// </summary>
    public void SwitchToZEDView()
    {
        TurnTrackingOff();
    }
    
    /// <summary>
    /// Switches the view to the observer view.
    /// </summary>
    public void SwitchToObserverView()
    {
        TurnTrackingOff();
    }
    
    /// <summary>
    /// Switches the view to the beroboy view.
    /// </summary>
    public void SwitchToBeRoboyView()
    {
        TurnTrackingOn();
    }
    
    
}
