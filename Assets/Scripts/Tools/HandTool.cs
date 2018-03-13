using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROSBridgeLib;

/// <summary>
/// Tool to push Roboy around and apply forces via direct contact.
/// </summary>
public class HandTool : ControllerTool
{
    #region PUBLIC_MEMBER_VARIABLES
    #endregion

    #region PRIVATE_MEMBER_VARIABLES

    /// <summary>
    /// object containing the visual form of a right hand
    /// </summary>
    [SerializeField]
    private Mesh m_RightHandMesh;

    /// <summary>
    /// object containing the visual form of a left hand
    /// </summary>
    [SerializeField]
    private Mesh m_LeftHandMesh;

    /// <summary>
    /// Factor is multiplied with the force that is created when pulling/ pushing Roboy
    /// </summary>
    [SerializeField]
    private float m_PullForceFactor = 100;

    /// <summary>
    /// Specifies max length of ray which is used to determine which bodypart is pointed at / should be selected
    /// </summary>
    [SerializeField]
    private float m_RayDistance = 0.3f;
    /// <summary>
    /// Specifies max length of ray which is used to determine which bodypart is pointed at / should be selected
    /// </summary>
    [SerializeField]
    private Vector3 POS;
    /// <summary>
    /// Specifies max length of ray which is used to determine which bodypart is pointed at / should be selected
    /// </summary>
    [SerializeField]
    private Vector3 DIR;


    /// <summary>
    /// TODO: is it used for anything?
    /// </summary>
    private MeshFilter m_MeshFilter;

    /// <summary>
    /// describing if hand model of left or right hand used
    /// </summary>
    private bool m_IsLeft = false;

    /// <summary>
    /// Variable to track the last highlighted object for comparison.
    /// </summary>
    private SelectableObject m_HighlightedObject;

    /// <summary>
    /// specifies, whether highlighted obj is selected(true) or just highlighted (false)
    /// </summary>
    private bool m_ObjectIsSelected = false;

    #region spring-related

    /// <summary>
    /// To calculate spring forces, the initial length needs to be known
    /// </summary>
    private float m_InitialLength;

    /// <summary>
    /// gameobj holding position reference to the point where Roboy is grabbed. 
    /// It moves along with Roboy since its parent is the respective Roboy part
    /// </summary>
    private GameObject m_RoboyPoint;

    /// <summary>
    /// For Debug purposes and to draw the force lines
    /// </summary>
    private LineRenderer m_Line;
    #endregion
    #endregion

    #region UNITY_MONOBEHAVIOUR

    /// <summary>
    /// Get which model to use for hand: right or left.
    /// </summary>
    private IEnumerator Start()
    {
        m_MeshFilter = GetComponent<MeshFilter>();

        while (!m_Initialized)
            yield return null;

        int rightIndex = SteamVR_Controller.GetDeviceIndex(SteamVR_Controller.DeviceRelation.Rightmost);
        int leftIndex = SteamVR_Controller.GetDeviceIndex(SteamVR_Controller.DeviceRelation.Leftmost);

        SteamVR_Controller.Device rightDevice = SteamVR_Controller.Input(rightIndex);
        SteamVR_Controller.Device leftDevice = SteamVR_Controller.Input(leftIndex);

        if (rightDevice == m_SteamVRDevice)
        {
            m_MeshFilter.mesh = m_RightHandMesh;
            m_IsLeft = false;
        }
        else if (leftDevice == m_SteamVRDevice)
        {
            m_MeshFilter.mesh = m_LeftHandMesh;
            m_IsLeft = true;
        }

    }

    /// <summary>
    /// Called once every frame, publishes hand position to gazebo
    /// </summary>
    private void FixedUpdate()
    {
        string linkName = (m_IsLeft) ? "left_hand" : "right_hand";
        List<string> linkNames = new List<string>();
        linkNames.Add(linkName);

        ROSBridgeLib.custom_msgs.RoboyPoseMsg msg = new ROSBridgeLib.custom_msgs.RoboyPoseMsg("hands",
        linkNames, new Vector3[] { transform.position }, new Quaternion[] { transform.rotation });
        ROSBridge.Instance.Publish(RoboyHandsPublisher.GetMessageTopic(), msg);
    }
    #endregion

    #region PUBLIC_METHODS
    /// <summary>
    /// Starts a ray from the controller. If the ray hits a roboy part, it changes its selection status. 
    /// Roboy parts are highlighted when pointing at them, and selected if the grab butten is held. 
    /// As soon as the button is released, the part is no longer selected (but might still be highlighed) //TODO: Test to be sure
    /// Function adapted from SelectorTool.cs
    /// </summary>
    public void CheckUserGrabbingRoboy()
    {
        // Start a ray from the controller
        RaycastHit hit;

        // If the ray hits something...
        if (Physics.Raycast(transform.position, transform.forward, out hit, m_RayDistance))
        {
            // set the end position to the hit point
            SelectableObject hittedObject = null;

            // verify that you are in selection mode -------------CHANGE THIS IN FUTURE ONLY TEST
            if (ModeManager.Instance.CurrentGUIMode == ModeManager.GUIMode.GUIViewer && ModeManager.Instance.CurrentGUIViewerMode != ModeManager.GUIViewerMode.Selection)
                return;
            //Depending on the tag (== UI elem type), call different fcts 
            switch (hit.transform.tag)
            {
                //for now: if no roboy part, then ignore
                case "RoboyUI":
                case "UIButton":
                case "UISlider":
                case "Floor":
                    break;
                default: //not UI -> Roboy parts 
                    //TODO: be careful, when new tags introduced and used in scene, this might blow up
                    hittedObject = hit.transform.gameObject.GetComponent<SelectableObject>();
                    break;
            }

            //if object found
            if (hittedObject != null)
            {
                //Ignore ModelSpawn- (see modemanager.SpawnViewerMode) stuff since this tool doesn't spawn/remove but change position
                // if the ray hits something different than last frame, then reset the last roboy part
                if (m_HighlightedObject != hittedObject)
                {
                    if (m_HighlightedObject != null)
                        m_HighlightedObject.SetStateDefault(); //no longer highlighted
                    // update the last roboy part as the current one
                    //ignore if we're already grabbing another roboy part 
                    if (!m_ObjectIsSelected)
                    {
                        m_HighlightedObject = hittedObject;

                    }
                }
                // otherwise set the roboy part to targeted
                else
                {
                    hittedObject.SetStateTargeted(); //only set to target if not selected/targeted already
                }
                // and select it if the user presses the trigger
                if (m_SteamVRDevice != null && m_SteamVRDevice.GetHairTriggerDown())
                {
                    hittedObject.SetStateSelected();
                    m_ObjectIsSelected = true;
                    Vibrate();
                    //TODO: if repositioning of arm desired, best do it here

                    //set point where Roboy was grabbed to reference for force calculation
                    m_RoboyPoint = new GameObject("GrabbedPoint");
                    m_RoboyPoint.transform.SetParent(hittedObject.transform);
                    m_RoboyPoint.transform.position = hit.point;
                    GameObject obj = Instantiate(Resources.Load("UI/Warning")) as GameObject;
                    obj.name = "Point";
                    obj.transform.parent = m_RoboyPoint.transform;
                    obj.transform.localPosition = Vector3.zero;

                    //initial spring length depending on hand and roboy part position (order of coords doesn'T matter)
                    m_InitialLength = (m_RoboyPoint.transform.position - transform.position).magnitude;
                }
            }
        }
        // if the ray does not hit anything, then just reset the last selected roboy part
        else
        {
            if (m_HighlightedObject != null && !m_ObjectIsSelected) //if element only was highlighted
            {
                m_HighlightedObject.SetStateDefault();
                m_HighlightedObject = null;
            }
        }
        //check if object thinks it's still held (no matter of ray hit sth)
        if (m_ObjectIsSelected && m_HighlightedObject)
        {
            if (m_SteamVRDevice.GetHairTriggerUp()) //only release obj if trigger is not held anymore
            {
                m_HighlightedObject.SetStateDefault(true); //force it to go back to previous state
                m_HighlightedObject = null;
                Destroy(m_RoboyPoint);
                m_RoboyPoint = null;
                m_ObjectIsSelected = false;
            }
        }
        //finally, handle user movement
        EvaluateHandPosition();
    }

    /// <summary>
    /// deselect everything so that Roboy appears in his default state
    /// </summary>
    public override void EndTool()
    {
        if (m_ObjectIsSelected)
        {
            m_ObjectIsSelected = false;
            m_HighlightedObject.SetStateDefault(true);
            m_HighlightedObject = null;
        }
    }
    #endregion

    #region PRUVATE_METHODS

    /// <summary>
    /// This method evaluates current position of the hand and computes forces acted on roboy by grabbing and moving. 
    /// Springs are used for this
    /// sends the info to gazebo
    /// </summary>
    private void EvaluateHandPosition()
    {
        if (m_RoboyPoint && m_ObjectIsSelected) //if we're currently grabbing sth and we have the means to calculate forces
        {
            //TODO: damping ? is it going to wiggle the whole time when in base position
            //TODO: check if transformations  correct / in right space & direction
            RoboyPart roboyPart;
            if ((roboyPart = m_HighlightedObject.gameObject.GetComponent<RoboyPart>()) != null)
            {
                // delta world direction * forces 
                Vector3 directionWorldSpace = (transform.position - m_RoboyPoint.transform.position) * m_PullForceFactor;
                //forces in local space of part that is affected
                Vector3 directionLocalSpace = GazeboUtility.WorldToLocalSpaceDirection(roboyPart.transform, directionWorldSpace);
                // the position in local space
                Vector3 forcePositionLocalSpace = GazeboUtility.WorldToLocalSpaceDirection(roboyPart.transform, m_RoboyPoint.transform.position);
                //force position in global space
                Vector3 forcePositionWorldSpace = m_RoboyPoint.transform.position;
                // DEBUG PURPOSES: draws a line between the points
                if (m_Line)
                {
                    //world space
                    //m_Line.SetPosition(0, forcePositionWorldSpace);
                    //m_Line.SetPosition(1, forcePositionWorldSpace + directionWorldSpace);
                    //local space
                    m_Line.SetPosition(0, forcePositionLocalSpace);
                    m_Line.SetPosition(1, forcePositionLocalSpace + directionLocalSpace);
                }
                else
                {
                    GameObject line = new GameObject();
                    m_Line = line.AddComponent<LineRenderer>();
                    m_Line.SetPosition(0, forcePositionLocalSpace);
                    m_Line.SetPosition(1, forcePositionLocalSpace + directionLocalSpace);
                    m_Line.startWidth = 0.001f;
                    m_Line.endWidth = 0.001f;
                }

                int duration = (int)(Time.smoothDeltaTime * 1000);  // time period during which force should be valid,, in milliseconds
                //send force message TODO: for now: WOlrd space even though supposed to be local space (link space) 
                //RoboyManager.Instance.SendExternalForce(roboyPart,forcePositionWorldSpace, directionWorldSpace, duration);
                RoboyManager.Instance.SendExternalForce(roboyPart, forcePositionLocalSpace, directionLocalSpace, duration);
            }
        }
    }

    #endregion
}