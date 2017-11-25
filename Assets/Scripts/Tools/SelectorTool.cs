using System.Collections;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

/// <summary>
/// SelectorTool provides a functionality to select parts of roboy on the mesh itself or through the GUI.
/// </summary>
[RequireComponent(typeof(LineRenderer))]
public class SelectorTool : ControllerTool
{
    /// <summary>
    /// Current preview model which holds a reference to the real simulation model.
    /// </summary>
    public PreviewModel CurrentPreviewModel;
    /// <summary>
    /// LineRenderer to draw the laser for selection.
    /// </summary>
    private LineRenderer m_LineRenderer;

    /// <summary>
    /// Variable to track the last selected object for comparison.
    /// </summary>
    private SelectableObject m_LastSelectedObject;

    /// <summary>
    /// Maximum ray length for selection.
    /// </summary>
    [SerializeField]
    private float m_RayDistance = 7f;

    /// <summary>
    /// Holds reference to object where eventtrigger GetHairTriggerDown() was invoked.
    /// Reference needed to call Reverse event.
    /// </summary>
    EventTrigger m_LastHeldObject = null;

    /// <summary>
    /// Initializes the lineRenderer component.
    /// </summary>
    void Start()
    {
        m_LineRenderer = GetComponent<LineRenderer>();
    }

    /// <summary>
    /// Starts a ray from the controller. If the ray hits a roboy part, it changes its selection status. Otherwise it resets the last selected/targeted roboy part.
    /// </summary>
    public void GetRayFromController()
    {
        if (InputManager.Instance.ModelSpawn_Controller!=null &&InputManager.Instance.ModelSpawn_Controller.Operating && ModeManager.Instance.CurrentSpawnViewerMode == ModeManager.SpawnViewerMode.Insert && CurrentPreviewModel != null && m_SteamVRDevice.GetHairTriggerDown())
        {
            CurrentPreviewModel.CreateSimulationModel();
        }

        // Start a ray from the controller
        RaycastHit hit;
        m_LineRenderer.SetPosition(0, transform.position);

        // If the ray hits something...
        if (Physics.Raycast(transform.position, transform.forward, out hit, m_RayDistance))
        {
            // set the end position to the hit point
            m_LineRenderer.SetPosition(1, hit.point);
            SelectableObject hittedObject = null;

            // verify that you are in selection mode -------------CHANGE THIS IN FUTURE ONLY TEST
            if (ModeManager.Instance.CurrentGUIMode == ModeManager.GUIMode.GUIViewer && ModeManager.Instance.CurrentGUIViewerMode != ModeManager.GUIViewerMode.Selection)
                return;
            //Depending on the tag (== UI elem type), call different fcts 
            switch (hit.transform.tag)
            {
                case "RoboyUI": // if the ray hits an UI component then retrieve the roboy part from RoboyManager
                    try
                    {
                        hittedObject = RoboyManager.Instance.RoboyParts[hit.transform.name].GetComponent<SelectableObject>();
                    } catch { }
                    break;
                case "UIButton":
                    //if button can be held (not just pressed), it is registered as being held
                    //therefore, it needs an eventtrigger component attached
                    Button b_pressed = hit.collider.GetComponent<Button>();
                    EventTrigger eventsystem = b_pressed.GetComponent<EventTrigger>();

                    if (m_SteamVRDevice.GetHairTriggerDown())
                    {
                        b_pressed.onClick.Invoke();
                        if (eventsystem) //if holding supported
                        {
                            eventsystem.OnPointerDown(null);
                            m_LastHeldObject = eventsystem;
                        }
                    }
                    break;
                case "UISlider": //slider elem in BeRoboy
                    Slider slid = hit.collider.GetComponent<Slider>();

                    //If the trigger is pressed(even gently), the slider fills up, the value increases until it reaches it's maximum.
                    if (m_SteamVRDevice.GetHairTrigger())
                    {
                        if (slid.value < slid.maxValue)
                        { slid.value += 0.01f; }

                    }
                    //If the trigger is not touched at all, the slider drains down, the value decreases until it reaches it's minimum.
                    else
                    {
                        if (slid.value > slid.minValue)
                        { slid.value -= 0.01f; }
                    }
                    break;
                case "Floor":
                    if (InputManager.Instance.ModelSpawn_Controller != null && InputManager.Instance.ModelSpawn_Controller.Operating && ModeManager.Instance.CurrentSpawnViewerMode == ModeManager.SpawnViewerMode.Insert && CurrentPreviewModel != null)
                    {
                        // move the current insert model above the point where we point on the floor
                        CurrentPreviewModel.transform.position = hit.point + new Vector3(0, 0.5f, 0);
                        //CurrentPreviewModel.transform.LookAt(transform);
                    }
                    break;
                default: //not UI -> Roboy parts
                    hittedObject = hit.transform.gameObject.GetComponent<SelectableObject>();
                    break;
            }
           
            //if object found
            if (hittedObject != null)
            {
                // if we hit an object on the model layer and are currently in removing state of spawn model controller
                if (InputManager.Instance.ModelSpawn_Controller != null && hittedObject.gameObject.layer == LayerMask.NameToLayer("ModelLayer")
                    /* && InputManager.Instance.ModelSpawn_Controller.Operating */
                    && ModeManager.Instance.CurrentSpawnViewerMode == ModeManager.SpawnViewerMode.Remove)
                {
                    if (m_SteamVRDevice.GetHairTriggerDown())
                    {
                        Transform rootParent = hittedObject.transform;
                        // replace this with a handler function in selectionmanager/modelmanager
                        // get the root parent
                        while (rootParent.transform.parent != null)
                        {
                            rootParent = rootParent.transform.parent;
                        }
                        Destroy(rootParent.gameObject);
                    }
                    return;
                }
                // if the ray hits something different than last frame, then reset the last roboy part
                if (m_LastSelectedObject != hittedObject)
                {
                    if (m_LastSelectedObject != null)
                        m_LastSelectedObject.SetStateDefault();

                    // update the last roboy part as the current one
                    m_LastSelectedObject = hittedObject;
                    Vibrate();
                }
                // otherwise set the roboy part to targeted
                else
                {
                    hittedObject.SetStateTargeted();
                }
                // and select it if the user presses the trigger
                if (m_SteamVRDevice.GetHairTriggerDown())
                {
                    hittedObject.SetStateSelected();
                    //Vibrate();
                }
            }
        }
        // if the ray does not hit anything, then just reset the last roboy part and cast it until maximum distance is reached
        else
        {
            m_LineRenderer.SetPosition(1, transform.position + transform.forward * m_RayDistance);

            if (m_LastSelectedObject != null)
                m_LastSelectedObject.SetStateDefault();

            m_LastSelectedObject = null;
        }

        //for grabbing and holding updates:
        if (m_LastHeldObject) //check if object thinks it's still held (no matter if ray hit sth)
        {
            if (m_SteamVRDevice.GetHairTriggerUp()) //if trigger not held anymore
            {
                m_LastHeldObject.OnPointerUp(null);
                m_LastHeldObject = null;
            }
        }
    }

    /// <summary>
    /// deselect everything so that Roboy appears in his default state
    /// </summary>
    public override void EndTool()
    {
        if (m_LastSelectedObject)
        {
            m_LastSelectedObject.SetStateDefault(true);
            m_LastSelectedObject = null;    
        }
        if(m_LastHeldObject)
        {
            m_LastHeldObject.OnPointerUp(null);
            m_LastHeldObject = null;
        }
    }
}

