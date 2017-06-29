using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using UnityEngine.UI;

/// <summary>
/// Object to create a selection wheel on a canvas. You need to place all objects which should be selectable as children.
/// These must have the <see cref="SelectionWheelPartUI"/> script attached. But you can create your own selectable parts
/// if your script derives from <see cref="SelectionWheelPartUI"/>. 
/// </summary>
public class SelectionWheel : MonoBehaviour {

    /// <summary>
    /// Width of the UI parts.
    /// </summary>
    [SerializeField]
    private float Width = 800;

    /// <summary>
    /// Height of the UI parts.
    /// </summary>
    [SerializeField]
    private float Height = 800;

    /// <summary>
    /// Default color for the parts.
    /// </summary>
    [SerializeField]
    private Color DefaultColor = Color.white;

    /// <summary>
    /// Highlight color for the parts.
    /// </summary>
    [SerializeField]
    private Color HighlightColor = Color.white;

    /// <summary>
    /// Selection color for the parts.
    /// </summary>
    [SerializeField]
    private Color SelectColor = Color.white;

    /// <summary>
    /// Default color of the outline of the parts.
    /// </summary>
    [SerializeField]
    private Color DefaultOutlineColor = Color.white;

    /// <summary>
    /// Selected color of the outline color.
    /// </summary>
    [SerializeField]
    private Color SelectedOutlineColor = Color.white;

    /// <summary>
    /// The controller which should should control the selection wheel.
    /// </summary>
    [SerializeField]
    private SteamVR_TrackedObject m_TrackedController;

    /// <summary>
    /// Actual component so we can check the current input of the controller.
    /// </summary>
    private SteamVR_Controller.Device m_Controller { get { return SteamVR_Controller.Input((int)m_TrackedController.index); } }

    /// <summary>
    /// Component so we can listen to events of the controller to make our life easier. Attach this component to the controller
    /// so it does work.
    /// </summary>
    private SteamVR_TrackedController m_ControllerEventListener;

    /// <summary>
    /// The sprite you want the parts to have.
    /// </summary>
    [SerializeField]
    private Sprite m_Sprite;

    /// <summary>
    /// Offset between each part.
    /// </summary>
    [SerializeField]
    private float m_Offset = 0.001f;

    /// <summary>
    /// All selectable parts.
    /// </summary>
    private List<SelectionWheelPartUI> m_Parts = new List<SelectionWheelPartUI>();

    /// <summary>
    /// Current selected part.
    /// </summary>
    private SelectionWheelPartUI m_SelectedPart;

    /// <summary>
    /// Is the wheel currently looking for touchpad input?
    /// </summary>
    private bool m_Recognizing = false;

    /// <summary>
    /// Initalize the controller event listener and parts.
    /// </summary>
    private void Awake()
    {
        m_ControllerEventListener = m_TrackedController.gameObject.GetComponent<SteamVR_TrackedController>();
    }

    /// <summary>
    /// Adjust the image component of the parts and subscribe to the controller events.
    /// </summary>
    public void Initialize(List<SelectionWheelPart> parts)
    {
        // add an image component to each part and adjust the settings
        for (int i = 0; i < parts.Count; i++)
        {
            // create a gameObject for each part as a child of this one
            GameObject wheelPartGO = new GameObject("SelectionWheelPart " + i);
            wheelPartGO.transform.parent = transform;
            wheelPartGO.transform.localScale = Vector3.one;
            wheelPartGO.transform.localPosition = Vector3.zero;
            // add the UI part
            SelectionWheelPartUI wheelPartUI = wheelPartGO.AddComponent<SelectionWheelPartUI>();
            wheelPartUI.Initialize(parts[i]);
            // set the image component
            Image img = wheelPartGO.GetComponent<Image>();
            img.rectTransform.sizeDelta = new Vector2(Width, Height);
            img.sprite = m_Sprite;
            img.type = Image.Type.Filled;
            img.fillMethod = Image.FillMethod.Radial360;
            img.fillAmount = 1.0f / parts.Count - m_Offset;
            img.rectTransform.localEulerAngles = new Vector3(0, 0, i * 360 / parts.Count);
            // set the colors
            wheelPartUI.DefaultColor = DefaultColor;
            wheelPartUI.HighlightColor = HighlightColor;
            wheelPartUI.SelectedColor = SelectColor;
            wheelPartUI.DefaultOutlineColor = DefaultOutlineColor;
            wheelPartUI.SelectedOutlineColor = SelectedOutlineColor;

            m_Parts.Add(wheelPartUI);
            wheelPartUI.Unhighlight();
        }
        // subcribe to the events
        m_ControllerEventListener.PadTouched += startRecognition;
        m_ControllerEventListener.PadUntouched += stopRecognition;
        m_ControllerEventListener.PadClicked += selectPart;
    }

    /// <summary>
    /// Starts the recognition.
    /// </summary>
    /// <param name="sender"></param>
    /// <param name="e"></param>
    private void startRecognition(object sender, ClickedEventArgs e)
    {
        StartCoroutine(recognitionCoroutine());
    }

    /// <summary>
    /// Stops the recognition and unhighlights all parts.
    /// </summary>
    /// <param name="sender"></param>
    /// <param name="e"></param>
    private void stopRecognition(object sender, ClickedEventArgs e)
    {
        m_Recognizing = false;

        for (int i = 0; i < m_Parts.Count; i++)
        {
            m_Parts[i].Unhighlight();
        }
    }

    /// <summary>
    /// Updates the selected part.
    /// </summary>
    /// <param name="sender"></param>
    /// <param name="e"></param>
    private void selectPart(object sender, ClickedEventArgs e)
    {
        int buttonID = getButtonID();
        if (buttonID == -1)
            return;

        if (m_SelectedPart)
            m_SelectedPart.Deselect();

        m_Parts[buttonID].Select();
        m_SelectedPart = m_Parts[buttonID];
    }

    /// <summary>
    /// Deselects a part. Use this function to deselect the last selected part.
    /// </summary>
    /// <param name="sender"></param>
    /// <param name="e"></param>
    private void deselectPart(object sender, ClickedEventArgs e)
    {
        int buttonID = getButtonID();
        if (buttonID == -1)
            return;
        m_Parts[buttonID].Deselect();
    }

    /// <summary>
    /// Coroutine to check the input of the controller and highlight the corresponding part. Stops if
    /// <see cref="m_Recognizing"/> is false. 
    /// </summary>
    /// <returns></returns>
    private IEnumerator recognitionCoroutine()
    {
        m_Recognizing = true;

        while (m_Recognizing)
        {
            int buttonID = getButtonID();
            if (buttonID == -1)
            {
                yield return null;
                continue;
            }
            
            m_Parts[buttonID].Highlight();

            for (int i = 0; i < m_Parts.Count; i++)
            {
                if (i == buttonID)
                    continue;
                m_Parts[i].Unhighlight();
            }
            yield return null;
        }
    }

    /// <summary>
    /// Get the button id based on the touchpad input. Returns -1 if the resulting ID is not in the parts list.
    /// </summary>
    /// <returns></returns>
    private int getButtonID()
    {
        Vector2 touchpadPosition = m_Controller.GetAxis();

        float touchAngle = MathUtility.VectorToAngle(touchpadPosition);
        float buttonAngleStep = 360 / m_Parts.Count;

        int buttonID =  (int)((MathUtility.WrapAngle(touchAngle + 180f)) / buttonAngleStep);
        if (buttonID < 0 || buttonID >= m_Parts.Count)
            return -1;
        return buttonID;
    }
}
