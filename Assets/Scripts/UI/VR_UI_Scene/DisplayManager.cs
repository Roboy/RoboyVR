using System.Collections;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Handles multiple displays.
/// Upon start, 2 displays are activated if 2 cameras and 2 monitors are found. 
/// Additional information is shortly displayed on the main screen.
/// Must be attached to a canvas directly to display the information
/// </summary>
public class DisplayManager : MonoBehaviour
{
    #region PUBLIC_MEMBER_VARIABLES
    /// <summary>
    /// This camera will be used for the second display
    /// </summary>
    public Camera Camera2;
    #endregion

    #region PRIVATE_VARIABLES
    /// <summary>
    /// object containing the displayed information
    /// </summary>
    GameObject m_Container;
    #endregion

    #region UNITY_MONOBEHAVIOUR_METHODS
    /// <summary>
    /// Activates 2 Displays if multiple detected. Displays short info about display number on Main screen.
    /// </summary>
    void Start()
    {

        Canvas canvas = CreateInfoScreen();
        m_Container = canvas.gameObject;
        Text t = canvas.GetComponentInChildren<Text>();
        Debug.Log(Display.displays.Length + " Displays found... activating");
        t.text = Display.displays.Length + " Displays found... activating";
        StartCoroutine(DisplayFor(canvas, 1.5f));
        // Display.displays[0] is the primary, default display and is always ON.
        // Check if additional displays are available and activate each.
        if (Display.displays.Length > 1 && (Camera2 != null))
        {
            //should be fullscreen mode for second screen....?
            Display.displays[1].Activate();
            if (Camera2 != null)
            {
                Camera2.GetComponent<Camera>().targetDisplay = 1;
            }
        }
    }
    #endregion
    #region PRIVATE_METHODS
    /// <summary>
    /// Create Text on Canvas to be displayed in centre of screen.
    /// </summary>
    /// <returns>Canvas containing text object</returns>
    private Canvas CreateInfoScreen()
    {
        GameObject gObject = new GameObject();
        gObject.transform.parent = transform;
        Canvas canvas = gObject.AddComponent<Canvas>();
        canvas.targetDisplay = 0;
        canvas.renderMode = RenderMode.ScreenSpaceOverlay;
        CanvasScaler canvasScaler = gObject.AddComponent<CanvasScaler>();
        canvasScaler.uiScaleMode = CanvasScaler.ScaleMode.ConstantPixelSize;
        canvasScaler.scaleFactor = 1;
        canvasScaler.referencePixelsPerUnit = 1000;
        GraphicRaycaster rc = gObject.AddComponent<GraphicRaycaster>();
        rc.ignoreReversedGraphics = true;
        gObject.transform.localScale = new Vector3(1, 1, 1);
        gObject.transform.localPosition = Vector3.zero;
        GameObject g2 = new GameObject();

        Text t = g2.AddComponent<Text>(); /*Text message displaying information about screens*/
        g2.transform.SetParent(gObject.transform, false);
        t.rectTransform.localPosition = Vector3.zero;
        t.horizontalOverflow = HorizontalWrapMode.Overflow;
        t.verticalOverflow = VerticalWrapMode.Overflow;
        t.alignment = TextAnchor.MiddleCenter;
        Font f = (Font)Resources.GetBuiltinResource(typeof(Font), "Arial.ttf");
        t.font = f;
        t.fontSize = 30;
        t.color = Color.black;
        t.enabled = true;
        return canvas;
    }

    /// <summary>
    /// Displays specified canvas before it is destroyed.
    /// </summary>
    /// <param name="c">Canvas to be displayed</param>
    /// <param name="sec">Time to display canvas</param>
    /// <returns></returns>
    private IEnumerator DisplayFor(Canvas c, float sec)
    {
        if (c == null) yield return null;
        c.enabled = true;
        yield return new WaitForSeconds(sec);
        Destroy(m_Container);
        Destroy(this);
    }
    #endregion
}