using System.Collections;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Handles multiple displays.
/// Upon start, 2 displays are activated if 2 cameras and 2 monitors are found. 
/// Additional information is shortly displayed on the main screen.
/// Must be attached to a canvas directly
/// </summary>
public class DisplayManager : MonoBehaviour
{
    #region PUBLIC_MEMBER:_VARIABLES
    /// <summary>
    /// This camera will be used for the second display
    /// </summary>
    public Camera cam2; 
    #endregion

    #region UNITY_MONOBEHAVIOUR_METHODS
    /// <summary>
    /// Activates 2 Displays if multiple detected. Displays short info about display number on Main screen.
    /// </summary>
    void Start()
    {
        Canvas c = CreateInfoScreen();
        Text t = c.GetComponentInChildren<Text>();
        Debug.Log(Display.displays.Length + " Displays found... activating");
        t.text = Display.displays.Length + " Displays found... activating";
        StartCoroutine(DisplayFor(c, 1.5f));
        // Display.displays[0] is the primary, default display and is always ON.
        // Check if additional displays are available and activate each.
        if (Display.displays.Length > 1 && (cam2 != null))
        {
            //should be fullscreen mode for second screen....?
            Display.displays[1].Activate();
            if(cam2 != null)
            {
                cam2.GetComponent<Camera>().targetDisplay = 1;
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
        GameObject g = new GameObject();
        g.transform.parent = transform;
        Canvas c = g.AddComponent<Canvas>();
        c.targetDisplay = 0;
        c.renderMode = RenderMode.ScreenSpaceOverlay;
        CanvasScaler cs = g.AddComponent<CanvasScaler>();
        cs.uiScaleMode = CanvasScaler.ScaleMode.ConstantPixelSize;
        cs.scaleFactor = 1;
        cs.referencePixelsPerUnit = 1000;
        GraphicRaycaster rc = g.AddComponent<GraphicRaycaster>();
        rc.ignoreReversedGraphics = true;
        g.transform.localScale = new Vector3(1, 1, 1);
        g.transform.localPosition = Vector3.zero;
        GameObject g2 = new GameObject();

        Text t = g2.AddComponent<Text>(); /*Text message displaying information about screens*/
        g2.transform.SetParent(g.transform, false);
        t.rectTransform.localPosition = Vector3.zero;
        t.horizontalOverflow = HorizontalWrapMode.Overflow;
        t.verticalOverflow = VerticalWrapMode.Overflow;
        t.alignment = TextAnchor.MiddleCenter;
        Font f = (Font)Resources.GetBuiltinResource(typeof(Font), "Arial.ttf");
        t.font = f;
        t.fontSize = 30;
        t.color = Color.black;
        t.enabled = true;
        return c;
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
        Destroy(this);
    }
    #endregion
}