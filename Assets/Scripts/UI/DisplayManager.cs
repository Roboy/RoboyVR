using System.Collections;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Handles multiple displays.
/// Upon start, 2 displays are activated if 2 cameras and 2 monitors are found. 
/// Additional information is shortly displayed on the main screen.
/// </summary>
public class DisplayManager : MonoBehaviour
{
    public Camera cam2; /*This camera will be used for the second display*/
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
        if (Display.displays.Length > 1 && (cam2!=null)) { 
            //TODO: windowed mode supposed to work under windows... might blow up under Linux / OSX !!
            Display.displays[1].Activate(400, 400, 60);
            cam2.GetComponent<Camera>().targetDisplay = 1;
        }
    }

    /// <summary>
    /// Create Text on Canvas to be displayed in centre of screen.
    /// </summary>
    /// <returns>Canvas containing text object</returns>
    Canvas CreateInfoScreen()
    {
        GameObject g = new GameObject();
        g.transform.parent = transform;
        Canvas c = g.AddComponent<Canvas>();
        c.targetDisplay =0;
        c.transform.position = Vector3.zero;
        c.renderMode = RenderMode.ScreenSpaceOverlay;
        CanvasScaler cs = g.AddComponent<CanvasScaler>();
        cs.uiScaleMode = CanvasScaler.ScaleMode.ConstantPixelSize;
        cs.scaleFactor = 1;
        cs.referencePixelsPerUnit = 100;
        GraphicRaycaster rc = g.AddComponent<GraphicRaycaster>();
        rc.ignoreReversedGraphics = true;
        g.transform.localScale = new Vector3(1, 1, 1);
        g.transform.localPosition = Vector3.zero;
        GameObject g2 = new GameObject();
        Text t = g2.AddComponent<Text>(); /*Text message displaying information about screens*/
        g2.transform.SetParent(g.transform, false);
        t.horizontalOverflow = HorizontalWrapMode.Overflow;
        t.verticalOverflow = VerticalWrapMode.Overflow;
        t.alignment = TextAnchor.MiddleLeft;
        Font f = (Font) Resources.GetBuiltinResource(typeof(Font), "Arial.ttf");
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
    IEnumerator DisplayFor( Canvas c, float sec)
    {
        if (c == null) yield return null;
        c.enabled = true;
        yield return new WaitForSeconds(sec);
        foreach (Component comp in c.GetComponentsInChildren<Component>())
        {
            Destroy(comp.gameObject);            
        }
    }
}