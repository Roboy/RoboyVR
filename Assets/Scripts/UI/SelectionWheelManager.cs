using System.Collections;
using UnityEngine;
using UnityEngine.UI;

public class SelectionWheelManager : MonoBehaviour
{
    private bool visible = false; /* Is selection wheel visible */
    private bool stillVisible = false; /* before disabling wheel:was it moved again? Should it still be visible*/
    private double curSpin = 0; /* rotational speed, changed and updated over time*/
    private float curAngle = 0;/* current angle around z axis (local)*/
    private int elems; /* number of text elems in selection wheel*/
    private int selected = -1; /* which text elem is selected*/
    public float speed; /* factor to multiply curSpin with, allows manual settings*/
    public float threshold; /* Spin below that point set 0*/
    public int selectIndex = 1; /* Specify where selected item should be positioned (clockwise, number of elem on circle)*/
    public Canvas canvas; /* canvas with selection wheel, en- and disabled depending on wheel */
    
    #region helpers
    /// <summary>
    /// x mod y in a function, as normal % just returns remainder and does not work for negative values.
    /// </summary>
    /// <param name="x">first operand</param>
    /// <param name="y">second operand</param>
    /// <returns></returns>
    int Mod(int x, int y)
    {
        return (x % y + y) % y;
    }

    /// <summary>
    /// Enables canvas containing wheel to be displayed
    /// </summary>
    void EnableCanvas()
    {
        if (canvas) { 
            canvas.GetComponent<Canvas>().enabled = true;
            visible = true;
        }
    }
    /// <summary>
    /// Coroutine to wait shortly (0.5s) and disable canvas if user did not give new input meanwhile.
    /// </summary>
    /// <returns></returns>
    IEnumerator DisableCanvas()
    {
        yield return new WaitForSeconds(0.5f);
        if (!stillVisible)
        {
            if (canvas)
            {
                canvas.GetComponent<Canvas>().enabled = false;
                visible = false;
            }
        }
    }

    /// <summary>
    /// Detects the currently selected item fom the selection wheel using the current angle and highlights it.
    /// </summary>
    void HighlightSelection()
    {
        int step = (360 / elems);
        int tmp = elems - (int)(curAngle) / step - 1;// map selection from 0 - (elems -1)
        tmp = Mod((tmp - selectIndex), elems); // select the desired item clockwise
        if (tmp == selected)
        {
            return;
        }
        selected = tmp;
        Debug.Log("selection: " + selected);
        Text[] texts = GetComponentsInChildren<Text>();
        for (int i = 0; i < texts.Length; i++)
        {
            if (selected != i)
            {
                texts[i].fontStyle = FontStyle.Normal;
                texts[i].color = Color.gray;

            }
            else
            {
                texts[i].fontStyle = FontStyle.Bold;
                texts[i].color = Color.black;
            }
        }
    }
    #endregion

    /// <summary>
    /// Initialize Wheel: Place each child (text) elem evenly on wheel. 
    /// </summary>
    void Start()
    {
        Vector2 radius;
        RectTransform t = GetComponent<RectTransform>();
        radius.x = t.rect.x * t.localScale.y; //assume square as base
        radius.y = 0;
        Component[] children = gameObject.GetComponentsInChildren(typeof(RectTransform));
        elems = children.Length - 1; //ignore this elem
        if (selectIndex > elems) selectIndex = selectIndex % elems; //stay within boundaries, not necessarily needed though
        for (int i = 0; i < children.Length; i++)
        {
            Transform transform = (Transform)children[i];
            if (transform.gameObject != gameObject) //ignore this elem
            {
                Debug.Log(children[i].name);
                // define point on outer circle called offset,start from 0 as 12 o'clock on the circle
                Vector2 offset = Quaternion.Euler(0, 0, 360 / elems * (i - 1)) * radius; 
                Debug.Log(offset);
                //move elem to that position on circle
                transform.Translate(offset.x, offset.y, 0);
            }
        }
        //counter turn elem to keep text straight
        if (canvas)
        {
            canvas.GetComponent<Canvas>().enabled = false;
            visible = false;
        }
        HighlightSelection();
    }

    /// <summary>
    /// Update once per frame
    /// </summary>
    void Update()
    {

        // update the current spin with the mouse wheel
        double friction;
        Component[] children;
        float value = Input.GetAxis("Mouse ScrollWheel");
        curSpin += value * speed * Time.deltaTime;
        if (curSpin > -threshold && curSpin < threshold && value == 0)
        {
            curSpin = 0;
            stillVisible = false;
            StartCoroutine(DisableCanvas());
        }
        Debug.Log(value);
        if (value != 0)
        {
            stillVisible = true;
            if (!visible) EnableCanvas();
        }
        curAngle = transform.rotation.eulerAngles.z;
        friction = curSpin * Time.deltaTime;
        curSpin -= (float)friction;
        // finally, do the actual rotation
        transform.Rotate(Vector3.back * (float)curSpin, Space.World);
        //counter rotate children
        children = gameObject.GetComponentsInChildren(typeof(RectTransform));
        for (int i = 0; i < children.Length; i++)
        {
            Transform transform = (Transform)children[i];
            if (transform.gameObject != gameObject)
            {
                transform.Rotate(Vector3.back * (float)(-curSpin), Space.Self);
            }
        }
        HighlightSelection();
    }
}

