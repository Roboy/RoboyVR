using System.Collections;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Implements a selectionwheel using the child objects as options.
/// This class is attached to a gameobject part of a canvas. It displays a selection wheel, checks for user input and updates the current selectioni saved in GameManager.
/// </summary>
public class SelectionWheelManager : MonoBehaviour
{
    #region PUBLIC_MEMBER_VARIABLES
    /// <summary>
    /// Threshold for display, if spin is below this value, DisableCanvas() will be called
    /// </summary>
    public float threshold = 0.1f;
    /// <summary>
    /// Multiplied when calculating friction, scales friction effect
    /// </summary>
    public float friction= 1;
    /// <summary>
    /// scales speed  that is applied to turn the wheel after touch input
    /// </summary>
    public float speed = 1;
    /// <summary>
    /// Index of controller to use for selection wheel (0/1)
    /// </summary>
    public int controllerIndex = 0;
    /// <summary>
    /// Specify where selected item should be positioned (clockwise, index in range of number of elem on circle)
    /// </summary>
    public int selectIndex = 1;
    /// <summary>
    /// canvas with selection wheel, en- and disabled depending on wheel 
    /// </summary>
    public Canvas canvas;
    #endregion

    #region PRIVATE_MEMBER_VARIABLES
    /// <summary>
    /// This is used to calculate the spin after touch input stopped
    /// </summary>
    private Vector2 prevPos = Vector2.zero;
    /// <summary>
    /// describes the spin, decreases over time with friction
    /// </summary>
    private float curSpin = 0;
    /// <summary>
    ///  Is selection wheel visible
    /// </summary>
    private bool visible = false;
    /// <summary>
    /// before disabling wheel:was it moved again? Should it still be disabled?
    /// </summary>
    private bool disabling = false;
    /// <summary>
    /// Radius of this object to rotate text on circle around centre
    /// </summary>
    private float radius;
    /// <summary>
    /// Overall angle of rotation around z axis, needed for selection.
    /// </summary>
    private float curAngle = 0;
    /// <summary>
    /// number of text elems in selection wheel
    /// </summary>
    private int elems;
    /// <summary>
    /// currently selected text elem by index.
    /// </summary>
    private int selected = -1;
    #endregion

    #region UNITY_MONOBEHAVIOUR_METHODS
    /// <summary>
    /// Initialize Wheel: Place each child (text) elem evenly on wheel. 
    /// </summary>
    void Start()
    {
        Component[] children;
        curAngle = 0;
        radius = GetComponent<RectTransform>().rect.width; //assuming width and height identical 
        children = gameObject.GetComponentsInChildren(typeof(RectTransform));
        elems = children.Length - 1; //ignore this elem
        if (selectIndex > elems) selectIndex = selectIndex % elems; //stay within boundaries, not necessarily needed though
        for (int i = 0; i < children.Length; i++)
        {
            RectTransform t = (RectTransform)children[i];
            if (t.gameObject != gameObject) //ignore this elem
            {
                // define point on outer circle called offset,start from 0 as 12 o'clock on the circle

                Vector2 offset = AngleToVector(360 / (elems) * (i - 1));
                offset *= radius; //apply length
                //move elem to that position on circle
                Debug.Log("offset unit size: " + offset);
                t.localPosition = new Vector3(offset.x, offset.y, 0);
            }
        }
        //counter turn elem to keep text straight
        if (canvas)
        {
            //canvas.GetComponent<Canvas>().enabled = false;
            visible = false;
        }
        HighlightSelection();
    }

    /// <summary>
    /// Update once per frame
    /// </summary>
    void Update()
    {

        //SpinOnTouch(false);
        SpinWithInertia();
        HighlightSelection();
    }
    #endregion

    #region PUBLIC_METHODS
    #endregion

    #region PRIVATE_METHODS
    /// <summary>
    /// x mod y in a function, as normal % just returns remainder and does not work for negative values.
    /// </summary>
    /// <param name="x">first operand</param>
    /// <param name="y">second operand</param>
    /// <returns></returns>
    private int Mod(int x, int y)
    {
        return (x % y + y) % y;
    }

    /// <summary>
    /// Enables canvas containing wheel to be displayed
    /// </summary>
    private void EnableCanvas()
    {
        if (canvas)
        {
            canvas.GetComponent<Canvas>().enabled = true;
            visible = true;
        }
    }
    /// <summary>
    /// Coroutine to wait shortly (0.5s) and disable canvas if user did not give new input meanwhile.
    /// </summary>
    /// <returns></returns>
    private IEnumerator DisableCanvas()
    {
        Debug.Log("disable gunction called...");
        yield return new WaitForSeconds(0.5f);

        if (disabling) //if disable still desired (whilst waiting further user input might have changed that)
        {
            if (canvas)
            {
                Debug.Log("Disabling...");
                canvas.GetComponent<Canvas>().enabled = false;
                visible = false;
            }
        }
    }

    /// <summary>
    /// Detects the currently selected item fom the selection wheel using the current angle and highlights it.
    /// </summary>
    private void HighlightSelection()
    {
        int step = (360 / elems);
        int tmp = elems - (int)(curAngle + step / 2) / step - 1;// map selection from 0 - (elems -1)
        tmp = Mod((tmp + selectIndex - 1), elems); // select the desired item clockwise, start at the top (-1)
        if (tmp == selected)
        {
            return;
        }
        selected = tmp;
        Debug.Log("selection: " + selected);
        VRUILogic.Instance.SelectedModeChanged(selected);
        Text[] texts = GetComponentsInChildren<Text>();
        for (int i = 0; i < texts.Length; i++)
        {
            if (selected != i)
            {
                texts[i].fontStyle = FontStyle.Normal;
                texts[i].color = Color.black;

            }
            else
            {
                texts[i].fontStyle = FontStyle.Bold;
                texts[i].color = Color.cyan;
            }
        }
    }
    /// <summary>
    /// returns a Vector two which is rotated by a degrees around the z axis
    /// </summary>
    /// <param name="a">anlge in range of 0 - 360°</param>
    /// <returns>rotated vector in unit size</returns>
    private Vector2 AngleToVector(float a)
    {
        a = Mathf.PI * a / 180;
        return new Vector2(Mathf.Cos(a), Mathf.Sin(a));
    }

    /// <summary>
    /// returns current angle given the vector
    /// </summary>
    /// <param name="v">vector</param>
    /// <returns>angle in degrees</returns>
    private float VectorToAngle(Vector2 v)
    {
        if (v.x == 0)
        {
            if (v.y > 0)
            {
                return 90;
            }
            else
            {
                return 270;
            }
        }
        float temp = Mathf.Atan(v.y / v.x) * 180 / Mathf.PI;
        if (v.x < 0)
        {
            return 180 + temp;
        }
        // TODOOOOOO
        if (temp < 0)
        {
            return temp + 360;
        }
        return temp;
    }

    /// <summary>
    /// Returns the angle within 0-360°
    /// </summary>
    /// <param name="a"></param>
    /// <returns></returns>
    private float AdjustAngle(float a)
    {
        if (a > 360)
        {
            a -= 360;
        }
        if (a < 0)
        {
            a += 360;
        }
        return a;
    }

    /// <summary>
    /// Calculates the current angle based on the user input.
    /// The angle is set with respect to the previous angle and the distance the finger tracked. 
    /// </summary>
    /// <param name="spin">should the wheel spin after touch input</param>
    private void SpinOnTouch(bool spin)
    {

        bool touched = VRUILogic.Instance.getTouchedInfo(controllerIndex);
        Vector2 curPos = VRUILogic.Instance.getTouchPosition(controllerIndex);

        if (touched) //if input found
        {
            disabling = false; //if change (input) occured whilst waiting for disable, do not disable
            if (!visible) EnableCanvas();
            if (!prevPos.Equals(Vector2.zero)) //angle does not change if new input just arrived
            {
                curAngle += VectorToAngle(prevPos) - VectorToAngle(curPos);
                curAngle = AdjustAngle(curAngle);
                Debug.Log("Cur ANgle: " + curAngle);
            }
            prevPos = curPos;

        }
        else
        {
            prevPos = Vector2.zero;
            if (visible && !disabling && !spin) //prevent from calling multiple disable canvases if we're already disabling
            {
                StartCoroutine(DisableCanvas());
                disabling = true;
            }
        }
    }
    /// <summary>
    /// Updates the position of the spinning wheel based on the touch input
    /// </summary>
    private void SpinWithInertia()
    {
        // update the current spin with the mouse wheel
        Component[] children;
        Vector3 newPos;

        bool touched = VRUILogic.Instance.getTouchedInfo(controllerIndex);
        Vector2 curPos = VRUILogic.Instance.getTouchPosition(controllerIndex);


        //Visibility settings for spin effect
        if (!touched && curSpin > -threshold && curSpin < threshold) //if too slow and no input
        {
            curSpin = 0;
            if (visible && !disabling) //prevent from calling multiple disable canvases if we're already disabling
            {
                StartCoroutine(DisableCanvas());
                disabling = true;
                return;
            }
        }

        if (touched)
        {
            if (curPos != null && !curPos.Equals(prevPos))
            {
                Debug.Log("Calculating spin");
                curSpin = Vector2.Distance(curPos, prevPos) * speed;
                if (!TurningClockwise(prevPos, curPos)) curSpin *= -1;
            }
            SpinOnTouch(true);
        }
        else
        {
            prevPos = Vector2.zero;
            curSpin -= curSpin * Time.deltaTime * friction;
            Debug.Log("applying spin and friction to abgle");
            curAngle = AdjustAngle(curAngle + (float)curSpin);
        }
        //rotate children (each text)
        children = gameObject.GetComponentsInChildren(typeof(RectTransform));
        for (int i = 0; i < children.Length; i++)
        {
            Transform transform = (Transform)children[i];
            if (transform.gameObject != gameObject)
            {
                float childangle = AdjustAngle( 360 -(curAngle + (360 / elems) * i));
                newPos = radius * AngleToVector(childangle); //adjust position using new point on circle
                children[i].transform.localPosition = new Vector3(newPos.x, newPos.y, 0);
            }
        }
    }

    private bool TurningClockwise(Vector2 prev, Vector2 cur)
    {
        if ((cur.x - prev.x) < 0) // if going towards the left
        {
            if (cur.y > 0)
            {
                return false;
            }
            else
            {
                return true;
            }
        }
        if (cur.y > 0)
        {
            return true;
        }
        return false;
    }
    #endregion
}

