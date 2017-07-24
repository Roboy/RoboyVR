using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class ScrollViewFunctionalities : MonoBehaviour
{

    #region PRIVATE_MEMBER_VARIABLES
    /// <summary>
    /// scroll Rectangle, adjusts content
    /// </summary>
    [SerializeField]
    private ScrollRect m_scroll;
    /// <summary>
    /// Reference to item, needed for size considerations
    /// </summary>
    [SerializeField]
    private RectTransform content;

    /// <summary>
    /// if set to true, one scroll action equals one item, otherwise whole page is skipped / moved
    /// </summary>
    [SerializeField]
    private bool m_ScrollItemwise = true;

    private bool m_buttonHeld = false;
    #endregion

    #region PUBLIC_METHODS
    /// <summary>
    /// Displays top part of the linked content canvas
    /// </summary>
    public void ScrollToTop()
    {
        Debug.Log("Scroll to top");
        m_scroll.verticalNormalizedPosition = 1;
    }

    /// <summary>
    /// moves content canvas to scroll down
    /// </summary>
    public void ScrollToBottom()
    {
        Debug.Log("Scroll to bottom");
        m_scroll.verticalNormalizedPosition = 0;
    }
    /// <summary>
    /// moves content canvas to scroll up
    /// </summary>
    public void ScrollUp()
    {
        Debug.Log("Scroll up");
        Scroll(1);
    }
    /// <summary>
    /// displays bottom part of linked content canvas
    /// </summary>
    public void ScrollDown()
    {
        Debug.Log("Scroll down");
        Scroll(-1);
    }
    #endregion
    /// <summary>
    /// when trigger is held (over button up), this function is called
    /// </summary>
    public void OnButtonUpDown()
    {
        m_buttonHeld = true;
        StartCoroutine(OnButtonUpHolding());
    }

    /// <summary>
    /// when trigger held (over button down), this function is called
    /// </summary>
    public void OnButtonDownDown()
    {
        m_buttonHeld = true;
        StartCoroutine(OnButtonDownHolding());
    }

    /// <summary>
    /// when trigger is not held anymore
    /// </summary>
    public void OnButtonUp()
    {
        m_buttonHeld = false;
        Debug.Log("Up registered");
    }
    #region PRIVATE_METHODS
    /// <summary>
    /// Scrolling up or down, factor specifies which direction
    /// </summary>
    /// <param name="factor">-1 == down, 1 == up</param>
    private void Scroll(int factor)
    {
        Button[] buttons = content.gameObject.GetComponentsInChildren<Button>();

        float itemheight = buttons[0].GetComponent<RectTransform>().rect.height;
        float contentheight = content.gameObject.GetComponent<RectTransform>().rect.height;
        if (buttons.Length > 0)
        {
            float step = (itemheight * buttons.Length) / contentheight;
            if (step < 1) //if no scrolling needed
            {//TODO: might not be necessary to set
                m_scroll.verticalNormalizedPosition = 1;
            }
            else
            { // adapt scrolling 
                //currentpos += numberofItemsfitting/ totalitemnumber
                float temp = m_scroll.verticalNormalizedPosition;
                if (m_ScrollItemwise)
                    temp += factor / (float) buttons.Length;
                else
                    temp += factor * (contentheight / (float) itemheight) / buttons.Length;
                if (temp > 1) temp = 1;
                m_scroll.verticalNormalizedPosition = temp;
                //adapt to 1
            }
        }
    }
    /// <summary>
    /// continuously scroll up if button held
    /// </summary>
    /// <returns></returns>
    IEnumerator OnButtonUpHolding()
    {
        Debug.Log("While holding up");
        while (m_buttonHeld)
        {
            ScrollUp();
            yield return new WaitForSeconds(0.1f);
        }
    }

    /// <summary>
    /// continuously scroll down if button held
    /// </summary>
    /// <returns></returns>
    IEnumerator OnButtonDownHolding()
    {
        Debug.Log("While holding down");
        while (m_buttonHeld)
        {
            ScrollDown();
            yield return new WaitForSeconds(0.1f);
        }
    }
    #endregion
}