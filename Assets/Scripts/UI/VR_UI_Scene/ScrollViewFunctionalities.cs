using System.Collections;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Provides scroll functionalities for a specified scrollView component.
/// These include toTop, toBottom, scrollUp, scrollDown.
/// the latter are either performed itemwise or using a certain percentage.
/// </summary>
public class ScrollViewFunctionalities : MonoBehaviour
{

    #region PRIVATE_MEMBER_VARIABLES
    /// <summary>
    /// scroll Rectangle, adjusts content
    /// </summary>
    [SerializeField]
    private ScrollRect m_Scroll;

    /// <summary>
    /// Reference to item, needed for size considerations
    /// </summary>
    [SerializeField]
    private RectTransform m_Content;

    /// <summary>
    /// RectTransform of the actual viewPort, used for percentage wise scrolling
    /// </summary>
    [SerializeField]
    private RectTransform m_ViewPort;

    /// <summary>
    /// if set to true, one scroll action equals one item(Button), otherwise whole page is skipped / moved
    /// </summary>
    [SerializeField]
    private bool m_ScrollButtonsItemwise = true;

    /// <summary>
    /// If scroll for a certain amount instead of itemwise, this value is used. 
    /// This value describes percentage of viewedContentbox to be scrolled down. 
    /// </summary>
    private float m_Percentage = 0.1f;

    /// <summary>
    /// is trigger currently pressed? Scrollfunctionalities depending on this. 
    /// </summary>
    private bool m_ButtonHeld = false;
    #endregion

    //this region contains all events which can be accessed from outside
    #region PUBLIC_METHODS

    /// <summary>
    /// Displays top part of the linked content canvas
    /// </summary>
    public void ObButtonTopClick()
    {
        Debug.Log("Scroll to top");
        m_Scroll.verticalNormalizedPosition = 1;
    }

    /// <summary>
    /// moves content canvas to scroll down
    /// </summary>
    public void OnButtonBottomClick()
    {
        Debug.Log("Scroll to bottom");
        m_Scroll.verticalNormalizedPosition = 0;
    }

    /// <summary>
    /// moves content canvas to scroll up
    /// </summary>
    public void OnButtonUpClick()
    {
        Debug.Log("Scroll up");
        if (m_ScrollButtonsItemwise)
            ScrollItemwise(true);
        else
            ScrollPercentage(true);
    }

    /// <summary>
    /// displays bottom part of linked content canvas
    /// </summary>
    public void OnButtonDownClick()
    {
        Debug.Log("Scroll down");
        if (m_ScrollButtonsItemwise)
            ScrollItemwise(false);
        else
            ScrollPercentage(false);
    }

    /// <summary>
    /// when trigger is held (over button up), this function is called
    /// </summary>
    public void OnButtonUpHold()
    {
        m_ButtonHeld = true;
        StartCoroutine(OnButtonUpHolding());
    }

    /// <summary>
    /// when trigger held (over button down), this function is called
    /// </summary>
    public void OnButtonDownHold()
    {
        m_ButtonHeld = true;
        StartCoroutine(OnButtonDownHolding());
    }

    /// <summary>
    /// when trigger is not held anymore
    /// </summary>
    public void OnButtonNotHeldAnymore()
    {
        m_ButtonHeld = false;
        Debug.Log("Up registered");
    }
    #endregion

    #region PRIVATE_METHODS
    /// <summary>
    /// Scrolling up or down, factor specifies which direction
    /// </summary>
    /// <param name="up">scrolling up or down?</param>
    private void ScrollItemwise(bool up)
    {
        Button[] buttons = m_Content.gameObject.GetComponentsInChildren<Button>();

        float itemheight = buttons[0].GetComponent<RectTransform>().rect.height;
        float contentheight = m_Content.gameObject.GetComponent<RectTransform>().rect.height;
        if (buttons.Length > 0)
        {
            //how many pages would be necessary to fit all items in the content rect 
            float step = (itemheight * buttons.Length) / contentheight;
            if (step < 1) //if no scrolling needed
            {//TODO: might not be necessary to set
                m_Scroll.verticalNormalizedPosition = 1;
            }
            else
            {
                // adapt scrolling 
                float temp = m_Scroll.verticalNormalizedPosition;

                if (up) temp += 1 / (float)buttons.Length;
                else temp -= 1 / (float)buttons.Length;
                //clamp
                if (temp > 1) temp = 1;
                if (temp < 0) temp = 0;
                m_Scroll.verticalNormalizedPosition = temp;
                //adapt to 1
            }
        }
    }

    /// <summary>
    ///  scrolls up or down a certain amount which is set in the editor (or default)
    /// </summary>
    /// <param name="up">scrolling up ?</param>
    private void ScrollPercentage(bool up)
    {
        float factor = 1;
        if (!up) factor = -1;
        float contentheight = m_Content.gameObject.GetComponent<RectTransform>().rect.height;
        float displayedHeight = m_ViewPort.rect.height;
        //scroll up or down (changed by factor) for a given percentage 
        //(scaled to fit normalized values by multiplying with last factor)
        m_Scroll.verticalNormalizedPosition += factor * m_Percentage * (displayedHeight / contentheight);


    }
    /// <summary>
    /// continuously scroll up if button held
    /// </summary>
    /// <returns></returns>
    private IEnumerator OnButtonUpHolding()
    {
        Debug.Log("While holding up");
        while (m_ButtonHeld)
        {
            yield return new WaitForSeconds(0.1f);
            if (m_ButtonHeld)
                OnButtonUpClick();
        }
    }

    /// <summary>
    /// continuously scroll down if button held
    /// </summary>
    /// <returns></returns>
    private IEnumerator OnButtonDownHolding()
    {
        Debug.Log("While holding down");
        while (m_ButtonHeld)
        {
            yield return new WaitForSeconds(0.1f);
            if (m_ButtonHeld)
                OnButtonDownClick();
        }
    }
    #endregion
}