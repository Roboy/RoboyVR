﻿using UnityEngine;
using System.Collections.Generic;
using System.Linq;
using UnityEngine.UI;

/// <summary>
/// Wrapper class around GraphRenderer and ExtensionMethod.
/// Creates a Graph with a list containing the displayed data. Provides Scaling, setting of boundaries, 
/// operations on and alteration of data. 
/// </summary>
public class GraphObject : MonoBehaviour
{
    #region PUBLIC_MEMBER_VARIABLES
    #endregion

    #region PRIVATE_MEMBER_VARIABLES

    /// <summary>
    /// for multiple curves in one Graph-plotter, this defines one curve
    /// </summary>
    private struct m_GraphInstance
    {
        /// <summary>
        /// instance of the plotting class
        /// </summary>
        GraphRenderer renderer;
        /// <summary>
        /// list of values to plot (pointer to same list that renderer has)
        /// </summary>
        List<float> values;
        /// <summary>
        /// List of buffered values in case graph is paused
        /// </summary>
        List<float> buffer;
        /// <summary>
        /// colour of displayed curve
        /// </summary>
        Color colour;
        /// <summary>
        /// default value which is used for undefined points/list elems
        /// </summary>
        float defaultValue;
        /// <summary>
        /// number of points to display, defines list sizes
        /// </summary>
        int numberOfDisplayedPoints;
    }

    /// <summary>
    /// Cached reference to our graph renderer.
    /// </summary>
    private GraphRenderer m_GraphRenderer;

    /// <summary>
    /// How many points do we currently show on the graph.
    /// </summary>
    private int m_DisplayedPointsCount = 0;

    /// <summary>
    /// List of all stored values.
    /// </summary>
    private List<float> m_Values = new List<float>();

    /// <summary>
    /// default value to fill list with if elems missing
    /// </summary>
    private float m_DefaultValue = 0f;
    /// <summary>
    /// List that saves further values if graph is being paused, replaces (parts of) current list as soon as continuing
    /// </summary>
    private List<float> m_Buffer;
    /// <summary>
    /// this restricts the addvalue function to only add when one fixed update frame is waited (for appropriate time representation)
    /// </summary>
    private bool m_UpdatePossible = true;
    #endregion

    #region UNITY_MONOBEHAVIOUR_METHODS
    /// <summary>
    /// this restricts addition of new values to fixed update times. Needed for coherent time representation
    /// </summary>
    void FixedUpdate()
    {
        //fixed update faster than update or no new values --> for correct time behaviour, fill list with additional values
        //might result in step like behaviour.... TODO
        if (m_UpdatePossible)
        {
            AddValue(m_Values[m_Values.Count - 1]);
        }
        m_UpdatePossible = true;
    }
    #endregion

    #region PUBLIC_METHODS
    /// <summary>
    /// Starts a graph with the given values.
    /// </summary>
    /// <param name="values">A list of floats which you want to display. Can be empty.</param>
    /// <param name="displayedSeconds">How many seconds you want to display (time frame).</param>
    public void Initialize(List<float> values, float displayedSeconds)
    {
        // return when the function is called with bad values
        if (values == null)
        {
            values = new List<float>();
        }
        if (displayedSeconds < 2 * Time.deltaTime)
        {
            Debug.Log("Cannot start a graph with less than 2 points!");
            return;
        }

        m_GraphRenderer = gameObject.AddComponent<GraphRenderer>();

        // fill the list if necessary
        if (values.Count < m_DisplayedPointsCount)
        {
            m_Values = values;
            m_Values.AddRange(Enumerable.Repeat(m_DefaultValue, m_DisplayedPointsCount).ToList());
        }
        // init values
        m_Values = values;
        // init graph renderer
        m_GraphRenderer.Initialize(m_Values, m_DisplayedPointsCount);
        DisplayForNumberOfSeconds(displayedSeconds);
    }

    /// <summary>
    /// starts the Graphrenderer. call initialize() beforehand.
    /// </summary>
    public void Play()
    {
        m_GraphRenderer.Play();
    }

    /// <summary>
    /// Resumes the graph if it was paused. Updates values based on buffered list. 
    /// </summary>
    public void Resume()
    {
        //Debug.Log("Resuming graph");
        if (m_Values == null) //if empty list found
        {
            m_Values = new List<float>();
            if (m_Buffer == null)
            {
                m_Values.AddRange(Enumerable.Repeat(m_DefaultValue, (m_DisplayedPointsCount)).ToList());
            }
        }
        //update list if buffered values found
        if (m_Buffer != null && m_Buffer.Count > 0)
        {
            //operate on this list (setting it to buffer list will not set list of graphrenderer)
            int numchanges = m_Buffer.Count;
            m_Values.RemoveRange(0, numchanges);
            m_Values.AddRange(m_Buffer);

            //delete buffer (praise the garbage collection)
            m_Buffer = null;
        }
        m_GraphRenderer.Play();
    }

    /// <summary>
    /// Pauses the graph.
    /// </summary>
    public void Pause()
    {
        //Debug.Log("Pausing graph");
        m_GraphRenderer.Pause();
    }

    /// <summary>
    /// Adds a value at the start of the list. Depending on whether graph is playing or not, real values updated / buffered list
    /// Needs to be called after Graphrenderer initialized  
    /// </summary>
    /// <param name="value">one value</param>
    public void AddValue(float value)
    {
        //this is used to visualise graph in right time frame
        if (!m_UpdatePossible)
        {
            return;
        }
        m_UpdatePossible = false;
        ///Debug.Log("List addition: Update (FPS info)" + (int)Time.time);

        if (m_GraphRenderer.IsPlaying()) // update displayed values list
        {
            m_Values.Add(value);
            m_Values.RemoveAt(0);
        }
        else //insert in buffered list
        {
            if (m_Buffer == null)
            {
                m_Buffer = new List<float>();
            }
            //Debug.Log("buffering");
            m_Buffer.Add(value);
            if (m_Buffer.Count > m_DisplayedPointsCount) //keep list cropped to max_size
            {
                m_Buffer.RemoveAt(0);
            }
        }

    }

    /// <summary>
    /// Adds a set of values at the start of the list. Depending on whether graph is playing or not, real values updated / buffered list
    /// Needs to be called after Graphrenderer initialized  
    /// </summary>
    /// <param name="values">Set of float values.</param>
    public void AddValues(List<float> values)
    {
        if (!m_UpdatePossible)
        {
            return;
        }
        m_UpdatePossible = false;
        /*TODO
        if (!m_WaitingOver) return;
        m_WaitingOver = false;*/
        List<float> change;
        if (m_GraphRenderer.IsPlaying()) //update displayed values if playing
        {
            change = m_Values;
        }
        else //update buffer if not
        {
            if (m_Buffer == null)
            {
                m_Buffer = new List<float>();
            }
            change = m_Buffer;
        }
        change.AddRange(values);
        int delta = change.Count - m_DisplayedPointsCount;
        if (delta > 0)
        {
            change.RemoveRange(0, delta);
        }

    }

    /// <summary>
    /// Changes the count of displayed points.
    /// Needs to be called after Graphrenderer initialized  
    /// </summary>
    /// <param name="count">New count of the displayed points.</param>
    public void NumberOfDisplayedPoints(int count)
    {
        if (count > 2)
        {
            m_GraphRenderer.ChangeGraphPointNumber(count);
        }
    }

    /// <summary>
    /// Sets the number of points to display the frame of specified number of seconds
    /// Needs to be called after Graphrenderer initialized  
    /// </summary>
    /// <param name="seconds">display all values in a time frame of (seconds) secs</param>
    public void DisplayForNumberOfSeconds(float seconds)
    {
        if (seconds > 0)
        {
            int pointNumber = (int)(seconds / Time.fixedDeltaTime);
            //Debug.Log("Number of points for " + seconds + " seconds: " + pointNumber);
            m_GraphRenderer.ChangeGraphPointNumber(pointNumber);
            m_DisplayedPointsCount = pointNumber;
            //Debug.Log(seconds + " sec range");
        }
        else
        {
            Debug.Log("Graph display frame must be positive!");
        }
    }

    /// <summary>
    /// changes the graph to not be dynamically adjusted depending on its current values, uses manual y axis value then
    /// Needs to be called after Graphrenderer initialized  
    /// </summary>
    public void SetNoAdjustment()
    {
        m_GraphRenderer.SetNoAdjustment();
    }

    /// <summary>
    /// changes graph to automatically scale y axis ´depending on current values
    /// Needs to be called after Graphrenderer initialized  
    /// </summary>
    public void SetAutomaticAdjust()
    {
        m_GraphRenderer.SetAutomaticAdjust();
    }

    /// <summary>
    /// sets range in for which the y values will be displayed
    /// Needs to be called after Graphrenderer initialized  
    /// </summary>
    /// <param name="range">x value == lower bownd, y upper</param>
    public void SetManualAdjust(Vector2 range)
    {
        m_GraphRenderer.SetManualAdjust(range);
    }

    /// <summary>
    /// sets range in for which the y values will be displayed
    /// Needs to be called after Graphrenderer initialized  
    /// </summary>
    /// <param name="range">x value == lower bownd, y upper</param>
    public void SetManualAdjust(float min, float max)
    {
        m_GraphRenderer.SetManualAdjust(min, max);
    }
    /// <summary>
    /// sets default value which is used to create new elems if list not filled
    /// Needs to be called after Graphrenderer initialized  
    /// </summary>
    /// <param name="val">any desired val</param>
    public void SetDefaultValue(float val)
    {
        m_DefaultValue = val;
        m_GraphRenderer.SetDefaultValue(val);
    }

    /// <summary>
    /// sets number of points to plot the graph. The list will be adapted to the new size
    /// Needs to be called after Graphrenderer initialized  
    /// </summary>
    /// <param name="number"></param>
    public void SetNumberOfPoints(int number)
    {
        m_GraphRenderer.ChangeGraphPointNumber(number);
    }

    /// <summary>
    /// Sets colour of the respective Graph.
    /// Needs to be called after Graphrenderer initialized  
    /// </summary>
    /// <param name="colour">Any colour</param>
    public void SetGraphColour(Color colour)
    {
        m_GraphRenderer.SetColour(colour);
    }
    #endregion

    #region PRIVATE_METHODS

    /// <summary>
    /// Creates a text field object for the current value. Set as child to this gameObject.
    /// </summary>
    /// <returns>text component of the new object</returns>
    private Text createTextfieldForCurrentValue()
    {
        GameObject m_Text = new GameObject();
        m_Text.name = "TextObject";
        m_Text.transform.parent = transform;
        m_Text.transform.localScale = transform.localScale;
        m_Text.transform.localPosition = Vector3.zero;
        m_Text.transform.localRotation = Quaternion.identity;

        RectTransform rectTransform = m_Text.GetComponent<RectTransform>();
        rectTransform.anchorMax = (new Vector2(0.2f, 1));
        rectTransform.anchorMin = new Vector2(0.2f, 1);
        rectTransform.sizeDelta = new Vector2(0.8f, 0.8f);
        return AddText(m_Text);
    }

    /// <summary>
    /// Adds text component to specified object with value presets such as overflowing text, anchoring, size, font and colour.
    /// </summary>
    /// <param name="obj">Object to which to attach the Text component</param>
    /// <returns>Reference to the added Textcomponent of the Object</returns>
    private Text AddText(GameObject obj)
    {
        if (obj.GetComponent<Text>() != null)
        {
            Debug.Log("Trying to add text Component to existing text component! Aborting");
            return null;
        }
        Text text = obj.AddComponent<Text>(); /*Text message displaying information about screens*/
        text.rectTransform.localPosition = Vector3.zero;
        text.horizontalOverflow = HorizontalWrapMode.Overflow;
        text.verticalOverflow = VerticalWrapMode.Overflow;
        text.alignment = TextAnchor.UpperLeft;
        Font f = (Font)Resources.GetBuiltinResource(typeof(Font), "Arial.ttf");
        text.font = f;
        text.fontSize = 56;
        text.color = Color.black;
        text.enabled = true;
        return text;
    }
    #endregion
}

