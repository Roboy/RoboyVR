using UnityEngine;
using System.Collections.Generic;
using System.Linq;

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
    private float defaultVal = 0f;
    /// <summary>
    /// List that saves further values if graph is being paused, replaces (parts of) current list as soon as continuing
    /// </summary>
    private List<float> m_buffered;

    /// <summary>
    /// How often do we want to update the graph renderer.
    /// </summary>
    private float m_TimeStep = 0f;

    #endregion

    #region UNITY_MONOBEHAVIOUR_METHODS
    /// <summary>
    /// Called once by Unity during startup / next frame when game obj created
    /// </summary>
    void Awake()
    {
        m_GraphRenderer = gameObject.AddComponent<GraphRenderer>();
    }

    #endregion

    #region PUBLIC_METHODS
    /// <summary>
    /// Starts a graph with the given values.
    /// </summary>
    /// <param name="values">A list of floats which you want to display. Can be empty.</param>
    /// <param name="displayedPointsCount">How many points of the list you want to display.</param>
    /// <param name="timeStep">How often the graph will be updated.</param>
    public void Run(List<float> values, int displayedPointsCount, float timeStep)
    {
        // return when the function is called with bad values
        if (values == null)
        {
            values = new List<float>();
        }

        if (displayedPointsCount < 2)
        {
            Debug.Log("Cannot start a graph with less than 2 points!");
            return;
        }

        if (timeStep < 0f)
            timeStep = 0f;

        // fill the list if necessary
        if (values.Count < displayedPointsCount )
        {
            m_Values = values;
            m_Values.AddRange(Enumerable.Repeat(defaultVal, (displayedPointsCount-values.Count)).ToList());
        }
        // init values
        m_Values = values;
        m_DisplayedPointsCount = displayedPointsCount;
        m_TimeStep = timeStep;
        // init graph renderer
        m_GraphRenderer.Initialize(m_Values, m_DisplayedPointsCount, m_TimeStep);
        m_GraphRenderer.Play();
    }

    /// <summary>
    /// Resumes the graph if it was paused. Updates values based on buffered list. 
    /// </summary>
    public void Resume()
    {
        Debug.Log("Resuming graph");
        if(m_Values == null) //if empty list found
        {
            m_Values = new List<float>();
            if(m_buffered == null)
            {
                m_Values.AddRange(Enumerable.Repeat(defaultVal, (m_DisplayedPointsCount)).ToList());
            }
        }
        //update list if buffered values found
        if(m_buffered != null && m_buffered.Count > 0)
        {
            int numchanges = m_buffered.Count;
            if(numchanges == m_DisplayedPointsCount || numchanges> m_Values.Count)
            {
                m_Values = m_buffered;
            }
            else
            {
                m_Values.RemoveRange(0, numchanges);
                m_Values.AddRange(m_buffered);
            }
            //delete buffer (praise the garbage collection)
            m_buffered = null;
        }
        m_GraphRenderer.Play();
    }

    /// <summary>
    /// Pauses the graph.
    /// </summary>
    public void Pause()
    {
        Debug.Log("Pausing graph");
        m_GraphRenderer.Pause();
    }

    /// <summary>
    /// Adds a value at the start of the list. Depending on whether graph is playing or not, real values updated / buffered list
    /// </summary>
    /// <param name="value">one value</param>
    public void AddValue(float value)
    {
        if (m_GraphRenderer.IsPlaying()) // update displayed values list
        {
        m_Values.Add(value);
        m_Values.RemoveAt(0);
        }
        else //insert in buffered list
        {
            if(m_buffered == null)
            {
                m_buffered = new List<float>();
            }
            m_buffered.Add(value);
            if(m_buffered.Count > m_DisplayedPointsCount) //keep list cropped to max_size
            {
                m_buffered.RemoveAt(0);
            }
        }
    }

    /// <summary>
    /// Adds a set of values at the start of the list. Depending on whether graph is playing or not, real values updated / buffered list
    /// </summary>
    /// <param name="values">Set of float values.</param>
    public void AddValues(List<float> values)
    {
        List<float> change;
        if (m_GraphRenderer.IsPlaying()) //update displayed values if playing
        {
            change = m_Values;
        }
        else //update buffer if not
        {
            if(m_buffered == null)
            {
                m_buffered = new List<float>();
            }
            change = m_buffered;
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
    /// </summary>
    /// <param name="count">New count of the displayed points.</param>
    public void ShowLastValues(int count)
    {
        int _count = Mathf.Min(m_Values.Count, count);
        m_GraphRenderer.ChangeGraphSize(_count);
    }


    /// <summary>
    /// changes the graph to not be dynamically adjusted depending on its current values, uses manual y axis value then
    /// </summary>
    public void SetNoAdjustment()
    {
        m_GraphRenderer.SetNoAdjustment();
    }

    /// <summary>
    /// changes graph to automatically scale y axis ´depending on current values
    /// </summary>
    public void SetAutomaticAdjust()
    {
        m_GraphRenderer.SetAutomaticAdjust();
    }

    /// <summary>
    /// sets range in for which the y values will be displayed
    /// </summary>
    /// <param name="range">x value == lower bownd, y upper</param>
    public void SetYAxisRange(Vector2 range)
    {
        m_GraphRenderer.SetYAxisRange(range);
    }
    #endregion

    /// <summary>
    /// sets default value which is used to create new elems if list not filled
    /// </summary>
    /// <param name="val">any desired val</param>
    public void SetDefaultValue(float val)
    {
        defaultVal = val;
        m_GraphRenderer.SetDefaultValue(val);
    }

}

