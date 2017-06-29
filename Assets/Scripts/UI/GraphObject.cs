using UnityEngine;
using System.Collections.Generic;

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
		Debug.Log ("New m_GraphRenderer");
    }

    #endregion

    #region PUBLIC_METHODS
    /// <summary>
    /// Starts a graph with the given values.
    /// </summary>
    /// <param name="values">A list of floats which you want to display.</param>
    /// <param name="displayedPointsCount">How many points of the list you want to display.</param>
    /// <param name="timeStep">How often the graph will be updated.</param>
    public void Run(List<float> values, int displayedPointsCount, float timeStep)
    {
        // return when the function is called with bad values
        if (values == null)
        {
            Debug.Log("Value list is null! Create it first!");
            return;
        }

        if (displayedPointsCount < 2)
        {
            Debug.Log("Cannot start a graph with less than 2 points!");
            return;
        }

        if (timeStep < 0f)
            timeStep = 0f;

        // fill the list if necessary
        if (values.Count == 0)
        {
            for (int i = 0; i < displayedPointsCount; i++)
            {
                values.Add(0f);
            }
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
    /// Resumes the graph if it was paused.
    /// </summary>
    public void Resume()
    {
        m_GraphRenderer.Play();
    }

    /// <summary>
    /// Pauses the graph.
    /// </summary>
    public void Pause()
    {
        m_GraphRenderer.Pause();
    }

    /// <summary>
    /// Adds a value at the start of the list.
    /// </summary>
    /// <param name="value"></param>
    public void AddValue(float value)
    {
        m_Values.Add(value);
        m_Values.RemoveAt(0);
    }

    /// <summary>
    /// Adds a set of values at the start of the list.
    /// </summary>
    /// <param name="values">Set of float values.</param>
    public void AddValues(List<float> values)
    {
        m_Values.AddRange(values);
        m_Values.RemoveRange(0, values.Count);
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


}

