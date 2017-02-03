using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

[RequireComponent(typeof(Image))]
public class GraphRenderer : MonoBehaviour
{
    // Color for the graph
    public Material GraphMaterial;

    // Offset values
    public float BorderLeft;
    public float BorderRight;
    public float BorderTop;
    public float BorderBottom;

    // LineRenderer to render the graph
    private LineRenderer m_OscillatorLineRenderer;

    private enum State
    {
        None,
        Initialized,
        Playing,
        Paused
    }

    // Current State
    private State m_CurrentState = State.None;

    // Reference of the origin value list in the initialize method
    private List<float> m_OriginValues;

    // The last updated values and positions in 3D of the graph
    private List<float> m_CurrentValues = new List<float>();
    private List<Vector3> m_Positions = new List<Vector3>();

    // Coroutines to handle the play process
    private IEnumerator m_PlayCoroutine;
    private IEnumerator m_UpdateValuesCoroutine;

    // Distance between each point
    private float m_StepSize = 0f;

    // Number of points the graph displays
    private int m_NumPoints = 0;

    // Minimum and maximum values of the data input
    private float m_MinValue = 0f;
    private float m_MaxValue = 1f;

    // Scaled width and height of the panel
    private float m_MaximumWidth;
    private float m_MaximumHeight;

    private RectTransform m_RectTransform;

    // How often we update the graph`s values
    private float m_TimeStep;

    void OnDisable()
    {
        Stop();
    }

    /// <summary>
    /// Initialize all the internal parameters 
    /// </summary>
    /// <param name="valueList"></param>
    /// <param name="numPoints"></param>
    /// <param name="timeStep"></param>
    public void Initialize(List<float> valueList, int numPoints, float timeStep)
    {
        if (m_CurrentState == State.None)
        {
            // Intialize the values list with values from the given list or set them to zero if numPoints exceeds the list length
            for (int i = 0; i < numPoints; i++)
            {
                if(i < valueList.Count)
                    m_CurrentValues.Add(valueList[i]);
                else
                    m_CurrentValues.Add(0f);
            }

            // Get the rect transform component
            m_RectTransform = GetComponent<RectTransform>();

            // Get the maximum scaled width and height
            m_MaximumWidth = (m_RectTransform.rect.width - BorderLeft - BorderRight) * transform.lossyScale.x;
            m_MaximumHeight = (m_RectTransform.rect.height - BorderBottom - BorderTop) * transform.lossyScale.y;
            
            // Number of points of the graph
            m_NumPoints = numPoints;
            m_StepSize = m_MaximumWidth / ((float)numPoints-1);

            m_OriginValues = valueList;
            m_TimeStep = timeStep;

            // For scaling purpose
            m_MinValue = findSmallestElement(m_CurrentValues);
            m_MaxValue = findBiggestElement(m_CurrentValues);

            for (int i = 0; i < numPoints; i++)
            {
                // Stick the graph to panel movement
                Vector3 pos = getGraphPositionAtIndex(i);
                m_Positions.Add(pos);
            }
            m_CurrentState = State.Initialized;
        }
    }
    /// <summary>
    /// Starts / Resumes the graph
    /// </summary>
    public void Play()
    {
        if (m_CurrentState == State.Initialized)
        {
            // Create a LineRenderer component and attach it with the given parameters
            m_OscillatorLineRenderer = gameObject.AddComponent<LineRenderer>();
            m_OscillatorLineRenderer.numPositions = m_NumPoints;
            m_OscillatorLineRenderer.material = GraphMaterial;
            m_OscillatorLineRenderer.startWidth = m_OscillatorLineRenderer.endWidth = 0.01f;
            m_OscillatorLineRenderer.SetPositions(m_Positions.ToArray());

            // Update graph position each frame
            m_PlayCoroutine = playCoroutine();
            // Update graph values each timestep
            m_UpdateValuesCoroutine = updateValuesCoroutine();           
            m_CurrentState = State.Playing;
            StartCoroutine(m_PlayCoroutine);
            StartCoroutine(m_UpdateValuesCoroutine);
        }
        else if (m_CurrentState == State.Paused)
        {
            StartCoroutine(m_UpdateValuesCoroutine);
            m_CurrentState = State.Playing;
        }
        else if (m_CurrentState == State.None)
        {
            Debug.Log("Graph not initialized yet! Call the initialize function first!");
        }
    }

    /// <summary>
    /// Pauses the graph value updates
    /// </summary>
    public void Pause()
    {
        if (m_CurrentState == State.Playing)
        {
            // Stop updating the values
            StopCoroutine(m_UpdateValuesCoroutine);
            m_CurrentState = State.Paused;
        }
    }
    /// <summary>
    /// Destroys the graph
    /// </summary>
    public void Stop()
    {
        if (m_CurrentState != State.None)
        {
            // Kill linerenderer, stop the coroutines
            StopCoroutine(m_PlayCoroutine);
            StopCoroutine(m_UpdateValuesCoroutine);
            Destroy(m_OscillatorLineRenderer);
            m_CurrentState = State.None;
        }
    }
    /// <summary>
    /// Renders the graph according to panel movement
    /// </summary>
    /// <returns></returns>
    private IEnumerator playCoroutine()
    {
        while (m_CurrentState == State.Playing || m_CurrentState == State.Paused)
        {
            m_Positions.Clear();

            // For scaling purpose
            m_MinValue = findSmallestElement(m_CurrentValues);
            m_MaxValue = findBiggestElement(m_CurrentValues);

            for (int i = 0; i < m_NumPoints; i++)
            {
                // Stick the graph to panel movement
                Vector3 pos = getGraphPositionAtIndex(i);
                m_Positions.Add(pos);
            }

            // Update the linerenderer with the new positions of the graph points
            m_OscillatorLineRenderer.SetPositions(m_Positions.ToArray());
            yield return null;
        }

        if (m_CurrentState == State.Initialized)
        {
            Debug.Log("Graph is initialized but not currently playing! Start the graph!");
        }
        else if (m_CurrentState == State.None)
        {
            Debug.Log("Graph is not initialized! Initialize the graph first");
        }
    }
    /// <summary>
    /// Refreshes value list each timestamp
    /// </summary>
    /// <returns></returns>
    private IEnumerator updateValuesCoroutine()
    {
        while (m_CurrentState == State.Playing)
        {
            m_CurrentValues.Clear();

            for (int i = 0; i < m_NumPoints; i++)
            {

                if (i < m_OriginValues.Count)
                    m_CurrentValues.Add(m_OriginValues[i]);
                // Origin list is shorter than the number of graph points you want to display 
                else
                    m_CurrentValues.Add(0f);
            }
            yield return new WaitForSeconds(m_TimeStep);
        }
    }

    float findSmallestElement(List<float> list )
    {
        if (list == null)
        {
            throw new ArgumentNullException("self");
        }

        if (list.Count == 0)
        {
            throw new ArgumentException("List is empty.", "self");
        }

        float minima = list[0];
        // Looking for the smallest element in a list
        foreach (float f in list)
        {
            if (f < minima)
            {
                minima = f;
            }
        }

        return minima;
    }

    float findBiggestElement(List<float> list)
    {
        if (list == null)
        {
            throw new ArgumentNullException("self");
        }

        if (list.Count == 0)
        {
            throw new ArgumentException("List is empty.", "self");
        }

        float maxima = list[0];
        // Looking for the biggest element in a list
        foreach (float f in list)
        {
            if (f > maxima)
            {
                maxima = f;
            }
        }

        return maxima;
    }

    /// <summary>
    /// Converts original values between [0, max. Height]
    /// </summary>
    /// <param name="min"></param>
    /// <param name="max"></param>
    /// <param name="value"></param>
    /// <returns></returns>
    float scaleValues(float min, float max, float value)
    {
        if (min == max)
            return value;
        return (value - min)*m_MaximumHeight/(max - min);
    }
    /// <summary>
    /// Transforms point from local space to panel space
    /// </summary>
    /// <param name="index"></param>
    /// <returns></returns>
    Vector3 getGraphPositionAtIndex(int index)
    {
        Vector3 pos = new Vector3(index * m_StepSize, scaleValues(m_MinValue, m_MaxValue, m_CurrentValues[index]), -0.025f);
        pos = transform.rotation * pos;
        pos += transform.position;
        pos -= m_RectTransform.pivot.x * m_MaximumWidth * transform.right;
        pos -= m_RectTransform.pivot.y * m_MaximumHeight * transform.up;
        return pos;
    }

    
}
