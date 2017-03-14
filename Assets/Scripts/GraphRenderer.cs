using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
#if UNITY_EDITOR
using UnityEditor;
#endif

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

    [ExposeProperty]
    public bool ShowCurrentValue
    {
        get { return m_ShowCurrentValue; }
        set
        {
            if (m_ShowCurrentValue != value)
            {
                m_ShowCurrentValue = value;

                if (m_ShowCurrentValue)
                    createTextfieldForCurrentValue();
                else
                    destroyTextfieldForCurrentValue();
            }
        }
    }
    [HideInInspector]
    [SerializeField]
    private bool m_ShowCurrentValue = false;

    [HideInInspector]
    [SerializeField]
    private GameObject m_TextForCurrentValue;

    // We make the border offsets read only in play mode so we need extra variables so we can reset them on change in play mode in OnValidate
    private float m_BorderLeftStatic;
    private float m_BorderRightStatic;
    private float m_BorderTopStatic;
    private float m_BorderBottomStatic;

    // We set the static offset variables to the initial values in Awake and mark the offsets as initialized
    private bool m_BorderInitialized;

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

    // In the unity editor we make sure that the values do not make up a negative area and make them read only in play mode
#if UNITY_EDITOR
    private void OnValidate()
    {
        if (!EditorApplication.isPlaying)
        {
            // Make sure that the inner part is at least 10% of the panel
            RectTransform rectTransform = GetComponent<RectTransform>();

            BorderLeft = Mathf.Clamp(BorderLeft, 0f, rectTransform.rect.width - BorderRight - 0.1f * rectTransform.rect.width);
            BorderRight = Mathf.Clamp(BorderRight, 0f, rectTransform.rect.width - BorderLeft - 0.1f * rectTransform.rect.width);
            BorderBottom = Mathf.Clamp(BorderBottom, 0f, rectTransform.rect.height - BorderTop - 0.1f * rectTransform.rect.height);
            BorderTop = Mathf.Clamp(BorderTop, 0f, rectTransform.rect.height - BorderBottom - 0.1f * rectTransform.rect.height);
        }
        else
        {
            // In play mode reset the values when they are changed
            if (m_BorderInitialized)
            {
                BorderLeft = m_BorderLeftStatic;
                BorderRight = m_BorderRightStatic;
                BorderTop = m_BorderTopStatic;
                BorderBottom = m_BorderBottomStatic;
            }
        }
    }


#endif

    /// <summary>
    /// Draw a rectangle to display the bordered panel where the graph will be rendered
    /// </summary>
    private void OnDrawGizmosSelected()
    {
        // Get the rect transform component for later use
        RectTransform rectTransform = GetComponent<RectTransform>();

        // calculate the center of the cube
        Vector2 pivotDifference = new Vector2(0.5f - rectTransform.pivot.x, 0.5f - rectTransform.pivot.y);

        float maximumWidth = transform.lossyScale.x * (rectTransform.rect.width);
        float maximumHeight = transform.lossyScale.y * (rectTransform.rect.height);

        float borderLeftScaled = BorderLeft / rectTransform.rect.width;
        float borderRightScaled = BorderRight / rectTransform.rect.width;
        float borderTopScaled = BorderTop / rectTransform.rect.height;
        float borderBottomScaled = BorderBottom / rectTransform.rect.height;

        // Get the center to be the real center of the panel independent of the pivot point
        Vector3 centerOfWholeImage = new Vector3(pivotDifference.x * maximumWidth / 2f, pivotDifference.y * maximumHeight * 2f, 0f);

        // Get the center of the panel with borders
        Vector3 centerOfBorderedImage = new Vector3(centerOfWholeImage.x - ((borderRightScaled - borderLeftScaled) * 0.5f), centerOfWholeImage.y - ((borderTopScaled - borderBottomScaled) * 0.5f), 0f);

        // Calculate the cube to display the offset by the borders
        Vector3 panelSize = new Vector3(maximumWidth, maximumHeight, 0f);

        // As we apply a transform matrix with panel size as scale vector we need to subtract the size of the bordered panel from a (1,1,1) vector
        Vector3 panelBorderedSize = Vector3.one - new Vector3(borderLeftScaled + borderRightScaled, borderTopScaled + borderBottomScaled, 0f);

        // Set the gizmos matrix to match the right transform values
        Matrix4x4 oldGizmosMatrix = Gizmos.matrix;
        Matrix4x4 transformMatrix = Matrix4x4.TRS(transform.position, transform.rotation, panelSize);
        Gizmos.matrix = transformMatrix;

        // Finally draw the wire cube
        Gizmos.DrawWireCube(centerOfWholeImage + (centerOfBorderedImage - centerOfWholeImage), panelBorderedSize);

        Gizmos.matrix = oldGizmosMatrix;
    }

    private void Awake()
    {
        // Intialize the static border offsets so we can reset the border values in OnValidate
        m_BorderBottomStatic = BorderBottom;
        m_BorderTopStatic = BorderTop;
        m_BorderLeftStatic = BorderLeft;
        m_BorderRightStatic = BorderRight;
        m_BorderInitialized = true;
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
                if (i < valueList.Count)
                    m_CurrentValues.Add(valueList[i]);
                else
                    m_CurrentValues.Add(0f);
            }

            // Get the rect transform component
            m_RectTransform = GetComponent<RectTransform>();

            // Get the maximum scaled width and height
            m_MaximumWidth = (m_RectTransform.rect.width - BorderLeft - BorderRight);
            m_MaximumHeight = (m_RectTransform.rect.height - BorderBottom - BorderTop);

            // Number of points of the graph
            m_NumPoints = numPoints;
            m_StepSize = m_MaximumWidth / ((float)numPoints - 1);

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
            m_OscillatorLineRenderer.useWorldSpace = false;
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
            m_CurrentState = State.Playing;
            m_UpdateValuesCoroutine = updateValuesCoroutine();
            StartCoroutine(m_UpdateValuesCoroutine);
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
            m_UpdateValuesCoroutine = null;
            m_OscillatorLineRenderer.enabled = false;
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
            if (m_PlayCoroutine != null)
                StopCoroutine(m_PlayCoroutine);
            if (m_UpdateValuesCoroutine != null)
                StopCoroutine(m_UpdateValuesCoroutine);
            m_UpdateValuesCoroutine = null;
            m_PlayCoroutine = null;
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

            if (m_ShowCurrentValue && m_TextForCurrentValue)
            {
                m_TextForCurrentValue.GetComponent<Text>().text = m_CurrentValues[0].ToString();
            }
            yield return new WaitForSeconds(m_TimeStep);
        }

        if (m_CurrentState != State.Playing)
        {
            Debug.Log("Graph is not playing! Cannot update the values!");
        }
    }

    float findSmallestElement(List<float> list)
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
        return (value - min) * m_MaximumHeight / (max - min);
    }
    /// <summary>
    /// Transforms point from local space to panel space
    /// </summary>
    /// <param name="index"></param>
    /// <returns></returns>
    Vector3 getGraphPositionAtIndex(int index)
    {
        Vector3 pos = new Vector3(index * m_StepSize, scaleValues(m_MinValue, m_MaxValue, m_CurrentValues[index]), -5f);
        pos -= m_RectTransform.pivot.x * m_MaximumWidth * Vector3.right;
        pos -= m_RectTransform.pivot.y * m_MaximumHeight * Vector3.up;
        Vector2 pivotDifference = Vector2.one * 0.5f - m_RectTransform.pivot;

        pos += new Vector3(m_MaximumWidth * pivotDifference.x + ((BorderLeft - BorderRight) * 0.5f), m_MaximumHeight * pivotDifference.y + ((BorderBottom - BorderTop) * 0.5f), 0f);
        return pos;
    }

    private void createTextfieldForCurrentValue()
    {
        m_TextForCurrentValue = new GameObject();
        m_TextForCurrentValue.name = "Current Value";
        m_TextForCurrentValue.transform.parent = transform;
        m_TextForCurrentValue.transform.localScale = transform.localScale;
        m_TextForCurrentValue.transform.localPosition = Vector3.zero;
        m_TextForCurrentValue.transform.localRotation = Quaternion.identity;

        m_TextForCurrentValue.AddComponent<Text>();
    }

    private void destroyTextfieldForCurrentValue()
    {
        DestroyImmediate(m_TextForCurrentValue);
    }


}
