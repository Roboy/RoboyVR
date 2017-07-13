using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System.Linq;
#if UNITY_EDITOR
using UnityEditor;
#endif

[RequireComponent(typeof(Image))]

public class GraphRenderer : MonoBehaviour
{
    #region PUBLIC_VARIABLES

    // Color for the graph
    public Material GraphMaterial;

    // Offset values    
    public float BorderLeft;
    public float BorderRight;
    public float BorderTop;
    public float BorderBottom;


    #endregion // PUBLIC_VARIABLES

    #region PRIVATE_VARIABLES

    [HideInInspector]
    [SerializeField]
    private GameObject m_TextForValueName;

    // We make the border offsets read only in play mode so we need extra variables so we can reset them on change in play mode in OnValidate
    private float m_BorderLeftStatic;
    private float m_BorderRightStatic;
    private float m_BorderTopStatic;
    private float m_BorderBottomStatic;

    // We set the static offset variables to the initial values in Awake and mark the offsets as initialized
    private bool m_BorderInitialized;

    // LineRenderer to render the graph
    private LineRenderer m_OscillatorLineRenderer;

    /// <summary>
    /// Indicates whether the graph curve is updating
    /// </summary>
    private bool m_Playing = false;
    /// <summary>
    /// Indicates whether all required objects and values are set
    /// </summary>
    private bool m_Initialized = false;

    /// <summary>
    /// Defines whether axis should be adjusted 
    /// </summary>
    private bool m_AdjustAxis = false;

    /// <summary>
    /// If axis not automatically adjusted, this range will be used. 
    /// x is the lower bound, y the upper. 
    /// </summary>
    private Vector2 m_YAxisRange = new Vector2(-1,1);

    /// <summary>
    /// Sets the distance from the graph to the canvas / panel
    /// </summary>
    private float m_ZDistance = -0.1f;

    /// <summary>
    /// The List of values to display. 
    /// since not primitive -> object pointer -> changes can be applied by other methods having same pointer
    /// </summary>
    private List<float> m_Values;

    /// <summary>
    /// Points on the lineRenderer in the local space.
    /// These points are already scaled and position on the graph.
    /// </summary>
    private List<Vector3> m_Positions = new List<Vector3>();

    /// <summary>
    /// default value that is used to initialize points on graph
    /// </summary>
    private float m_DefaultValue = 0f;

    /// <summary>
    ///  Coroutines to handle the play process. References needed to stop these. 
    /// </summary>
    private IEnumerator m_PlayCoroutine = null;

    // Distance between each point
    private float m_StepSize = 0f;

    // Number of points the graph displays
    private int m_NumPoints = 0;

    // Scaled width and height of the panel
    private float m_MaximumWidth;
    private float m_MaximumHeight;

    /// <summary>
    /// rectangle containing size, position & rotation (local space)
    /// </summary>
    private RectTransform m_RectTransform;

    #endregion // PRIVATE_VARIABLES

    #region UNITY_MONOBEHAVIOR_METHODS

    /// <summary>
    /// Destroy the graph when attached gameObject is disabled.
    /// </summary>
    void OnDisable()
    {
        Pause();
    }

    /// <summary>
    /// Play the graph when the gameObject is enabled and initialized.
    /// </summary>
    void OnEnable()
    {
        if (m_Initialized && !m_Playing)
        {
            Play();
        }
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

    ///// <summary> TODO: scaling not working as of now -> manual scale
    ///// event, called by unity as soon as RectTransform Component recognised changed. 
    ///// Updates m_MaximumWidth and m_MaximumHeight for graph plotter
    ///// </summary>
    //void OnRectTransformDimensionsChange()
    //{

    //    if (m_RectTransform)
    //    {
    //        m_MaximumWidth = (m_RectTransform.rect.width - BorderLeft - BorderRight);
    //        m_MaximumHeight = (m_RectTransform.rect.height - BorderBottom - BorderTop);

    //        m_StepSize = m_MaximumWidth / ((float)m_NumPoints - 1);
    //        m_MaximumWidth = (m_RectTransform.rect.width - BorderLeft - BorderRight);

    //    }
    //}

    #endregion // UNITY_MONOBEHAVIOR_METHODS

    #region PUBLIC_METHODS

    /// <summary>
    /// Initialize all the internal parameters.
    /// Default y axis range = (-1,1);
    /// default scale of y axis: false
    /// </summary>
    /// <param name="valueList"></param>
    /// <param name="numPoints"></param>
    public void Initialize(List<float> valueList, int numPoints)
    {
        if (!m_Initialized)
        {
            // Debug.Log("Initializing graph renderer");
            // Intialize the values list with values from the given list or set them to zero if numPoints exceeds the list length
            m_Values = valueList;
            // Get the rect transform component
            m_RectTransform = GetComponent<RectTransform>();
            if (!m_RectTransform)
            {
                m_RectTransform = gameObject.AddComponent<RectTransform>();
            }
            // Get the maximum scaled width and height
            m_MaximumWidth = (m_RectTransform.rect.width - BorderLeft - BorderRight);
            m_MaximumHeight = (m_RectTransform.rect.height - BorderBottom - BorderTop);

            // Number of points of the graph
            m_NumPoints = numPoints;
            m_StepSize = m_MaximumWidth / ((float)numPoints - 1);

            m_Values = valueList;

            if (m_Values.Count < m_NumPoints) //need to add additional values if count smaller than requested data
            {
                int additions = m_NumPoints - m_Values.Count;
                m_Values.AddRange(Enumerable.Repeat(m_DefaultValue, additions).ToList());
            }
            for (int i = 0; i < numPoints; i++)
            {
                // Stick the graph to panel movement
                Vector3 pos = getGraphPositionAtIndex(i);
                m_Positions.Add(pos);
            }

            if (GetComponent<LineRenderer>() == null) // if no component created yet
            {
                // Create a LineRenderer component and attach it with the given parameters
                m_OscillatorLineRenderer = gameObject.AddComponent<LineRenderer>();
                m_OscillatorLineRenderer.useWorldSpace = false;
                m_OscillatorLineRenderer.positionCount = m_NumPoints;
                m_OscillatorLineRenderer.material = GraphMaterial;
                m_OscillatorLineRenderer.startWidth = m_OscillatorLineRenderer.endWidth = 0.005f;
                m_OscillatorLineRenderer.SetPositions(m_Positions.ToArray());
            }
            if (m_YAxisRange.Equals(Vector2.zero)) m_YAxisRange = new Vector2(-1, 1);
            m_Initialized = true;
        }
    }
    /// <summary>
    /// Starts / Resumes the graph.
    /// </summary>
    public void Play()
    {
        //Debug.Log("Graph renderer: Play called");
        //TODO: apparently concurrency issues -> lock?
        if (m_Initialized && !m_Playing)
        {
            m_Playing = true;
            if (m_PlayCoroutine == null) // if no coroutine running yet
            {
                // Update graph position each frame
                m_PlayCoroutine = playCoroutine();
                // Update graph values each timestep
                StartCoroutine(m_PlayCoroutine);
            }
            else
            {
                StartCoroutine(m_PlayCoroutine);
            }
        }
        if (!m_Initialized)
        {
            Debug.Log("Graph not initialized yet! Call the initialize function first!");
        }

    }

    /// <summary>
    /// Pauses the graph value updates.
    /// </summary>
    public void Pause()
    {
        if (m_Initialized && m_Playing)
        {
            //Debug.Log("Graph renderer Pause");
            m_Playing = false;
        }
    }
    /// <summary>
    /// Stops and destroys the graph.
    /// </summary>
    public void Stop()
    {
        if (m_Initialized) //if not stopped already
        {
            // Kill linerenderer, stop the coroutines
            if (m_PlayCoroutine != null)
            {
                StopCoroutine(m_PlayCoroutine);
                m_PlayCoroutine = null;
            }

            Destroy(m_OscillatorLineRenderer);
            // m_CurrentState = State.None;
            m_Playing = false;
            m_Initialized = false;

            Debug.Log("Graph renderer Stopped");
        }
    }

    /// <summary>
    /// Changes the number of points to be plotted to the given number.
    /// </summary>
    /// <param name="numPoints">The new size of the graph</param>
    public void ChangeGraphPointNumber(int numPoints)
    {
        if (m_Initialized)
        {
            if (numPoints == m_NumPoints)
                return;

            m_NumPoints = numPoints;
            m_OscillatorLineRenderer.positionCount = numPoints;
            m_StepSize = m_MaximumWidth / ((float)numPoints - 1);

            int deltaSize = numPoints - m_Values.Count;

            // create new 0 values for the values list so we dont get a null reference in the coroutines
            if (deltaSize > 0)
            {
                m_Values.AddRange(Enumerable.Repeat(m_DefaultValue, deltaSize).ToList());
            }
            else
            {
                m_Values.RemoveRange(0, -deltaSize);
            }
        }
        else
        {
            Debug.Log("Please initialize first before changing graph size");
        }
    }

    /// <summary>
    /// changes the graph to not be dynamically adjusted depending on its current values, uses manual y axis value then
    /// </summary>
    public void SetNoAdjustment()
    {
        m_AdjustAxis = false;
    }

    /// <summary>
    /// changes graph to automatically scale y axis ´depending on current values
    /// </summary>
    public void SetAutomaticAdjust()
    {
        m_AdjustAxis = true;
    }

    /// <summary>
    /// Sets range in for which the y values will be displayed
    /// </summary>
    /// <param name="range"></param>
    public void SetManualAdjust(Vector2 range)
    {
        if (range.x >= range.y)
        {
            Debug.Log("Your given minimum value is bigger or equal to the maximum!");
            return;
        }
        m_YAxisRange = range;
        m_AdjustAxis = false;
    }

    /// <summary>
    /// Sets range in for which the y values will be displayed
    /// </summary>
    /// <param name="range"></param>
    public void SetManualAdjust(float min, float max)
    {
        if (min >= max)
        {
            Debug.Log("Your given minimum value is bigger or equal to the maximum!");
            return;
        }
        m_YAxisRange = new Vector2(min, max);
        m_AdjustAxis = false;
    }

    /// <summary>
    /// Sets default value which is being used to initialize missing points
    /// </summary>
    /// <param name="val">default value</param>
    public void SetDefaultValue(float val)
    {
        m_DefaultValue = val;
    }

    /// <summary>
    /// returns, whether the coroutines are being executed and state is set to play. 
    /// </summary>
    /// <returns>playing (true), not playing(false)</returns>
    public bool IsPlaying()
    {
        return m_Playing;
    }
    #endregion // PUBLIC_METHODS

    #region PRIVATE_METHODS

    /// <summary>
    /// Renders the graph according to panel movement.
    /// </summary>
    /// <returns></returns>
    private IEnumerator playCoroutine()
    {
        while (m_Initialized)
        {
            // Debug.Log("playCoroutine");

            m_Positions.Clear();

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
        if (!m_Initialized)
        {
            Debug.Log("Graph is not initialized! Initialize the graph first");
        }
    }


    /// <summary>
    /// Transforms point from local space to panel space
    /// </summary>
    /// <param name="index"></param>
    /// <returns></returns>
    Vector3 getGraphPositionAtIndex(int index)
    {
        Vector3 pos;
        if (m_AdjustAxis)
        {//adapt range depending on 
            m_YAxisRange = new Vector2(m_Values.Min(), m_Values.Max());
        }
        //check for boundaries
        float temp = m_Values[index];
        if (temp < m_YAxisRange.x) temp = m_YAxisRange.x;
        if (temp > m_YAxisRange.y) temp = m_YAxisRange.y;
        //adjust to range
        float y = m_MaximumHeight * (temp - m_YAxisRange.x) / (m_YAxisRange.y - m_YAxisRange.x); // size * (my_dist / overall_dist)
        pos = new Vector3(index * m_StepSize, y, m_ZDistance);

        pos -= m_RectTransform.pivot.x * m_MaximumWidth * Vector3.right;
        pos -= m_RectTransform.pivot.y * m_MaximumHeight * Vector3.up;
        Vector2 pivotDifference = Vector2.one * 0.5f - m_RectTransform.pivot;

        pos += new Vector3(m_MaximumWidth * pivotDifference.x + ((BorderLeft - BorderRight) * 0.5f), m_MaximumHeight * pivotDifference.y + ((BorderBottom - BorderTop) * 0.5f), 0f);
        return pos;
    }

    /// <summary>
    /// sets the colour of the respective graph. Must be set after initialization
    /// </summary>
    /// <param name="color"></param>
    public void SetColour(Color color)
    {
        if (m_OscillatorLineRenderer)
            m_OscillatorLineRenderer.material.color = color;
    }
    #endregion // PRIVATE_METHODS
}
