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

    /// <summary>
    /// Public property to create a text object to show the current value of the graph.
    /// </summary>
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

    /// <summary>
    /// Text object for the current value. Is only there if <see cref="ShowCurrentValue"/> is true. 
    /// </summary>
    public Text TextForValueName
    {
        get
        {
            if (m_TextForValueName)
                return m_TextForValueName.GetComponent<Text>();
            return null;
        }
    }

    /// <summary>
    /// Public property to enable a text object to show the value name of the current value.
    /// </summary>
    [ExposeProperty]
    public bool ShowValueName
    {
        get { return m_ShowValueName; }
        set
        {
            if (m_ShowValueName != value)
            {
                m_ShowValueName = value;

                if (m_ShowValueName)
                    createTextfieldForValueName();
                else
                    destroyTextfieldForValueName();
            }
        }
    }

    #endregion // PUBLIC_VARIABLES

    #region PRIVATE_VARIABLES

    [HideInInspector]
    [SerializeField]
    private bool m_ShowCurrentValue = false;

    [HideInInspector]
    [SerializeField]
    private bool m_ShowValueName = false;

    [HideInInspector]
    [SerializeField]
    private GameObject m_TextForCurrentValue;

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
    /// indicates whether the graph curve is updating
    /// </summary>
    private bool playing = false;
    /// <summary>
    /// indicates whether all required objects and values are set
    /// </summary>
    private bool initialized = false;

    /// <summary>
    /// defines whether axis should be adjusted 
    /// </summary>
    private bool adjustAxis = false;

    /// <summary>
    /// If axis not automatically adjusted, this range will be used. 
    /// x is the lower bound, y the upper. 
    /// </summary>
    private Vector2 yAxisRange = Vector2.zero;

    /// <summary>
    /// Sets the distance from the graph to the canvas / panel
    /// </summary>
    private float zdistance = -0.1f;

    /// <summary>
    /// The last updated values and positions in 3D of the graph
    /// since not primitive -> pointer -> changes can be applied by other methods with same pointer
    /// </summary>
    private List<float> m_Values;

    /// <summary>
    /// List of positions of graph points??? (No clue)
    /// </summary>
    private List<Vector3> m_Positions = new List<Vector3>();

    /// <summary>
    /// default value that is used to initialize points on graph
    /// </summary>
    private float defaultVal = 0f;

    // Coroutines to handle the play process
    private IEnumerator m_PlayCoroutine;
    private IEnumerator m_UpdateValuesCoroutine;

    // Distance between each point
    private float m_StepSize = 0f;

    // Number of points the graph displays
    private int m_NumPoints = 0;

    // Scaled width and height of the panel
    private float m_MaximumWidth;
    private float m_MaximumHeight;

    private RectTransform m_RectTransform;

    // How often we update the graph`s values
    private float m_TimeStep;
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
        if (initialized && !playing)
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

    #endregion // UNITY_MONOBEHAVIOR_METHODS

    #region PUBLIC_METHODS

    /// <summary>
    /// Initialize all the internal parameters.
    /// Default y axis range = (-1,1);
    /// default scale of y axis: false
    /// </summary>
    /// <param name="valueList"></param>
    /// <param name="numPoints"></param>
    /// <param name="timeStep"></param>
    public void Initialize(List<float> valueList, int numPoints, float timeStep)
    {
        if (!initialized)
        {
            Debug.Log("Initializing graph");
            // Intialize the values list with values from the given list or set them to zero if numPoints exceeds the list length
            m_Values = valueList;
            // Get the rect transform component
            m_RectTransform = GetComponent<RectTransform>();

            // Get the maximum scaled width and height
            m_MaximumWidth = (m_RectTransform.rect.width - BorderLeft - BorderRight);
            m_MaximumHeight = (m_RectTransform.rect.height - BorderBottom - BorderTop);

            // Number of points of the graph
            m_NumPoints = numPoints;
            m_StepSize = m_MaximumWidth / ((float)numPoints - 1);

            m_Values = valueList;
            m_TimeStep = timeStep;

            if (m_Values.Count < m_NumPoints) //need to add additional values if count smaller than requested data
            {
                int additions = m_NumPoints - m_Values.Count;
                m_Values.AddRange(Enumerable.Repeat(defaultVal, additions).ToList());
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
            if (yAxisRange.Equals(Vector2.zero)) yAxisRange = new Vector2(-1, 1);
            initialized = true;
        }
    }
    /// <summary>
    /// Starts / Resumes the graph.
    /// </summary>
    public void Play()
    {
        //TODO: apparently concurrency issues -> lock?
        if (initialized && !playing)
        {
            playing = true;
            if (m_PlayCoroutine == null) // if no coroutine running yet
            {
                // Update graph position each frame
                m_PlayCoroutine = playCoroutine();
                // Update graph values each timestep
                m_UpdateValuesCoroutine = updateValuesCoroutine();
                StartCoroutine(m_PlayCoroutine);
                StartCoroutine(m_UpdateValuesCoroutine);
            }
            else
            {//when paused, this coroutine is deleted 
                m_UpdateValuesCoroutine = updateValuesCoroutine();
                StartCoroutine(m_UpdateValuesCoroutine);
                StartCoroutine(m_PlayCoroutine);
            }
        }
        if (!initialized)
        {
            Debug.Log("Graph not initialized yet! Call the initialize function first!");
        }

    }

    /// <summary>
    /// Pauses the graph value updates.
    /// </summary>
    public void Pause()
    {
        if (initialized && playing)
        {
            //Debug.Log("Pause");

            // Stop updating the values
            StopCoroutine(m_UpdateValuesCoroutine);
            m_UpdateValuesCoroutine = null;
            //m_OscillatorLineRenderer.enabled = false;
            playing = false;
        }
    }
    /// <summary>
    /// Stops and destroys the graph.
    /// </summary>
    public void Stop()
    {
        if (initialized) //if not stopped already
        {
            // Kill linerenderer, stop the coroutines
            if (m_PlayCoroutine != null)
                StopCoroutine(m_PlayCoroutine);
            if (m_UpdateValuesCoroutine != null)
                StopCoroutine(m_UpdateValuesCoroutine);
            m_UpdateValuesCoroutine = null;
            m_PlayCoroutine = null;
            Destroy(m_OscillatorLineRenderer);
            // m_CurrentState = State.None;
            playing = false;
            initialized = false;

            Debug.Log("Graph renderer Stopped");
        }
    }

    /// <summary>
    /// Changes the graph size to the given size.
    /// </summary>
    /// <param name="numPoints">The new size of the graph</param>
    public void ChangeGraphSize(int numPoints)
    {
        if (initialized)
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
                 m_Values.AddRange(Enumerable.Repeat(defaultVal, deltaSize).ToList());
            }
            else
            {
                m_Values.RemoveRange(numPoints, -deltaSize);
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
        adjustAxis = false;
    }

    /// <summary>
    /// changes graph to automatically scale y axis ´depending on current values
    /// </summary>
    public void SetAutomaticAdjust()
    {
        adjustAxis = true;
    }

    /// <summary>
    /// sets range in for which the y values will be displayed
    /// </summary>
    /// <param name="range"></param>
    public void SetYAxisRange(Vector2 range)
    {
        yAxisRange = range;
    }

    /// <summary>
    /// sets default value which is being used to initialize missing points
    /// </summary>
    /// <param name="val">default value</param>
    public void SetDefaultValue(float val)
    {
        defaultVal = val;
    }

    /// <summary>
    /// returns, whether the coroutines are being executed and state is set to play. 
    /// </summary>
    /// <returns>playing (true), not playing(false)</returns>
    public bool IsPlaying(){
        return playing;
    }
    #endregion // PUBLIC_METHODS

    #region PRIVATE_METHODS

    /// <summary>
    /// Renders the graph according to panel movement.
    /// </summary>
    /// <returns></returns>
    private IEnumerator playCoroutine()
    {
        while (initialized)
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
        if (!initialized)
        {
            Debug.Log("Graph is not initialized! Initialize the graph first");
        }
    }

    /// <summary>
    /// Refreshes value list each timestamp. Fills List if count smaller than requested elems. Uses defined default value 
    /// </summary>
    /// <returns></returns>
    private IEnumerator updateValuesCoroutine()
    {
        while (playing)
        {
            //Debug.Log("updateValuesCoroutine");

            if (m_Values.Count < m_NumPoints) //need to add additional values
            {
                int additions = m_NumPoints - m_Values.Count;
                m_Values.AddRange(Enumerable.Repeat(defaultVal, additions).ToList());
            }

            if (m_ShowCurrentValue && m_TextForCurrentValue)
            {
                Text tmp = m_TextForCurrentValue.GetComponent<Text>();
                if(tmp) tmp.text = m_Values[0].ToString("n2");
            }
            yield return new WaitForSeconds(m_TimeStep);
        }
        // Print warning to console if this function is called manually when graph is not playing
        if (!playing)
        {
            Debug.Log("Graph is not playing! Cannot update the values!");
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
        if (adjustAxis)
        {//adapt range depending on 
            yAxisRange = new Vector2(m_Values.Min(), m_Values.Max());
        }
        //check for boundaries
        float temp = m_Values[index];
        if (temp < yAxisRange.x) temp = yAxisRange.x;
        if (temp > yAxisRange.y) temp = yAxisRange.y;
        //adjust to range
        float y = m_MaximumHeight * (temp - yAxisRange.x) / (yAxisRange.y - yAxisRange.x); // size * (my_dist / overall_dist)
        pos = new Vector3(index * m_StepSize, y, zdistance);

        pos -= m_RectTransform.pivot.x * m_MaximumWidth * Vector3.right;
        pos -= m_RectTransform.pivot.y * m_MaximumHeight * Vector3.up;
        Vector2 pivotDifference = Vector2.one * 0.5f - m_RectTransform.pivot;

        pos += new Vector3(m_MaximumWidth * pivotDifference.x + ((BorderLeft - BorderRight) * 0.5f), m_MaximumHeight * pivotDifference.y + ((BorderBottom - BorderTop) * 0.5f), 0f);
        return pos;
    }

    /// <summary>
    /// Creates a text field object for the current value. Set as child to this gameObject.
    /// </summary>
    private void createTextfieldForCurrentValue()
    {
        m_TextForCurrentValue = new GameObject();
        m_TextForCurrentValue.name = "Current Value";
        m_TextForCurrentValue.transform.parent = transform;
        m_TextForCurrentValue.transform.localScale = transform.localScale;
        m_TextForCurrentValue.transform.localPosition = Vector3.zero;
        m_TextForCurrentValue.transform.localRotation = Quaternion.identity;

        object temp = m_TextForCurrentValue.AddComponent<Text>();
    }

    /// <summary>
    /// Destroys the text field object.
    /// </summary>
    private void destroyTextfieldForCurrentValue()
    {
        DestroyImmediate(m_TextForCurrentValue);
    }

    /// <summary>
    /// Creates a text field for the current value name. Set as child to this gameObject.
    /// </summary>
    private void createTextfieldForValueName()
    {
        m_TextForValueName = new GameObject();
        m_TextForValueName.name = "Value Name";
        m_TextForValueName.transform.parent = transform;
        m_TextForValueName.transform.localScale = transform.localScale;
        m_TextForValueName.transform.localPosition = Vector3.zero;
        m_TextForValueName.transform.localRotation = Quaternion.identity;
        m_TextForValueName.AddComponent<Text>();
    }

    /// <summary>
    /// Destroys the text field object for the value name.
    /// </summary>
    private void destroyTextfieldForValueName()
    {
        DestroyImmediate(m_TextForValueName);
    }

    #endregion // PRIVATE_METHODS
}
