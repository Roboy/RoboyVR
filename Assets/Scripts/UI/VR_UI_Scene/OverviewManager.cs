using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// INSERT CLASS NAME. 
/// This is the long description separated from the short description with a .
/// </summary>
public class OverviewManager : MonoBehaviour
{
    #region PUBLIC_MEMBER_VARIABLES
    #endregion

    #region PRIVATE_MEMBER_VARIABLES
    /// <summary>
    /// graph displaying a heartbeat, linked to the screen
    /// </summary>
    private GraphObject heart;
    /// <summary>
    /// screen of this mode
    /// </summary>
    [SerializeField]
    private Canvas screen;

    private bool testing = false;
    /* For test purposes to adjust heartbeat
     *  #region heartbeat values
     [SerializeField]
     private float a= 0.2f;
     [SerializeField]
     private float d= 1.4f;
     [SerializeField]
     private float h= 3;
     [SerializeField]
     private float s= 0.05f;
     [SerializeField]
     private float w= 0.02f;
     #endregion*/
    #endregion

    #region UNITY_MONOBEHAVIOUR_METHODS
    /// <summary>
    /// Called once by Unity during startup
    /// </summary>
    void Awake()
    {

        GameObject obj = new GameObject();
        obj.name = "heartbeat";
        obj.transform.parent = screen.transform; //linked to screen
        obj.transform.localScale = Vector3.one;
        obj.transform.localPosition = Vector3.zero;
        obj.transform.localRotation = Quaternion.identity;
        //add graph
        obj.AddComponent<GraphObject>();
        heart = obj.GetComponent<GraphObject>();

        heart.SetDefaultValue(2);
        heart.SetNoAdjustment();
        heart.SetYAxisRange(new Vector2(0, 4));
        heart.Run(null, 200, 1 / 200);
    }

    /// <summary>
    /// do stuff as soon as enabled again
    /// </summary>
    void OnEnable()
    {
        if (heart)
        {
            heart.Resume();
        }
    }
    void OnDisable()
    {
        if (heart)
        {
            heart.Pause();
        }
    }
    /// <summary>
    /// Called once every frame
    /// </summary>
    void Update()
    {
        //test graph renderer behaviour
        if (!testing)
        {
            testing = true;
            StartCoroutine(test());
        }
        heart.AddValue(GetBeat());
    }
    #endregion

    #region PUBLIC_METHODS
    /// <summary>
    /// coroutine to test graph pause and resume functions / behaviour. pauses and restarts after short amount of waiting time
    /// </summary>
    /// <returns></returns>
    IEnumerator test()
    {
        yield return new WaitForSeconds(3);
        heart.Pause();

        yield return new WaitForSeconds(2f);
        heart.Resume();
        testing = false;
    }
    #endregion

    #region PRIVATE_METHODS

    /// <summary>
    /// returns heartbeat value of current time step
    /// </summary>
    /// <returns></returns>
    private float GetBeat()
    {

        //create heartbeat function
        float L = 3f; // factor when to loop function again
        float x = Time.time;
        x = x - Mathf.Ceil(x / L - 0.5f) * L;
        /* a lot of adjusting parameters */
        float a = 2f;
        float d = 0.4f;
        float h = 2;
        float s = 0.05f;
        float w = 0.03f;
        float e = Mathf.Exp(1);

        if (L < 2 * d) // must be met
        {
            Debug.Log("some issues with heartbeat period");
        }

        float y_1 = a * Mathf.Pow(e, (-Mathf.Pow(x + d, 2) / (2 * w)));
        float y_2 = Mathf.Pow(e, (-Mathf.Pow(x - d, 2) / (2 * w)));
        float y_3 = h - (Mathf.Abs(x / s) - x) * Mathf.Pow(e, (-Mathf.Pow(7 * x, 2) / 2));
        float y = y_1 + y_2 + y_3;

        return y;

    }
    #endregion
}

