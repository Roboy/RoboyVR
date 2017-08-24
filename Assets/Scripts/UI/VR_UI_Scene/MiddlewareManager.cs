using System.Collections;
using UnityEngine;

/// <summary>
/// Provides all functionalities and manages Middleware mode
/// </summary>
public class MiddlewareManager : MonoBehaviour
{
    #region PUBLIC_MEMBER_VARIABLES
    #endregion

    #region PRIVATE_MEMBER_VARIABLES
    private GameObject arm;
    #endregion

    #region UNITY_MONOBEHAVIOUR_METHODS
    /// <summary>
    /// Called once by Unity during startup
    /// </summary>
    void Start()
    {
        Vector3[] points1 = { new Vector3(0.242f, 0.355f, -0.034f), //upper arm left
                              new Vector3(0.2358f, 0.4768f, -0.024f),}; //torso

        Vector3[] points2 = { new Vector3(0.18f, 0.342f, -0.034f), //upper arm left
                              new Vector3(0.1771f, 0.4585f, 0) }; //torso

        Vector3[] points3 = { new Vector3(0.2337f, 0.258f, -0.0643f), //upper arm left
                              new Vector3(0.2204f, 0.1817f, -0.0833f), //lower arm left
                              new Vector3(0.2109f, 0.1362f, -0.1178f), //lower arm left
                              new Vector3(0.2109f, 0.1193f, -0.134f)}; //lower arm left
        string[] names = { "oberarm_left", "torso" };

        string[] names3 = { "oberarm_left", "unterarm_left", "unterarm_left", "unterarm_left" };
        VRUILogic.Instance.AddTendon(0, points1, names, 1f);
        VRUILogic.Instance.AddTendon(1, points2, names, 1f);
        VRUILogic.Instance.AddTendon(2, points3, names3, 1f);
        arm = VRUILogic.Instance.GetBodyPart("unterarm_left");
    }

    /// <summary>
    /// Called once every frame
    /// </summary>
    void Update()
    {
        float x = Mathf.Cos(Time.realtimeSinceStartup) * 0.5f + 0.5f;
        VRUILogic.Instance.UpdateTendon(0, x);
        VRUILogic.Instance.UpdateTendon(1, x);
        VRUILogic.Instance.UpdateTendon(2, x);
        if (arm)
        {
            Vector3 temp = arm.transform.position;
            Vector3 offset = new Vector3();
            float angleparts = 4 / Time.deltaTime; // how many parts fit into 4 secs
            float time = Time.time;
            if(((int)time/4)%2 == 0) //0-4 8-12 ....
            {
                arm.transform.RotateAround(temp + offset, Vector3.up, -90 / angleparts);
            }
            else //4-8, 12-16 ....
            {
                arm.transform.RotateAround(temp + offset, Vector3.up, 90 / angleparts);
            }
        }
    }
    #endregion

    #region PUBLIC_METHODS
    #endregion

    #region PRIVATE_METHODS

    #endregion
}