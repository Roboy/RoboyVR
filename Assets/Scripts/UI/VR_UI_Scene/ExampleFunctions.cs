using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ExampleFunctions : MonoBehaviour {

    #region PUBLIC_MEMBER_VARIABLES
    #endregion

    #region PRIVATE_MEMBER_VARIABLES   
    /// <summary>
    /// Reference needed to continuously rotate arm
    /// </summary>
    private GameObject m_Arm;

    /// <summary>
    /// Boolean managing continuous notification spawn (for coroutine management)
    /// </summary>
    private bool testing = false;
    #endregion

    #region UNITY_MONOBEHAVIOUR_METHODS

    /// <summary>
    /// initializes tendons and so on
    /// </summary>
    void Start()
    {
        //TENDON EXAMPLE
        Vector3[] points1 = { new Vector3(0.242f, 0.355f, -0.034f), //upper arm left
                              new Vector3(0.2358f, 0.4768f, -0.024f),}; //torso

        Vector3[] points2 = { new Vector3(0.18f, 0.342f, -0.034f), //upper arm left
                              new Vector3(0.1771f, 0.4585f, 0) }; //torso

        Vector3[] points3 = { new Vector3(0.2337f, 0.258f, -0.0643f), //upper arm left
                              new Vector3(0.2204f, 0.1817f, -0.0833f), //lower arm left
                              new Vector3(0.2109f, 0.1362f, -0.1178f), //lower arm left
                              new Vector3(0.2109f, 0.1193f, -0.134f)}; //lower arm left

        Vector3[] points4 = { new Vector3(0.1814f, 0.274f, -0.047f), //upper arm left
                              new Vector3(0.1814f, 0.164f, -0.072f), //lower arm left
                              new Vector3(0.1814f, 0.1287f, -0.102f), //lower arm left
                              new Vector3(0.1814f, 0.0947f, -0.157f)}; //lower arm left
        string[] names = { "oberarm_left", "torso" };

        string[] names3 = { "oberarm_left", "unterarm_left", "unterarm_left", "unterarm_left" };
        VRUILogic.Instance.AddTendon(0, points1, names, 1f);
        VRUILogic.Instance.AddTendon(1, points2, names, 1f);
        VRUILogic.Instance.AddTendon(2, points3, names3, 1f);
        VRUILogic.Instance.AddTendon(3, points4, names3, 1f);
        m_Arm = VRUILogic.Instance.GetBodyPart("unterarm_left");

    }

    /// <summary>
    /// Called once every frame
    /// </summary>
    void Update()
    {
        //TENDON EXAMPLE
        float x = Mathf.Cos(Time.realtimeSinceStartup * 8 / (2 * 3.14159f)) * 0.5f + 0.5f; //amplitude: 0-1, period length = 8
        VRUILogic.Instance.UpdateTendon(0, x);
        VRUILogic.Instance.UpdateTendon(1, x);
        VRUILogic.Instance.UpdateTendon(2, 1-x);
        VRUILogic.Instance.UpdateTendon(3, 1 - x);

        if (m_Arm)
        {
            Vector3 temp = m_Arm.transform.position;
            Vector3 offset = new Vector3();
            float angleparts = 4 / Time.deltaTime; // how many parts fit into 4 secs
            float time = Time.time;
            if (((int)time / 4) % 2 == 0) //0-4 8-12 .... for first 4 secs turn arm outwards
            {
                m_Arm.transform.RotateAround(temp + offset, Vector3.up, -90 / angleparts);
            }
            else //4-8, 12-16 .... for next 4 secs turn arm back to original pos
            {
                m_Arm.transform.RotateAround(temp + offset, Vector3.up, 90 / angleparts);
            }
        }
        //NOTIFICATIONS
        //test notifications display
        if (!testing)
        {
            testing = true;
            StartCoroutine(test());
        }

    }
    #endregion

    #region PUBLIC_METHODS
    #endregion

    #region PRIVATE_METHODS
    /// <summary>
    /// used for testing of notifications display
    /// </summary>
    /// <returns></returns>
    private IEnumerator test()
    {


        yield return new WaitForSeconds(2);
        VRUILogic.Instance.AddNewNotification(DummyStates.MessageType.WARNING, DummyStates.State.MOTOR_DEAD, "oberarm_right", 3);
        //Debug.Log("Added Warning");

        yield return new WaitForSeconds(4);
        VRUILogic.Instance.AddNewNotification(DummyStates.MessageType.ERROR, DummyStates.State.MOTOR_DEAD, "hip", 5);
        //Debug.Log("Added Error");

        yield return new WaitForSeconds(2);
        VRUILogic.Instance.AddNewNotification(DummyStates.MessageType.WARNING, DummyStates.State.MOTOR_DEAD, "foot_left", 5);
        //Debug.Log("Added Debug");

        yield return new WaitForSeconds(2);
        VRUILogic.Instance.AddNewNotification(DummyStates.MessageType.ERROR, DummyStates.State.MOTOR_DEAD, "head", 5f);
        // Debug.Log("Added debug");
        testing = false;
    }
    #endregion
}

