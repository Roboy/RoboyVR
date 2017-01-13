using System;
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using ROSBridgeLib;


public class RoboyManager : Singleton<RoboyManager> {


    #region PUBLIC_MEMBER_VARIABLES

    public string VM_IP = "";

    public Transform Roboy;

    #endregion //PUBLIC_MEMBER_VARIABLES

    #region PRIVATE_MEMBER_VARIABLES

    private ROSBridgeWebSocketConnection m_Ros = null;
    private RoboyPoseMsg m_RoboyPoseMessage;

    private Dictionary<string, Transform> m_RoboyParts = new Dictionary<string, Transform>();

    #endregion //PRIVATE_MEMBER_VARIABLES

    #region MONOBEHAVIOR_METHODS

    void Start()
    {
        if (string.IsNullOrEmpty(VM_IP))
            return;

        m_Ros = new ROSBridgeWebSocketConnection("ws://" + VM_IP, 9090);

        m_Ros.AddSubscriber(typeof(RoboyPoseSubscriber));
        m_Ros.AddServiceResponse(typeof(RoboyServiceResponse));
        m_Ros.AddPublisher(typeof(RoboyPosePublisher));

        m_Ros.Connect();

        foreach (Transform t in Roboy)
        {
            if(t == null | !t.CompareTag("RoboyPart"))
                continue;
            m_RoboyParts.Add(t.name, t);
        }

    }

    void Update()
    {
        m_Ros.Render();

        if (Input.GetKeyDown(KeyCode.R))
            m_Ros.CallService("/roboy/reset_world", "");
    }

    void OnApplicationQuit()
    {
        if (m_Ros != null)
            m_Ros.Disconnect();
    }
    #endregion //MONOBEHAVIOR_METHODS

    #region PUBLIC_METHODS

    public void ReceiveMessage(RoboyPoseMsg msg)
    {
        Debug.Log("Received message");

        m_RoboyPoseMessage = msg;

        Dictionary<string, float> xPositionsDictionary = m_RoboyPoseMessage.XDic;
        Dictionary<string, float> yPositionsDictionary = m_RoboyPoseMessage.YDic;
        Dictionary<string, float> zPositionsDictionary = m_RoboyPoseMessage.ZDic;

        Dictionary<string, float> qxRotationsDictionary = m_RoboyPoseMessage.QxDic;
        Dictionary<string, float> qyRotationsDictionary = m_RoboyPoseMessage.QyDic;
        Dictionary<string, float> qzRotationsDictionary = m_RoboyPoseMessage.QzDic;
        Dictionary<string, float> qwRotationsDictionary = m_RoboyPoseMessage.QwDic;

        foreach (KeyValuePair<string, Transform> roboyPart in m_RoboyParts)
        {
            string index = roboyPart.Key;
            Vector3 originPosition = new Vector3(xPositionsDictionary[index], yPositionsDictionary[index], zPositionsDictionary[index]);
            Quaternion originRotation = new Quaternion(qxRotationsDictionary[index], qyRotationsDictionary[index], qzRotationsDictionary[index], qwRotationsDictionary[index]);

            roboyPart.Value.localPosition = gazeboPositionToUnity(originPosition);
            roboyPart.Value.localRotation = gazeboRotationToUnity(originRotation);


        }

    }

    Quaternion gazeboRotationToUnity(Quaternion gazeboRot)
    {
        Quaternion rotX = Quaternion.AngleAxis(180f, Vector3.right);
        Quaternion rotZ = Quaternion.AngleAxis(180f, Vector3.forward);

        Quaternion tempRot = new Quaternion(-gazeboRot.x, -gazeboRot.z, -gazeboRot.y, gazeboRot.w);

        Quaternion finalRot = tempRot*rotZ*rotX;

        return finalRot;
    }

    Vector3 gazeboPositionToUnity(Vector3 gazeboPos)
    {
        return new Vector3(gazeboPos.x, gazeboPos.z, gazeboPos.y);
    }

    Vector3 unityPositionToGazebo(Vector3 unityPos)
    {
        return new Vector3(unityPos.x, unityPos.z, unityPos.y);
    }

    Quaternion unityRotationToGazebo(Quaternion unityRot)
    {
        Quaternion rotX = Quaternion.AngleAxis(180f, Vector3.right);
        Quaternion rotZ = Quaternion.AngleAxis(180f, Vector3.forward);

        Quaternion tempRot = unityRot*rotX*rotZ;

        Quaternion finalRot = new Quaternion(-tempRot.x, -tempRot.z, -tempRot.y, tempRot.w);

        return finalRot;
    }
    #endregion //PUBLIC_METHODS

    #region PRIVATE_METHODS

    void drawTendons()
    {
        Dictionary<int, List<Vector3>> tendonsDictionary = new Dictionary<int, List<Vector3>>();
        int tendonsCount = 5;
        int tendonsLength = UnityEngine.Random.Range(2, 10);

        for (int i = 0; i < tendonsCount; i++)
        {
            List<Vector3> lv = new List<Vector3>();
            for (int j = 0; j < tendonsLength; j++)
            {
                Vector3 position = new Vector3(UnityEngine.Random.Range(-10.0f, 10.0f), 0, UnityEngine.Random.Range(-10.0f, 10.0f));
                lv.Add(position);
            }
            tendonsDictionary.Add(i, lv);
            tendonsLength = UnityEngine.Random.Range(2, 10);
        }

        foreach (KeyValuePair<int, List<Vector3>> t in tendonsDictionary)
        {
            //Get number of points for linerenderer
            int points = t.Value.Count;

            GameObject g = new GameObject();
            g.AddComponent<LineRenderer>();
            LineRenderer lr = g.GetComponent<LineRenderer>();
            lr.numPositions = points - 1;
            lr.SetPositions(t.Value.ToArray());
            lr.startWidth = lr.endWidth = 0.1f;
        }
    }

    #endregion //PRIVATE_METHODS
}
