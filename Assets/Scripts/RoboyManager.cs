using System;
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using ROSBridgeLib;


public class RoboyManager : Singleton<RoboyManager> {


    #region PUBLIC_MEMBER_VARIABLES

    public string VM_IP = "";

    public GameObject[] RoboyParts;

    #endregion //PUBLIC_MEMBER_VARIABLES

    #region PRIVATE_MEMBER_VARIABLES

    private ROSBridgeWebSocketConnection m_Ros = null;
    private RoboyPoseMsg m_RoboyPoseMessage;

    private Dictionary<string, GameObject> m_RoboyParts = new Dictionary<string, GameObject>();

    #endregion //PRIVATE_MEMBER_VARIABLES

    #region MONOBEHAVIOR_METHODS

    void Start()
    {
        if (string.IsNullOrEmpty(VM_IP))
            return;

        m_Ros = new ROSBridgeWebSocketConnection("ws://" + VM_IP, 9090);

        m_Ros.AddSubscriber(typeof(RoboyPoseSubscriber));
        m_Ros.AddServiceResponse(typeof(RoboyServiceResponse));

        m_Ros.Connect();

        foreach (GameObject g in RoboyParts)
        {
            if(g == null | !g.CompareTag("RoboyPart"))
                continue;
            m_RoboyParts.Add(g.name, g);
        }
    }

    void Update()
    {
        m_Ros.Render();

        if (Input.GetKeyDown(KeyCode.A))
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

        Dictionary<string, float> rollRotationsDictionary = m_RoboyPoseMessage.RollDic;
        Dictionary<string, float> pitchRotationsDictionary = m_RoboyPoseMessage.PitchDic;
        Dictionary<string, float> yawRotationsDictionary = m_RoboyPoseMessage.YawDic;

        Dictionary<string, float> qxRotationsDictionary = m_RoboyPoseMessage.QxDic;
        Dictionary<string, float> qyRotationsDictionary = m_RoboyPoseMessage.QyDic;
        Dictionary<string, float> qzRotationsDictionary = m_RoboyPoseMessage.QzDic;
        Dictionary<string, float> qwRotationsDictionary = m_RoboyPoseMessage.QwDic;
        Dictionary<string, float> v00Dictionary = m_RoboyPoseMessage.V00Dic;
        Dictionary<string, float> v01Dictionary = m_RoboyPoseMessage.V01Dic;
        Dictionary<string, float> v02Dictionary = m_RoboyPoseMessage.V02Dic;
        Dictionary<string, float> v03Dictionary = m_RoboyPoseMessage.V03Dic;
        Dictionary<string, float> v10Dictionary = m_RoboyPoseMessage.V10Dic;
        Dictionary<string, float> v11Dictionary = m_RoboyPoseMessage.V11Dic;
        Dictionary<string, float> v12Dictionary = m_RoboyPoseMessage.V12Dic;
        Dictionary<string, float> v13Dictionary = m_RoboyPoseMessage.V13Dic;
        Dictionary<string, float> v20Dictionary = m_RoboyPoseMessage.V20Dic;
        Dictionary<string, float> v21Dictionary = m_RoboyPoseMessage.V21Dic;
        Dictionary<string, float> v22Dictionary = m_RoboyPoseMessage.V22Dic;
        Dictionary<string, float> v23Dictionary = m_RoboyPoseMessage.V23Dic;
        Dictionary<string, float> v30Dictionary = m_RoboyPoseMessage.V30Dic;
        Dictionary<string, float> v31Dictionary = m_RoboyPoseMessage.V31Dic;
        Dictionary<string, float> v32Dictionary = m_RoboyPoseMessage.V32Dic;
        Dictionary<string, float> v33Dictionary = m_RoboyPoseMessage.V33Dic;


        foreach (KeyValuePair<string, GameObject> roboyPart in m_RoboyParts)
        {
            string index = roboyPart.Key;
            Vector3 originPosition = new Vector3(xPositionsDictionary[index], yPositionsDictionary[index], zPositionsDictionary[index]);
            Quaternion originRotation = new Quaternion(qxRotationsDictionary[index], qyRotationsDictionary[index], qzRotationsDictionary[index], qwRotationsDictionary[index]);

            //if(index.Equals("torso"))
            //    Debug.Log(string.Format("X: {0}, Y: {1}, Z: {2}", originRotation.eulerAngles.x, originRotation.eulerAngles.y, originRotation.eulerAngles.z));

            roboyPart.Value.transform.localPosition = gazeboPositionToUnity(originPosition);
            roboyPart.Value.transform.localRotation = gazeboRotationToUnity(originRotation);


        }

    }

    Quaternion gazeboRotationToUnity(Quaternion gazeboRot)
    {
        Quaternion rotX = Quaternion.AngleAxis(-90f, Vector3.right);
        Quaternion rotY = Quaternion.AngleAxis(90f, Vector3.up);

        Quaternion inverse = new Quaternion(gazeboRot.x, gazeboRot.y, gazeboRot.z, gazeboRot.w);

        Quaternion leftHandedRotation = inverse;

        return  rotY * rotX * gazeboRot;


    }

    Vector3 gazeboPositionToUnity(Vector3 gazeboPos)
    {
        Quaternion rotX = Quaternion.AngleAxis(-90f, Vector3.right);
        Quaternion rotY = Quaternion.AngleAxis(90f, Vector3.up);

        gazeboPos = rotY * rotX * gazeboPos;
        gazeboPos.z *= -1f;

        return gazeboPos;
    }

    #endregion //PUBLIC_METHODS

    #region PRIVATE_METHODS

    #endregion //PRIVATE_METHODS
}
