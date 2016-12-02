using System;
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using ROSBridgeLib;


public class RoboyManager : Singleton<RoboyManager> {


    #region PUBLIC_MEMBER_VARIABLES

    public string VM_IP = "";

    public GameObject[] RoboyParts;

    public float TEST_ROTATION;

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

        Vector3 iPart = Vector3.right * Mathf.Sin(Mathf.Deg2Rad * TEST_ROTATION * 0.5f);
        float rPart = Mathf.Cos(TEST_ROTATION * Mathf.Deg2Rad * 0.5f);
        Quaternion myRot = new Quaternion(iPart.x, iPart.y, iPart.z, rPart);

        //m_RoboyParts["torso"].transform.position = myRot * m_RoboyParts["torso"].transform.position;
        m_RoboyParts["torso"].transform.rotation *= myRot;
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

            //Vector3 position = new Vector3(yPositionsDictionary[index] * -1f, zPositionsDictionary[index], xPositionsDictionary[index]);
            Vector3 originPosition = new Vector3(xPositionsDictionary[index], yPositionsDictionary[index], zPositionsDictionary[index]);
            Vector3 eulerAngles = new Vector3(Mathf.Rad2Deg * rollRotationsDictionary[index], Mathf.Rad2Deg * yawRotationsDictionary[index] * -1f + 90f, Mathf.Rad2Deg * pitchRotationsDictionary[index]);

            Quaternion roboyQ = new Quaternion(qxRotationsDictionary[index], qyRotationsDictionary[index], qzRotationsDictionary[index], qwRotationsDictionary[index]);

            //Quaternion rotX = Quaternion.AngleAxis(90f, Vector3.right);
            //Quaternion rotY = Quaternion.AngleAxis(90f, Vector3.up);

            Quaternion rotX = Quaternion.AngleAxis(-90f, Vector3.right);
            Quaternion rotY = Quaternion.AngleAxis(90f, Vector3.up);

            //Vector3 pos = new Vector3(x, y, z);

            originPosition = rotY * rotX * originPosition;
            //pos = rotY * pos;

            originPosition.z *= -1f;

            Quaternion q2 = new Quaternion(-roboyQ.x, -roboyQ.y, -roboyQ.z, roboyQ.w);

            Quaternion qX2 = Quaternion.AngleAxis(90f, Vector3.right);

            roboyQ = rotY * qX2 * q2;


            //roboyQ *= rotX * rotY;

            //Quaternion final2Q = new Quaternion(-roboyQ.x, -roboyQ.z, -roboyQ.y, roboyQ.w);

            //Quaternion positionQ = new Quaternion(xPositionsDictionary[index], yPositionsDictionary[index], zPositionsDictionary[index], 0);
            //positionQ *= rotX;

            //Quaternion position2Q = new Quaternion(-positionQ.x, -positionQ.z, -positionQ.y, positionQ.w);

            //originPosition = rotX * originPosition;

            roboyPart.Value.transform.localPosition = originPosition;
            roboyPart.Value.transform.rotation = roboyQ;

            continue;


        }

    }

    public static Quaternion GetRotationFromMatrix(Matrix4x4 matrix)
    {
        var qw = Mathf.Sqrt(1f + matrix.m00 + matrix.m11 + matrix.m22) / 2;
        var w = 4 * qw;
        var qx = (matrix.m21 - matrix.m12) / w;
        var qy = (matrix.m02 - matrix.m20) / w;
        var qz = (matrix.m10 - matrix.m01) / w;

        return new Quaternion(qx, qy, qz, qw);
    }


    public static Quaternion QuaternionFromMatrix(Matrix4x4 m)
    {
        // Adapted from: http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
        Quaternion q = new Quaternion
        {
            w = Mathf.Sqrt(Mathf.Max(0, 1 + m[0, 0] + m[1, 1] + m[2, 2]))/2,
            x = Mathf.Sqrt(Mathf.Max(0, 1 + m[0, 0] - m[1, 1] - m[2, 2]))/2,
            y = Mathf.Sqrt(Mathf.Max(0, 1 - m[0, 0] + m[1, 1] - m[2, 2]))/2,
            z = Mathf.Sqrt(Mathf.Max(0, 1 - m[0, 0] - m[1, 1] + m[2, 2]))/2
        };
        q.x *= Mathf.Sign(q.x * (m[2, 1] - m[1, 2]));
        q.y *= Mathf.Sign(q.y * (m[0, 2] - m[2, 0]));
        q.z *= Mathf.Sign(q.z * (m[1, 0] - m[0, 1]));
        return q;
    }

    public static Matrix4x4 MatrixFromQuaternion(Quaternion q)
    {
        //Adapted from dick
        Matrix4x4 m = new Matrix4x4();

        m.m00 = 1.0f - 2.0f*q.y*q.y - 2.0f*q.z*q.z;
        m.m01 = 2.0f*q.x*q.y - 2.0f*q.z*q.w;
        m.m02 = 2.0f*q.x*q.z + 2.0f*q.y*q.w;
        m.m03 = 0.0f;

        m.m10 = 2.0f*q.x*q.y + 2.0f*q.z*q.w;
        m.m11 = 1.0f - 2.0f*q.x*q.x - 2.0f*q.z*q.z;
        m.m12 = 2.0f*q.y*q.z - 2.0f*q.x*q.w;
        m.m13 = 0.0f;

        m.m20 = 2.0f*q.x*q.z - 2.0f*q.y*q.w;
        m.m21 = 2.0f*q.y*q.z + 2.0f*q.x*q.w;
        m.m22 = 1.0f - 2.0f*q.x*q.x - 2.0f*q.y*q.y;
        m.m23 = 0.0f;

        m.m30 = 0.0f;
        m.m31 = 0.0f;
        m.m32 = 0.0f;
        m.m33 = 1.0f;

        return m;
    }

    static Quaternion ConvertToRightHand(Vector3 Euler)
    {
        Quaternion x = Quaternion.AngleAxis(-Euler.x, Vector3.right);
        Quaternion y = Quaternion.AngleAxis(Euler.y, Vector3.up);
        Quaternion z = Quaternion.AngleAxis(Euler.z, Vector3.forward);
        return (z * y * x);
    }

    static Quaternion ConvertToRightHand2(Vector3 Euler)
    {
        Quaternion x = Quaternion.AngleAxis(-Euler.x, Vector3.right);
        Quaternion y = Quaternion.AngleAxis(-Euler.y + 180, Vector3.up);
        Quaternion z = Quaternion.AngleAxis(Euler.z, Vector3.forward);
        return (z * y * x);
    }

    #endregion //PUBLIC_METHODS

    #region PRIVATE_METHODS

    #endregion //PRIVATE_METHODS
}
