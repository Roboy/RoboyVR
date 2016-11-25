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

        if(Input.GetKeyDown(KeyCode.A))
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

            Vector3 position = new Vector3(yPositionsDictionary[index] * -1f, zPositionsDictionary[index], xPositionsDictionary[index]);
            Vector3 eulerAngles = new Vector3(Mathf.Rad2Deg * rollRotationsDictionary[index], Mathf.Rad2Deg * yawRotationsDictionary[index] * -1f + 90f, Mathf.Rad2Deg * pitchRotationsDictionary[index]);


            Matrix4x4 m = new Matrix4x4
            {
                m00 = v00Dictionary[index],
                m01 = v01Dictionary[index],
                m02 = v02Dictionary[index],
                m03 = v03Dictionary[index],
                m10 = v10Dictionary[index],
                m11 = v11Dictionary[index],
                m12 = v12Dictionary[index],
                m13 = v13Dictionary[index],
                m20 = v20Dictionary[index],
                m21 = v21Dictionary[index],
                m22 = v22Dictionary[index],
                m23 = v23Dictionary[index],
                m30 = v30Dictionary[index],
                m31 = v31Dictionary[index],
                m32 = v32Dictionary[index],
                m33 = v33Dictionary[index]
            };

            Quaternion tempQ = GetRotationFromMatrix(m);

            Vector3 euler = tempQ.eulerAngles;

            float gamma = euler.y;
            float beta = euler.z;
            float alpha = euler.x;

            float tempPitch = alpha*-1 + Mathf.PI/2f;
            alpha = beta;
            beta = tempPitch;

            Matrix4x4 a = new Matrix4x4
            {
                m00 = Mathf.Cos(alpha) * Mathf.Cos(beta),
                m01 = Mathf.Cos(alpha) * Mathf.Sin(beta) * Mathf.Sin(gamma) - Mathf.Sin(alpha)* Mathf.Cos(gamma),
                m02 = Mathf.Cos(alpha)* Mathf.Sin(beta) * Mathf.Cos(gamma) + Mathf.Sin(alpha) * Mathf.Sin(gamma),
                m10 = Mathf.Sin(alpha) * Mathf.Cos(beta),
                m11 = Mathf.Sin(alpha) * Mathf.Sin(beta) * Mathf.Sin(gamma) + Mathf.Cos(alpha)* Mathf.Cos(gamma),
                m12 = Mathf.Sin(alpha)* Mathf.Sin(beta) * Mathf.Cos(gamma) - Mathf.Cos(alpha) * Mathf.Sin(gamma),
                m20 = -Mathf.Sin(beta),
                m21 = Mathf.Cos(beta) * Mathf.Sin(gamma),
                m22 = Mathf.Cos(beta) * Mathf.Cos(gamma),
                m03 = 0,
                m13 = 0,
                m23 = 0,
                m30 = 0,
                m31 = 0,
                m32 = 0,
                m33 = 1
            };


            Matrix4x4 mNew = Matrix4x4.Inverse(m);

            Quaternion qTemp2 = QuaternionFromMatrix(m);

            Quaternion rotX = Quaternion.AngleAxis(-90, Vector3.forward);
            Quaternion rotZ = Quaternion.AngleAxis(90f, Vector3.up);
            Quaternion rotY = Quaternion.AngleAxis(-90f, Vector3.right);

            qTemp2 = rotZ*rotX*qTemp2;

            Quaternion qTemp3 = Quaternion.Inverse(qTemp2);

            //Quaternion qTemp = GetRotationFromMatrix(m);
            //qTemp *= Quaternion.Euler(-90,0,0);

            //m = MatrixFromQuaternion(qTemp);
              
            //Matrix4x4 mNew = new Matrix4x4();;

            //SEE http://stackoverflow.com/questions/1263072/changing-a-matrix-from-right-handed-to-left-handed-coordinate-system

            //ry => rz
            //mNew.m01 = -m.m02;
            ////rz => ry
            //mNew.m02 = m.m01;
            ////ux => lx
            //mNew.m10 = m.m20;
            ////uy => lz
            //mNew.m11 = -m.m22;
            ////uz => ly
            //mNew.m12 = m.m21;
            ////lx => ux
            //mNew.m20 = m.m10;
            ////ly => uz
            //mNew.m21 = -m.m12;
            ////lz => uy
            //mNew.m22 = m.m11;
            //py => pz
            //mNew.m31 = m.m32;
            //pz => py
            //mNew.m32 = m.m31;

            ////rx => uy
            //mNew.m00 = -m.m12;
            ////ry => ux
            //mNew.m01 = m.m10;
            ////rz => uz
            //mNew.m02 = m.m11;

            ////ux => lx
            //mNew.m10 = -m.m22;
            ////uy => lz
            //mNew.m11 = m.m20;
            ////uz => ly
            //mNew.m12 = m.m21;

            ////lx => -rz
            //mNew.m20 = -m.m02;
            ////ly => -ry
            //mNew.m21 = m.m00;
            ////lz => -rx
            //mNew.m22 = -m.m01;

            //mNew.m33 = 1;


            Quaternion q = GetRotationFromMatrix(a);


            
            //q *= Quaternion.Euler(90f, 0, 0);
            //Matrix4x4 n = MatrixFromQuaternion(q);

            //for (int i = 0; i < 4; i++)
            //{
            //    for (int j = 0; j < 4; j++)
            //    {
            //        if (Math.Abs(m[i, j] - n[i, j]) > 0.0005)
            //        {
            //            Debug.Log("Error @"+i+","+j+" :"+ (m[i, j] - n[i, j]));
            //        }

            //    }
            //}
            roboyPart.Value.transform.localPosition = position;
            roboyPart.Value.transform.rotation = qTemp2;

            ////Quaternion qNew = QuaternionFromMatrix(MatrixFromQuaternion(q));
            //Quaternion qNew = GetRotationFromMatrix(MatrixFromQuaternion(q));

            //if (Math.Abs(qNew.x - q.x) > 0.0005)
            //{
            //    Debug.Log("error x: " + (qNew.x - q.x));
            //}
            //if (Math.Abs(qNew.y - q.y) > 0.0005)
            //{
            //    Debug.Log("error y: " + (qNew.y - q.y));
            //}
            //if (Math.Abs(qNew.z - q.z) > 0.0005)
            //{
            //    Debug.Log("error z: " + (qNew.z - q.z));
            //}
            //if (Math.Abs(qNew.w - q.w) > 0.0005)
            //{
            //    Debug.Log("error w: " + (qNew.w - q.w));
            //}
            //Debug.Log(string.Format("XDifference: {0}, YDiff: {1}, ZDiff: {2}, WDiff: {3}",qNew.x - q.x, qNew.y - q.y, qNew.z - q.z, qNew.w - q.w));

            //roboyPart.Value.transform.eulerAngles = eulerAngles;
            //if (index.Equals("torso"))
            //{
            //    for (int i = 0; i < 4; i++)
            //        for (int j = 0; j < 4; j++)
            //            Debug.Log("M@ " + i + "," + j + ": " + m[i, j]);

            //    Debug.Log("ROLL : " + rollRotationsDictionary[index] + " PITCH : " + pitchRotationsDictionary[index] + " YAW : " + yawRotationsDictionary[index]);
            //}
            //Debug.Log();
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
