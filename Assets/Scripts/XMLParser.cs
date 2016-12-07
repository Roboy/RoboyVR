using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using System.Xml;
using System.IO;

public class XMLParser : MonoBehaviour
{

    #region PUBLIC_MEMBER_VARIABLES
    public TextAsset XML_FILE;
    #endregion //PUBLIC_MEMBER_VARIABLES

    #region PRIVATE_MEMBER_VARIABLES
    #endregion //PRIVATE_MEMBER_VARIABLES

    #region MONOBEHAVIOR_METHODS
    void Awake()
    {
        getInitParameters();
    }

    void OnApplicationQuit()
    {
        Quaternion curRot = transform.rotation;

        curRot = unityRotationToGazebo(curRot);

        //Debug.Log(string.Format("Name: {0}, X: {1}, Y: {2}, Z: {3}, W: {4}", gameObject.name, curRot.x, curRot.y, curRot.z, curRot.w));
    }

    #endregion //MONOBEHAVIOR_METHODS

    #region PUBLIC_METHODS

    #endregion //PUBLIC_METHODS

    #region PRIVATE_METHODS
    private void getInitParameters()
    {
        XmlDocument xmlDoc = new XmlDocument();
        xmlDoc.LoadXml(XML_FILE.text);

        XmlNode node = xmlDoc.SelectSingleNode("/sdf/model/link[@name='" + gameObject.name + "']/pose");

        string[] poseString = node.InnerText.Split(null);

        float x = float.Parse(poseString[0]);
        float y = float.Parse(poseString[1]);
        float z = float.Parse(poseString[2]);

        float alpha = float.Parse(poseString[3]);
        float beta = float.Parse(poseString[4]);
        float gamma = float.Parse(poseString[5]);


        //Gazebo to Unity coordinate system

        //POSITION
        //X => -Y
        //Y => Z
        //Z => X

        Quaternion rotX = Quaternion.AngleAxis(-90f, Vector3.right);
        Quaternion rotY = Quaternion.AngleAxis(90f, Vector3.up);
        Quaternion rotZ = Quaternion.AngleAxis(-180f, Vector3.right);

        Vector3 pos = new Vector3(x, y, z);

        pos = rotY * rotX * pos;

        pos.z *= -1f;

        transform.localPosition = pos;

        //ROTATION
        //alpha => alpha
        //beta => gamma * -1 + 90°
        //gamma => beta

        Quaternion q = Quaternion.Euler(new Vector3(alpha, beta, gamma));


        //if (gameObject.name.Equals("thigh_left"))
        //{
        //    float qx = float.Parse(poseString[3]);
        //    float qy = float.Parse(poseString[4]);
        //    float qz = float.Parse(poseString[5]);
        //    float qw = float.Parse(poseString[6]);

        //    Quaternion rot = new Quaternion(qx, qy, qz, qw);

        //    transform.localRotation = gazeboRotationToUnity(rot);
        //    return;
        //}

        transform.localRotation = gazeboRotationToUnity(q);
    }

    Quaternion gazeboRotationToUnity(Quaternion gazeboRot)
    {
        Quaternion rotX = Quaternion.AngleAxis(90f, Vector3.right);
        Quaternion rotY = Quaternion.AngleAxis(90f, Vector3.up);

        return rotY*rotX*gazeboRot;
    }

    Quaternion unityRotationToGazebo(Quaternion unityRot)
    {
        Quaternion rotX = Quaternion.AngleAxis(-90f, Vector3.right);
        Quaternion rotY = Quaternion.AngleAxis(-90f, Vector3.up);

        return rotX*rotY*unityRot;
    }

    #endregion //PRIVATE_METHODS
}
