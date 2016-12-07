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

        transform.localPosition = gazeboPositionToUnity(pos);

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
        //Quaternion rotZ = Quaternion.AngleAxis(90f, Vector3.up);
        //Quaternion rotX = Quaternion.AngleAxis(-90f, Vector3.left);
        //Quaternion rot = rotX * rotZ;

        ////Quaternion inverse = new Quaternion(gazeboRot.x, gazeboRot.y, gazeboRot.z, gazeboRot.w);

        //gazeboRot *= rot;


        //return Quaternion.Inverse(gazeboRot);

        Quaternion meal = new Quaternion(gazeboRot.x, gazeboRot.z, gazeboRot.y, gazeboRot.w);
        Quaternion rot = Quaternion.AngleAxis(-90f, Vector3.up);

        return rot * meal;
        //Debug.Log("gRotx: " + gazeboRot.x + " gRoty: " + gazeboRot.y + " gRotz: " + gazeboRot.z +
        //    " gRotw: " + gazeboRot.w);
        //Debug.Log("lRotx: " + leftHandedRotation.x + " lRoty: " + leftHandedRotation.y + " lRotz: " + leftHandedRotation.z +
        //    " lRotw: " + leftHandedRotation.w);
        //return  rot * leftHandedRotation;


    }

    Vector3 gazeboPositionToUnity(Vector3 gazeboPos)
    {
        Quaternion rotX = Quaternion.AngleAxis(-90f, Vector3.forward);
        Quaternion rotY = Quaternion.AngleAxis(-90f, Vector3.up);

        Quaternion rot = rotX * rotY;

        //gazeboPos = rotY * gazeboPos;
        //gazeboPos = rotX * gazeboPos;

        //gazeboPos = rotY * rotX * gazeboPos;
        gazeboPos = rot * gazeboPos;
        gazeboPos.z *= -1f;

        return gazeboPos;

        //return new Vector3(-gazeboPos.y, gazeboPos.z, gazeboPos.x);
    }

    #endregion //PRIVATE_METHODS
}
