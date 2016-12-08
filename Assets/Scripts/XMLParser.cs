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

        Vector3 pos = new Vector3(x, y, z);
        Quaternion q = Quaternion.Euler(new Vector3(alpha, beta, gamma));

        transform.localPosition = gazeboPositionToUnity(pos);
        transform.localRotation = gazeboRotationToUnity(q);
    }

    Quaternion gazeboRotationToUnity(Quaternion gazeboRot)
    {
        Quaternion rotX = Quaternion.AngleAxis(180f, Vector3.right);
        Quaternion rotZ = Quaternion.AngleAxis(180f, Vector3.forward);

        Quaternion tempRot = new Quaternion(-gazeboRot.x, -gazeboRot.z, -gazeboRot.y, gazeboRot.w);

        Quaternion finalRot = gazeboRot * rotZ * rotX;

        return finalRot;
    }

    Vector3 gazeboPositionToUnity(Vector3 gazeboPos)
    {
        return new Vector3(gazeboPos.x, gazeboPos.z, gazeboPos.y);
    }

    #endregion //PRIVATE_METHODS
}
