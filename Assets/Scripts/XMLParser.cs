﻿using UnityEngine;
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

        transform.position = new Vector3(-y, z, x);

        //ROTATION
        //alpha => alpha
        //beta => gamma * -1 + 90°
        //gamma => beta

        transform.eulerAngles = new Vector3(Mathf.Rad2Deg * alpha, Mathf.Rad2Deg * gamma * -1f + 90f, Mathf.Rad2Deg * beta);
    }
    #endregion //PRIVATE_METHODS
}
