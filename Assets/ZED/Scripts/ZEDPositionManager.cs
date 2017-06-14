using System.Collections;
using System.Collections.Generic;

using UnityEngine;


/// <summary>
/// Enables to save/load the position of the ZED, is useful especially for the greenScreen
/// </summary>
public class ZEDPositionManager : MonoBehaviour {

    /// <summary>
    /// ZED pose file name
    /// </summary>
    private const string pathFileZEDPos = "pos_zed.cfg";

    /// <summary>
    /// Save the position of the ZED
    /// </summary>
    public void SaveZEDPos()
    {
        using (System.IO.StreamWriter file = new System.IO.StreamWriter(pathFileZEDPos))
        {
            string tx = "tx=" + transform.localPosition.x.ToString();
            string ty = "ty=" + transform.localPosition.y.ToString();
            string tz = "tz=" + transform.localPosition.z.ToString();
            string rx = "rx=" + transform.localRotation.eulerAngles.x.ToString();
            string ry = "ry=" + transform.localRotation.eulerAngles.y.ToString();
            string rz = "rz=" + transform.localRotation.eulerAngles.z.ToString();
            file.WriteLine(tx);
            file.WriteLine(ty);
            file.WriteLine(tz);
            file.WriteLine(rx);
            file.WriteLine(ry);
            file.WriteLine(rz);
            file.Close();
        }
    }

    void Awake()
    {
#if !UNITY_EDITOR
        Debug.Log("Load ZED Pos");
        LoadZEDPos();
#endif
    }

    /// <summary>
    /// Load the position of the ZED from a file
    /// </summary>
    public void LoadZEDPos()
    {
        string[] lines = null;
        try
        {
            lines = System.IO.File.ReadAllLines(pathFileZEDPos);
        }
        catch (System.Exception )
        {
            
        }
        if (lines == null) return;
        Vector3 position = new Vector3(0, 0, 0);
        Vector3 eulerRotation = new Vector3(0, 0, 0);
        foreach (string line in lines)
        {
            string[] splittedLine = line.Split('=');
            if (splittedLine.Length == 2)
            {
                string key = splittedLine[0];
                string field = splittedLine[1];

                if (key == "tx")
                {
                    position.x = float.Parse(field);
                }
                else if (key == "ty")
                {
                    position.y = float.Parse(field);
                }
                else if (key == "tz")
                {
                    position.z = float.Parse(field);
                }
                else if (key == "rx")
                {
                    eulerRotation.x = float.Parse(field);
                }
                else if (key == "ry")
                {
                    eulerRotation.y = float.Parse(field);
                }
                else if (key == "rz")
                {
                    eulerRotation.z = float.Parse(field);
                }
            }
        }

        transform.localPosition = position;
        transform.localRotation = Quaternion.Euler(eulerRotation.x, eulerRotation.y, eulerRotation.z);
    }
}
