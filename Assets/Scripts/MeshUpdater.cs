using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;
using System.Diagnostics;


public class MeshUpdater : MonoBehaviour {


    //"pathToBlender/blender.exe"
    public string pathToBlender = "C:\\Program Files\\Blender Foundation\\Blender\\blender.exe";

    //"pathToScript\meshDownloadScript.py"
    public string pathToScript = "D:\\Unity Projects\\RoboyVR\\Assets\\RoboyModel\\OriginModels\\meshDownloadScript.py";

    //"gitPathToMeshes\" 
    public string pathToMeshes = "https://github.com/Roboy/roboy_models/tree/master/legs_with_upper_body/cad";

    // Use this for initialization
    void Start () {
		
	}
	
	// Update is called once per frame
	void Update () {
		
	}

    public void MeshUpdate(string cmd, string args) {
        UnityEngine.Debug.Log("I built this with my bear hands!");
        UnityEngine.Debug.Log(pathToBlender);
        UnityEngine.Debug.Log(pathToScript);
        UnityEngine.Debug.Log(pathToMeshes);
        //ProcessStartInfo start = new ProcessStartInfo();
        //start.FileName = pathToScript;
        //start.Arguments = string.Format("{0} {1}", cmd, args);
        //start.UseShellExecute = false;
        //start.RedirectStandardOutput = true;
        //using (Process process = Process.Start(start))
        //{
        //    using (StreamReader reader = process.StandardOutput)
        //    {
        //        string result = reader.ReadToEnd();
        //        Console.Write(result);
        //    }
        //}
    }
}
