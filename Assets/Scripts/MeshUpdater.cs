using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;
using System.Diagnostics;


public class MeshUpdater : MonoBehaviour {


    //"pathToBlender/blender.exe"
    public string pathToBlender = @"C:\Program Files\Blender\blender.exe";

    //"pathToScript\meshDownloadScript.py"
    public string pathToScript = @"D:\RoboyVR\Assets\SimulationModels\RoboyModel\OriginModels\meshDownloadScript.py";

    //"gitPathToMeshes\" 
    public string pathToMeshes = @"https://github.com/Roboy/roboy_models/tree/master/legs_with_upper_body/cad/";

    [Tooltip("This script scans for all models in the github repo. For this to work you have to install python and setup the PATH variable (Windows)")]
    public string pathToScanScript = @"D:\RoboyVR\Assets\SimulationModels\RoboyModel\OriginModels\ModelScanner.py";

    public string pathToModels = @"https://github.com/Roboy/roboy_models/";

    // Use this for initialization
    void Start () {
		
	}
	
	// Update is called once per frame
	void Update () {
		
	}

    public void RunCMD(string command) {
        //UnityEngine.Debug.Log("I built this with my bear hands!");
        //UnityEngine.Debug.Log(pathToBlender);
        //UnityEngine.Debug.Log(pathToScript);
        //UnityEngine.Debug.Log(pathToMeshes);
        UnityEngine.Debug.Log(command);


        var processInfo = new ProcessStartInfo("cmd.exe", "/C" +command);
        processInfo.CreateNoWindow = true;
        processInfo.UseShellExecute = false;
        processInfo.RedirectStandardError = true;
        processInfo.RedirectStandardOutput = true;

        var process = Process.Start(processInfo);

        process.OutputDataReceived += (object sender, DataReceivedEventArgs e) =>
            Console.WriteLine("output>>" + e.Data);
        process.BeginOutputReadLine();

        process.ErrorDataReceived += (object sender, DataReceivedEventArgs e) =>
            Console.WriteLine("error>>" + e.Data);
        process.BeginErrorReadLine();

        process.WaitForExit();

        Console.WriteLine("ExitCode: {0}", process.ExitCode);
        process.Close();
    }
}
