using System;
using System.Diagnostics;
using UnityEngine;
using Microsoft.Win32;

/// <summary>
/// This class provides the functionality to run commandline with given arguments on a Windows machine.
/// </summary>
public class CommandlineUtility {

    private const string keyBase = @"SOFTWARE\Microsoft\Windows\CurrentVersion\App Paths";

    /// <summary>
    /// Runs the windows "cmd" commandline.
    /// </summary>
    /// <param name="arguments">Array of arguments, where the first argument should be the command, like "python" or "start" and the other ones are the parameters.</param>
	public static void RunCommandLine(string[] arguments)
    {
        string command = argumentsToCommandlineString(arguments);
        UnityEngine.Debug.Log("Running the following command: \n"  + command);

        var processInfo = new ProcessStartInfo("cmd.exe", "/C" + command);
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

    /// <summary>
    /// Returns the path of the given application. Does not work when application path key is not set in the registry.
    /// </summary>
    /// <param name="fileName">Should contain the applicationame with .exe, f.e. "blender.exe"</param>
    /// <returns></returns>
    public static string GetPathForExe(string fileName)
    {
        RegistryKey localMachine = Registry.LocalMachine;
        RegistryKey fileKey = localMachine.OpenSubKey(string.Format(@"{0}\{1}", keyBase, fileName));
        object result = null;
        if (fileKey != null)
        {
            result = fileKey.GetValue(string.Empty);
        }
        fileKey.Close();

        return (string)result;
    }

    /// <summary>
    /// Converts an array of arguments to a string so it can be executed. Wraps all parameters with quotation marks.
    /// </summary>
    /// <param name="arguments"></param>
    /// <returns></returns>
    private static string argumentsToCommandlineString(string[] arguments)
    {
        string command = arguments[0];
        for (int i = 1; i < arguments.Length; i++)
        {
            command += " \"" + arguments[i] + "\"";
        }
        return command;
    }
}
