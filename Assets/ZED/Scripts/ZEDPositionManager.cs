using UnityEngine;
using System.IO;

/// <summary>
/// Enables to save/load the position of the ZED, is useful especially for the greenScreen
/// </summary>
public class ZEDPositionManager : MonoBehaviour
{

    /// <summary>
    /// ZED pose file name
    /// </summary>
    [SerializeField]
    public static string ZEDOffsetFile = "ZED_Position_Offset.conf";

    private string path = @"Stereolabs\steamvr";
    /// <summary>
    /// Save the position of the ZED
    /// </summary>
    public void SaveZEDPos()
    {
        using (System.IO.StreamWriter file = new System.IO.StreamWriter(path))
        {
            string tx = "x=" + transform.localPosition.x.ToString() + " //Translation x";
            string ty = "y=" + transform.localPosition.y.ToString() + " //Translation y";
            string tz = "z=" + transform.localPosition.z.ToString() + " //Translation z";
            string rx = "rx=" + transform.localRotation.eulerAngles.x.ToString() + " //Rotation x";
            string ry = "ry=" + transform.localRotation.eulerAngles.y.ToString() + " //Rotation y";
            string rz = "rz=" + transform.localRotation.eulerAngles.z.ToString() + " //Rotation z";


            file.WriteLine(tx);
            file.WriteLine(ty);
            file.WriteLine(tz);
            file.WriteLine(rx);
            file.WriteLine(ry);
            file.WriteLine(rz);
            if (sl.ZEDCamera.GetInstance().CameraIsReady)
            {
                string fov = "fov=" + (sl.ZEDCamera.GetInstance().GetFOV() * Mathf.Rad2Deg).ToString();
                file.WriteLine(fov);

            }
#if ZED_STEAM_VR
            if (padManager)
            {
                string i = "indexController=" + (padManager.controllerIndexZEDHolder > 0 ? SteamVR.instance.GetStringProperty(Valve.VR.ETrackedDeviceProperty.Prop_SerialNumber_String, (uint)padManager.controllerIndexZEDHolder) : "NONE") + " //SN of the pad attached to the camera (NONE to set no pad on it)";
                file.WriteLine(i);
            }
#endif
            file.Close();
        }
    }
    public ZEDPadManager padManager;
    void Awake()
    {
		string folder = System.Environment.GetFolderPath(System.Environment.SpecialFolder.ApplicationData);
        string specificFolder = Path.Combine(folder, @"Stereolabs\steamvr");
        path = Path.Combine(specificFolder, ZEDOffsetFile);

        // Check if folder exists and if not, create it
        if (!Directory.Exists(specificFolder))
			Directory.CreateDirectory(specificFolder);

		padManager = GetComponent<ZEDPadManager>();

        LoadZEDPos();
		CreateFileWatcher(specificFolder);


    }

    /// <summary>
    /// Load the position of the ZED from a file
    /// </summary>
    public void LoadZEDPos()
    {
		if (!System.IO.File.Exists(path)) return;

        string[] lines = null;
        try
        {
			lines = System.IO.File.ReadAllLines(path);
        }
        catch (System.Exception)
        {

        }
        if (lines == null) return;
        Vector3 position = new Vector3(0, 0, 0);
        Vector3 eulerRotation = new Vector3(0, 0, 0);
        foreach (string line in lines)
        {
            string[] splittedLine = line.Split('=');
            if (splittedLine.Length >= 2)
            {
                string key = splittedLine[0];
                string field = splittedLine[1].Split(' ')[0];

                if (key == "x")
                {
                    position.x = float.Parse(field);
                }
                else if (key == "y")
                {
                    position.y = float.Parse(field);
                }
                else if (key == "z")
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
#if ZED_STEAM_VR
        if (padManager)
        {
            padManager.LoadIndex(path);
        }
#endif
        transform.localPosition = position;
        transform.localRotation = Quaternion.Euler(eulerRotation.x, eulerRotation.y, eulerRotation.z);
    }

    public void CreateFileWatcher(string path)
    {
        // Create a new FileSystemWatcher and set its properties.
        FileSystemWatcher watcher = new FileSystemWatcher();
        watcher.Path = path;
        /* Watch for changes in LastAccess and LastWrite times, and 
           the renaming of files or directories. */
        watcher.NotifyFilter = NotifyFilters.LastAccess | NotifyFilters.LastWrite
           | NotifyFilters.FileName | NotifyFilters.DirectoryName;
        // Only watch text files.
        watcher.Filter = ZEDOffsetFile;

        // Add event handlers.
        watcher.Changed += new FileSystemEventHandler(OnChanged);

        // Begin watching.
        watcher.EnableRaisingEvents = true;
    }

    // Define the event handlers.
    private void OnChanged(object source, FileSystemEventArgs e)
    {
#if ZED_STEAM_VR
        if (padManager)
        {
            padManager.LoadIndex(path);
        }
#endif

    }
}
