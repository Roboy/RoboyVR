
using System.Collections.Generic;
using UnityEngine;
using System.IO;
#if ZED_STEAM_VR
using Valve.VR;
#endif
public class ZEDPadManager : MonoBehaviour
{


    /**** Structures used to delay the pads*****/
    public struct TimedPoseData
    {
        public float timestamp;
        public Quaternion rotation;
        public Vector3 position;
    }
#if ZED_STEAM_VR
    private int previousStateIndex = -1;
#endif
    public delegate void ZEDOnPadIndexSetAction();
    public static event ZEDOnPadIndexSetAction ZEDOnPadIndexSet;


    public struct Pose
    {
        public Vector3 pose;
        public Quaternion rot;
    }
#if ZED_STEAM_VR
    public struct DelayedPad
    {
        public GameObject o;
        public SteamVR_Controller.Device d;
    }
    [HideInInspector]
    public Dictionary<int, DelayedPad> delayedPads = new Dictionary<int, DelayedPad>();

    /**** Steam VR ****/
    private SteamVR_ControllerManager controllerManager;
    [HideInInspector]
    public List<SteamVR_TrackedController> controllers = new List<SteamVR_TrackedController>();
    [HideInInspector]
    public List<GameObject> controllersGameObject = new List<GameObject>();
    [HideInInspector]
    public List<SteamVR_Controller.Device> devices = new List<SteamVR_Controller.Device>();

    [HideInInspector]
    public GameObject controllerObject;

    private Dictionary<int, List<TimedPoseData>> poseData = new Dictionary<int, List<TimedPoseData>>();
    [HideInInspector]
    [Tooltip("Controller ID holding the ZED")]
    public int controllerIndexZEDHolder = -1;
    public string SNHolder = "";
    /**** Latency correction ****/
    [Range(0, 200)]
    public int delay = 78;

    private float timerVive = 0.0f;
    private float timerMaxVive = 1.0f;



    private SteamVR_Events.Action newPoses;
    [HideInInspector]
    public bool padsSet = false;
    [HideInInspector]
    public GameObject cameraRig = null;
    public GameObject modelToReplaceVivePads = null;
    private bool ZEDControllerSet = false;

    private void Awake()
    {
        cameraRig = GameObject.Find("[CameraRig]");
        if (cameraRig == null)
        {
            Debug.LogWarning("Camera Rig needed, the script is disabled ");
            enabled = false;
        }
    }
    void Start()
    {
        newPoses = SteamVR_Events.NewPosesAction(OnNewPoses);
        newPoses.enabled = true;

        RegisterExistingPads();
        HideGameObjectFromZED();
    }




    public void LoadIndex(string path)
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

        foreach (string line in lines)
        {
            string[] splittedLine = line.Split('=');
            if (splittedLine.Length >= 2)
            {
                string key = splittedLine[0];
                string field = splittedLine[1].Split(' ')[0];

                if (key == "indexController")
                {
                    SNHolder = field;
                }
            }
        }

    }

    /**********************************/

    //To each new poses from the VIVE, the pos is registered and a delay is applied
    private void OnNewPoses(TrackedDevicePose_t[] poses)
    {
        foreach (KeyValuePair<int, DelayedPad> pad in delayedPads)
        {
            var i = (int)pad.Key;

            if (i == -1)
                return;

            if (poses.Length <= i)
                return;

            if (!poses[i].bDeviceIsConnected)
                return;

            if (!poses[i].bPoseIsValid)
                return;

            var pose = new SteamVR_Utils.RigidTransform(poses[i].mDeviceToAbsoluteTracking);

            RegisterPosition(i, pose.pos, pose.rot);
            Pose p = GetValuePosition(i, (float)(delay / 1000.0f));
            DelayedPad delayedPad;
            if (delayedPads.TryGetValue(i, out delayedPad))
            {
                delayedPads[i].o.transform.position = p.pose;
                delayedPads[i].o.transform.rotation = p.rot;
            }

            if (i == controllerIndexZEDHolder)
            {
                transform.parent.position = delayedPads[i].o.transform.position;
                transform.parent.rotation = delayedPads[i].o.transform.rotation;

            }
        }
    }

    //Register the existing pads, the pads registered will be checked if their IDs are correct
    void RegisterExistingPads()
    {
        if (cameraRig != null)
        {
            controllerManager = cameraRig.transform.GetComponent<SteamVR_ControllerManager>();
            controllerManager.Refresh();
            controllersGameObject.Clear();
            devices.Clear();
            controllersGameObject.Add(controllerManager.left);
            controllersGameObject.Add(controllerManager.right);
            foreach (GameObject o in controllerManager.objects)
            {
                if (!controllersGameObject.Contains(o))
                {
                    controllersGameObject.Add(o);
                }
            }
        }
        else
        {
            Debug.LogWarning("Missing [Camera_Rig] !");

        }
    }

    void OnApplicationQuit()
    {
        if (newPoses != null)
        {
            newPoses.enabled = false;
        }
    }

    public bool GetPadIndex()
    {
        foreach (GameObject c in controllersGameObject)
        {
            if (CheckIndexExist(c))
            {
                devices.Add(SteamVR_Controller.Input((int)c.GetComponent<SteamVR_TrackedObject>().index));
            }
        }
        return devices.Count >= 2;
    }

    //The new pads are registered and created. The preview is also created
    public void CreateNewPads()
    {
        poseData.Clear();
        foreach (DelayedPad o in delayedPads.Values)
        {
            Destroy(o.o);
        }
        delayedPads.Clear();

        foreach (SteamVR_Controller.Device d in devices)
        {
            if (poseData.ContainsKey((int)d.index))
                continue;
            DelayedPad p = new DelayedPad();
            if (modelToReplaceVivePads == null)
            {
                p.o = new GameObject();
                p.o.name = "VirtualDelayedPad_" + (int)d.index;
                p.o.layer = sl.ZEDCamera.Tag;
                SteamVR_RenderModel m = p.o.AddComponent<SteamVR_RenderModel>();
                m.modelOverride = "vr_controller_vive_1_5";

                m.SetDeviceIndex((int)d.index);
            }
            else
            {
                p.o = Instantiate(modelToReplaceVivePads);
            }
            p.d = d;

            poseData.Add((int)d.index, new List<TimedPoseData>());
            delayedPads.Add((int)d.index, p);
        }
    }

    // If the index is tracked, not an hmd and not already registered => true
    private bool CheckIndexExist(GameObject c)
    {
        SteamVR_TrackedObject tracked = c.GetComponent<SteamVR_TrackedObject>();
        return tracked != null && (int)tracked.index != -1 && (int)tracked.index != 0 && !devices.Contains(SteamVR_Controller.Input((int)tracked.index));
    }

    public bool SetPads(bool noCameraTracking)
    {

        bool controllerFound = false;
        uint leftIndex = OpenVR.k_unTrackedDeviceIndexInvalid, rightIndex = OpenVR.k_unTrackedDeviceIndexInvalid;
        var system = OpenVR.System;

        if (system != null)
        {
            leftIndex = system.GetTrackedDeviceIndexForControllerRole(ETrackedControllerRole.LeftHand);
            rightIndex = system.GetTrackedDeviceIndexForControllerRole(ETrackedControllerRole.RightHand);
        }

        int index = 0;
        if (devices.Count == 0) return false;
        if (!noCameraTracking)
        {
            foreach (SteamVR_Controller.Device d in devices)
            {
                if ((d.index != OpenVR.k_unTrackedDeviceIndexInvalid && d.index != leftIndex && d.index != rightIndex && leftIndex != OpenVR.k_unTrackedDeviceIndexInvalid && rightIndex != OpenVR.k_unTrackedDeviceIndexInvalid)
                    || d.GetPressDown(Valve.VR.EVRButtonId.k_EButton_SteamVR_Trigger))
                {
                    controllerIndexZEDHolder = (int)d.index;
                    ZEDControllerSet = true;
                    controllerFound = true;
                    break;
                }
            }
        }
        else if (noCameraTracking)
        {
            ZEDControllerSet = true;
            controllerFound = true;
            controllerIndexZEDHolder = -2;
        }


        foreach (SteamVR_Controller.Device d in devices)
        {

            if (ZEDControllerSet && controllerIndexZEDHolder != (int)d.index && d.valid)
            {
                while (index < controllersGameObject.Count && !controllersGameObject[index].activeInHierarchy)
                {
                    index++;
                }
                SteamVR_TrackedController c = controllersGameObject[index].GetComponent<SteamVR_TrackedController>();
                if (c == null)
                {
                    c = controllersGameObject[index].AddComponent<SteamVR_TrackedController>();
                }
                if (controllerObject == null)
                {
                    controllerObject = controllersGameObject[index];

                }
                c.SetDeviceIndex((int)d.index);
                controllers.Add(c);


                padsSet = true;

            }
            index++;
        }

        return controllerFound;
    }
    private const int layerToHide = 10;

    //Hides the Camera rig (mostly pads) from the ZED
    public void HideGameObjectFromZED()
    {
        Camera zedCamera = transform.GetChild(0).GetComponent<Camera>();

        if (zedCamera != null && cameraRig != null)
        {
            zedCamera.cullingMask &= ~(1 << layerToHide);
            SetLayerRecursively(cameraRig, layerToHide);
        }
    }

    public static void SetLayerRecursively(GameObject go, int layerNumber)
    {
        if (go == null) return;
        foreach (Transform trans in go.GetComponentsInChildren<Transform>(true))
        {
            trans.gameObject.layer = layerNumber;
        }
    }



    //Compute the delayed position and rotation from history
    private Pose GetValuePosition(int index, float timeDelay)
    {
        Pose p = new Pose();
        p.pose = poseData[index][poseData[index].Count - 1].position;
        p.rot = poseData[index][poseData[index].Count - 1].rotation;

        float idealTS = (Time.time - timeDelay);

        for (int i = 0; i < poseData[index].Count; ++i)
        {
            if (poseData[index][i].timestamp > idealTS)
            {
                int currentIndex = i;
                if (currentIndex > 0)
                {
                    float timeBetween = poseData[index][currentIndex].timestamp - poseData[index][currentIndex - 1].timestamp;
                    float alpha = ((Time.time - poseData[index][currentIndex - 1].timestamp) - timeDelay) / timeBetween;

                    Vector3 pos = Vector3.Lerp(poseData[index][currentIndex - 1].position, poseData[index][currentIndex].position, alpha);
                    Quaternion rot = Quaternion.Lerp(poseData[index][currentIndex - 1].rotation, poseData[index][currentIndex].rotation, alpha);

                    p = new Pose();
                    p.pose = pos;
                    p.rot = rot;
                    poseData[index].RemoveRange(0, currentIndex - 1);
                }
                return p;
            }
        }
        return p;
    }

    private void RegisterPosition(int index, Vector3 position, Quaternion rot)
    {
        TimedPoseData currentPoseData = new TimedPoseData();
        currentPoseData.timestamp = Time.time;
        currentPoseData.rotation = rot;
        currentPoseData.position = position;

        poseData[index].Add(currentPoseData);
    }

    private void Update()
    {
        

        timerVive += Time.deltaTime;
        if (timerVive > timerMaxVive)
        {
            timerVive = 0;
            if (!padsSet)
            {
                GetPadIndex();
            }

            if (devices.Count != delayedPads.Count)
            {
                CreateNewPads();
            }

            if (controllerIndexZEDHolder == -1)
            {
                if (SNHolder.Equals("NONE"))
                {
                    controllerIndexZEDHolder = -2;
                }
                else
                {
                    foreach (SteamVR_Controller.Device d in devices)
                    {
                        if (SteamVR.instance.GetStringProperty(Valve.VR.ETrackedDeviceProperty.Prop_SerialNumber_String, (uint)d.index).Equals(SNHolder))
                        {
                            controllerIndexZEDHolder = (int)d.index;
                        }
                    }
                }
            }
            if (controllerIndexZEDHolder == -1)
            {
                Debug.LogWarning("The serial number " + SNHolder + " is not found");
            }


            if (previousStateIndex != controllerIndexZEDHolder && controllerIndexZEDHolder != -1)
            {
                previousStateIndex = controllerIndexZEDHolder;
                if (ZEDOnPadIndexSet != null)
                {
                    ZEDOnPadIndexSet();
                }
            }
        }
    }
#endif
}
