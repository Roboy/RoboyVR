using UnityEngine;

namespace ROSBridgeLib
{
    /// <summary>
    /// This defines a ROS service. Basically a service serves as function call. Therefore you need the service aka the function and arguments when you call a service.
    /// As soon as you send a service call the service waits for a response.
    /// </summary>
    public class ROSBridgeService : MonoBehaviour
    {
        public static void ServiceCallBack(string service, string response) { }
    }
}
