using UnityEngine;

namespace ROSBridgeLib
{
    public class RoboyServiceResponse : ROSBridgeService
    {
        #region PUBLIC_METHODS

        public static new void ServiceCallBack(string service, string response)
        {
            if (response == null)
                Debug.Log("ServiceCallback for service " + service);
            else
                Debug.Log("ServiceCallback for service " + service + " response " + response);
        }

        #endregion //PUBLIC_METHODS
    }
}
