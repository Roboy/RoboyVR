using UnityEngine;

public class RoboyServiceResponse{

    #region PUBLIC_MEMBER_VARIABLES
    #endregion //PUBLIC_MEMBER_VARIABLES

    #region PRIVATE_MEMBER_VARIABLES
    #endregion //PRIVATE_MEMBER_VARIABLES

    #region MONOBEHAVIOR_METHODS
    #endregion //MONOBEHAVIOR_METHODS

    #region PUBLIC_METHODS

    public static void ServiceCallBack(string service, string response)
    {
        if (response == null)
            Debug.Log("ServiceCallback for service " + service);
        else
            Debug.Log("ServiceCallback for service " + service + " response " + response);
    }

    #endregion //PUBLIC_METHODS

    #region PRIVATE_METHODS
    #endregion //PRIVATE_METHODS
}
