using ROSBridgeLib;

public class RoboyPosePublisher : ROSBridgePublisher {

    #region PUBLIC_MEMBER_VARIABLES
    #endregion //PUBLIC_MEMBER_VARIABLES

    #region PRIVATE_MEMBER_VARIABLES
    #endregion //PRIVATE_MEMBER_VARIABLES

    #region MONOBEHAVIOR_METHODS
    #endregion //MONOBEHAVIOR_METHODS

    #region PUBLIC_METHODS

    public new static string GetMessageTopic()
    {
        return "/roboy/pose";

        //return "/roboy/id";
    }

    public new static string GetMessageType()
    {
        return "common_utilities/Pose";

        //return "std_msgs/Int32";
    }

    public static string ToYAMLString(RoboyPoseMsg msg)
    {
        return msg.ToYAMLString();
    }

    #endregion //PUBLIC_METHODS

    #region PRIVATE_METHODS
    #endregion //PRIVATE_METHODS
}
