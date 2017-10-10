#if !__LINE__ // if C# Usage
/// <summary>
/// This class is a prototype providing c# and c++ support. 
/// This is to be shared by control team / the UI 
/// Control defines and keeps this updated
/// </summary>
public class DummyStates

{/// <summary>
/// Different states a notification communicates (how is the respective part doing)
/// FOR NOW ONLY DUMMY STATES, NEED TO BE ADAPTED 
/// </summary>
    public
#endif
        enum State
    {
        MOTOR_DEAD = 0,
        MOTOR_LOCKED = 1
    };
#if !__LINE__ // if C# Usage
    /// <summary>
    /// Type of the sent message 
    /// NEEDS TO BE SYNCHED WITH CONTROL TEAM(Danger missing / ignored)
    /// </summary>
    public
#endif
        enum MessageType
    {
        INFO,
        DEBUG,
        WARNING,
        ERROR
    }

#if !__LINE__ // if C# Usage
}
#endif