#if !__LINE__ // if C# Usage
/// <summary>
/// This class is a prototype providing c# and c++ support. 
/// This is to be shared by control team / the UI 
/// Control defines and keeps this updated
/// </summary>
public class DummyStates

{
    public 
#endif
   
        
        
        enum State
    {
        MOTOR_DEAD  = 0 ,
        MOTOR_LOCKED = 1
    };
#if !__LINE__ // if C# Usage
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