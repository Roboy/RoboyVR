using UnityEngine;

/// <summary>
/// Utility class to convert a pose between the Gazebo and Unity coordinate frame.
/// IMPORTANT: EVEN THOUGH THESE UTILITIES ARE PUBLICLY AVAILABLE, THE COORD. SYS. TRANSLATION HAPPENS AUTOMATICALLY WHEN
/// MESSAGES ARE SENT OR RECEIVED!!!
/// </summary>
public class GazeboUtility {

    /// <summary>
    /// Converts a vector in gazebo coordinate frame to unity coordinate frame.
    /// *MUST BE CALLED WHEN MESSAGE RECEIVED & CREATED -> SEE IN MESSAGE DEFINITION*
    /// </summary>
    /// <param name="gazeboPos">Vector in gazebo coordinate frame.</param>
    /// <returns>Vector in unity coordinate frame.</returns>
    public static Vector3 GazeboPositionToUnity(Vector3 gazeboPos)
    {
        if (gazeboPos == null)
        {
            return Vector3.zero;
        }
        return new Vector3(gazeboPos.x, gazeboPos.z, gazeboPos.y);
    }

    /// <summary>
    /// Converts a vector in unity coordinate frame to gazebo coordinate frame. 
    /// *MUST BE CALLED WHEN MESSAGE CREATED TO BE SENT-> SEE IN MESSAGE DEFINITION*
    /// </summary>
    /// <param name="unityPos">Vector in unity coordinate frame.</param>
    /// <returns>Vector in gazebo coordinate frame.</returns>
    public static Vector3 UnityPositionToGazebo(Vector3 unityPos)
    {
            if (unityPos == null)
        {
            return Vector3.zero;
        }
        return new Vector3(unityPos.x, unityPos.z, unityPos.y);
    }

    /// <summary>
    /// Converts a quaternion in gazebo coordinate frame to unity coordinate frame.
    /// *MUST BE CALLED WHEN MESSAGE RECEIVED & CREATED -> SEE IN MESSAGE DEFINITION*
    /// </summary>
    /// <param name="gazeboRot">Quaternion in gazebo coordinate frame.</param>
    /// <returns>Quaternion in unity coordinate frame.</returns>
    public static Quaternion GazeboRotationToUnity(Quaternion gazeboRot)
    {
        Quaternion rotX = Quaternion.AngleAxis(180f, Vector3.right);
        Quaternion rotZ = Quaternion.AngleAxis(180f, Vector3.forward);

        Quaternion tempRot = new Quaternion(-gazeboRot.x, -gazeboRot.z, -gazeboRot.y, gazeboRot.w);

        Quaternion finalRot = tempRot * rotZ * rotX;

        return finalRot;
    }

    /// <summary>
    /// Converts a quaternion in unity coordinate frame to gazebo coordinate frame.
    /// *MUST BE CALLED WHEN MESSAGE CREATED TO BE SENT-> SEE IN MESSAGE DEFINITION*
    /// </summary>
    /// <param name="unityRot">Quaternion in unity coordinate frame.</param>
    /// <returns>Quaternion in gazebo coordinate frame.</returns>
    public static Quaternion UnityRotationToGazebo(Quaternion unityRot)
    {
        Quaternion rotX = Quaternion.AngleAxis(180f, Vector3.right);
        Quaternion rotZ = Quaternion.AngleAxis(180f, Vector3.forward);

        Quaternion tempRot = unityRot * rotX * rotZ;

        Quaternion finalRot = new Quaternion(-tempRot.x, -tempRot.z, -tempRot.y, tempRot.w);

        return finalRot;
    }
    
    /// <summary>
    /// Returns the given local direction in worldspace coordinates
    /// IMPORTANT: Not influenced by scale etc
    /// </summary>
    /// <param name="localSpace">local space in which the direction is currently defined</param>
    /// <param name="localdirection">direction in local space</param>
    /// <returns></returns>
    public static Vector3 LocalToWorldSpaceDirection(Transform localSpace, Vector3 localdirection)
    {
        if (localSpace != null)
        {
            return localSpace.TransformDirection(localdirection);
        }
        return Vector3.zero;
    }

    /// <summary>
    /// Returns quaternion which was translated from world space into local space
    /// TODO: further testing needed
    /// </summary>
    /// <param name="localspace"></param>
    /// <param name="worldrotation"></param>
    /// <returns></returns>
    public static Quaternion WorldToLocalSpaceRotation(Transform localspace, Quaternion worldrotation)
    {
        if (localspace != null)
        {
            return Quaternion.Inverse(localspace.rotation) * worldrotation;
        }
        return Quaternion.identity;
    }

    /// <summary>
    /// returns the given direction transformed into the given local space
    /// IMPORTANT: not influenced by scales
    /// </summary>
    /// <param name="localspace">desired local space</param>
    /// <param name="worlddirection">direction in worldspace coordinates</param>
    /// <returns></returns>
    public static Vector3 WorldToLocalSpaceDirection(Transform localspace, Vector3 worlddirection)
    {
        if (localspace != null)
        {
            return localspace.TransformDirection(worlddirection);
        }
        return Vector3.zero;
    }
    /// <summary>
    /// Removes first and last string character (intended for unnecessary quotation marks but works for anything)
    /// </summary>
    /// <param name="s">string to be cropped</param>
    /// <returns></returns>
    public static string RemoveQuotationMarks(string s)
    {
        string temp = s.Remove(0, 1);
        return temp.Remove(s.Length - 2, 1);
    }
}
