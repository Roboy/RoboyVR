using UnityEngine;

/// <summary>
/// Utility class to convert a pose between the Gazebo and Unity coordinate frame.
/// </summary>
public class GazeboUtility {

    /// <summary>
    /// Converts a vector in gazebo coordinate frame to unity coordinate frame.
    /// </summary>
    /// <param name="gazeboPos">Vector in gazebo coordinate frame.</param>
    /// <returns>Vector in unity coordinate frame.</returns>
    public static Vector3 GazeboPositionToUnity(Vector3 gazeboPos)
    {
        return new Vector3(gazeboPos.x, gazeboPos.z, gazeboPos.y);
    }

    /// <summary>
    /// Converts a vector in unity coordinate frame to gazebo coordinate frame.
    /// </summary>
    /// <param name="unityPos">Vector in unity coordinate frame.</param>
    /// <returns>Vector in gazebo coordinate frame.</returns>
    public static Vector3 UnityPositionToGazebo(Vector3 unityPos)
    {
        return new Vector3(unityPos.x, unityPos.z, unityPos.y);
    }

    /// <summary>
    /// Converts a quaternion in gazebo coordinate frame to unity coordinate frame.
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
