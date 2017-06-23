using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Utility class for several math functions.
/// </summary>
public static class MathUtility {

    /// <summary>
    /// Returns the angle within 0-360°.
    /// </summary>
    /// <param name="a"></param>
    /// <returns></returns>
    public static float WrapAngle(float a)
    {
        if (a > 360)
        {
            a -= 360;
        }
        if (a < 0)
        {
            a += 360;
        }
        return a;
    }

    /// <summary>
    /// Detects whether the second vector is turning clockwise in respect to the first one.
    /// Assumes that both vectors describe positions on the same circle.
    /// </summary>
    /// <param name="first">The vector which is compared to.</param>
    /// <param name="second">The vector which is compared to the first one.</param>
    /// <returns></returns>
    public static bool TurningClockwise(Vector2 first, Vector2 second)
    {
        if ((second.x - first.x) < 0) // if going towards the left
        {
            if (second.y > 0)
            {
                return false;
            }
            else
            {
                return true;
            }
        }
        if (second.y > 0)
        {
            return true;
        }
        return false;
    }

    /// <summary>
    /// Modulo function, as normal % just returns remainder and does not work for negative values.
    /// </summary>
    /// <param name="x">First operand</param>
    /// <param name="y">Second operand</param>
    /// <returns></returns>
    public static int Mod(int x, int y)
    {
        return (x % y + y) % y;
    }

    /// <summary>
    /// Returns a Vector2 which is rotated by the given angle (degrees) around the Z-axis.
    /// </summary>
    /// <param name="rotation">Angle in degrees.</param>
    /// <returns>Rotated vector in unit size.</returns>
    public static Vector2 RotateVectorDegrees(float rotation)
    {
        return new Vector2(Mathf.Cos(rotation * Mathf.Deg2Rad), Mathf.Sin(rotation * Mathf.Deg2Rad));
    }

    /// <summary>
    /// Returns a Vector2 which is rotated by the given angle (degrees) around the Z-axis.
    /// </summary>
    /// <param name="rotation">Angle in radians.</param>
    /// <returns>Rotated vector in unit size.</returns>
    public static Vector2 RotateVectorRadian(float rotation)
    {
        return new Vector2(Mathf.Cos(rotation), Mathf.Sin(rotation));
    }

    /// <summary>
    /// Returns an angle given a vector which describes a position on a circle.
    /// </summary>
    /// <param name="v">Vector describing the position.</param>
    /// <returns>Angle in degrees.</returns>
    public static float VectorToAngle(Vector2 v)
    {
        if (v == Vector2.zero)
            return float.NaN;
        // return the angle between the x-axis in degrees
        float angle = Mathf.Atan2(v.y, v.x) * Mathf.Rad2Deg;
        // wrap angle
        return WrapAngle(angle);
    }

    /// <summary>
    /// Converts an vector to an angle clockwise respective to the up vector.
    /// </summary>
    /// <param name="v">Vector describing the arrow position.</param>
    /// <returns>Angle in degrees.</returns>
    public static float VectorToClockAngle(Vector2 v)
    {
        if (v == Vector2.zero)
            return float.NaN;
        // do some magic shit
        float scaleProduct = Vector3.Dot(Vector3.up, v);
        float resultAngle = Mathf.Acos(scaleProduct / v.magnitude);
        int sign;

        if (v.x > 0)
            sign = 1;
        else
            sign = -1;

        resultAngle *= sign * Mathf.Rad2Deg;

        return WrapAngle(resultAngle);
    }

}
