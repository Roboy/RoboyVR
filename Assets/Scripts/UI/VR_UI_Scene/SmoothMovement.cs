using UnityEngine;

/// <summary>
/// Smoothes Movement of attached object.
/// This skript can be added to any moving object to smooth its movement, especially cameras and controlles.
/// </summary>
public class SmoothMovement : MonoBehaviour
{
    #region PRIVATE_MEMBER_VARIABLES
    /// <summary>
    /// previous position to take into consideration
    /// </summary>
    private Vector3 m_PrevPos = Vector3.zero;
    #endregion
    #region UNITY_MONOBEHAVIOR_METHODS
    /// <summary>
    /// Corrects position after update function (and position update) called
    /// </summary>
    void LateUpdate()
    {
        //TODO: local / world position differentiation?
        transform.position = Vector3.Lerp(transform.position, m_PrevPos, 0.5f);
        m_PrevPos = transform.position;
    }
    #endregion
}