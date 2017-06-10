using UnityEngine;

/// <summary>
/// Smoothes Movement of attached object.
/// This skript can be added to any moving object to smooth its movement, especially cameras and controlles.
/// </summary>
public class SmoothMovement : MonoBehaviour {

    private Vector3 prevPos = Vector3.zero;
    /// <summary>
    /// Corrects position after update function (and position update) called
    /// </summary>
    void LateUpdate () {
        //TODO: local / world position differentiation?
        transform.position = Vector3.Lerp(transform.position, prevPos, 0.5f);
        prevPos = transform.position;
	}
}