using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// Projectile moves forward and triggers a ROS external force message if it hits roboy.
public class Projectile : MonoBehaviour
{
    /// <summary>
    /// The speed of the projectile.
    /// </summary>
    public float projectileSpeed;
	
    /// <summary>
    /// Move forward and destroy yourself if you are not in the roboy cave.
    /// </summary>
	void Update () {
		transform.Translate(Vector3.back * projectileSpeed);
	    if (transform.position.magnitude > 20.0f)
	    {
            Destroy(gameObject);
	    }
	}

    /// <summary>
    /// Triggers a ROS external force message. Transforms the hit point from world space to roboy local space.
    /// </summary>
    /// <param name="collision"></param>
    void OnCollisionEnter(Collision collision)
    {
        RoboyPart roboyPart;
        // If the hitted object is roboy
        if ((roboyPart = collision.gameObject.GetComponent<RoboyPart>()) != null)
        {   
            // Transform the position to roboy space
            Vector3 forcePosition = collision.transform.InverseTransformPoint(transform.position);
            // transform the direction to roboy space
            Vector3 forceDirection = collision.transform.InverseTransformDirection(transform.forward * -1f * 1500);
            int duration = 200;
            // trigger the message in RoboyManager
            RoboyManager.Instance.ReceiveExternalForce(roboyPart, forcePosition, forceDirection, duration);
        }
    }
}
