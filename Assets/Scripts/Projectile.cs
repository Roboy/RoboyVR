using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Projectile : MonoBehaviour
{

    public float projectileSpeed;
	// Use this for initialization
	void Start () {
		
	}
	
	// Update is called once per frame
	void Update () {
		transform.Translate(Vector3.back * projectileSpeed);
	    if (transform.position.magnitude > 20.0f)
	    {
            Destroy(gameObject);
	    }
	}

    void OnCollisionEnter(Collision collision)
    {
        RoboyPart roboyPart;
        if ((roboyPart = collision.gameObject.GetComponent<RoboyPart>()) != null)
        {          
            Vector3 forcePosition = collision.transform.InverseTransformPoint(transform.position);
            Vector3 forceDirection = collision.transform.InverseTransformDirection(transform.forward * -1f * 1500);
            int duration = 200;
            RoboyManager.Instance.ReceiveExternalForce(roboyPart, forcePosition, forceDirection, duration);

            Debug.Log("Hit at: " + collision.gameObject.name);
            //Debug.Log(forceDirection);
        }
    }
}
