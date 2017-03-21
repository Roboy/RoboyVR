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
}
