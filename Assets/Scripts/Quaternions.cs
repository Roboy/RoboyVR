using UnityEngine;
using System.Collections;

public class Quaternions : MonoBehaviour
{
   

	// Use this for initialization
	void Start () {
        Debug.Log(transform.rotation.eulerAngles.ToString());
        Quaternion rot = Quaternion.Inverse(transform.rotation);
	    Vector3 euler = rot.eulerAngles;
	    float eulerY = euler.y;
	    euler.x = -euler.x;
	    euler.y = -euler.z;
	    euler.z = eulerY;
        transform.rotation = Quaternion.Euler(euler);
	}
	
	// Update is called once per frame
	void Update ()
	{
        Vector3 rot = transform.rotation.eulerAngles;
        //rot = new Vector3(rot.x, rot.y + 45, rot.z);
        //transform.rotation = Quaternion.Euler(rot);


	    
        //Debug.Log(transform.rotation.ToString());

    }
}
