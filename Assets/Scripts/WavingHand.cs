using UnityEngine;
using System.Collections;

public class WavingHand : MonoBehaviour {
    bool reverse;
    public float speed = 10f;

    // Use this for initialization
    void Start () {

        reverse = false;
        
	
	}
	
	// Update is called once per frame
	void Update () {

        
	    if (reverse == false)
        {
            transform.Rotate(-speed * Time.deltaTime, 0, 0);
        }
        if (transform.rotation.eulerAngles.x < 15.0f | transform.rotation.eulerAngles.x > 75.0f)
        {
            reverse = !(reverse);
        }
        if (reverse == true)
        {
            transform.Rotate(speed * Time.deltaTime, 0, 0);
        }

        Debug.Log(reverse);
        Debug.Log(transform.rotation.eulerAngles.x);

    }
}
