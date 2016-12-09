using UnityEngine;
using System.Collections;

public class MovementMenu : MonoBehaviour {

	// Use this for initialization
	void Start () {
	
	}
	
	// Update is called once per frame
	void Update () {
    transform.Rotate(0, 20 * Time.deltaTime, 0);

    }
}
