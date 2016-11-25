using System;
using UnityEngine;
using System.Collections;

public class Rosbridge : MonoBehaviour
{
    public String Host;
    public Int32 Port;

    private Client c;

	// Use this for initialization
	void Start () {

	c = new Client();
    c.setHost(Host);
    c.setPort(Port);
    c.setupSocket();

	}
	
	// Update is called once per frame
	void Update ()
	{
	    Debug.Log("Ros sagt: "+c.readSocket());
	}
}
