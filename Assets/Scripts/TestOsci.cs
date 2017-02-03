using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TestOsci : MonoBehaviour
{

    public GraphRenderer g;

    private List<float> values = new List<float>();

	// Use this for initialization
	void Start () {
	    	
            //g.Play();
	    for (int i = 0; i < 30; i++)
	    {
	        values.Add(0f);
	    }
	}
	
	// Update is called once per frame
	void Update () {
        if(Input.GetKeyDown(KeyCode.I))
            g.Initialize(values, 10, 5f);
        if (Input.GetKeyDown(KeyCode.P))
            g.Play();
        if(Input.GetKeyDown(KeyCode.U))
            g.Pause();
        if(Input.GetKeyDown(KeyCode.A))
            addValue();
        if(Input.GetKeyDown(KeyCode.S))
            g.Stop();
        if(Input.GetKeyDown(KeyCode.R))
            addValueOver9000();
        if (Input.GetKeyDown(KeyCode.T))
            addValueUnder9000();
    }

    void addValue()
    {
        float newValue = Random.Range(0, 100);
        ExtensionMethods.ShiftRight(values, 1);

        values[0] = newValue;
    }

    void addValueOver9000()
    {
        float newValue = 9001;
        ExtensionMethods.ShiftRight(values, 1);

        values[0] = newValue;
    }
    void addValueUnder9000()
    {
        float newValue = -9001;
        ExtensionMethods.ShiftRight(values, 1);

        values[0] = newValue;
    }
}
