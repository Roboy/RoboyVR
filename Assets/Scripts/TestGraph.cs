using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// UGLY SHIT ONLY FOR TESTING; DELETE THIS WHEN GRAPH OBJECT/ RENDERER IS DONE OR
/// REFACTOR TO A NICE EXAMPLE SCRIPT IN A DEMO FOLDER
/// </summary>
public class TestGraph : MonoBehaviour {

    GraphObject graph;

    float currentTimer = 0f;

	// Use this for initialization
	void Start () {
        graph = gameObject.AddComponent<GraphObject>();
        List<float> values = new List<float>();

        for (int i = 0; i < 100; i++)
        {
            values.Add(Random.Range(-100, 100));
        }
        graph.SetAutomaticAdjust();
        graph.Run(values, 100);

        

	}
	
	// Update is called once per frame
	void Update () {
        if (currentTimer > 0.1f)
        {
            graph.AddValue(Random.Range(-100, 100));
            currentTimer = 0f;
        }
        else
            currentTimer += Time.deltaTime;
        if (Input.GetKeyDown(KeyCode.A))
        {
            graph.SetManualAdjust(new Vector2(-1, 1));
            Debug.Log("Set");
        }
            
    }
}
