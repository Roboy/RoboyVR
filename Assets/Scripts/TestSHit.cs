using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROSBridgeLib;
using ROSBridgeLib.custom_msgs;

public class TestSHit : MonoBehaviour {

	// Use this for initialization
	void Start () {
		
	}
	
	// Update is called once per frame
	void Update () {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            List<string> objects = new List<string>();
            objects.Add("Roboy_simplified");
            List<Vector3> positions = new List<Vector3>();
            positions.Add(GazeboUtility.UnityPositionToGazebo(new Vector3(2.3123f, 2.2311f, 0.5f)));
            ModelMsg msg = new ModelMsg(1, 1, objects, positions);
            Debug.Log(msg.ToYAMLString());
            ROSBridge.Instance.Publish(RoboyModelPublisher.GetMessageTopic(), msg);
        }
        else if (Input.GetKeyDown(KeyCode.Escape))
        {
            List<string> objects = new List<string>();
            objects.Add("Roboy_simplified");
            List<Vector3> positions = new List<Vector3>();
            positions.Add(Vector3.zero);
            ModelMsg msg = new ModelMsg(0, 1, objects, positions);

            ROSBridge.Instance.Publish(RoboyModelPublisher.GetMessageTopic(), msg);
        }
    }
}
