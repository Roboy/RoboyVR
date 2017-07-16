
using UnityEngine;

/// <summary>
/// Sets the sorting layer of any found LineRenderer Component to "Ray"
/// </summary>
public class RayOrder : MonoBehaviour {

	// Use this for initialization
	void Awake () {
        gameObject.GetComponent<LineRenderer>().sortingLayerName = "Ray";
        Debug.Log("Layer set");
	}
}
