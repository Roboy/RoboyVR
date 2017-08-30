using UnityEngine;

/// <summary>
/// Sets the sorting layer of any found LineRenderer Component to "Ray"
/// Needed for Canvas/Ray order (see SelectorTool)
/// </summary>
public class RayOrder : MonoBehaviour {

	/// <summary>
    /// Sets sorting layer of any found linerenderer
    /// </summary>
	void Awake () {
        gameObject.GetComponent<LineRenderer>().sortingLayerName = "Ray";
        Debug.Log("[" + gameObject.name + "] LineRenderer Layer set");
	}
}
