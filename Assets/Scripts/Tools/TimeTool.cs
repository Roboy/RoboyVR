using UnityEngine;

public class TimeTool : ControllerTool {

    public GameObject Arrow;
	
	// Update is called once per frame
	void Update () {
        positionArrow();
        //Debug.Log("TimeToolID: " + m_SteamVRDevice.index);
    }

    /// <summary>
    /// Sets rotation such that finger position on the touchpad indicates the angle (center of touchpad = (0,0))
    /// </summary>
    void positionArrow()
    {
        Vector2 touchPos = Controller.GetAxis(Valve.VR.EVRButtonId.k_EButton_Axis0);

        float resultAngle = MathUtility.VectorToClockAngle(touchPos);

        if (float.IsNaN(resultAngle))
            return;

        Arrow.transform.localEulerAngles = new Vector3(0, resultAngle, 0);
    }
}
