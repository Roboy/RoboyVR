using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TimeTool : ControllerTool {

    public GameObject Arrow;
	
	// Update is called once per frame
	void Update () {
        positionArrow();
        //Debug.Log("TimeToolID: " + m_SteamVRDevice.index);
    }

    void positionArrow()
    {
        Vector2 touchPos = m_SteamVRDevice.GetAxis(Valve.VR.EVRButtonId.k_EButton_Axis0);

        float scaleProduct = Vector3.Dot(Vector3.up, touchPos);
        float lengthProduct = touchPos.magnitude;
        float resultAngle = Mathf.Acos(scaleProduct / lengthProduct);
        int sign;

        if (touchPos.x > 0)
            sign = 1;
        else
            sign = -1;

        resultAngle *= sign * Mathf.Rad2Deg;

        if (float.IsNaN(resultAngle))
            return;

        Arrow.transform.localEulerAngles = new Vector3(0, resultAngle, 0);

        //Debug.Log(resultAngle * Mathf.Rad2Deg * sign);

        //Debug.Log("X: " + touchPos.x + " Y: " + touchPos.y);
        //Vector3 newPos = new Vector3(touchPos.x, touchPos.y, );
        //Arrow.transform.LookAt(newPos);
    }
}
