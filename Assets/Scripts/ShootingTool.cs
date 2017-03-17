using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ShootingTool : ControllerTool {

    public Projectile ProjectilePrefab;

    public Transform SpawnPoint;
	
	// Update is called once per frame
	void Update () {

        if (m_SteamVRDevice.GetHairTriggerDown())
        {
            Shoot();
            Vibrate();
        }
    }

    void Shoot()
    {
        Instantiate(ProjectilePrefab, SpawnPoint.position, transform.rotation);
    }
}
