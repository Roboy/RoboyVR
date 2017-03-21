using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ShootingTool : ControllerTool {

    public Projectile ProjectilePrefab;

    public Transform SpawnPoint;

    private Animator m_Animator;

    void Start()
    {
        m_Animator = GetComponent<Animator>();
    }

    // Update is called once per frame
	void Update () {

        if (m_SteamVRDevice.GetHairTriggerDown())
        {
            Shoot();
            Vibrate();
        }

        //Debug.Log("ShootingToolID: " + m_SteamVRDevice.index);
    }

    void Shoot()
    {
        Instantiate(ProjectilePrefab, SpawnPoint.position, transform.rotation);
        m_Animator.SetTrigger("shootTrigger");       
    }
}
