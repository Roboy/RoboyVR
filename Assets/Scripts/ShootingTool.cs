using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ShootingTool : ControllerTool {

    public Projectile ProjectilePrefab;

    public Transform SpawnPoint;

    public Transform Trigger;

    public Transform TriggerBack;

    [Range(0.1f, 1f)]
    public float ShootDelay = 0.5f;

    private Vector3 m_InitTriggerPosition;

    private float m_CurrentShootCooldown = 0f;

    void Start()
    {
        m_InitTriggerPosition = Trigger.localPosition;
    }

    // Update is called once per frame
	void Update () {

        if (m_SteamVRDevice.GetAxis(Valve.VR.EVRButtonId.k_EButton_Axis1).x > 0.99f && m_CurrentShootCooldown > ShootDelay /*&& m_SteamVRDevice.GetHairTriggerDown()*/)
        {          
            Shoot();
            Vibrate();

            m_CurrentShootCooldown = 0f;
        }

        m_CurrentShootCooldown += Time.deltaTime;

        animateTrigger();
    }

    void Shoot()
    {
        Instantiate(ProjectilePrefab, SpawnPoint.position, transform.rotation);
    }

    void animateTrigger()
    {
        float triggerValue = m_SteamVRDevice.GetAxis(Valve.VR.EVRButtonId.k_EButton_Axis1).x;

        Trigger.localPosition = m_InitTriggerPosition + triggerValue * (TriggerBack.localPosition - m_InitTriggerPosition); 
    }
}
