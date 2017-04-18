using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// ShootingTool is used to shoot a projectile on roboy. The projectile then triggers a ROS message to send an external force to the simulation.
/// </summary>
public class ShootingTool : ControllerTool {

    /// <summary>
    /// Projectile prefab which is responsible to send the ROS message.
    /// </summary>
    public Projectile ProjectilePrefab;

    /// <summary>
    /// Spawn transform to retrieve the spawn position and direction.
    /// </summary>
    public Transform SpawnPoint;

    /// <summary>
    /// Trigger transform for trigger animation.
    /// </summary>
    public Transform Trigger;

    /// <summary>
    /// Transform of the position when trigger is fully pressed.
    /// </summary>
    public Transform TriggerBack;
    
    /// <summary>
    /// Reload time between shots.
    /// </summary>
    [Range(0.1f, 1f)]
    public float ShootDelay = 0.5f;

    /// <summary>
    /// The standard trigger position.
    /// </summary>
    private Vector3 m_InitTriggerPosition;

    /// <summary>
    /// Variable for tracking current shooting cooldown.
    /// </summary>
    private float m_CurrentShootCooldown = 0f;

    /// <summary>
    /// Initializes trigger position.
    /// </summary>
    void Start()
    {
        m_InitTriggerPosition = Trigger.localPosition;
    }

    /// <summary>
    /// Shoots when the user presses the trigger to maximum value if shooting is not on cooldown.
    /// </summary>
	void Update () {

        // Shoot if trigger is pressed and shooting is not on cooldown
        if (m_SteamVRDevice.GetAxis(Valve.VR.EVRButtonId.k_EButton_Axis1).x > 0.99f && m_CurrentShootCooldown > ShootDelay /*&& m_SteamVRDevice.GetHairTriggerDown()*/)
        {          
            Shoot();
            Vibrate();

            m_CurrentShootCooldown = 0f;
        }

        // otherwise update cooldown
        m_CurrentShootCooldown += Time.deltaTime;
        // and animate trigger
        animateTrigger();
    }

    /// <summary>
    /// Instantiates a projectile prefab on the SpawnPoint.
    /// </summary>
    void Shoot()
    {
        Instantiate(ProjectilePrefab, SpawnPoint.position, transform.rotation);
    }

    /// <summary>
    /// Animates trigger based on current trigger value.
    /// </summary>
    void animateTrigger()
    {
        float triggerValue = m_SteamVRDevice.GetAxis(Valve.VR.EVRButtonId.k_EButton_Axis1).x;

        Trigger.localPosition = m_InitTriggerPosition + triggerValue * (TriggerBack.localPosition - m_InitTriggerPosition); 
    }
}
