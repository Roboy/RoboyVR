using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

/// <summary>
/// Interface class to spawn a simulation model in Unity and a ROS node.
/// </summary>
public class PreviewModel : MonoBehaviour {

    /// <summary>
    /// The actual simulation model which can be spawned and is synchronized with a ROS node.
    /// </summary>
    [SerializeField]
    private GameObject SimulationModel;

    /// <summary>
    /// We need this boolean to avoid spawning models which collide and therefore trigger a collision in ROS and making boom boom.
    /// </summary>
    private bool m_Colliding = false;

    /// <summary>
    /// Helper variable so we save performance and do not need to change the materials every frame but every change instead.
    /// </summary>
    private bool m_LastCollidingValue = false;

    /// <summary>
    /// We need this list so we can cache all materials to indicate whether the model can be inserted or not.
    /// </summary>
    private Material[] m_Materials;

    private Material m_Material_True;

    private Material m_Material_Fail;

    void Start()
    {
        m_Material_True = Resources.Load("Insertinger/mat_insertModel_true") as Material;
        m_Material_Fail = Resources.Load("Insertinger/mat_insertModel_fail") as Material;

        Transform[] allChildren = GetComponentsInChildren<Transform>();
        
        foreach (var child in allChildren)
        {
            // get the renderer and copy all materials of the current child to the cached array
            Renderer renderer = child.gameObject.GetComponent<Renderer>();
            Array.Copy(renderer.materials, m_Materials, renderer.materials.Length);
        }
    }

    public void FixedUpdate()
    {
        checkCollision();
    }

    public void CreateSimulationModel()
    {
        if (m_Colliding)
            return;
        Instantiate(SimulationModel, transform.position, transform.rotation);
        ModeManager.Instance.CurrentSpawnViewerMode = ModeManager.SpawnViewerMode.Idle;
    }

    /// <summary>
    /// Checks for collision with other models via an overlap sphere and changes the material when needed.
    /// </summary>
    private void checkCollision()
    {
        Collider[] hitColliders = Physics.OverlapSphere(transform.position, 2.0f);
        if (hitColliders.Length == 0)
        {
            m_Colliding = false;
        }
        else
        {
            m_Colliding = true;
        }
        if (m_LastCollidingValue != m_Colliding)
        {
            m_LastCollidingValue = m_Colliding;
            changeMaterials(m_Colliding);
        }
    }

    /// <summary>
    /// Change the materials of the preview model to fail or true depending whether the model is colliding with another model or not.
    /// </summary>
    /// <param name="state"></param>
    private void changeMaterials(bool state)
    {
        Material stateMaterial;
        // if we are colliding set the material to fail otherwise to true
        if (state)
        {
            stateMaterial = m_Material_Fail;
        }
        else
        {
            stateMaterial = m_Material_True;
        }
        foreach (var material in m_Materials)
        {
            material.color = stateMaterial.color;
        }
    }


}
