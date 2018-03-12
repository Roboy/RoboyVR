using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using ROSBridgeLib.custom_msgs;
using ROSBridgeLib;

/// <summary>
/// Interface class to spawn a simulation model in Unity and a ROS node.
/// </summary>
public class PreviewModel : MonoBehaviour {

    /// <summary>
    /// The actual simulation model which can be spawned and is synchronized with a ROS node.
    /// </summary>
    [SerializeField]
    private GameObject SimulationModel;

    [SerializeField]
    private float CollisionRadius = 1f;

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
    private List<Renderer> m_Renderers = new List<Renderer>();

    private Material m_Material_True;

    private Material m_Material_Fail;

    private LayerMask m_LayerMask;

    void Start()
    {
        m_Material_True = Resources.Load("Insertinger/mat_insertModel_true") as Material;
        m_Material_Fail = Resources.Load("Insertinger/mat_insertModel_fail") as Material;

        Transform[] allChildren = GetComponentsInChildren<Transform>();
        
        foreach (var child in allChildren)
        {
            // get the renderer and copy all materials of the current child to the cached array
            Renderer renderer = child.gameObject.GetComponent<Renderer>();
            if (renderer != null)
            {
                m_Renderers.Add(renderer);
            }
                
        }
        // at the start set the state to non colliding
        changeMaterials(false);

        m_LayerMask = 1 << LayerMask.NameToLayer("ModelLayer");
    }

    public void FixedUpdate()
    {
        checkCollision();
    }

    public void CreateSimulationModel()
    {
        if (m_Colliding)
            return;
        GameObject simModel = Instantiate(SimulationModel, transform.position, transform.rotation);
        simModel.name = SimulationModel.name;
        //untag: this way, only our real roboy can be manipulated
        simModel.tag = "SpawnedModel";
        for (int i = 0; i < simModel.transform.childCount; i++)
        {
            simModel.transform.GetChild(i).gameObject.tag = "SpawnedModel";
        }
        InputManager.Instance.ModelSpawn_Controller.Operating = false;
        Destroy(InputManager.Instance.Selector_Tool.CurrentPreviewModel.gameObject);
        InputManager.Instance.Selector_Tool.CurrentPreviewModel = null;
        if (SimulationModel.GetComponent<ROSObject>() == null)
            return;
        // publish a message to add the object to ros node
        List<string> objects = new List<string>();
        objects.Add(SimulationModel.name);
        List<Vector3> positions = new List<Vector3>();
        positions.Add(GazeboUtility.UnityPositionToGazebo(transform.position));
        ModelMsg msg = new ModelMsg(1, 1, objects, positions);
        ROSBridge.Instance.Publish(RoboyModelPublisher.GetMessageTopic(), msg);
        RoboyManager.Instance.AddRoboy(simModel.transform);
    }

    /// <summary>
    /// Checks for collision with other models via an overlap sphere and changes the material when needed.
    /// </summary>
    private void checkCollision()
    {
        Collider[] hitColliders = Physics.OverlapSphere(transform.position, CollisionRadius, m_LayerMask);
        // this means we only collide with ourself
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

    private void OnDrawGizmosSelected()
    {
        Gizmos.DrawWireSphere(transform.position, CollisionRadius);
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
        foreach (var renderer in m_Renderers)
        {
            renderer.material = stateMaterial;
            for (int i = 0; i < renderer.materials.Length; i++)
            {
                renderer.materials[i] = stateMaterial;
            }
        }
        
    }
}
