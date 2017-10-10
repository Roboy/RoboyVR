using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// A button on the SpawnModelController which spawns a preview model of a simulation model when pressed.
/// </summary>
[RequireComponent(typeof(Button))]
public class ModelButton : MonoBehaviour {

    /// <summary>
    /// The instance of the preview model which should be spawned.
    /// </summary>
    public PreviewModel SpawnedModel
    {
        get { return m_SpawnedModel; }
    }

    /// <summary>
    /// Serialized so we can set this in the editor. 
    /// </summary>
    [SerializeField]
    private PreviewModel PreviewModel;

    private PreviewModel m_SpawnedModel;

	// Use this for initialization
	void Start () {
        GetComponent<Button>().onClick.AddListener(createPreviewModel);
	}

    /// <summary>
    /// Intermediary function so we can wait in the coroutine one frame between changing the spawn viewer mode and setting the preview model in selectorTool.
    /// </summary>
    private void createPreviewModel()
    {
        if (ModeManager.Instance.CurrentSpawnViewerMode != ModeManager.SpawnViewerMode.Insert)
            return;
        StartCoroutine(createPreviewModelCoroutine());
    }

    /// <summary>
    /// Instantiate the preview model and change the current spawn viewer mode in the ModeManager.
    /// </summary>
    private IEnumerator createPreviewModelCoroutine()
    {
        // do nothing if model spawn controller is already in removing/add state
        if (InputManager.Instance.ModelSpawn_Controller.Operating == true)
            yield break;
        if (PreviewModel == null)
            yield break;

        m_SpawnedModel = Instantiate(PreviewModel, Vector3.zero, PreviewModel.transform.rotation);
        m_SpawnedModel.gameObject.name = PreviewModel.gameObject.name;
        InputManager.Instance.ModelSpawn_Controller.Operating = true;
        yield return null;
        InputManager.Instance.Selector_Tool.CurrentPreviewModel = m_SpawnedModel;
    }
}
