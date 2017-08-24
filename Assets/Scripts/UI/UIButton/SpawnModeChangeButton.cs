using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// A button on the SpawnModelController which changes the mode between inserting and removing.
/// </summary>
[RequireComponent(typeof(Button))]
public class SpawnModeChangeButton : MonoBehaviour {

    private Text m_Text;
	// Use this for initialization
	void Start () {
        GetComponent<Button>().onClick.AddListener(changeSpawnMode);
        m_Text = GetComponentInChildren<Text>();
        m_Text.text = "Insert Model";
	}
	
	void changeSpawnMode()
    {
        switch (ModeManager.Instance.CurrentSpawnViewerMode)
        {
            case ModeManager.SpawnViewerMode.Insert:
                ModeManager.Instance.CurrentSpawnViewerMode = ModeManager.SpawnViewerMode.Remove;
                m_Text.text = "Remove Model";
                break;
            case ModeManager.SpawnViewerMode.Remove:
                ModeManager.Instance.CurrentSpawnViewerMode = ModeManager.SpawnViewerMode.Insert;
                m_Text.text = "Insert Model";
                break;
        }
    }
}
