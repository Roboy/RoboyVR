using UnityEngine;
using UnityEngine.UI;
using System.Collections;

public class SimButton : MonoBehaviour
{
    public Button button;

    void Start()
    {
        Button btn = button.GetComponent<Button>();
        btn.onClick.AddListener(TaskOnClick);
    }

    void TaskOnClick()
    {
        ViewSelectionManager.Instance.SwitchToSimulationView();
    }
}