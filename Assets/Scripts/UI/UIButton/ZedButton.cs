using UnityEngine;
using UnityEngine.UI;
using System.Collections;

public class ZedButton : MonoBehaviour
{
    public Button button;

    void Start()
    {
        Button btn = button.GetComponent<Button>();
        btn.onClick.AddListener(TaskOnClick);
    }

    void TaskOnClick()
    {
        Debug.Log("You have clicked the button!");
        ViewSelectionManager.Instance.SwitchToZEDView();
    }
}