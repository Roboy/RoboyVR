using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class JSONSAVE : MonoBehaviour
{
    [HideInInspector]
    public Vector3 Position;
    [HideInInspector]
    public Quaternion Rotation;
    [HideInInspector]
    public List<Test> TestList = new List<Test>();

    private List<string> savedData = new List<string>();

    public int m_Index = 0;
    public string Value;

    private enum State
    {
        Saving,
        Playing
    }

    private State m_CurrentState = State.Saving;

    // Update is called once per frame
	void Update () {
        if(m_CurrentState == State.Saving)
		    saveData();

        if(Input.GetKeyDown(KeyCode.L))
            loadData();
	}

    void saveData()
    {
        Position = transform.position;
        Rotation = transform.rotation;
        Test t = new Test () { Index = m_Index++, Value = Random.Range(500, 700).ToString()};
        Value = t.Value;
        TestList.Add(t);
        savedData.Add(JsonUtility.ToJson(this));
    }

    void loadData()
    {
        Debug.Log("Start loading");
        m_CurrentState = State.Playing;
        StartCoroutine(loadDataCoroutine());    
    }

    IEnumerator loadDataCoroutine()
    {
        List<string> originSavedData = savedData;
        savedData.Reverse();
        foreach (string save in savedData)
        {
            JsonUtility.FromJsonOverwrite(save, this);
            transform.position = Position;
            transform.rotation = Rotation;
            m_Index = TestList[originSavedData.IndexOf(save)].Index;
            Value = TestList[originSavedData.IndexOf(save)].Value;

            yield return null;
        }

        m_CurrentState = State.Saving;

        Debug.Log("Saving again!");
    }

    public class Test
    {
        public int Index;
        public string Value;
    }
}
