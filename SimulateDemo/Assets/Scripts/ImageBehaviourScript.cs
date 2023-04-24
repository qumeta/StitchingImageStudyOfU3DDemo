using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;


/*
 * Qumeta：
*/
public class ImageBehaviourScript : MonoBehaviour
{
    public Information Information;
    public TestBehaviourScript RootScript;

    [SerializeField]
    private bool showFlag = true;

    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    void OnDrawGizmos()
    {
        if (Information == null || RootScript == null)
            return;

        if (showFlag && RootScript.ShowFlag)
        {
            var lastColor = Gizmos.color;

            Gizmos.color = Color.black;
#if UNITY_EDITOR
            //Handles.Label(this.transform.position, Information.DateTime.ToString(), new GUIStyle());
            Handles.Label(this.transform.position, this.name, new GUIStyle());
#endif
            Gizmos.color = lastColor;
        }
    }    
}
