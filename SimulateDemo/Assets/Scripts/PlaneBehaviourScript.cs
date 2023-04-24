using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;


/*
 * Qumeta：
*/
public class PlaneBehaviourScript : MonoBehaviour
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

        var color = transform.GetComponent<MeshRenderer>().sharedMaterial.color;
        transform.GetComponent<MeshRenderer>().sharedMaterial.color = new Color(color.r, color.g, color.b, RootScript.ShowFlag ? (float)100/255 : 0);

        //如果是当前选择节点时
        if (Selection.activeObject == this.gameObject 
            || Selection.activeObject == this.transform.parent.gameObject
            || Selection.activeObject == this.transform.parent.parent.gameObject)
        {
            transform.GetComponent<MeshRenderer>().sharedMaterial.color = new Color(color.r, color.g, color.b, (float)100 / 255);
        }

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
