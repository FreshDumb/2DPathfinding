using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;


[CustomEditor(typeof(NavMeshManager))]
public class NavManagerCustomInspector : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        NavMeshManager myScript = (NavMeshManager)target;
        if (GUILayout.Button("Rebuild Navmesh"))
        {
            myScript.RebuildNavMesh();
        }
    }
}
