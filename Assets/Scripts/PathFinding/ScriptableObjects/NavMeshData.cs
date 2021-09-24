using System;
using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

[Serializable]
public class NavDataContainer_Row
{
    public NavigationDataContainer[] nodeData;

    public NavDataContainer_Row(int _size)
    {
        nodeData = new NavigationDataContainer[_size];
    }
}


[Serializable]
public class NavigationDataContainer
{
    public float m_distance = float.MaxValue;
    public int m_predecessorId = -1;
    public bool m_visited = false;

    public NavigationDataContainer(float _distance, int _predecessorId)
    {
        m_distance = _distance;
        m_predecessorId = _predecessorId;
        m_visited = false;
    }
}

[Serializable]
[CreateAssetMenu(fileName = "Data", menuName = "ScriptableObjects/SpawnManagerScriptableObject", order = 1)]
public class NavMeshData : ScriptableObject
{
    // TempContainers
    public List<NavNode2D> m_GraphNodes_temp;
    public List<Edge> m_GraphEdges_temp;
    public List<Edge> m_JumpEdges_temp;

    // Built Data
    public NavNode2D[] m_GraphNodes;

    public NavDataContainer_Row[] m_GraphNavigationData;
    
    public float m_iJumpTestSubsteps;
    public float m_gravityMagnitude;

    public void InitNavMeshData()
    {
        m_GraphNodes = new NavNode2D[m_GraphNodes_temp.Count];
        for (int i = 0; i < m_GraphNodes_temp.Count; i++)
        {
            m_GraphNodes[i] = m_GraphNodes_temp[i];
            m_GraphNodes[i].m_id = i;
#if UNITY_EDITOR
            EditorUtility.SetDirty(m_GraphNodes[i]);
#endif
        }

        BuildNavData();
    }

    void BuildNavData()
    {
        m_GraphNavigationData = new NavDataContainer_Row[m_GraphNodes.Length];
        for (int i = 0; i < m_GraphNavigationData.Length; i++)
        {
            m_GraphNavigationData[i] = new NavDataContainer_Row(m_GraphNodes.Length);
        }

        for (int i = 0; i < m_GraphNavigationData.Length; i++)
        {
            Dijkstra(i);
        }

#if UNITY_EDITOR
        EditorUtility.SetDirty(this);
#endif

        Debug.Log("Navigation Data Built!");
    }


    void Dijkstra(int _sourceID)
    {
        List<int> nodeSetIDs = new List<int>();
        NavDataContainer_Row tempRow = m_GraphNavigationData[_sourceID];

        for (int i = 0; i < tempRow.nodeData.Length; i++)
        {
            tempRow.nodeData[i] = new NavigationDataContainer(float.MaxValue, -1);
        }
        tempRow.nodeData[_sourceID].m_distance = 0;
        nodeSetIDs.Add(_sourceID);

        int Count = 0;
        while (nodeSetIDs.Count > 0)
        {
            int tempNodeID = nodeSetIDs[0];
            float closestNodeDistance = tempRow.nodeData[tempNodeID].m_distance;

            int indexToRemove = 0;
            for (int i = 0; i < nodeSetIDs.Count; i++)
            {
                if (tempRow.nodeData[nodeSetIDs[i]].m_distance < closestNodeDistance)
                {
                    tempNodeID = nodeSetIDs[i];
                    closestNodeDistance = tempRow.nodeData[nodeSetIDs[i]].m_distance;
                    indexToRemove = i;
                }
            }
            nodeSetIDs.RemoveAt(indexToRemove);

            tempRow.nodeData[tempNodeID].m_visited = true;
            foreach (Edge edge in m_GraphNodes[tempNodeID].m_neighbours)
            {
                float tempDistanceToNode = tempRow.nodeData[tempNodeID].m_distance + Mathf.Abs(edge.m_weight);
                if (tempDistanceToNode < tempRow.nodeData[edge.m_destination.m_id].m_distance)
                {
                    tempRow.nodeData[edge.m_destination.m_id].m_distance = tempDistanceToNode;
                    tempRow.nodeData[edge.m_destination.m_id].m_predecessorId = edge.m_source.m_id;
                }

                if (tempRow.nodeData[edge.m_destination.m_id].m_visited == false && 
                    nodeSetIDs.Contains(edge.m_destination.m_id) == false)
                {
                    nodeSetIDs.Add(edge.m_destination.m_id);
                }
            }
            Count++;
        }
    }

    public Stack<int> GetPathBacktracking(int _sourceID, int _targetID)
    {
        int currentNodeId = _targetID;

        Stack<int> path = new Stack<int>();
        Stack<int> path2 = new Stack<int>();
        path.Push(currentNodeId);
        path2.Push(currentNodeId);
        while (currentNodeId != _sourceID)
        {
            currentNodeId = m_GraphNavigationData[_sourceID].nodeData[currentNodeId].m_predecessorId;
            if (currentNodeId == -1)
            {
                return null;
            }
            path.Push(currentNodeId);
            path2.Push(currentNodeId);
        }
        VisualizePath(m_GraphNodes[_sourceID], path2);
        return path;
    }

    void VisualizePath(NavNode2D _source, Stack<int> _path)
    {
        NavNode2D currentNode = _source;
        _path.Pop();
        NavNode2D nextNode;
        while (_path.Count > 0)
        {
            if (currentNode.GetNeighbour(_path.Peek()).m_isJump == true)
            {
                Edge tempEdge = currentNode.GetNeighbour(_path.Pop());
                VisualizeJumpPath(tempEdge);
                nextNode = tempEdge.m_destination;
                currentNode = nextNode;
            }
            else
            {
                nextNode = currentNode.GetNeighbour(_path.Pop()).m_destination;
                Debug.DrawLine(currentNode.m_position, nextNode.m_position, Color.red);
                currentNode = nextNode;
            }

        }
    }

    void VisualizeJumpPath(Edge _jumpEdge)
    {
        float jumpTime = (_jumpEdge.m_destination.m_position.x - _jumpEdge.m_source.m_position.x) / _jumpEdge.m_jumpXVelocity;
        float tempVelocity = MathUtils.GetPreciseJumpStartVelocity(_jumpEdge.m_source.m_position.y, _jumpEdge.m_destination.m_position.y, jumpTime, m_gravityMagnitude);
        float singleTimeStep = jumpTime / m_iJumpTestSubsteps;
        List<Vector2> JumpSubsteps = new List<Vector2>();
        for (int i = 0; i < m_iJumpTestSubsteps; i++)
        {
            JumpSubsteps.Add(MathUtils.GetCurrentPositionInJump(singleTimeStep + singleTimeStep * i, _jumpEdge.m_source.m_position.x,
                _jumpEdge.m_source.m_position.y, tempVelocity, _jumpEdge.m_jumpXVelocity, m_gravityMagnitude));
        }
        Debug.DrawLine(_jumpEdge.m_source.m_position, JumpSubsteps[0], Color.red);

        for (int i = 0; i < JumpSubsteps.Count - 1; i++)
        {
            Debug.DrawLine(JumpSubsteps[i], JumpSubsteps[i + 1], Color.red);
        }
    }
}
