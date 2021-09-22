using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[Serializable]
public class NavigationDataContainer_Row
{
    public NavigationDataContainer[] nodeData;

    public NavigationDataContainer_Row(int _size)
    {
        nodeData = new NavigationDataContainer[_size];
    }
}


[Serializable]
public class NavigationDataContainer
{
    public float distance;
    public int predecessorId;
    public bool visited;

    public NavigationDataContainer(float _distance, int _predecessorId)
    {
        distance = _distance;
        predecessorId = _predecessorId;
        visited = false;
    }
    public void setDistance(float _distance)
    {
        distance = _distance;
    }
    public void setPredecessor(int _predecessorId)
    {
        predecessorId = _predecessorId;
    }
    public void setVisited()
    {
        visited = true;
    }
}


[CreateAssetMenu(fileName = "Data", menuName = "ScriptableObjects/SpawnManagerScriptableObject", order = 1)]
public class NavMeshData : ScriptableObject
{
    // TempContainers
    public List<NavNode2D> m_GraphNodes_temp;
    public List<Edge> m_GraphEdges_temp;
    public List<Edge> m_JumpEdges_temp;

    // Built Data
    public NavNode2D[] m_GraphNodes;
    public Vector2[] m_NodePositions;

    public NavigationDataContainer_Row[] m_GraphNavigationData;
    //public NavigationDataContainer[,] m_GraphNavigationData;

    public void InitNavMeshData()
    {
        m_GraphNodes = new NavNode2D[m_GraphNodes_temp.Count];
        m_NodePositions = new Vector2[m_GraphNodes_temp.Count];
        for (int i = 0; i < m_GraphNodes_temp.Count; i++)
        {
            m_NodePositions[i] = m_GraphNodes_temp[i].m_position;
            m_GraphNodes[i] = m_GraphNodes_temp[i];
            m_GraphNodes[i].m_id = i;
        }

        BuildNavData();
    }

    void BuildNavData()
    {
        m_GraphNavigationData = new NavigationDataContainer_Row[m_GraphNodes.Length];
        for (int i = 0; i < m_GraphNavigationData.Length; i++)
        {
            m_GraphNavigationData[i] = new NavigationDataContainer_Row(m_GraphNodes.Length);
        }

        for (int i = 0; i < m_GraphNavigationData.Length; i++)
        {
            Dijkstra(i);
        }
        Debug.Log("Navigation Data Built!");
    }


    bool Dijkstra(int _sourceID)
    {
        List<int> nodeSetIDs = new List<int>();
        NavigationDataContainer_Row tempRow = m_GraphNavigationData[_sourceID];

        for (int i = 0; i < tempRow.nodeData.Length; i++)
        {
            tempRow.nodeData[i] = new NavigationDataContainer(float.MaxValue, -1);
        }
        tempRow.nodeData[_sourceID].setDistance(0);
        nodeSetIDs.Add(_sourceID);

        int Count = 0;
        while (nodeSetIDs.Count > 0)
        {
            int tempNodeID = nodeSetIDs[0];
            float closestNodeDistance = tempRow.nodeData[tempNodeID].distance;

            int indexToRemove = 0;
            for (int i = 0; i < nodeSetIDs.Count; i++)
            {
                if (tempRow.nodeData[nodeSetIDs[i]].distance < closestNodeDistance)
                {
                    tempNodeID = nodeSetIDs[i];
                    closestNodeDistance = tempRow.nodeData[nodeSetIDs[i]].distance;
                    indexToRemove = i;
                }
            }
            nodeSetIDs.RemoveAt(indexToRemove);

            tempRow.nodeData[tempNodeID].setVisited();
            foreach (Edge edge in m_GraphNodes[tempNodeID].m_neighbours)
            {
                float tempDistanceToNode = tempRow.nodeData[tempNodeID].distance + Mathf.Abs(edge.m_weight);
                if (tempDistanceToNode < tempRow.nodeData[edge.m_destination.m_id].distance)
                {
                    tempRow.nodeData[edge.m_destination.m_id].setDistance(tempDistanceToNode);
                    tempRow.nodeData[edge.m_destination.m_id].setPredecessor(edge.m_source.m_id);
                }

                if (tempRow.nodeData[edge.m_destination.m_id].visited == false && 
                    nodeSetIDs.Contains(edge.m_destination.m_id) == false)
                {
                    nodeSetIDs.Add(edge.m_destination.m_id);
                }
            }
            Count++;
        }
        return true;
    }

    public Stack<int> GetRecursivePath(int _sourceID, int _targetID)
    {
        int currentNodeId = _targetID;

        Stack<int> path = new Stack<int>();
        Stack<int> path2 = new Stack<int>();
        path.Push(currentNodeId);
        path2.Push(currentNodeId);
        while (currentNodeId != _sourceID)
        {
            currentNodeId = m_GraphNavigationData[_sourceID].nodeData[currentNodeId].predecessorId;
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
            //if (currentNode.getNeighbour(_path.Peek()).m_isJump == true)
            //{
            //    Edge tempEdge = currentNode.getNeighbour(_path.Pop());
            //    VisualizeJumpPath(tempEdge);
            //    nextNode = tempEdge.m_destination;
            //    currentNode = nextNode;
            //}
            //else
            //{
            //    nextNode = currentNode.getNeighbour(_path.Pop()).m_destination;
            //    Debug.DrawLine(currentNode.m_position, nextNode.m_position, Color.red);
            //    currentNode = nextNode;
            //}
            nextNode = currentNode.getNeighbour(_path.Pop()).m_destination;
            Debug.DrawLine(currentNode.m_position, nextNode.m_position, Color.red);
            currentNode = nextNode;
        }
    }
}
