using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.UI;

public class NavMeshManager : MonoBehaviour
{
    public NavMeshData m_navMeshDataSORef;

    public List<Vector2> m_allNodes;
    public List<Vector2> m_doubledNodes;

    Queue<Edge> m_checkForJump;

    public float m_fPointDistance = 0.1f;
    public float m_fDoubleNodeDistance = 0.01f;
    public int m_iJumpTestSubsteps = 20;

    public float m_fJumpHeight = 3.0f;
    public float m_fmoveSpeed = 4.0f;
    public float m_fjumpHorMoveSpeed = 4.0f;

    public float m_fJumpStartVelocity;
    float m_fJumpTime;

    public float m_gravityMagnitude = 10.0f;

    public bool RebuildNavMesh()
    {
        if (m_navMeshDataSORef == null)
        {
            Debug.Log("Nav Mesh Data Container Scriptable Object Ref Missing!");
            return false;
        }

        m_allNodes.Clear();
        m_doubledNodes.Clear();

        m_navMeshDataSORef.m_GraphNodes_temp = new List<NavNode2D>();
        m_navMeshDataSORef.m_GraphEdges_temp = new List<Edge>();
        m_navMeshDataSORef.m_JumpEdges_temp = new List<Edge>();
        m_navMeshDataSORef.m_GraphNodes = new NavNode2D[0];

        m_checkForJump = new Queue<Edge>();

        m_fJumpStartVelocity = Mathf.Sqrt(2.0f * m_gravityMagnitude * m_fJumpHeight);
        m_fJumpTime = m_fJumpStartVelocity / m_gravityMagnitude;

        print("JumpTime: " + m_fJumpTime);
        print("Jumptime Quadratic: " + MathUtils.MinInAirTime(0, 100, m_fJumpStartVelocity, m_gravityMagnitude));
        print("Jumptime Quadratic: " + MathUtils.MaxInAirTime(0, 100, m_fJumpStartVelocity, m_gravityMagnitude));

        CreateNodePositions();

        Debug.Log("Number of Nodes: " + m_allNodes.Count);
        ClearDoubleNodes();

        CreateGraphNodesFromPositions();
        ConstructEdges();

        CheckAllNodesForJumpEdges();
        EvaluateJumpEdges();

        //foreach (Edge edge in m_JumpEdges)
        //{
        //    VisualizeJumpPath(edge);
        //}

        //GetRecursivePath(m_GraphNodes[0], m_GraphNodes[m_GraphNodes.Count - 1]);

        m_navMeshDataSORef.m_gravityMagnitude = m_gravityMagnitude;
        m_navMeshDataSORef.m_iJumpTestSubsteps = m_iJumpTestSubsteps;
        m_navMeshDataSORef.InitNavMeshData();

#if UNITY_EDITOR
        EditorUtility.SetDirty(this);
#endif
        return true;
    }

    void CreateNodePositions()
    {
        PolygonCollider2D[] allColliders = FindObjectsOfType<PolygonCollider2D>();
        foreach (PolygonCollider2D col in allColliders)
        {
            Vector2 point1 = col.transform.TransformPoint(col.points[0]);
            Vector2 point2 = col.transform.TransformPoint(col.points[col.points.Length - 1]);

            AddNodesAlongLine(point1, point2);

            for (int i = 0; i < col.points.Length - 1; i++)
            {
                point1 = col.transform.TransformPoint(col.points[i]);
                point2 = col.transform.TransformPoint(col.points[i + 1]);

                AddNodesAlongLine(point1, point2);
            }
        }
    }

    void AddNodesAlongLine(Vector2 _point1, Vector2 _point2)
    {
        TryAddingNodeToList(_point1);

        Vector2 DirectionsOfLine = (_point2 - _point1).normalized;
        float DistanceBetweenPoints = (_point2 - _point1).magnitude;

        if(DistanceBetweenPoints >= m_fPointDistance * 2)
        {
            float stepsBetweenPoints = DistanceBetweenPoints / m_fPointDistance;
            int stepsBetweenPointsRounded = (int)stepsBetweenPoints;


            for (int i = 1; i <= stepsBetweenPointsRounded; i++)
            {
                Vector2 tempPointToAdd = _point1 + DirectionsOfLine * i * m_fPointDistance;

                TryAddingNodeToList(tempPointToAdd);
            }
        }
        TryAddingNodeToList(_point2);
    }

    bool TryAddingNodeToList(Vector2 _pointToAdd)
    {
        RaycastHit2D tempHit = Physics2D.Raycast(_pointToAdd + Vector2.up * 0.01f, Vector2.down, 0.015f);
        if (tempHit.collider == null || tempHit.distance < 0.008f)
        {
            //  Debug.Log("Not Added: " + tempHit.distance);
            return false;
        }
        else
        {
            //  Debug.Log("Added Node: " + tempHit.distance);
            m_allNodes.Add(_pointToAdd);
            return true;
        }
    }

    void ClearDoubleNodes()
    {
        List<int> tempIndicesMarkedForSkipping = new List<int>();
        List<Vector2> newListOfNodes = new List<Vector2>();

        for (int i = 0; i < m_allNodes.Count; i++)
        {
            for (int j = 0; j < m_allNodes.Count; j++)
            {
                if(i == j)
                {

                }
                else
                {
                    if (Vector2.Distance(m_allNodes[i], m_allNodes[j]) <= m_fDoubleNodeDistance)
                    {
                        int tempIndice = Mathf.Min(i, j);
                        if (tempIndicesMarkedForSkipping.Contains(tempIndice) == false)
                        {
                            //  Debug.Log("DoubledIndex: " + tempIndice);
                            tempIndicesMarkedForSkipping.Add(tempIndice);
                        }
                    }
                }
            }
        }

        for (int i = 0; i < m_allNodes.Count; i++)
        {
            if(tempIndicesMarkedForSkipping.Contains(i) == false)
            {
                newListOfNodes.Add(m_allNodes[i]);
            }
            else
            {
                //  Debug.Log("Skipped Nodes because doubled!");
                m_doubledNodes.Add(m_allNodes[i]);
            }
        }
        m_allNodes.Clear();
        m_allNodes = newListOfNodes;
    }

    void CreateGraphNodesFromPositions()
    {
        string[] nodeAssetPaths = { "Assets/BuiltData/Nodes" };
        List<string> outFailedPaths = new List<string>();
        AssetDatabase.DeleteAssets(nodeAssetPaths, outFailedPaths);
        AssetDatabase.CreateFolder("Assets/BuiltData", "Nodes");
        AssetDatabase.SaveAssets();
        AssetDatabase.Refresh();

        EditorUtility.UnloadUnusedAssetsImmediate();
        Resources.UnloadUnusedAssets();
        System.GC.Collect();


        for (int i = 0; i < m_allNodes.Count; i++)
        {
            m_navMeshDataSORef.m_GraphNodes_temp.Add(ScriptableObject.CreateInstance<NavNode2D>());
            AssetDatabase.CreateAsset(m_navMeshDataSORef.m_GraphNodes_temp[i], "Assets/BuiltData/Nodes/Node_" + i + ".asset");
            m_navMeshDataSORef.m_GraphNodes_temp[i].m_id = i;
            m_navMeshDataSORef.m_GraphNodes_temp[i].m_position = m_allNodes[i];

            Debug.Log("Added Node " + i);
        }

        AssetDatabase.SaveAssets();
    }

    void ConstructEdges()
    {
        for (int i = 0; i < m_navMeshDataSORef.m_GraphNodes_temp.Count; i++)
        {
            for (int j = 0; j < m_navMeshDataSORef.m_GraphNodes_temp.Count; j++)
            {
                if((i == j) == false)
                {
                    Edge tempEdge = TryAddingEdge(m_navMeshDataSORef.m_GraphNodes_temp[i], m_navMeshDataSORef.m_GraphNodes_temp[j]);
                    if (tempEdge != null)
                    {
                        SetNodeNeighbour(tempEdge);
                    }
                }
            }
        }
        for (int i = 0; i < m_navMeshDataSORef.m_GraphNodes_temp.Count; i++)
        {
            if(m_navMeshDataSORef.m_GraphNodes_temp[i].m_neighbours == null)
            {
                m_navMeshDataSORef.m_GraphNodes_temp.RemoveAt(i);
                i--;
            }
            else if(m_navMeshDataSORef.m_GraphNodes_temp[i].m_neighbours.Count == 0)
            {
                m_navMeshDataSORef.m_GraphNodes_temp.RemoveAt(i);
                i--;
            }
        }
    }

    Edge TryAddingEdge(NavNode2D _startNode, NavNode2D _targetNode)
    {
        float tempDistance = Vector2.Distance(_startNode.m_position, _targetNode.m_position);
        if (tempDistance <= m_fPointDistance * 1.1f)
        {
            Vector2 tempDirection = (_targetNode.m_position - _startNode.m_position).normalized;
            float tempSignedAngle = Vector2.SignedAngle(Vector2.right, tempDirection);

            Vector2 tempLocalUp;
            if (Mathf.Abs(tempSignedAngle) <= 90)
            {
                float tempAngle = tempSignedAngle + 90;
                tempLocalUp = new Vector2(Mathf.Cos(tempAngle * Mathf.Deg2Rad),
                                          Mathf.Sin(tempAngle * Mathf.Deg2Rad));
            }
            else
            {
                float tempAngle = tempSignedAngle - 90;
                tempLocalUp = new Vector2(Mathf.Cos(tempAngle * Mathf.Deg2Rad),
                                          Mathf.Sin(tempAngle * Mathf.Deg2Rad));
            }


            RaycastHit2D tempHit = Physics2D.Raycast(_startNode.m_position + tempLocalUp * 0.001f, tempDirection, tempDistance);

            if ((tempHit.collider == null) == false)
            {
                Debug.Log("! Cant Go Through Collider !  " + tempHit.distance);

                return null;
            }
            else
            {
                float tempSign = Mathf.Sign(tempSignedAngle);
                Debug.Log(tempSignedAngle);
                if (Mathf.Abs(tempSignedAngle) == 180)
                {
                    tempSign = -1;
                }
                Edge tempEdge = new Edge(m_navMeshDataSORef.m_GraphEdges_temp.Count, ((tempDistance * tempSign) / m_fmoveSpeed), _startNode, _targetNode);
                m_navMeshDataSORef.m_GraphEdges_temp.Add(tempEdge);

                return tempEdge;
            }
        }

        return null;
    }

    void CheckAllNodesForJumpEdges()
    {

        for (int i = 0; i < m_navMeshDataSORef.m_GraphNodes_temp.Count; i++)
        {
            for (int j = 0; j < m_navMeshDataSORef.m_GraphNodes_temp.Count; j++)
            {
                if (i == j)
                {

                }
                else
                {
                    if(CanReachNode(m_navMeshDataSORef.m_GraphNodes_temp[i], m_navMeshDataSORef.m_GraphNodes_temp[j]) == false)
                    {
                        m_checkForJump.Enqueue(new Edge(m_navMeshDataSORef.m_GraphNodes_temp[i], m_navMeshDataSORef.m_GraphNodes_temp[j]));
                    }
                }
            }
        }
    }

    bool CanReachNode(NavNode2D _start, NavNode2D _target)
    {
        Queue<NavNode2D> nodesToCheck = new Queue<NavNode2D>();

        List<int> reachedNodesIds = new List<int>();

        nodesToCheck.Enqueue(_start);
        reachedNodesIds.Add(_start.m_id);

        while (nodesToCheck.Count > 0)
        {
            NavNode2D nodeToCheck = nodesToCheck.Dequeue();
            if (nodeToCheck.m_id == _target.m_id)
            {
                return true;
            }
            foreach (Edge edge in nodeToCheck.m_neighbours)
            {
                if (reachedNodesIds.Contains(edge.m_destination.m_id) == false)
                {
                    nodesToCheck.Enqueue(edge.m_destination);
                    reachedNodesIds.Add(edge.m_destination.m_id);
                }
            }
        }
        return false;
    }

    bool SetNodeNeighbour(Edge _newEdge)
    {
        if(_newEdge.m_source.m_neighbours != null)
        {
            for (int i = 0; i < _newEdge.m_source.m_neighbours.Count; i++)
            {
                if (_newEdge.m_destination.m_id == _newEdge.m_source.m_neighbours[i].m_destination.m_id)
                {
                    return false;
                }
            }
        }


        _newEdge.m_source.AddNeighbour(_newEdge);
        return true;
    }



    void EvaluateJumpEdges()
    {
        while(m_checkForJump.Count > 0)
        {
            Edge currentEdgeToCheck = m_checkForJump.Dequeue();
            if (EvaluateSingleJumpEdge(currentEdgeToCheck))
            {
                m_navMeshDataSORef.m_JumpEdges_temp.Add(currentEdgeToCheck);
            }
        }
    }

    bool EvaluateSingleJumpEdge(Edge _edgeToCheck)
    {
        float yDistance = _edgeToCheck.m_destination.m_position.y - _edgeToCheck.m_source.m_position.y;
        if (yDistance > m_fJumpHeight)
        {
            return false;
        }


        float maxAirTime = MathUtils.MaxInAirTime(
                                _edgeToCheck.m_source.m_position.y, 
                                _edgeToCheck.m_destination.m_position.y,
                                m_fJumpStartVelocity,
                                m_gravityMagnitude
                                );

        float maxXDistance = maxAirTime * m_fjumpHorMoveSpeed;
        float xDistance = Mathf.Abs(_edgeToCheck.m_destination.m_position.x - _edgeToCheck.m_source.m_position.x);
        float tempSign = Mathf.Sign(_edgeToCheck.m_destination.m_position.x - _edgeToCheck.m_source.m_position.x);
        if (xDistance < maxXDistance)
        {


            float jumpTime = (_edgeToCheck.m_destination.m_position.x - _edgeToCheck.m_source.m_position.x) / m_fjumpHorMoveSpeed;
            float tempYVelocity = MathUtils.GetPreciseJumpStartVelocity(_edgeToCheck.m_source.m_position.y, _edgeToCheck.m_destination.m_position.y, Mathf.Abs(jumpTime), m_gravityMagnitude);
            float tempXVelocity = m_fjumpHorMoveSpeed;
            if (tempYVelocity < 0)
            {
                return false;
            }

            if (CollisionCheckJump(_edgeToCheck, Mathf.Abs(jumpTime), Mathf.Abs(tempYVelocity), tempXVelocity * tempSign) == false)
            {
                jumpTime = maxAirTime;
                tempYVelocity = m_fJumpStartVelocity;
                tempXVelocity = Mathf.Abs(xDistance / jumpTime);
                if(CollisionCheckJump(_edgeToCheck, Mathf.Abs(jumpTime), Mathf.Abs(tempYVelocity), tempXVelocity * tempSign) == false)
                {
                    return false;
                }
            }


            _edgeToCheck.m_weight = (Mathf.Sign(yDistance) * jumpTime);
            _edgeToCheck.m_isJump = true;
            _edgeToCheck.m_jumpYVelocity = tempYVelocity;
            _edgeToCheck.m_jumpXVelocity = tempXVelocity;
            SetNodeNeighbour(_edgeToCheck);
            return true;
        }

        return false;
    }

    public NavNode2D GetClosestNodeToPosition(Vector2 _position)
    {
        float smallestDistance = float.MaxValue;
        NavNode2D closestNode = null;
        foreach (NavNode2D node in m_navMeshDataSORef.m_GraphNodes)
        {
            float tempDistance = Vector2.Distance(_position, node.m_position);
            if (tempDistance < smallestDistance)
            {
                smallestDistance = tempDistance;
                closestNode = node;
            }
        }
        return closestNode;
    }

    bool CollisionCheckJump(Edge _edgeToCheck, float _jumpTime, float _yVelocity, float _xVelocity)
    {
        float singleTimeStep = _jumpTime / m_iJumpTestSubsteps;
        List<Vector2> JumpSubsteps = new List<Vector2>();
        for (int i = 0; i < m_iJumpTestSubsteps - 1; i++)
        {
            JumpSubsteps.Add(MathUtils.GetCurrentPositionInJump(singleTimeStep + singleTimeStep * i, _edgeToCheck.m_source.m_position.x,
                _edgeToCheck.m_source.m_position.y, _yVelocity, _xVelocity, m_gravityMagnitude));

            Vector2 substepPosition = MathUtils.GetCurrentPositionInJump(singleTimeStep + singleTimeStep * i, _edgeToCheck.m_source.m_position.x,
                _edgeToCheck.m_source.m_position.y, _yVelocity, _xVelocity, m_gravityMagnitude);

            if (Physics2D.OverlapArea(new Vector2(substepPosition.x - 0.25f, substepPosition.y), new Vector2(substepPosition.x + 0.25f, substepPosition.y + 1.0f)) != null)
            {
                return false;
            }
        }
        return true;
    }


    private void OnDrawGizmos()
    {
        /// Uncomment to see Nodes as Gizmos
        /*
        Gizmos.color = Color.red;
        if (m_navMeshDataSORef.m_GraphNodes_temp != null)
        {
            for (int i = 0; i < m_navMeshDataSORef.m_GraphNodes_temp.Count; i++)
            {
                Gizmos.DrawSphere(m_navMeshDataSORef.m_GraphNodes_temp[i].m_position, 0.04f);
            }
            Gizmos.color = Color.blue;
            foreach (Vector2 pos in m_doubledNodes)
            {
                Gizmos.DrawSphere(pos, 0.02f);
            }
        }
        */

        /// Uncomment to see Edges as Line Gizmos
        /*
        if (m_navMeshDataSORef.m_GraphEdges_temp != null)
        {
            foreach (Edge edge in m_navMeshDataSORef.m_GraphEdges_temp)
            {
                if (edge.m_weight < 0)
                {
                    Gizmos.color = Color.red;
                    Gizmos.DrawLine(edge.m_source.m_position, edge.m_destination.m_position);
                }
                else
                {
                    Gizmos.color = Color.blue;
                    Gizmos.DrawLine(edge.m_source.m_position + Vector2.up * 0.05f, edge.m_destination.m_position + Vector2.up * 0.05f);
                }
            }
        }
        */
    }
}
