using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.UI;

//public class DijkstraContainer
//{
//    public float distance;
//    public int predecessorId;
//    public bool visited;

//    public DijkstraContainer(float _distance, int _predecessorId)
//    {
//        distance = _distance;
//        predecessorId = _predecessorId;
//        visited = false;
//    }
//    public void setDistance(float _distance)
//    {
//        distance = _distance;
//    }
//    public void setPredecessor(int _predecessorId)
//    {
//        predecessorId = _predecessorId;
//    }
//    public void setVisited()
//    {
//        visited = true;
//    }
//}


public class NavMeshManager : MonoBehaviour
{
    public NavMeshData NavMeshDataSORef;

    public List<Vector2> allNodes;
    public List<Vector2> DoubledNodes;

    //  List<NavNode2D> m_GraphNodes;
    //  List<Edge> m_GraphEdges;
    //  List<Edge> m_JumpEdges;

    Queue<Edge> m_CheckForJump;

    public float m_fPointAccuracy = 0.1f;
    public float m_fDoubleNodeDeleteDistance = 0.01f;
    public int m_iJumpTestSubsteps = 10;

    public float m_fJumpHeight = 2.0f;
    public float m_moveSpeed = 3.0f;
    public float m_jumpHorMoveSpeed = 4.0f;

    float m_jumpStartVelocity;
    float m_jumpTime;

    public float m_gravityMagnitude = 10.0f;

    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        //  Debug.Log(m_GraphNodes.Count);
        //  Dijkstra(m_GraphNodes[0]);
    }

    void AddNodesAlongLine(Vector2 _point1, Vector2 _point2)
    {
        TryAddingNodeToList(_point1);

        Vector2 DirectionsOfLine = (_point2 - _point1).normalized;
        float DistanceBetweenPoints = (_point2 - _point1).magnitude;

        if(DistanceBetweenPoints >= m_fPointAccuracy * 2)
        {
            float stepsBetweenPoints = DistanceBetweenPoints / m_fPointAccuracy;
            int stepsBetweenPointsRounded = (int)stepsBetweenPoints;


            for (int i = 1; i <= stepsBetweenPointsRounded; i++)
            {
                Vector2 tempPointToAdd = _point1 + DirectionsOfLine * i * m_fPointAccuracy;

                /*try
                {
                    Debug.Log(((int)((tempPointToAdd - allNodes[allNodes.Count - 1]).magnitude * 100)) / 100.0f);

                }
                catch (System.Exception)
                {
                    Debug.Log("SomethingWentWrong");
                    throw;
                }*/
                TryAddingNodeToList(tempPointToAdd);
            }
        }
        TryAddingNodeToList(_point2);
    }

    void ClearDoubleNodes()
    {
        List<int> tempIndicesMarkedForSkipping = new List<int>();
        List<Vector2> newListOfNodes = new List<Vector2>();

        for (int i = 0; i < allNodes.Count; i++)
        {
            for (int j = 0; j < allNodes.Count; j++)
            {
                if(i == j)
                {

                }
                else
                {
                    if (Vector2.Distance(allNodes[i], allNodes[j]) <= m_fDoubleNodeDeleteDistance)
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

        for (int i = 0; i < allNodes.Count; i++)
        {
            if(tempIndicesMarkedForSkipping.Contains(i) == false)
            {
                newListOfNodes.Add(allNodes[i]);
            }
            else
            {
                //  Debug.Log("Skipped Nodes because doubled!");
                DoubledNodes.Add(allNodes[i]);
            }
        }
        allNodes.Clear();
        allNodes = newListOfNodes;
    }

    void CreateGraphNodesFromPositions()
    {
        string[] nodeAssetPaths = { "Assets/BuiltData/Nodes" };
        List<string> outFailedPaths = new List<string>();
        AssetDatabase.DeleteAssets(nodeAssetPaths, outFailedPaths);
        AssetDatabase.CreateFolder("Assets/BuiltData", "Nodes");

        for (int i = 0; i < allNodes.Count; i++)
        {
            NavMeshDataSORef.m_GraphNodes_temp.Add(ScriptableObject.CreateInstance<NavNode2D>());
            AssetDatabase.CreateAsset(NavMeshDataSORef.m_GraphNodes_temp[i], "Assets/BuiltData/Nodes/Node_" + i + ".asset");
            NavMeshDataSORef.m_GraphNodes_temp[i].m_id = i;
            NavMeshDataSORef.m_GraphNodes_temp[i].m_position = allNodes[i];

            Debug.Log("Added Node " + i);
        }
        AssetDatabase.SaveAssets();
    }

    void ConstructEdges()
    {
        for (int i = 0; i < NavMeshDataSORef.m_GraphNodes_temp.Count; i++)
        {
            for (int j = 0; j < NavMeshDataSORef.m_GraphNodes_temp.Count; j++)
            {
                if((i == j) == false)
                {
                    Edge tempEdge = TryAddingEdge(NavMeshDataSORef.m_GraphNodes_temp[i], NavMeshDataSORef.m_GraphNodes_temp[j]);
                    if (tempEdge != null)
                    {
                        SetNodeNeighbour(tempEdge);
                    }
                }
            }
        }
        for (int i = 0; i < NavMeshDataSORef.m_GraphNodes_temp.Count; i++)
        {
            if(NavMeshDataSORef.m_GraphNodes_temp[i].m_neighbours.Count == 0)
            {
                NavMeshDataSORef.m_GraphNodes_temp.RemoveAt(i);
                i--;
            }
        }
    }

    bool TryAddingNodeToList(Vector2 _pointToAdd)
    {
        //  RaycastHit2D tempHit = Physics2D.CircleCast(_pointToAdd + Vector2.up * 0.01f, 0.0005f, Vector2.down, 5);
        RaycastHit2D tempHit = Physics2D.Raycast(_pointToAdd + Vector2.up * 0.01f, Vector2.down, 0.015f);
        if (tempHit.collider == null || tempHit.distance < 0.008f)
        {
            //  Debug.Log("Not Added: " + tempHit.distance);
            return false;
        }
        else
        {
            //  Debug.Log("Added Node: " + tempHit.distance);
            allNodes.Add(_pointToAdd);
            return true;
        }
    }

    Edge TryAddingEdge(NavNode2D _startNode, NavNode2D _targetNode)
    {
        float tempDistance = Vector2.Distance(_startNode.m_position, _targetNode.m_position);
        if (tempDistance <= m_fPointAccuracy * 1.1f)
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

            //  Vector2 tempLocalRight = new Vector2(Mathf.Cos((tempSignedAngle) * Mathf.Deg2Rad), Mathf.Sin((tempSignedAngle) * Mathf.Deg2Rad));

            RaycastHit2D tempHit = Physics2D.Raycast(_startNode.m_position + tempLocalUp * 0.001f, tempDirection, tempDistance);

            //  Debug.DrawRay(m_GraphNodes[i].m_position, tempLocalUp * 0.1f, Color.cyan, 100);
            //  Debug.DrawRay(m_GraphNodes[i].m_position + tempLocalUp * 0.001f, tempDirection * tempDistance, Color.green, 100);

            //  Debug.DrawRay(m_GraphNodes[i].m_position, tempLocalRight * 0.1f, Color.red, 100);


            if ((tempHit.collider == null) == false)
            {
                Debug.Log("! Cant Go Through Collider !  " + tempHit.distance);
                //  Debug.Log(tempHit.collider.name);

                return null;
            }
            else
            {
                //  Debug.Log("Angle of Edge" + Vector2.SignedAngle(Vector2.right, tempDirection));
                //  m_GraphEdges.Add(new Edge(Counter, tempDistance * tempSign, m_GraphNodes[i], m_GraphNodes[j]));
                float tempSign = Mathf.Sign(tempSignedAngle);
                Debug.Log(tempSignedAngle);
                if (Mathf.Abs(tempSignedAngle) == 180)
                {
                    tempSign = -1;
                }
                Edge tempEdge = new Edge(NavMeshDataSORef.m_GraphEdges_temp.Count, ((tempDistance * tempSign) / m_moveSpeed), _startNode, _targetNode);
                NavMeshDataSORef.m_GraphEdges_temp.Add(tempEdge);

                return tempEdge;
            }
        }

        return null;
    }

    void CheckAllNodesForJumpEdges()
    {

        for (int i = 0; i < NavMeshDataSORef.m_GraphNodes_temp.Count; i++)
        {
            for (int j = 0; j < NavMeshDataSORef.m_GraphNodes_temp.Count; j++)
            {
                if (i == j)
                {

                }
                else
                {
                    if(CanReachNode(NavMeshDataSORef.m_GraphNodes_temp[i], NavMeshDataSORef.m_GraphNodes_temp[j]) == false)
                    {
                        m_CheckForJump.Enqueue(new Edge(NavMeshDataSORef.m_GraphNodes_temp[i], NavMeshDataSORef.m_GraphNodes_temp[j]));
                    }
                }
            }
        }
    }

    bool SetNodeNeighbour(Edge _newEdge)
    {
        for (int i = 0; i < _newEdge.m_source.m_neighbours.Count; i++)
        {
            if(_newEdge.m_destination.m_id == _newEdge.m_source.m_neighbours[i].m_destination.m_id)
            {
                return false;
            }
        }

        _newEdge.m_source.m_neighbours.Add(_newEdge);
        return true;
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
            if(nodeToCheck.m_id == _target.m_id)
            {
                return true;
            }
            foreach (Edge edge in nodeToCheck.m_neighbours)
            {
                if(reachedNodesIds.Contains(edge.m_destination.m_id) == false)
                {
                    nodesToCheck.Enqueue(edge.m_destination);
                    reachedNodesIds.Add(edge.m_destination.m_id);
                }
            }
        }
        return false;
    }

    void EvaluateJumpEdges()
    {
        while(m_CheckForJump.Count > 0)
        {
            Edge currentEdgeToCheck = m_CheckForJump.Dequeue();
            if (EvaluateSingleJumpEdge(currentEdgeToCheck))
            {
                NavMeshDataSORef.m_JumpEdges_temp.Add(currentEdgeToCheck);
            }
        }
        NavMeshDataSORef.m_JumpEdges_temp.Clear();
    }

    bool EvaluateSingleJumpEdge(Edge _edgeToCheck)
    {
        float yDistance = _edgeToCheck.m_destination.m_position.y - _edgeToCheck.m_source.m_position.y;
        if (yDistance > m_fJumpHeight)
        {
            return false;
        }

        float maxAirTime = MaxInAirTime(
                                _edgeToCheck.m_source.m_position.y, 
                                _edgeToCheck.m_destination.m_position.y,
                                m_jumpStartVelocity,
                                m_gravityMagnitude
                                );

        float maxXDistance = maxAirTime * m_jumpHorMoveSpeed;
        float xDistance = Mathf.Abs(_edgeToCheck.m_destination.m_position.x - _edgeToCheck.m_source.m_position.x);
        if (xDistance < maxXDistance)
        {


            float jumpTime = (_edgeToCheck.m_destination.m_position.x - _edgeToCheck.m_source.m_position.x) / m_jumpHorMoveSpeed;
            float tempYVelocity = GetPreciseJumpStartVelocity(_edgeToCheck.m_source.m_position.y, _edgeToCheck.m_destination.m_position.y, jumpTime, m_gravityMagnitude);
            float tempXVelocity = m_jumpHorMoveSpeed;

            if (CollisionCheckJump(_edgeToCheck, jumpTime, tempYVelocity, tempXVelocity) == false)
            {
                jumpTime = maxAirTime;
                tempYVelocity = m_jumpStartVelocity;
                tempXVelocity = Mathf.Abs(xDistance / jumpTime);
                if(CollisionCheckJump(_edgeToCheck, jumpTime, tempYVelocity, tempXVelocity) == false)
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

    private void OnDrawGizmos()
    {
        Gizmos.color = Color.red;
        if(NavMeshDataSORef.m_GraphNodes_temp != null)
        {
            for (int i = 0; i < NavMeshDataSORef.m_GraphNodes_temp.Count; i++)
            {
                Gizmos.DrawSphere(NavMeshDataSORef.m_GraphNodes_temp[i].m_position, 0.04f);
            }
            //Gizmos.color = Color.blue;
            //foreach (Vector2 pos in DoubledNodes)
            //{
            //    Gizmos.DrawSphere(pos, 0.02f);
            //}
        }


        if(NavMeshDataSORef.m_GraphEdges_temp != null)
        {
            foreach (Edge edge in NavMeshDataSORef.m_GraphEdges_temp)
            {
                if (edge.m_weight < 0)
                {
                    Gizmos.color = Color.red;
                    Gizmos.DrawLine(edge.m_source.m_position, edge.m_destination.m_position);
                }
                else
                {
                    Gizmos.color = Color.blue;
                    Gizmos.DrawLine(edge.m_source.m_position + Vector2.up, edge.m_destination.m_position + Vector2.up);
                }
            }
        }

        //if(NavMeshDataSORef.m_JumpEdges_temp != null)
        //{
        //    foreach (Edge edge in NavMeshDataSORef.m_JumpEdges_temp)
        //    {
        //        if (edge.m_weight > 0)
        //        {
        //            Gizmos.color = Color.magenta;
        //            Gizmos.DrawLine(edge.m_source.m_position, edge.m_destination.m_position);
        //        }
        //        else
        //        {
        //            Gizmos.color = Color.yellow;
        //            Gizmos.DrawLine(edge.m_source.m_position, edge.m_destination.m_position);
        //        }
        //    }
        //}
    }



    /// Utils
    /// 
    float[] SolveQuadraticFormula(float _a, float _b, float _c)
    {
        float[] result = new float[2];
        float determinant = _b * _b - 4 * _a * _c;
        if(determinant < 0)
        {
            return null;
        }
        float determinantSqrt = Mathf.Sqrt(determinant);
        print("Determinante: " + determinant);
        result[0] = (-_b + determinantSqrt) / (2 * _a);
        result[1] = (-_b - determinantSqrt) / (2 * _a);
        return result;
    }
    float[] InAirTime(float _startingY, float _targetY, float _yVelocity, float _gravity)
    {
        float tempC = _startingY - _targetY;
        float tempB = _yVelocity;
        float tempA = -0.5f * _gravity;

        return SolveQuadraticFormula(tempA, tempB, tempC);
    }
    float MaxInAirTime(float _startingY, float _targetY, float _yVelocity, float _gravity)
    {
        float tempC = _startingY - _targetY;
        float tempB = _yVelocity;
        float tempA = -0.5f * _gravity;

        float[] tempResult = SolveQuadraticFormula(tempA, tempB, tempC);
        if(tempResult == null)
        {
            return float.MinValue;
        }
        else
        {
            return Mathf.Max(tempResult[0], tempResult[1]);
        }
    }
    float MinInAirTime(float _startingY, float _targetY, float _yVelocity, float _gravity)
    {
        float tempC = _startingY - _targetY;
        float tempB = _yVelocity;
        float tempA = -0.5f * _gravity;

        float[] tempResult = SolveQuadraticFormula(tempA, tempB, tempC);
        if (tempResult == null)
        {
            return float.MinValue;
        }
        else
        {
            return Mathf.Min(tempResult[0], tempResult[1]);
        }
    }
    float GetPreciseJumpStartVelocity(float _startingY, float _targetY, float _jumpTime, float _gravity)
    {
        return (_targetY - _startingY + 0.5f * _gravity * _jumpTime * _jumpTime) / _jumpTime;
    }
    public Vector2 GetCurrentPositionInJump(float _currentTime, float _startingX, float _startingY, float _ystartVelocity, float _xVelocity, float _gravity)
    {
        float tempX = _startingX + _currentTime * _xVelocity;
        float tempY = _startingY + _ystartVelocity * _currentTime - 0.5f * _gravity * _currentTime * _currentTime;
        return new Vector2(tempX, tempY);
    }

    void VisualizeJumpPath(Edge _jumpEdge)
    {
        float jumpTime = (_jumpEdge.m_destination.m_position.x - _jumpEdge.m_source.m_position.x) / _jumpEdge.m_jumpXVelocity;
        float tempVelocity = GetPreciseJumpStartVelocity(_jumpEdge.m_source.m_position.y, _jumpEdge.m_destination.m_position.y, jumpTime, m_gravityMagnitude);
        float singleTimeStep = jumpTime / m_iJumpTestSubsteps;
        List<Vector2> JumpSubsteps = new List<Vector2>();
        for (int i = 0; i < m_iJumpTestSubsteps; i++)
        {
            JumpSubsteps.Add(GetCurrentPositionInJump(singleTimeStep + singleTimeStep * i, _jumpEdge.m_source.m_position.x,
                _jumpEdge.m_source.m_position.y, tempVelocity, _jumpEdge.m_jumpXVelocity, m_gravityMagnitude));
        }
        Debug.DrawLine(_jumpEdge.m_source.m_position, JumpSubsteps[0], Color.red);

        for (int i = 0; i < JumpSubsteps.Count - 1; i++)
        {
            Debug.DrawLine(JumpSubsteps[i], JumpSubsteps[i + 1], Color.red);
        }
    }



    /*
    Dictionary<int, DijkstraContainer> Dijkstra(NavNode2D _source)
    {
        Dictionary<int, DijkstraContainer> resultDictionary = new Dictionary<int, DijkstraContainer>();
        List<NavNode2D> nodeSet = new List<NavNode2D>();

        foreach (NavNode2D node in NavMeshDataSORef.m_GraphNodes_temp)
        {
            resultDictionary.Add(node.m_id, new DijkstraContainer(float.MaxValue, -1));
        }
        resultDictionary[_source.m_id].setDistance(0);
        nodeSet.Add(_source);

        int Count = 0;
        while (nodeSet.Count > 0)
        {
            NavNode2D tempNode = nodeSet[0];
            float closestNodeDistance = resultDictionary[tempNode.m_id].distance;

            int indexToRemove = 0;
            for (int index = 0 ; index < nodeSet.Count; index++)
            {
                if(resultDictionary[nodeSet[index].m_id].distance < closestNodeDistance)
                {
                    tempNode = nodeSet[index];
                    closestNodeDistance = resultDictionary[nodeSet[index].m_id].distance;
                    indexToRemove = index;
                }
            }
            nodeSet.RemoveAt(indexToRemove);

            resultDictionary[tempNode.m_id].setVisited();
            foreach (Edge edge in tempNode.m_neighbours)
            {
                float tempDistanceToNode = resultDictionary[tempNode.m_id].distance + Mathf.Abs(edge.m_weight);
                if(tempDistanceToNode < resultDictionary[edge.m_destination.m_id].distance)
                {
                    resultDictionary[edge.m_destination.m_id].setDistance(tempDistanceToNode);
                    resultDictionary[edge.m_destination.m_id].setPredecessor(edge.m_source.m_id);
                }

                if(resultDictionary[edge.m_destination.m_id].visited == false && nodeSet.Contains(edge.m_destination) == false)
                {
                    nodeSet.Add(edge.m_destination);
                }
            }
            Count++;
        }
        return resultDictionary;
    }

    public Stack<int> GetRecursivePath(NavNode2D _source, NavNode2D _target)
    {
        Dictionary<int, DijkstraContainer> tempDictionary = Dijkstra(_source);
        int currentNodeId = _target.m_id;

        Stack<int> path = new Stack<int>();
        Stack<int> path2 = new Stack<int>();
        path.Push(currentNodeId);
        path2.Push(currentNodeId);
        while (currentNodeId != _source.m_id)
        {
            currentNodeId = tempDictionary[currentNodeId].predecessorId;
            if(currentNodeId == -1)
            {
                return null;
            }
            path.Push(currentNodeId);
            path2.Push(currentNodeId);
        }
        VisualizePath(_source, path2);
        return path;
    }*/

    void VisualizePath(NavNode2D _source, Stack<int> _path)
    {
        NavNode2D currentNode = _source;
        _path.Pop();
        NavNode2D nextNode;
        while (_path.Count > 0)
        {
            if (currentNode.getNeighbour(_path.Peek()).m_isJump == true)
            {
                Edge tempEdge = currentNode.getNeighbour(_path.Pop());
                VisualizeJumpPath(tempEdge);
                nextNode = tempEdge.m_destination;
                currentNode = nextNode;
            }
            else
            {
                nextNode = currentNode.getNeighbour(_path.Pop()).m_destination;
                Debug.DrawLine(currentNode.m_position, nextNode.m_position, Color.red);
                currentNode = nextNode;
            }

        }
    }

    public NavNode2D GetClosestNodeToPosition(Vector2 _position)
    {
        float smallestDistance = float.MaxValue;
        NavNode2D closestNode = null;
        foreach (NavNode2D node in NavMeshDataSORef.m_GraphNodes_temp)
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
            JumpSubsteps.Add(GetCurrentPositionInJump(singleTimeStep + singleTimeStep * i, _edgeToCheck.m_source.m_position.x,
                _edgeToCheck.m_source.m_position.y, _yVelocity, _xVelocity, m_gravityMagnitude));

            Vector2 substepPosition = GetCurrentPositionInJump(singleTimeStep + singleTimeStep * i, _edgeToCheck.m_source.m_position.x,
                _edgeToCheck.m_source.m_position.y, _yVelocity, _xVelocity, m_gravityMagnitude);

            if (Physics2D.OverlapArea(new Vector2(substepPosition.x - 0.25f, substepPosition.y), new Vector2(substepPosition.x + 0.25f, substepPosition.y + 1.0f)) != null)
            {
                return false;
            }
        }
        return true;
    }

    public bool RebuildNavMesh()
    {
        if(NavMeshDataSORef == null)
        {
            Debug.Log("Nav Mesh Data Container Scriptable Object Ref Missing!");
            return false;
        }

        NavMeshDataSORef.m_GraphNodes_temp = new List<NavNode2D>();
        NavMeshDataSORef.m_GraphEdges_temp = new List<Edge>();
        NavMeshDataSORef.m_JumpEdges_temp = new List<Edge>();
        m_CheckForJump = new Queue<Edge>();

        m_jumpStartVelocity = Mathf.Sqrt(2.0f * m_gravityMagnitude * m_fJumpHeight);
        m_jumpTime = m_jumpStartVelocity / m_gravityMagnitude;

        print("JumpTime: " + m_jumpTime);
        print("Jumptime Quadratic: " + MinInAirTime(0, 100, m_jumpStartVelocity, m_gravityMagnitude));
        print("Jumptime Quadratic: " + MaxInAirTime(0, 100, m_jumpStartVelocity, m_gravityMagnitude));

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

        Debug.Log("Number of Nodes: " + allNodes.Count);
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

        ParseNodesToScriptableObject();

        return true;
    }

    void ParseNodesToScriptableObject()
    {
        NavMeshDataSORef.InitNavMeshData(NavMeshDataSORef.m_GraphNodes_temp);
    }
}