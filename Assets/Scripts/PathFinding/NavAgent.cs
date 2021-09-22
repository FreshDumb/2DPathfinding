using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class NavAgent : MonoBehaviour
{

    public NavNode2D currentNode;
    public NavNode2D closestNode;
    public int headingNodeId;
    public int currentNodeId;

    Edge currentEdge;

    public List<int> neighbours;
    public NavMeshManager navManagerRef;
    Stack<int> currentPath;
    public int[] testArray;

    float bufferedTime;
    float currentMoveTime;
    public float timeToHeading;

    // Update is called once per frame
    void Update()
    {

        if(currentPath != null)
        {
            testArray = currentPath.ToArray();
        }

        if (currentNode == null)
        {
            currentNode = navManagerRef.GetClosestNodeToPosition(transform.position);
            if (currentNode != null)
            {
                transform.position = currentNode.m_position;
                headingNodeId = currentNode.m_id;
                foreach (Edge edge in currentNode.m_neighbours)
                {
                    neighbours.Add(edge.m_destination.m_id);
                }
            }
        }
        else
        {
            setClosestMouseNode();
            currentNodeId = currentNode.m_id;


            if (closestNode != null)
            {
                currentPath = navManagerRef.NavMeshDataSORef.GetRecursivePath(headingNodeId, closestNode.m_id);
                if(currentPath.Count > 1)
                {
                    if (headingNodeId == currentNode.m_id)
                    {
                        currentPath.Pop();
                        headingNodeId = currentPath.Pop();
                        currentEdge = currentNode.getNeighbour(headingNodeId);
                    }
                }
            }

            if (headingNodeId == currentNode.m_id)
            {

            }
            else
            {
                if(currentEdge != null)
                {
                    if(currentEdge.m_isJump == false)
                    {
                        MoveGrounded();
                    }
                    else
                    {
                        MoveInair();
                    }
                }
            }
        }

        bool setHeadingNodeAndAdjustPosition(float _timeBuffer)
        {
            currentEdge = currentNode.getNeighbour(headingNodeId);
            if(currentEdge == null)
            {
                return false;
            }
            currentMoveTime = 0;
            transform.position = currentEdge.m_destination.m_position;
            currentNode = currentEdge.m_destination;

            neighbours.Clear();
            foreach (Edge edge in currentNode.m_neighbours)
            {
                neighbours.Add(edge.m_destination.m_id);
            }

            if(currentPath.Count > 0)
            {
                headingNodeId = currentPath.Pop();
                currentEdge = currentNode.getNeighbour(headingNodeId);
            }


            bufferedTime = _timeBuffer;
            return true;
        }

        float getCurrentBufferedTime()
        {
            float tempBufferedTime = bufferedTime;
            bufferedTime = 0;
            return tempBufferedTime;
        }

        void setClosestMouseNode()
        {
            Vector2 mouseWorldpos = Camera.main.ScreenToWorldPoint(Input.mousePosition);
            closestNode = navManagerRef.GetClosestNodeToPosition(mouseWorldpos);
        }

        void MoveGrounded()
        {
            currentMoveTime += Time.deltaTime + getCurrentBufferedTime();
            timeToHeading = Vector2.Distance(currentEdge.m_destination.m_position, currentEdge.m_source.m_position) / navManagerRef.m_moveSpeed;
            if (currentMoveTime > timeToHeading)
            {
                setHeadingNodeAndAdjustPosition(currentMoveTime - timeToHeading);
            }
            else
            {
                transform.position = currentEdge.m_source.m_position + currentMoveTime *
                                        navManagerRef.m_moveSpeed * (currentEdge.m_destination.m_position - currentEdge.m_source.m_position).normalized;
            }
        }  
        
        void MoveInair()
        {
            float tempSign = Mathf.Sign(currentEdge.m_destination.m_position.x - currentEdge.m_source.m_position.x);
            currentMoveTime += (Time.deltaTime + getCurrentBufferedTime());
            if (currentMoveTime > Mathf.Abs(currentEdge.m_weight))
            {
                setHeadingNodeAndAdjustPosition(currentMoveTime - Mathf.Abs(currentEdge.m_weight));
            }
            else
            {
                transform.position = MathUtils.GetCurrentPositionInJump(currentMoveTime * tempSign, currentNode.m_position.x, currentNode.m_position.y,
                                        Mathf.Abs(currentEdge.m_jumpYVelocity) * tempSign, currentEdge.m_jumpXVelocity, navManagerRef.m_gravityMagnitude);

            }
        }
    }
}
