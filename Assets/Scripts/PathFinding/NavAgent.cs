using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class NavAgent : MonoBehaviour
{

    public NavNode2D m_currentNode;
    public NavNode2D m_closestNode;
    public int m_headingNodeID;
    public int m_currentNodeID;

    Edge m_currentEdge;

    //  Used for debugging
    //  public List<int> m_neighbours;

    public NavMeshManager m_navManagerRef;
    Stack<int> m_currentPath;

    float m_bufferedTime;
    float m_currentMoveTime;
    public float m_timeToHeading;

    void Update()
    {
        if (m_currentNode == null)
        {
            m_currentNode = m_navManagerRef.GetClosestNodeToPosition(transform.position);
            if (m_currentNode != null)
            {
                transform.position = m_currentNode.m_position;
                m_headingNodeID = m_currentNode.m_id;
            }
        }
        else
        {
            SetClosestMouseNode();
            m_currentNodeID = m_currentNode.m_id;


            if (m_closestNode != null)
            {
                m_currentPath = m_navManagerRef.m_navMeshDataSORef.GetPathBacktracking(m_headingNodeID, m_closestNode.m_id);
                if(m_currentPath.Count > 1)
                {
                    if (m_headingNodeID == m_currentNode.m_id)
                    {
                        m_currentPath.Pop();
                        m_headingNodeID = m_currentPath.Pop();
                        m_currentEdge = m_currentNode.GetNeighbour(m_headingNodeID);
                    }
                }
            }

            if (m_headingNodeID == m_currentNode.m_id)
            {

            }
            else
            {
                if(m_currentEdge != null)
                {
                    if(m_currentEdge.m_isJump == false)
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

        bool SetHeadingNodeAndAdjustPosition(float _timeBuffer)
        {
            m_currentEdge = m_currentNode.GetNeighbour(m_headingNodeID);
            if(m_currentEdge == null)
            {
                return false;
            }
            m_currentMoveTime = 0;
            transform.position = m_currentEdge.m_destination.m_position;
            m_currentNode = m_currentEdge.m_destination;

            if(m_currentPath.Count > 0)
            {
                m_headingNodeID = m_currentPath.Pop();
                m_currentEdge = m_currentNode.GetNeighbour(m_headingNodeID);
            }


            m_bufferedTime = _timeBuffer;
            return true;
        }

        float GetCurrentBufferedTime()
        {
            float tempBufferedTime = m_bufferedTime;
            m_bufferedTime = 0;
            return tempBufferedTime;
        }

        void SetClosestMouseNode()
        {
            Vector2 mouseWorldpos = Camera.main.ScreenToWorldPoint(Input.mousePosition);
            m_closestNode = m_navManagerRef.GetClosestNodeToPosition(mouseWorldpos);
        }

        void MoveGrounded()
        {
            m_currentMoveTime += Time.deltaTime + GetCurrentBufferedTime();
            m_timeToHeading = Vector2.Distance(m_currentEdge.m_destination.m_position, m_currentEdge.m_source.m_position) / m_navManagerRef.m_fmoveSpeed;
            if (m_currentMoveTime > m_timeToHeading)
            {
                SetHeadingNodeAndAdjustPosition(m_currentMoveTime - m_timeToHeading);
            }
            else
            {
                transform.position = m_currentEdge.m_source.m_position + m_currentMoveTime *
                                        m_navManagerRef.m_fmoveSpeed * (m_currentEdge.m_destination.m_position - m_currentEdge.m_source.m_position).normalized;
            }
        }  
        
        void MoveInair()
        {
            float tempSign = Mathf.Sign(m_currentEdge.m_destination.m_position.x - m_currentEdge.m_source.m_position.x);
            m_currentMoveTime += (Time.deltaTime + GetCurrentBufferedTime());
            if (m_currentMoveTime > Mathf.Abs(m_currentEdge.m_weight))
            {
                SetHeadingNodeAndAdjustPosition(m_currentMoveTime - Mathf.Abs(m_currentEdge.m_weight));
            }
            else
            {
                transform.position = MathUtils.GetCurrentPositionInJump(m_currentMoveTime, m_currentNode.m_position.x, m_currentNode.m_position.y,
                                        Mathf.Abs(m_currentEdge.m_jumpYVelocity), m_currentEdge.m_jumpXVelocity * tempSign, m_navManagerRef.m_gravityMagnitude);

            }
        }
    }
}
