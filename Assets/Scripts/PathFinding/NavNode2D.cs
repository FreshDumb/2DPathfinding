using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[Serializable]
public class NavNode2D : ScriptableObject
{
    public int m_id;
    public Vector2 m_position;
    public List<Edge> m_neighbours = new List<Edge>();

    public Edge GetNeighbour(int _id)
    {
        foreach (Edge edge in m_neighbours)
        {
            if(edge.m_destination.m_id == _id)
            {
                return edge;
            }
        }
        return null;
    }

    public void AddNeighbour(Edge _newNeighbour)
    {
        if(m_neighbours != null)
        {
            m_neighbours.Add(_newNeighbour);
        }
        else
        {
            m_neighbours = new List<Edge>();
            m_neighbours.Add(_newNeighbour);
        }
    }
}