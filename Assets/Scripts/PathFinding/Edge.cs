using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[Serializable]
public class Edge
{
    public float m_weight;
    public NavNode2D m_source;
    public NavNode2D m_destination;
    public bool m_isJump = false;
    public float m_jumpYVelocity;
    public float m_jumpXVelocity;

    public Edge(int _id, float _weight, NavNode2D _source, NavNode2D _destination)
    {
        m_weight = _weight;
        m_source = _source;
        m_destination = _destination;
    }
    public Edge(NavNode2D _source, NavNode2D _destination)
    {
        m_weight = 0;
        m_source = _source;
        m_destination = _destination;
    }
}
