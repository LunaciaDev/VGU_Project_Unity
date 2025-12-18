using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MoveHuman : MonoBehaviour
{
    [SerializeField]
    GameObject m_Human;
    public GameObject Human { get => m_Human; set => m_Human = value; }
    
    public void move() {
        // Apply a bit of force to move the cube
        Rigidbody rb = m_Human.GetComponent<Rigidbody>();
        rb.AddForce(Vector3.forward * 100);    
    }
}
