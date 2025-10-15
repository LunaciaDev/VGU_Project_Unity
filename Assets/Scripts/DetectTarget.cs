using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DetectTarget : MonoBehaviour
{
    private Color baseColor = Color.white;
    private Color hasTargetColor = Color.green;
    private Renderer cubeRenderer;

    // Start is called before the first frame update
    void Start()
    {
        cubeRenderer = GetComponent<Renderer>();
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.CompareTag("TargetZone"))
        {
            cubeRenderer.material.color = hasTargetColor;
        }
    }

    private void OnTriggerExit(Collider other)
    {
        if (other.gameObject.CompareTag("TargetZone"))
        {
            cubeRenderer.material.color = baseColor;
        }
    }
}
