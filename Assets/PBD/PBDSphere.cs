using Unity.Mathematics;
using UnityEngine;

public class PBDSphere : MonoBehaviour 
{
    PBDSolver solver;

    public float3 position = new float3();
    public float3 predicted = new float3();
    public float3 velocity = new float3();
    public float inverseMass = 1f;
    public float radius = .5f;

    [HideInInspector]
    public int index = 0;

    void OnEnable()
    {
        solver = GameObject.FindObjectOfType<PBDSolver>();
        solver.AddSphere(this);
    }
    
    void OnDisable()
    {
        solver.RemoveSphere(this.index);
        solver = null;
    }
}