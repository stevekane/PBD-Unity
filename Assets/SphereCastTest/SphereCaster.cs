using UnityEngine;

public class SphereCaster : MonoBehaviour 
{
	public float r = .5f;
	public float d = .5f;
	public RaycastHit rh = new RaycastHit();
	public Mesh mesh;

	void OnDrawGizmos()
	{
		// Conclusion from these tests is that sphere casting does not detect ANYTHING that
		// overlaps the initial sphere. You have to imagine that it will strike ONLY points that
		// are not contained in the original sphere's volume.
		var p = transform.position;
		var dir = transform.right;
		var didHit = Physics.SphereCast(p, r, dir, out rh, d);

		Gizmos.color = didHit ? Color.green : Color.red;
		Gizmos.DrawWireSphere(transform.position, r);
		Gizmos.DrawWireSphere(transform.position + dir * d, r);
	}
}