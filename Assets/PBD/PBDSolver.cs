using Unity.Mathematics;
using UnityEngine;

public struct ColliderContactConstraint
{
	public PBDSphere sphere;		
	public float3 normal;
	public float3 velocity;

	public static void Solve(ColliderContactConstraint c)
	{
		var constraint = math.dot(c.normal, c.sphere.predicted) - c.sphere.radius;

		if (constraint >= 0)
			return;

		c.sphere.predicted -= c.normal * constraint;
	}
}

public class PBDSolver : MonoBehaviour 
{
	static int iterationCount = 4;
	static float invIterations = .25f;
	static float3 Gravity = new float3(0, -85, 0);

	PBDSphere[] spheres = new PBDSphere[1024];
	int sphereCount = 0;
	ColliderContactConstraint[] cccs = new ColliderContactConstraint[1024];
	int cccCount = 0;

	RaycastHit[] collisions = new RaycastHit[1024];

	public int AddSphere(PBDSphere s)
	{
		var index = sphereCount;

		spheres[sphereCount] = s;
		sphereCount++;
		return index;
	}

	public int AddColliderContactConstraint(ColliderContactConstraint ccc)
	{
		var index = cccCount;

		cccs[index] = ccc;
		cccCount++;
		return index;
	}

	public void RemoveSphere(int index)
	{
		sphereCount--;

		var lastSphere = spheres[sphereCount];

		lastSphere.index = index;
		spheres[index] = lastSphere;
	}

	void FixedUpdate()
	{
		var dt = Time.fixedDeltaTime;		

		// apply forward euler and predict positions
		for (var i = 0; i < sphereCount; i++)
		{
			var s = spheres[i];

			s.position = s.transform.position;
			// probably should add some velocity damping here ... or somewhere else?
			s.velocity = s.velocity + Gravity * dt;
			s.predicted = s.position + s.velocity * dt;

			var delta = s.predicted - s.position;
			var direction = math.normalize(delta);
			var distance = math.length(direction);
			var collisionCount = Physics.SphereCastNonAlloc(s.position, s.radius, direction, collisions, distance);

			for (var c = 0; c < collisionCount; c++)
			{
				// Let's use the following formula to compute the velocity 
				// http://gamedevs.org/uploads/continuous-collision-detection-and-physics.pdf
				float3 pContact = collisions[c].point;
				float3 vContact = math.sqrt(s.velocity * s.velocity + 2 * Gravity * (pContact - s.position));
				Debug.Log(vContact + " " + s.velocity);
				var nccc = new ColliderContactConstraint
				{
					sphere = s,
					normal = collisions[c].normal,
					velocity = s.velocity
				};

				AddColliderContactConstraint(nccc);
			}
		}

		// iteratively apply constraints
		for (var k = 0; k < iterationCount; k++)
		{
			for (var i = 0; i < cccCount; i++)
			{
				ColliderContactConstraint.Solve(cccs[i]);
			}
		}

		// update velocity and position
		for (var i = 0; i < sphereCount; i++)
		{
			spheres[i].velocity = (spheres[i].predicted - spheres[i].position) / dt;
			spheres[i].position = spheres[i].predicted;
			spheres[i].transform.position = spheres[i].position;
		}

		for (var i = 0; i < cccCount; i++)
		{
			var mvn = math.dot(cccs[i].velocity, cccs[i].normal);
			var vn = cccs[i].normal * mvn;
			var vt = cccs[i].velocity - vn;

			// could modulate both by elastic and frictional responses
			cccs[i].sphere.velocity = vt + vn * -1;
		}

		// flush the collisions
		cccCount = 0;
	}
}