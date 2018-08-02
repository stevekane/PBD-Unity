using Unity.Mathematics;
using UnityEngine;

public struct ColliderContactConstraint
{
	public PBDSphere sphere;		
	public float3 normal;
	public float3 point;
	public float3 velocity;
	public float elasticity;
	public float friction;

	public static void Solve(ColliderContactConstraint c)
	{
		var constraint = math.dot(c.normal, c.sphere.predicted) - c.sphere.radius;

		if (constraint >= 0)
			return;
			
		c.sphere.position -= c.normal * constraint;
		c.sphere.predicted -= c.normal * constraint;
	}
}

public class PBDSolver : MonoBehaviour 
{
	static int iterationCount = 4;
	static float invIterations = .25f;
	public float3 Gravity = new float3(0, -85, 0);
	public float Damping = .98f;
	public float IdleThreshold = .5f;

	PBDSphere[] spheres = new PBDSphere[1024];
	int sphereCount = 0;
	ColliderContactConstraint[] cccs = new ColliderContactConstraint[1024];
	int cccCount = 0;

	RaycastHit[] collisions = new RaycastHit[1024];

	public int AddSphere(PBDSphere s)
	{
		var index = sphereCount;

		s.index = index;
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
			s.velocity = s.velocity * Damping + Gravity * dt;
			s.predicted = s.position + s.velocity * dt;

			var delta = s.predicted - s.position;
			var direction = math.normalize(delta);
			var distance = math.length(delta);
			var collisionCount = Physics.SphereCastNonAlloc(s.position, s.radius, direction, collisions, distance);

			for (var c = 0; c < collisionCount; c++)
			{
				// float3 pContact = collisions[c].point;
				// float3 displacement = pContact - s.position;
				// TODO: should calculate the contact time and velocity
				// float3 vContact = math.sqrt(s.velocity * s.velocity + 2 * Gravity * displacement);
				// vContact *= math.dot(vContact, s.velocity) < 0 ? -1 : 1;

				var nccc = new ColliderContactConstraint
				{
					sphere = s,
					normal = collisions[c].normal,
					point = collisions[c].point,
					velocity = s.velocity,
					elasticity = collisions[c].collider.material.bounciness,
					friction = collisions[c].collider.material.dynamicFriction
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
			var isIdle = math.lengthSquared(spheres[i].velocity) < IdleThreshold;

			spheres[i].velocity = isIdle ? new float3(0) : (spheres[i].predicted - spheres[i].position) / dt;
			spheres[i].position = spheres[i].predicted;
			spheres[i].transform.position = spheres[i].position;
		}

		for (var i = 0; i < cccCount; i++)
		{
			var mvn = math.dot(cccs[i].velocity, cccs[i].normal);
			var vn = cccs[i].normal * mvn;
			var vt = cccs[i].velocity - vn;

			cccs[i].sphere.velocity = vt * cccs[i].friction - vn * cccs[i].elasticity;
		}

		// flush the collisions
		cccCount = 0;
	}
}