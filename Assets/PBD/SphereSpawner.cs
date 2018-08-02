using UnityEngine;

public class SphereSpawner : MonoBehaviour
{
	public PBDSolver solver;
	public PBDSphere SpherePrefab;
	public float timeTillNextSpawn = 0f;
	public float SpawnFrequency = 1f;

	void FixedUpdate()
	{
		timeTillNextSpawn -= Time.fixedDeltaTime;

		if (timeTillNextSpawn > 0)
			return;

		timeTillNextSpawn = SpawnFrequency;		

		var sphere = Instantiate(SpherePrefab, transform.position + Random.onUnitSphere * 2f, transform.rotation);

		Destroy(sphere.gameObject, 20f);
	}
}