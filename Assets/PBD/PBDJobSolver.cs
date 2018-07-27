using Unity.Mathematics;
using Unity.Mathematics.Experimental;
using Unity.Jobs;
using Unity.Burst;
using Unity.Collections;
using Unity.Transforms;
using UnityEngine;
using UnityEngine.Jobs;
using System;
using Random = UnityEngine.Random;

public struct DistanceConstraint
{
	public int i0;	
	public int i1;	
	public float distance;
	public float stiffness;
}

public struct PositionConstraint
{
	public int i;
	public float3 position;
}

public struct ShapeConstraint
{
	public int start;
	public int end;
	public float stiffness;
	public float3x3 inverseMassMatrix;
	public float3 restCenterOfMass;
}

public class PBDJobSolver : MonoBehaviour 
{
	public Vector3 GRAVITY = new Vector3(0, -100, 0);
	public float VELOCITY_SPREAD = 10f;
	public int JOB_SIZE_POWER_OF_TWO = 6;
	public int ATOM_RESOLUTION = 4;
	public int ITERATION_COUNT = 10;
	[Range(0f, 1f)]
	public float STIFFNESS = 1f;

	[SerializeField]
	GameObject AtomPrefab;

	GameObject[] atoms;
	NativeArray<float3> restPositions;
	NativeArray<float3> positions;
	NativeArray<float3> predicteds;
	NativeArray<float3> velocities;
	Transform[] transforms;
	TransformAccessArray transformAccessArray;

	NativeArray<DistanceConstraint> distanceConstraints;
	NativeArray<PositionConstraint> positionConstraints;
	NativeArray<ShapeConstraint> shapeConstraints;

	CopyPositionFromTransform copyPositionFromTransformJob;
	ApplyExternalForces applyExternalForcesJob;
	EstimatePosition estimatePositionJob;
	RunSolver runSolverJob;
	UpdateVelocity updateVelocityJob;
	UpdatePosition updatePositionJob;
	CopyPositionToTransform copyPositionToTransformJob;

	JobHandle copyPositionFromTransformJobHandle;
	JobHandle applyExternalForcesJobHandle;
	JobHandle estimatePositionJobHandle;
	JobHandle runSolverJobHandle;
	JobHandle updateVelocityJobHandle;
	JobHandle updatePositionJobHandle;
	JobHandle copyPositionToTransformJobHandle;

	void Awake() 
	{ 
		int atomCount = ATOM_RESOLUTION * ATOM_RESOLUTION;
		int dcCount = 0;
		int pcCount = 0;
		int scCount = 1;

		atoms = new GameObject[atomCount];
		transforms = new Transform[atomCount];
		velocities = new NativeArray<float3>(atomCount, Allocator.Persistent);
		predicteds = new NativeArray<float3>(atomCount, Allocator.Persistent);
		positions = new NativeArray<float3>(atomCount, Allocator.Persistent);
		restPositions = new NativeArray<float3>(atomCount, Allocator.Persistent);
		distanceConstraints = new NativeArray<DistanceConstraint>(dcCount, Allocator.Persistent);
		positionConstraints = new NativeArray<PositionConstraint>(pcCount, Allocator.Persistent);
		shapeConstraints = new NativeArray<ShapeConstraint>(scCount, Allocator.Persistent);

		// layout particles in grid
		for (var x = 0; x < ATOM_RESOLUTION; x++)
		{
			for (var y = 0; y < ATOM_RESOLUTION; y++)
			{
				var index = x * ATOM_RESOLUTION + y;
				var position = Random.onUnitSphere * 10f + transform.position;

				atoms[index] = Instantiate(AtomPrefab, position, Quaternion.identity);
				positions[index] = position;
				predicteds[index] = position;
				transforms[index] = atoms[index].transform;
			}
		}

		// Side-effecting function currently... it writes to restPositions and creates the constraint
		var sc = CreateShapeConstraint(restPositions, positions, 0, predicteds.Length, STIFFNESS);

		shapeConstraints[0] = sc;
		transformAccessArray = new TransformAccessArray(transforms);
	}

	void OnDestroy()
	{
		velocities.Dispose();	
		predicteds.Dispose();	
		positions.Dispose();	
		restPositions.Dispose();
		transformAccessArray.Dispose();
		distanceConstraints.Dispose();
		positionConstraints.Dispose();
		shapeConstraints.Dispose();
	}

	void Update() 
	{
		float deltaTime = Time.fixedDeltaTime;
		float3 deltaGravity = deltaTime * GRAVITY;
		float damping = .98f;
		int batchSize = (int)Mathf.Pow(2, JOB_SIZE_POWER_OF_TWO);
		int count = atoms.Length;

		copyPositionFromTransformJob = new CopyPositionFromTransform
		{
			positions = positions
		};
		applyExternalForcesJob = new ApplyExternalForces
		{
			damping = damping,
			deltaGravity = deltaGravity,
			velocities = velocities
		};
		estimatePositionJob = new EstimatePosition
		{
			deltaTime = deltaTime,
			velocities = velocities,
			positions = positions,
			predicteds = predicteds
		};
		runSolverJob = new RunSolver
		{
			iterationCount = ITERATION_COUNT,
			restPositions = restPositions,
			predicteds = predicteds,
			positionConstraints = positionConstraints,
			distanceConstraints = distanceConstraints,
			shapeConstraints = shapeConstraints
		};
		updateVelocityJob = new UpdateVelocity
		{
			deltaTime = deltaTime,
			velocities = velocities,
			positions = positions,
			predicted = predicteds
		};
		updatePositionJob = new UpdatePosition
		{
			deltaTime = deltaTime,
			positions = positions,
			predicteds = predicteds
		};
		copyPositionToTransformJob = new CopyPositionToTransform
		{
			positions = positions
		};

		copyPositionFromTransformJobHandle = copyPositionFromTransformJob.Schedule(transformAccessArray);
		applyExternalForcesJobHandle = applyExternalForcesJob.Schedule(count, batchSize);
		estimatePositionJobHandle = estimatePositionJob.Schedule(count, batchSize, JobHandle.CombineDependencies(applyExternalForcesJobHandle, copyPositionFromTransformJobHandle));
		runSolverJobHandle = runSolverJob.Schedule(estimatePositionJobHandle);
		updateVelocityJobHandle = updateVelocityJob.Schedule(count, batchSize, runSolverJobHandle);
		updatePositionJobHandle = updatePositionJob.Schedule(count, batchSize, updateVelocityJobHandle);
		copyPositionToTransformJobHandle = copyPositionToTransformJob.Schedule(transformAccessArray, updatePositionJobHandle);
	}

	void LateUpdate()
	{
		copyPositionToTransformJobHandle.Complete();
	}

	struct CopyPositionFromTransform: IJobParallelForTransform
	{
		public NativeArray<float3> positions;

		public void Execute(int i, TransformAccess transform) => positions[i] = transform.position;
	}

	struct ApplyExternalForces: IJobParallelFor
	{
		public float damping;
		public float3 deltaGravity;
		public NativeArray<float3> velocities;

        public void Execute(int i) => velocities[i] = damping * (velocities[i] + deltaGravity);
    }

	struct EstimatePosition: IJobParallelFor
	{
		public float deltaTime;
		[ReadOnly]
		public NativeArray<float3> velocities;
		[ReadOnly]
		public NativeArray<float3> positions;
		public NativeArray<float3> predicteds;

        public void Execute(int i) => predicteds[i] = velocities[i] * deltaTime + positions[i];
    }

	struct RunSolver : IJob
	{
		public int iterationCount;
		[ReadOnly]
		public NativeArray<float3> restPositions;
		public NativeArray<float3> predicteds;
		[ReadOnly]
		public NativeArray<PositionConstraint> positionConstraints;
		[ReadOnly]
		public NativeArray<DistanceConstraint> distanceConstraints;
		[ReadOnly]
		public NativeArray<ShapeConstraint> shapeConstraints;

		public void Execute()
		{
			float invIterations = 1f / (float)iterationCount;
			float inverseMass = 1f; // THIS SHOULD VARY BY ATOM! add native array of inversemasses to this overall solver

			for (var k = 0; k < iterationCount; k++)
			{
				for (var j = 0; j < distanceConstraints.Length; j++)
				{
					// TODO: could use local refs here to avoid stupid copying
					var dc = distanceConstraints[j];
					var a = predicteds[dc.i0];
					var b = predicteds[dc.i1];
					var delta = a - b;
					var direction = math.normalize(delta);
					var C = math.length(delta) - dc.distance;
					var dstiffness = 1f - math.pow(1f - dc.stiffness, invIterations);

					if (C > -float.Epsilon && C < float.Epsilon)
						continue;
					
					predicteds[dc.i0] -= inverseMass / (inverseMass + inverseMass) * dstiffness * C * direction;
					predicteds[dc.i1] += inverseMass / (inverseMass + inverseMass) * dstiffness * C * direction;
				}

				for (var j = 0; j < shapeConstraints.Length; j++)
				{
					ShapeConstraint sc = shapeConstraints[j];
					float dstiffness = 1f - Mathf.Pow(1f - sc.stiffness, invIterations);
					float totalMass = 0f;
					float3 centerOfMass = Vector3.zero;

					for (var i = sc.start; i < sc.end; i++)
					{
						centerOfMass += predicteds[i] * inverseMass;
						totalMass += inverseMass;
					}
					centerOfMass /= totalMass;

					// compute rest matrix
					float a00 = 0f;
					float a01 = 0f;
					float a02 = 0f; 
					float a10 = 0f;
					float a11 = 0f;
					float a12 = 0f;
					float a20 = 0f;
					float a21 = 0f;
					float a22 = 0f;

					for (var i = sc.start; i < sc.end; i++)
					{
						float3 q = restPositions[i];
						float3 p = predicteds[i] - centerOfMass;

						a00 += inverseMass * p.x * q.x;
						a01 += inverseMass * p.x * q.y;
						a02 += inverseMass * p.x * q.z;

						a10 += inverseMass * p.y * q.x;
						a11 += inverseMass * p.y * q.y;
						a12 += inverseMass * p.y * q.z;

						a20 += inverseMass * p.z * q.x;
						a21 += inverseMass * p.z * q.y;
						a22 += inverseMass * p.z * q.z;
					}

					float3x3 currentMatrix = new float3x3(
						a00, a01, a02,
						a10, a11, a12,
						a20, a21, a22
					);
					float3x3 covarianceMatrix = math.mul(currentMatrix, sc.inverseMassMatrix);
					float3x3 rotationMatrix = Decomposition.FastExtractRotationFrom(covarianceMatrix, 10);

					for (var i = sc.start; i < sc.end; i++)
					{
						float3 goal = centerOfMass + math.mul(rotationMatrix, restPositions[i]);		

						predicteds[i] += (goal - predicteds[i]) * sc.stiffness * dstiffness;
					}
				}

				for (var i = 0; i < predicteds.Length; i++)
				{
					if (predicteds[i].y > 0)
						continue;

					var aboveGround = predicteds[i];

					aboveGround.y = 0;
					predicteds[i] = aboveGround;
				}

				for (var j = 0; j < positionConstraints.Length; j++)
				{
					// TODO: could use local refs here to avoid stupid copying
					var pc = positionConstraints[j];

					predicteds[pc.i] = pc.position;
				}
			}
		}
	}

	struct UpdateVelocity : IJobParallelFor
	{
		public float deltaTime; // Must be non-zero for division

		[ReadOnly]
		public NativeArray<float3> positions;
		[ReadOnly]
		public NativeArray<float3> predicted;
		public NativeArray<float3> velocities;

        public void Execute(int i) => velocities[i] = (predicted[i] - positions[i]) / deltaTime;
    }

	struct UpdatePosition : IJobParallelFor
	{
		public float deltaTime;
		[ReadOnly]
		public NativeArray<float3> predicteds;
		public NativeArray<float3> positions;

        public void Execute(int i) => positions[i] = predicteds[i];
    }

	struct CopyPositionToTransform: IJobParallelForTransform
	{
		[ReadOnly]
		public NativeArray<float3> positions;

		public void Execute(int i, TransformAccess transform) => transform.position = positions[i];
	}

	// TODO: this mutates restPositions...unsure what to do about this
	static ShapeConstraint CreateShapeConstraint(NativeArray<float3> restPositions, NativeArray<float3> positions, int Start, int End, float Stiffness)
	{
		float a00 = 0f;
		float a01 = 0f;
		float a02 = 0f; 
		float a10 = 0f;
		float a11 = 0f;
		float a12 = 0f;
		float a20 = 0f;
		float a21 = 0f;
		float a22 = 0f;
		float inverseMass = 1; // Could be defined on the atoms themselves
		float totalMass = 0f;
		ShapeConstraint sc = new ShapeConstraint();

		sc.end = End;
		sc.start = Start;
		sc.stiffness = Stiffness;
		sc.restCenterOfMass = Vector3.zero;

		// compute center of mass of resting pose
		for (var i = sc.start; i < sc.end; i++)
		{
			sc.restCenterOfMass += positions[i] * inverseMass;
			totalMass += inverseMass;
		}
		sc.restCenterOfMass /= totalMass;

		// compute rest matrix
		for (var i = sc.start; i < sc.end; i++)
		{
			float3 q = positions[i] - sc.restCenterOfMass;

			a00 += inverseMass * q.x * q.x;
			a01 += inverseMass * q.x * q.y;
			a02 += inverseMass * q.x * q.z;

			a10 += inverseMass * q.y * q.x;
			a11 += inverseMass * q.y * q.y;
			a12 += inverseMass * q.y * q.z;

			a20 += inverseMass * q.z * q.x;
			a21 += inverseMass * q.z * q.y;
			a22 += inverseMass * q.z * q.z;
			restPositions[i] = q;
		}

		float3x3 restMatrix = new float3x3(
			a00, a01, a02,
			a10, a11, a12,
			a20, a21, a22
		);

		sc.inverseMassMatrix = math.inverse(restMatrix);
		return sc;
	}
}