#include "blocks.h"
#include <queue>

// Source: https://github.com/leomccormack/convhull_3d
#define CONVHULL_3D_ENABLE
#include "convhull_3d.h"

/*!
\brief
*/
static bool BreakFractureConstraint(const Vector3& p, const Vector3& candidate, const FractureSet& fractures)
{
	bool intersect = false;
	float tmax = Magnitude(p - candidate);
	Ray ray = Ray(p, Normalize(candidate - p));
	for (int c = 0; c < fractures.Size(); c++)
	{
		float t;
		if (fractures.At(c).Intersect(ray, t) && t < tmax)
		{
			intersect = true;
			break;
		}
	}
	return intersect;
}

/*!
\brief
*/
static bool CanBeLinkedToCluster(const Vector3& candidate, const std::vector<Vector3>& cluster, const FractureSet& fractures, float R_Max)
{
	// Check if the candidate can be linked to all other node in the cluster
	// Without breaking a constraint or being too far
	bool canBeLinked = true;
	for (int i = 0; i < cluster.size(); i++)
	{
		Vector3 p = cluster[i];

		// Distance criteria
		if (SquaredMagnitude(candidate - p) > R_Max)
		{
			canBeLinked = false;
			break;
		}

		// Fracture criteria
		for (int c = 0; c < fractures.Size(); c++)
		{
			float tmax = Magnitude(p - candidate);
			float t;
			if (fractures.At(c).Intersect(Ray(p, Normalize(candidate - p)), t) && t < tmax)
			{
				canBeLinked = false;
				break;
			}
		}
		if (!canBeLinked)
			break;
	}
	return canBeLinked;
}


/*!
\brief
*/
PointSet3 PoissonSamplingCube(const Box& box, float r, int n)
{
	PointSet3 set;
	float c = 4.0f * r * r;
	for (int i = 0; i < n; i++)
	{
		Vector3 t = box.RandomInside();
		bool hit = false;
		for (int j = 0; j < set.Size(); j++)
		{
			if (SquaredMagnitude(t - set.At(j)) < c)
			{
				hit = true;
				break;
			}
		}
		if (hit == false)
			set.pts.push_back(t);
	}
	return set;
}

/*!
\brief
*/
FractureSet GenerateFractures(FractureType type, const Box& box, float r)
{
	FractureSet set;
	if (type == FractureType::Equidimensional)
	{
		// "Three dominant sets of joints, approximately orthogonal, with occasional irregular joints, giving equidimensional blocks"
		Box inflatedRockDomain = box.Extended(Vector3(-3.0));
		PointSet3 samples = PoissonSamplingCube(inflatedRockDomain, 3.0, 1000);
		for (int i = 0; i < samples.Size(); i++)
		{
			int a = Random::Integer() % 3;
			Vector3 axis = a == 0 ? Vector3(1, 0, 0) : a == 1 ? Vector3(0, 1, 0) : Vector3(0, 0, 1);
			float r = Random::Uniform(10.0f, 15.0f);
			set.fractures.push_back(Circle(samples.At(i), axis, r));
		}
	}
	else if (type == FractureType::Rhombohedral)
	{
		// "Three (or more) dominant mutually oblique sets of joints, giving oblique-shaped, equidimensional blocks."
		Box inflatedRockDomain = box.Extended(Vector3(-3.0));
		PointSet3 samples = PoissonSamplingCube(inflatedRockDomain, 3.0, 1000);
		for (int i = 0; i < samples.Size(); i++)
		{
			int a = Random::Integer() % 3;
			Vector3 axis = a == 0 ? Vector3(0.5, 0.5, 0) : a == 1 ? Vector3(0, 0.5, 0.5) : Vector3(0.5, 0, 0.5);
			float r = Random::Uniform(3.0, 7.0);
			set.fractures.push_back(Circle(samples.At(i), axis, r));
		}
	}
	else if (type == FractureType::Polyhedral)
	{
		// "Irregular jointing without arrangement into distinct sets, and of small joints"
		PointSet3 samples = PoissonSamplingCube(box, 1.0, 1000);
		for (int i = 0; i < samples.Size(); i++)
		{
			float r = Random::Uniform(2.0, 8.0);
			Vector3 axis = Sphere(Vector3(0), 1.0).RandomSurface();
			set.fractures.push_back(Circle(samples.At(i), axis, r));
		}
	}
	else if (type == FractureType::Tabular)
	{
		// "One dominant set of parallel joints, for example bedding planes, with other non-persistent joints; thickness of blocks much less than length or width."
		// Z Axis
		const int fracturing = 15;
		Vector3 p = box[0];
		p[0] += box.Diagonal()[0] / 2.0f;
		p[1] += box.Diagonal()[1] / 2.0f;
		float step = box.Size()[2] / float(fracturing);
		float noiseStep = step / 10.0f;
		for (int i = 0; i < fracturing - 1; i++)
		{
			p[2] += step + Random::Uniform(-noiseStep, noiseStep);
			Vector3 axis = Vector3(0, 0, 1);
			Vector3 tiltedAxis = axis; // No tilting for tabular tiles
			set.fractures.push_back(Circle(p, tiltedAxis, 20.0));
		}

		PointSet3 poissonSamples = PoissonSamplingCube(box, 1.0, fracturing * 6.0);
		for (int i = 0; i < poissonSamples.Size(); i++)
		{
			float r = Random::Uniform(1.5f, 3.0f);
			Vector3 axis = (Random::Integer() % 2) == 0 ? Vector3(1, 0, 0) : Vector3(0, 1, 0);
			set.fractures.push_back(Circle(poissonSamples.At(i), axis, r));
		}
	}
	return set;
}

/*!
\brief
*/
std::vector<BlockCluster> ComputeBlockClusters(PointSet3& set, const FractureSet& frac)
{
	const int	 allPtsSize		= set.Size();
	const float	 R_Neighborhood = 2.5f * 2.5f;

	// Build constrained nearest neighor graph on tiled points
	std::vector<std::vector<int>> graph;
	graph.resize(allPtsSize);
	for (int i = 0; i < allPtsSize; i++)
	{
		Vector3 p = set.At(i);
		for (int j = 0; j < allPtsSize; j++)
		{
			if (i == j)
				continue;
			Vector3 q = set.At(j);
			if (SquaredMagnitude(p - q) < R_Neighborhood && !BreakFractureConstraint(p, q, frac))
				graph[i].push_back(j);
		}
	}

	// Flood fill algorithm to find clusters
	const float R_Max_Block = 10.5f * 10.5f;
	std::vector<bool> visitedFlags;
	visitedFlags.resize(allPtsSize, false);
	int averageClusterSize = 0;
	std::vector<BlockCluster> clusters;
	for (int j = 0; j < allPtsSize; j++)
	{
		std::queue<int> toVisit;
		toVisit.push(j);
		std::vector<Vector3> cluster;
		while (toVisit.empty() == false)
		{
			int index = toVisit.front();
			toVisit.pop();
			if (visitedFlags[index])
				continue;

			Vector3 q = set.At(index); 
			if (!CanBeLinkedToCluster(q, cluster, frac, R_Max_Block))
				continue;
			cluster.push_back(q);
			visitedFlags[index] = true;
			for (int i = 0; i < graph[index].size(); i++)
			{
				if (visitedFlags[graph[index][i]])
					continue;
				toVisit.push(graph[index][i]);
			}
		}
		if (cluster.size() > 10)
		{
			averageClusterSize += cluster.size();
			clusters.push_back({ cluster });
		}
	}
	std::cout << "Average cluster size: " << float(averageClusterSize) / clusters.size() << std::endl;
	return clusters;
}

/*!
\brief
*/
std::vector<BlockMesh> ComputeBlockMeshes(const std::vector<BlockCluster>& clusters)
{
	std::vector<BlockMesh> meshes;
	for (int k = 0; k < clusters.size(); k++)
	{
		std::vector<Plane> ret;
		std::vector<Vector3> allPts = clusters[k].pts;

		// Compute convex hull
		int n = allPts.size();
		if (n <= 4)
			continue;
		ch_vertex* vertices = new ch_vertex[n];
		for (int i = 0; i < n; i++)
			vertices[i] = { allPts[i][0], allPts[i][1], allPts[i][2] };
		int* faceIndices = NULL;
		int nFaces;
		convhull_3d_build(vertices, n, &faceIndices, &nFaces);
		if (nFaces == 0)
			continue;

		// Extract the mesh.
		std::vector<Triangle> meshTriangles;
		for (int i = 0; i < nFaces; i++)
		{
			const int j = i * 3;
			Vector3 v1 = Vector3(vertices[faceIndices[j + 0]].x, vertices[faceIndices[j + 0]].y, vertices[faceIndices[j + 0]].z);
			Vector3 v2 = Vector3(vertices[faceIndices[j + 1]].x, vertices[faceIndices[j + 1]].y, vertices[faceIndices[j + 1]].z);
			Vector3 v3 = Vector3(vertices[faceIndices[j + 2]].x, vertices[faceIndices[j + 2]].y, vertices[faceIndices[j + 2]].z);
			meshTriangles.push_back(Triangle(v1, v2, v3));
		}
		meshes.push_back(BlockMesh({ meshTriangles }));

		// Free memory
		delete[] vertices;
		delete[] faceIndices;
	}
	return meshes;
}
