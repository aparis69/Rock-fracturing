#include "blocks.h"
#include <queue>

// Source: https://github.com/leomccormack/convhull_3d
#define CONVHULL_3D_ENABLE
#include "convhull_3d.h"

// Source: https://github.com/aparis69/MarchingCubeCpp
#define MC_IMPLEM_ENABLE
#include "MC.h"

// Multithread field function computation
#include <omp.h>


/*!
\brief
*/
BlockSDF::BlockSDF(const std::vector<Plane>& pl, float sr)
{
	planes = pl;
	smoothRadius = sr;
}

/*!
\brief Generalized polynomial smoothing function between two distances.
Union:  SmoothingPolynomial(a, b, smooth);
Inter: -SmoothingPolynomial(-a, -b, smooth);
Diffe:  SmoothingPolynomial(-d1, d2, smooth);
\param a first distance
\param b second distance
\param sr smoothing radius
*/
float BlockSDF::SmoothingPolynomial(float d1, float d2, float sr) const
{
	float h = Math::Max(sr - Math::Abs(d1 - d2), 0.0f) / sr;
	return Math::Min(d1, d2) - h * h * sr * 0.25f;
}

/*!
\brief
*/
float BlockSDF::SignedSmoothConvex(const Vector3& p) const
{
	float d = planes.at(0).Signed(p);
	for (int i = 1; i < planes.size(); i++)
	{
		float dd = planes.at(i).Signed(p);
		d = -SmoothingPolynomial(-d, -dd, smoothRadius);
	}
	return d;
}

/*!
\brief
*/
Vector3 BlockSDF::Gradient(const Vector3& p) const
{
	static const float Epsilon = 0.01f;
	float x = SignedSmoothConvex(Vector3(p[0] - Epsilon, p[1], p[2])) - SignedSmoothConvex(Vector3(p[0] + Epsilon, p[1], p[2]));
	float y = SignedSmoothConvex(Vector3(p[0], p[1] - Epsilon, p[2])) - SignedSmoothConvex(Vector3(p[0], p[1] + Epsilon, p[2]));
	float z = SignedSmoothConvex(Vector3(p[0], p[1], p[2] - Epsilon)) - SignedSmoothConvex(Vector3(p[0], p[1], p[2] + Epsilon));
	return Vector3(x, y, z) * (0.5f / Epsilon);
}

/*!
\brief
*/
float BlockSDF::WarpingStrength(const Vector3& p, const Vector3& n) const
{
	const float texScale = 0.1642f;						// Hardcoded because it looks good
	Vector2 x = Abs(Vector2(p[2], p[1])) * texScale;
	Vector2 y = Abs(Vector2(p[0], p[2])) * texScale;
	Vector2 z = Abs(Vector2(p[1], p[0])) * texScale;

	float tmp;
	x = Vector2(modff(x[0], &tmp), modff(x[1], &tmp));
	y = Vector2(modff(y[0], &tmp), modff(y[1], &tmp));
	z = Vector2(modff(z[0], &tmp), modff(z[1], &tmp));

	// Blend weights
	Vector3 ai = Abs(n);
	ai = ai / (ai[0] + ai[1] + ai[2]);

	/*
		Todo(Axel): Replace "1.0f" with a bilinear interpolation from a texture 
		to add micro-scale details to the blocks.
	*/
	// Blend everything
	return    ai[0] * 0.0f
			+ ai[1] * 0.0f
			+ ai[2] * 0.0f;
}

/*!
\brief Compute the signed distance to the block primitive.
\param p point
*/
float BlockSDF::Signed(const Vector3& p) const
{
	// First compute gradient-based warping
	Vector3 g = Gradient(p);
	float s = 1.0f * WarpingStrength(p, -Normalize(g));

	// Then compute contribution from the convex block
	return SignedSmoothConvex(p + g * s);
}



/*!
\brief Check if a given point can be linked to a candidate.
\param p point
\param c the candidate
\param fractures fracture set
*/
static bool BreakFractureConstraint(const Vector3& p, const Vector3& c, const FractureSet& fractures)
{
	bool intersect = false;
	float tmax = Magnitude(p - c);
	Ray ray = Ray(p, Normalize(c - p));
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
\brief Check if the candidate can be linked to all other node in the cluster, without breaking a constraint or being too far.
*/
static bool CanBeLinkedToCluster(const Vector3& candidate, const std::vector<Vector3>& cluster, const FractureSet& fractures, float R_Max)
{
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
\brief Poisson sampling inside a 3D box, using dart-throwing.
\param box
\param r poisson radius
\param n number of try
*/
PointSet3 PoissonSamplingBox(const Box& box, float r, int n)
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
\brief Fracture generation inside a box, from the fracture type.
Code is sometimes duplicated in this function.
\param type fracture type
\param box the box in which to distribute fractures
\param r radius influencing the distance between fracture centers.
*/
FractureSet GenerateFractures(FractureType type, const Box& box, float r)
{
	FractureSet set;
	if (type == FractureType::Equidimensional)
	{
		// "Three dominant sets of joints, approximately orthogonal, with occasional irregular joints, giving equidimensional blocks"
		Box inflatedRockDomain = box.Extended(Vector3(-3.0));
		PointSet3 samples = PoissonSamplingBox(inflatedRockDomain, 3.0, 1000);
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
		PointSet3 samples = PoissonSamplingBox(inflatedRockDomain, 3.0, 1000);
		for (int i = 0; i < samples.Size(); i++)
		{
			int a = Random::Integer() % 3;
			Vector3 axis = a == 0 ? Vector3(0.5, 0.5, 0) : a == 1 ? Vector3(0, 0.5, 0.5) : Vector3(0.5, 0, 0.5);
			float r = Random::Uniform(10.0f, 15.0f);
			set.fractures.push_back(Circle(samples.At(i), axis, r));
		}
	}
	else if (type == FractureType::Polyhedral)
	{
		// "Irregular jointing without arrangement into distinct sets, and of small joints"
		PointSet3 samples = PoissonSamplingBox(box, 1.0, 1000);
		for (int i = 0; i < samples.Size(); i++)
		{
			float r = Random::Uniform(2.0f, 12.0f);
			Vector3 axis = Sphere(Vector3(0), 1.0).RandomSurface();
			set.fractures.push_back(Circle(samples.At(i), axis, r));
		}
	}
	else if (type == FractureType::Tabular)
	{
		// "One dominant set of parallel joints, for example bedding planes, with other non-persistent joints; thickness of blocks much less than length or width."
		// Y Axis
		const int fracturing = 10;
		Vector3 p = box[0];
		p.x += box.Diagonal()[0] / 2.0f;
		p.z += box.Diagonal()[1] / 2.0f;
		float step = box.Size()[2] / float(fracturing);
		float noiseStep = step / 10.0f;
		for (int i = 0; i < fracturing - 1; i++)
		{
			p.y += step + Random::Uniform(-noiseStep, noiseStep);
			Vector3 axis = Vector3(0, -1, 0);
			set.fractures.push_back(Circle(p, axis, 20.0));
		}
	}
	return set;
}

/*!
\brief Clustering step of the algorithm. We first build the constrained nearest neighbour graph
between samples, then perform a flood fill algorithm to get isolated clusters of points.
\param set samples of the cubic tile
\param frac the fractures
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
			clusters.push_back({ cluster });
		}
	}
	return clusters;
}

/*!
\brief Implicit primitive generation from the block clusters. This function is very similar to ComputeBlockMeshes,
but instead of extracting a mesh, we create an implicit primitive for each block defined as the smooth intersection
of half spaces. This is the function used in the paper - ComputeBlockMeshes is only here if you want to play with the
raw meshes.
\param clusters
*/
std::vector<BlockSDF> ComputeBlockSDF(const std::vector<BlockCluster>& clusters)
{
	std::vector<BlockSDF> primitives;
	for (int k = 0; k < clusters.size(); k++)
	{
		std::vector<Plane> ret;
		std::vector<Vector3> allPts = clusters[k].pts;

		// Compute convex hull
		int n = int(allPts.size());
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

		// Extract planes from hull triangle
		std::vector<Plane> planes;
		for (int i = 0; i < nFaces; i++)
		{
			const int j = i * 3;
			Vector3 v1 = Vector3(float(vertices[faceIndices[j + 0]].x), float(vertices[faceIndices[j + 0]].y), float(vertices[faceIndices[j + 0]].z));
			Vector3 v2 = Vector3(float(vertices[faceIndices[j + 1]].x), float(vertices[faceIndices[j + 1]].y), float(vertices[faceIndices[j + 1]].z));
			Vector3 v3 = Vector3(float(vertices[faceIndices[j + 2]].x), float(vertices[faceIndices[j + 2]].y), float(vertices[faceIndices[j + 2]].z));
			Vector3 pn = Triangle(v1, v2, v3).Normal();
			planes.push_back(Plane(v1, pn));
		}
		
		// @Warning: this cannot work in all cases: the correct smoothing radius must depend on
		// The actual size of the convex primitive formed by the intersection of planes.
		// So artifacts may occur by using a constant.
		const double smoothRadius = 0.25;	
		primitives.push_back(BlockSDF(planes, smoothRadius));

		// Free memory
		delete[] vertices;
		delete[] faceIndices;
	}
	return primitives;
}

/*!
\brief Polygonization function of the implicit primitives using marching cubes.
\param sdf implicit block functions
*/
MC::mcMesh PolygonizeSDF(const Box& box, const std::vector<BlockSDF>& sdf)
{
	// Compute field function
	const int n = 100;
	MC::MC_FLOAT* field = new MC::MC_FLOAT[n * n * n];
	Vector3 cellDiagonal = (box[1] - box[0]) / (n - 1);
	{
#pragma omp parallel for num_threads(16) shared(field)
		for (int i = 0; i < n; i++)
		{
			for (int j = 0; j < n; j++)
			{
				for (int k = 0; k < n; k++)
				{
					Vector3 p = Vector3(box[0][0] + i * cellDiagonal[0], box[0][1] + j * cellDiagonal[1], box[0][2] + k * cellDiagonal[2]);
					float d = 10000000.0f;
					for (int l = 0; l < sdf.size(); l++)
						d = Math::Min(d, sdf[l].Signed(p));
					field[(k * n + j) * n + i] = d;
				}
			}
		}
	}
	
	// Polygonize
	MC::mcMesh mesh;
	MC::marching_cube(field, n, n, n, mesh);
	return mesh;
}
