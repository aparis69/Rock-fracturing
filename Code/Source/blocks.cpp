#include "blocks.h"
#include <queue>

// Source: https://github.com/leomccormack/convhull_3d
#define CONVHULL_3D_ENABLE
#include "convhull_3d.h"

// Source: https://github.com/aparis69/MarchingCubeCpp
#define MC_IMPLEM_ENABLE
#include "MC.h"

// Source: http://nothings.org/stb
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

// Multithread field function computation
#include <omp.h>

// Warping displacement strength stored as global
static ScalarField2D warpingField;


/*!
\brief Default constructor for node.
*/
SDFNode::SDFNode()
{
}

/*!
\brief Constructor from a bounding box.
\param box the bounding box.
*/
SDFNode::SDFNode(const Box& box) : box(box)
{
}

/*!
\brief Computes the gradient numerically at a given point.
\param p point
*/
Vector3 SDFNode::Gradient(const Vector3& p) const
{
	static const double Epsilon = 0.01f;
	double x = Signed(Vector3(p[0] - Epsilon, p[1], p[2])) - Signed(Vector3(p[0] + Epsilon, p[1], p[2]));
	double y = Signed(Vector3(p[0], p[1] - Epsilon, p[2])) - Signed(Vector3(p[0], p[1] + Epsilon, p[2]));
	double z = Signed(Vector3(p[0], p[1], p[2] - Epsilon)) - Signed(Vector3(p[0], p[1], p[2] + Epsilon));
	return Vector3(x, y, z) * (0.5f / Epsilon);
}


/*!
\brief Constructor for a binary bounding sphere node.
\param a, b child nodes
\param re transition radius
*/
SDFUnionSphereLOD::SDFUnionSphereLOD(SDFNode* a, SDFNode* b, double re) : SDFNode(Box(a->box, b->box).Extended(Vector3(re))), re(re), sphere(Sphere(box.Center(), box.Size().Max()))
{
	e[0] = a;
	e[1] = b;
}

/*!
\brief Evaluates the node.
\param p point
*/
double SDFUnionSphereLOD::Signed(const Vector3& p) const
{
	// Signed distance to bounding sphere
	double sd = sphere.Distance(p);
	if (sd > re)
		return sd;

	// Returns only sub-trees
	double se = Math::Min(e[0]->Signed(p), e[1]->Signed(p));
	if (sd < sphere.Radius())
		return se;

	// Interpolate between bounding sphere and sub-trees
	double a = (sd - sphere.Radius()) / (re - sphere.Radius());
	return (1.0 - a) * se + a * sd;
}

/*!
\brief Computes the bounding volume hiearchy using bounding sphere binary nodes.
\param nodes the nodes to organize
\param re transition radius
*/
SDFNode* SDFUnionSphereLOD::OptimizedBVH(std::vector<SDFNode*>& nodes, double re)
{
	if (nodes.size() == 0)
		return nullptr;
	return OptimizedBVHRecursive(nodes, 0, int(nodes.size()), re);
}

/*!
\brief
*/
SDFNode* SDFUnionSphereLOD::OptimizedBVHRecursive(std::vector<SDFNode*>& pts, int begin, int end, double re)
{
	/*
	\brief BVH space partition predicate. Could also use a lambda function to avoid the structure.
	*/
	struct BVHPartitionPredicate
	{
		int axis;
		double cut;

		BVHPartitionPredicate(int a, double c) : axis(a), cut(c)
		{
		}

		bool operator()(SDFNode* p) const
		{
			return (p->box.Center()[axis] < cut);
		}
	};

	// If leaf, returns primitive
	int nodeCount = end - begin;
	if (nodeCount <= 1)
		return pts[begin];

	// Bounding box of primitive in [begin, end] range
	Box bbox = pts[begin]->box;
	for (int i = begin + 1; i < end; i++)
		bbox = Box(bbox, pts[i]->box);

	// Find the most stretched axis of the bounding box
	// Cut the box in the middle of this stretched axis
	int stretchedAxis = bbox.Diagonal().MaxIndex();

	double axisMiddleCut = (bbox[0][stretchedAxis] + bbox[1][stretchedAxis]) / 2.0;

	// Partition our primitives in relation to the axisMiddleCut
	auto pmid = std::partition(pts.begin() + begin, pts.begin() + end, BVHPartitionPredicate(stretchedAxis, axisMiddleCut));

	// Ensure the partition is not degenerate : all primitives on the same side
	unsigned int midIndex = std::distance(pts.begin(), pmid);
	if (midIndex == begin || midIndex == end)
		midIndex = (begin + end) / 2;

	// Recursive construction of sub trees
	SDFNode* left = OptimizedBVHRecursive(pts, begin, midIndex, re);
	SDFNode* right = OptimizedBVHRecursive(pts, midIndex, end, re);

	// Union of the two child nodes
	return new SDFUnionSphereLOD(left, right, re);
}


/*!
\brief Constructor from a node.
\param e child node
*/
SDFGradientWarp::SDFGradientWarp(SDFNode* e) : e(e)
{
	box = e->box;
}

/*!
\brief Computes the warping strength as a triplanar parameterization of the texture.
\param p point
\param n normal
*/
double SDFGradientWarp::WarpingStrength(const Vector3& p, const Vector3& n) const
{
	const double texScale = 0.1642f;						// Hardcoded because it looks good
	Vector2 x = Abs(Vector2(p[2], p[1])) * texScale;
	Vector2 y = Abs(Vector2(p[0], p[2])) * texScale;
	Vector2 z = Abs(Vector2(p[1], p[0])) * texScale;

	double tmp;
	x = Vector2(modf(x[0], &tmp), modf(x[1], &tmp));
	y = Vector2(modf(y[0], &tmp), modf(y[1], &tmp));
	z = Vector2(modf(z[0], &tmp), modf(z[1], &tmp));

	// Blend weights
	Vector3 ai = Abs(n);
	ai = ai / (ai[0] + ai[1] + ai[2]);

	// Blend everything
	return  ai[0] * warpingField.GetValueBilinear(x)
			+ ai[1] * warpingField.GetValueBilinear(y)
			+ ai[2] * warpingField.GetValueBilinear(z);
}

/*!
\brief Evalutes the warped signed distance to the node.
\param p point
*/
double SDFGradientWarp::Signed(const Vector3& p) const
{
	// First compute gradient-based warping
	Vector3 g = e->Gradient(p);
	float s = 0.65 * WarpingStrength(p, -Normalize(g));

	// Then compute contribution from the convex block
	return e->Signed(p + g * s);
}


/*!
\brief Constructor for a smooth block, from a set of (closed) planes and a smoothing radius.
\param pl the planes
\param sr smoothing radius
*/
SDFBlock::SDFBlock(const std::vector<Plane>& pl, double sr) : SDFNode(Box(Plane::ConvexPoints(pl)).Extended(Vector3(0.01f)))
{
	planes = pl;
	smoothRadius = sr;
	box = box.Extended(Vector3(sr));
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
double SDFBlock::SmoothingPolynomial(double d1, double d2, double sr) const
{
	double h = Math::Max(sr - Math::Abs(d1 - d2), 0.0) / sr;
	return Math::Min(d1, d2) - h * h * sr * 0.25;
}

/*!
\brief Compute the signed distance to the block primitive.
\param p point
*/
double SDFBlock::Signed(const Vector3& p) const
{
	double d = planes.at(0).Signed(p);
	for (int i = 1; i < planes.size(); i++)
	{
		double dd = planes.at(i).Signed(p);
		d = -SmoothingPolynomial(-d, -dd, smoothRadius);
	}
	return d;
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
	double tmax = Magnitude(p - c);
	Ray ray = Ray(p, Normalize(c - p));
	for (int c = 0; c < fractures.Size(); c++)
	{
		double t;
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
static bool CanBeLinkedToCluster(const Vector3& candidate, const std::vector<Vector3>& cluster, const FractureSet& fractures, double R_Max)
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
			double tmax = Magnitude(p - candidate);
			double t;
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
\brief Load a greyscale image file for later use in gradient-based warping.
\param str image path
\param a min
\param b max
*/
void LoadImageFileForWarping(const char* str, double a, double b)
{
	int nx, ny, n;
	unsigned char* idata = stbi_load(str, &nx, &ny, &n, 1);
	warpingField = ScalarField2D(nx, ny, Box2D(Vector2(0), Vector2(1)));
	for (int i = 0; i < nx; i++)
	{
		for (int j = 0; j < ny; j++)
		{
			unsigned char g = idata[i * ny + j];
			double t = double(g) / 255.0f;
			warpingField.Set(i, j, t);
		}
	}
}

/*!
\brief Poisson sampling inside a 3D box, using dart-throwing.
\param box
\param r poisson radius
\param n number of try
*/
PointSet3 PoissonSamplingBox(const Box& box, double r, int n)
{
	PointSet3 set;
	double c = 4.0f * r * r;
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
FractureSet GenerateFractures(FractureType type, const Box& box, double r)
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
			double r = Random::Uniform(10.0f, 15.0f);
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
			double r = Random::Uniform(10.0f, 15.0f);
			set.fractures.push_back(Circle(samples.At(i), axis, r));
		}
	}
	else if (type == FractureType::Polyhedral)
	{
		// "Irregular jointing without arrangement into distinct sets, and of small joints"
		PointSet3 samples = PoissonSamplingBox(box, 1.0, 1000);
		for (int i = 0; i < samples.Size(); i++)
		{
			double r = Random::Uniform(2.0f, 12.0f);
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
		double step = box.Size()[2] / float(fracturing);
		double noiseStep = step / 10.0f;
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
SDFNode* ComputeBlockSDF(const std::vector<BlockCluster>& clusters)
{
	std::vector<SDFNode*> primitives;
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
			Vector3 pc = Triangle(v1, v2, v3).Center();
			planes.push_back(Plane(pc, pn));
		}
		
		// Discard if domain is not closed.
		auto convex = Plane::ConvexPoints(planes);
		if (convex.size() > 0)
		{
			// @Warning: this cannot work in all cases: the correct smoothing radius must depend on
			// The actual size of the convex primitive formed by the intersection of planes.
			// So artifacts may occur by using a constant.
			const double smoothRadius = 0.25;
			primitives.push_back(new SDFGradientWarp(new SDFBlock(planes, smoothRadius)));
		}

		// Free memory
		delete[] vertices;
		delete[] faceIndices;
	}
	return SDFUnionSphereLOD::OptimizedBVH(primitives, 0.5);
}

/*!
\brief Polygonization function of the implicit primitives using marching cubes.
\param sdf implicit block functions
*/
MC::mcMesh PolygonizeSDF(const Box& box, SDFNode* node)
{
	// Compute field function
	const int n = 200;
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
					field[(k * n + j) * n + i] = node->Signed(p);
				}
			}
		}
	}

	// Polygonize
	MC::mcMesh mesh;
	MC::marching_cube(field, n, n, n, mesh);
	return mesh;
}
