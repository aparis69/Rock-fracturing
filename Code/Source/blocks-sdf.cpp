#include "blocks.h"

// for std::partition in the bounding volume hierarchy
#include <algorithm>

// Source: http://nothings.org/stb
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

// Gradient Warping strength stored as global
static ScalarField2D warpingField;


/*!
\brief Default constructor for a node.
*/
SDFNode::SDFNode()
{
}

/*!
\brief Constructor from a bounding box.
*/
SDFNode::SDFNode(const Box& box) : box(box)
{
}

/*!
\brief Computes the gradient at a given point using central differences.
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
\brief Constructor for a binary union node, with a simple LOD system.
\param a, b child
\param re
*/
SDFUnionSphereLOD::SDFUnionSphereLOD(SDFNode* a, SDFNode* b, double re) : SDFNode(Box(a->box, b->box)), re(re), sphere(Sphere(box.Center(), box.Size().Max()))
{
	e[0] = a;
	e[1] = b;
}

/*!
\brief Compute the signed distance.
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
\brief Utility function for constructing a BVH of signed distance field nodes.
*/
SDFNode* SDFUnionSphereLOD::OptimizedBVH(std::vector<SDFNode*>& nodes, double re)
{
	if (nodes.size() == 0)
		return nullptr;
	return OptimizedBVHRecursive(nodes, 0, int(nodes.size()), re);
}

/*!
\brief Utility function for constructing a BVH of signed distance field nodes.
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
\brief Constructor for gradient-warping operator. Parameters are hardcoded in the operator.
\param e child.
*/
SDFGradientWarp::SDFGradientWarp(SDFNode* e) : e(e)
{
	box = e->box;
}

/*!
\brief
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
\brief Compute the signed distance.
\param p point
*/
double SDFGradientWarp::Signed(const Vector3& p) const
{
	// First compute gradient-based warping
	Vector3 g = e->Gradient(p);
	double s = 0.65 * WarpingStrength(p, -Normalize(g));	// Hardcoded strength

	// Then compute contribution from the convex block
	return e->Signed(p + g * s);
}


/*!
\brief Constructor for a block, from a set of plane and a smoothing radius.
\param pl set of planes forming a closed convex shape.
\param sr smoothing radius.
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
