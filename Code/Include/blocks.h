#pragma once
#include "basics.h"
#include "MC.h"
#include "noise.h"

enum FractureType
{
	Equidimensional,
	Tabular,
	Rhombohedral,
	Polyhedral
};

struct PointSet3
{
public:
	std::vector<Vector3> pts;

	inline Vector3 At(int i) const
	{
		return pts[i];
	}

	inline Vector3& At(int i)
	{
		return pts[i];
	}

	inline int Size() const
	{
		return int(pts.size());
	}
};

struct FractureSet
{
public:
	std::vector<Circle> fractures;

	inline Circle At(int i) const
	{
		return fractures[i];
	}

	inline Circle& At(int i)
	{
		return fractures[i];
	}

	inline int Size() const
	{
		return int(fractures.size());
	}
};

struct BlockCluster
{
public:
	std::vector<Vector3> pts;
};


struct SDFNode
{
public:
	Box box;

	SDFNode();
	SDFNode(const Box& box);
	Vector3 Gradient(const Vector3& p) const;
	virtual double Signed(const Vector3& p) const = 0;
};

struct SDFUnionSphereLOD : public SDFNode
{
public:
	Sphere sphere;
	double re;
	SDFNode* e[2];

	SDFUnionSphereLOD(SDFNode*, SDFNode*, double re);
	double Signed(const Vector3& p) const;

	static SDFNode* OptimizedBVH(std::vector<SDFNode*>& nodes, double re);
	static SDFNode* OptimizedBVHRecursive(std::vector<SDFNode*>& nodes, int begin, int end, double re);
};

struct SDFGradientWarp : public SDFNode
{
public:
	SDFNode* e;

	SDFGradientWarp(SDFNode* e);
	double WarpingStrength(const Vector3& p, const Vector3& n) const;
	double Signed(const Vector3& p) const;
};

struct SDFBlock : public SDFNode
{
public:
	std::vector<Plane> planes;
	double smoothRadius;

	SDFBlock(const std::vector<Plane>& pl, double sr);
	double SmoothingPolynomial(double d1, double d2, double sr) const;
	double Signed(const Vector3& p) const;
};

void LoadImageFileForWarping(const char* str, double a, double b);
PointSet3 PoissonSamplingBox(const Box& box, double r, int n);
FractureSet GenerateFractures(FractureType type, const Box& box, double r);
std::vector<BlockCluster> ComputeBlockClusters(PointSet3& set, const FractureSet& frac);
SDFNode* ComputeBlockSDF(const std::vector<BlockCluster>& clusters);
MC::mcMesh PolygonizeSDF(const Box& box, SDFNode* node);
