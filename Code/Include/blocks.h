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

struct BlockSDF
{
public:
	std::vector<Plane> planes;
	float smoothRadius;

	BlockSDF(const std::vector<Plane>& pl, float sr);
	float SmoothingPolynomial(float d1, float d2, float sr) const;
	float SignedSmoothConvex(const Vector3& p) const;
	Vector3 Gradient(const Vector3& p) const;
	float WarpingStrength(const Vector3& p, const Vector3& n) const;
	float Signed(const Vector3& p) const;
};

PointSet3 PoissonSamplingBox(const Box& box, float r, int n);
FractureSet GenerateFractures(FractureType type, const Box& box, float r);
std::vector<BlockCluster> ComputeBlockClusters(PointSet3& set, const FractureSet& frac);
std::vector<BlockSDF> ComputeBlockSDF(const std::vector<BlockCluster>& clusters);
MC::mcMesh PolygonizeSDF(const Box& box, const std::vector<BlockSDF>& clusters);
