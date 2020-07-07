#pragma once
#include "basics.h"

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
		return pts.size();
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
		return fractures.size();
	}
};

struct BlockCluster
{
public:
	std::vector<Vector3> pts;
};

PointSet3 PoissonSamplingCube(const Box& box, float r, int n);
FractureSet GenerateFractures(FractureType type, const Box& box, float r);
std::vector<BlockCluster> ComputeBlockClusters(PointSet3& set, const FractureSet& frac);
void ComputeBlockPrimitives(const std::vector<BlockCluster>& clusters);
