#pragma once
#include "basics.h"
#include "MC.h"

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

struct BlockSDF
{
public:
	std::vector<Plane> planes;
	float smoothRadius;

	inline BlockSDF(const std::vector<Plane>& pl, float sr)
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
	inline float SmoothingPolynomial(float d1, float d2, float sr) const
	{
		float h = Math::Max(sr - Math::Abs(d1 - d2), 0.0f) / sr;
		return Math::Min(d1, d2) - h * h * sr * 0.25f;
	}

	/*!
	\brief Compute the signed distance to the block primitive.
	\param p point
	*/
	inline float Signed(const Vector3& p) const
	{
		float d = planes.at(0).Signed(p);
		for (int i = 1; i < planes.size(); i++)
		{
			float dd = planes.at(i).Signed(p);
			d = -SmoothingPolynomial(-d, -dd, smoothRadius);
		}
		return d;
	}
};

PointSet3 PoissonSamplingBox(const Box& box, float r, int n);
FractureSet GenerateFractures(FractureType type, const Box& box, float r);
std::vector<BlockCluster> ComputeBlockClusters(PointSet3& set, const FractureSet& frac);
std::vector<BlockSDF> ComputeBlockSDF(const std::vector<BlockCluster>& clusters);
MC::mcMesh PolygonizeSDF(const Box& box, const std::vector<BlockSDF>& clusters);
