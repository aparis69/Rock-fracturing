#include "blocks.h"

/*!
\brief
*/
PointSet3 PoissonSamplingCube(const Box& box, float r, int n)
{
	PointSet3 set;
	double c = 4.0 * r * r;
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
			double r = Random::Uniform(10.0, 15.0);
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
			double r = Random::Uniform(3.0, 7.0);
			set.fractures.push_back(Circle(samples.At(i), axis, r));
		}
	}
	else if (type == FractureType::Polyhedral)
	{
		// "Irregular jointing without arrangement into distinct sets, and of small joints"
		PointSet3 samples = PoissonSamplingCube(box, 1.0, 1000);
		for (int i = 0; i < samples.Size(); i++)
		{
			double r = Random::Uniform(2.0, 8.0);	
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
		p[0] += box.Diagonal()[0] / 2.0;
		p[1] += box.Diagonal()[1] / 2.0;
		double step = box.Size()[2] / double(fracturing);
		double noiseStep = step / 10.0;
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
			double r = Random::Uniform(1.5, 3.0);
			Vector3 axis = (Random::Integer() % 2) == 0 ? Vector3(1, 0, 0) : Vector3(0, 1, 0);
			set.fractures.push_back(Circle(poissonSamples[i], axis, r));
		}
	}
	return set;
}

/*!
\brief
*/
std::vector<BlockCluster> ComputeBlockClusters(PointSet3& set, const FractureSet& frac)
{

}

/*!
\brief
*/
void ComputeBlockPrimitives(const std::vector<BlockCluster>& clusters)
{

}