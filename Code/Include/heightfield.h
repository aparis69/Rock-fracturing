#pragma once

#include "basics.h"

// This class implements some state of the art algorithms on 2D terrains.
class HeightField : public ScalarField2D
{
protected:
	static Vector2i next[8];

public:
	HeightField();
	HeightField(int nx, int ny, const Box2D& bbox);
	HeightField(int nx, int ny, const Box2D& bbox, float value);
	~HeightField();
	
	ScalarField2D DrainageArea() const;
	ScalarField2D Wetness() const;
	ScalarField2D StreamPower() const;
	ScalarField2D Slope() const;
	Vector2i Next(int i, int j, int k) const;
};
