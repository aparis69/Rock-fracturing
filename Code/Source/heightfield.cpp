#include "heightfield.h"
#include "vec.h"

#include <algorithm>
#include <array>
#include <deque>
#include <queue>

/*
\brief Main Class for representing 2D HeightField. Implements state of the art map computation: Slope, StreamPower, Drainage etc...
*/

Vector2i HeightField::next[8] = { Vector2i(1, 0), Vector2i(1, 1), Vector2i(0, 1), Vector2i(-1, 1), Vector2i(-1, 0), Vector2i(-1, -1), Vector2i(0, -1), Vector2i(1, -1) };

/*
\brief Default constructor. Call Scalarfield2D default constructor.
*/
HeightField::HeightField() : ScalarField2D()
{
}

/*
\brief Constructor
\param nx width size of field
\param ny height size of field
\param bottomLeft bottom left vertex world coordinates
\param topRight top right vertex world coordinates
*/
HeightField::HeightField(int nx, int ny, const Box2D& bbox) : ScalarField2D(nx, ny, bbox)
{
}

/*
\brief Constructor
\param nx width size of field
\param ny height size of field
\param bbox bounding box in world space
\param value default value for the field
*/
HeightField::HeightField(int nx, int ny, const Box2D& bbox, float value) : ScalarField2D(nx, ny, bbox, value)
{

}

/*
\brief Destructor
*/
HeightField::~HeightField()
{
}

/*!
\brief Todo
*/
Vector2i HeightField::Next(int i, int j, int k) const
{
	return Vector2i(i, j) + next[k];
}

/*
\brief Compute the Drainage Area field. For an explanation, see https://hal.inria.fr/hal-01262376/file/2016_cordonnier.pdf
or https://arches.liris.cnrs.fr/publications/articles/SIGGRAPH2013_PCG_Terrain.pdf.
*/
ScalarField2D HeightField::DrainageArea() const
{
	// Utility struct
	typedef struct ScalarValue
	{
		int x, y;
		float value;
		inline ScalarValue() { }
		inline ScalarValue(int a, int b, float h) : x(a), y(b), value(h) { }
	} ScalarValue;

	// Sort all point by decreasing height
	std::deque<ScalarValue> points;
	for (int i = 0; i < ny; i++)
	{
		for (int j = 0; j < nx; j++)
			points.push_back(ScalarValue(i, j, Get(i, j)));
	}
	std::sort(points.begin(), points.end(), [](ScalarValue p1, ScalarValue p2) { return p1.value > p2.value; });

	std::array<float, 8> slopes;
	std::array<Vector2i, 8> coords;
	ScalarField2D DA = ScalarField2D(nx, ny, box, 1.0);
	while (!points.empty())
	{
		ScalarValue p = points.front();
		points.pop_front();

		slopes.fill(0.0f);
		int i = p.x, j = p.y;
		float h = Get(i, j);
		int neighbourCount = 0;
		for (int k = -1; k <= 1; k++)
		{
			for (int l = -1; l <= 1; l++)
			{
				if ((k == 0 && l == 0) || Inside(i + k, j + l) == false)
					continue;
				// If current point has lower neighbour : compute slope to later distribute accordingly.
				float nH = Get(i + k, j + l);
				if (h > nH)
				{
					float dH = h - nH;
					if (k + l == -1 || k + l == 1)
						slopes[neighbourCount] = dH;
					else
						slopes[neighbourCount] = dH / sqrt(2.0f);

					coords[neighbourCount] = Vector2i(i + k, j + l);
					neighbourCount++;
				}
			}
		}

		// Distribute to those lower neighbours
		/*float sum = Math::Sum<float, 8>(slopes);
		for (int k = 0; k < neighbourCount; k++)
			DA.Set(coords[k], DA.Get(coords[k]) + DA.Get(i, j) * (slopes[k] / sum));*/
	}
	return DA;
}

/*
\brief Compute the slope field, ie Magnitude(Gradient(i, j)).
*/
ScalarField2D HeightField::Slope() const
{
	ScalarField2D S = ScalarField2D(nx, ny, box);
	for (int i = 0; i < ny; i++)
	{
		for (int j = 0; j < nx; j++)
			S.Set(i, j, Magnitude(Gradient(i, j)));
	}
	return S;
}

/*
\brief Compute the Wetness Index Field.
*/
ScalarField2D HeightField::Wetness() const
{
	ScalarField2D DA = DrainageArea();
	ScalarField2D S = Slope();
	for (int i = 0; i < ny; i++)
	{
		for (int j = 0; j < nx; j++)
			DA.Set(i, j, abs(log(DA.Get(i, j) / (1.0f + S.Get(i, j)))));
	}
	return DA;
}

/*
\brief Compute the StreamPower field, as described by http://geosci.uchicago.edu/~kite/doc/Whipple_and_Tucker_1999.pdf.
*/
ScalarField2D HeightField::StreamPower() const
{
	ScalarField2D DA = DrainageArea();
	ScalarField2D S = Slope();
	for (int i = 0; i < ny; i++)
	{
		for (int j = 0; j < nx; j++)
			DA.Set(i, j, sqrt(DA.Get(i, j)) * S.Get(i, j));
	}
	return DA;
}
