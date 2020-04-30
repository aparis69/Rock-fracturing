#pragma once
#include "vec.h"
#include <time.h>

#include <vector>

// Random (Dirty, C-style)
class Random
{
public:
	/*!
	\brief Constructor.
	*/
	Random()
	{
		// Empty
	}

	/*!
	\brief Compute a random number in a given range.
	\param a min
	\param b max
	*/
	static inline float Uniform(float a, float b)
	{
		return a + (b - a) * Uniform();
	}

	/*!
	\brief Compute a uniform random number in [0, 1]
	*/
	static inline float Uniform()
	{
		return float(rand()) / RAND_MAX;
	}

	/*!
	\brief Compute a random positive integer.
	*/
	static inline int Integer()
	{
		return rand();
	}
};


// AABB 3D class
class Box
{
protected:
	Vector3 a;
	Vector3 b;

public:
	explicit Box(const Vector3& A, const Vector3& B);
	explicit Box(const Vector3& C, float R);
	explicit Box(const Box& b1, const Box& b2);

	bool Contains(const Vector3&) const;
	Box Extended(const Vector3&) const;
	float Distance(const Vector3& p) const;
	Vector3 RandomInside() const;
	void SetParallelepipedic(float size, int& x, int& y, int& z);
	void SetParallelepipedic(int n, int& x, int& y, int& z);
	Vector3 Vertex(int) const;
	Vector3 BottomLeft() const;
	Vector3 TopRight() const;
	Vector3& operator[](int i);
	Vector3 operator[](int i) const;
};

/*
\brief Constructor
\param A lower left vertex in world coordinates
\param B upper right vertex in world coordinates
*/
inline Box::Box(const Vector3& A, const Vector3& B) : a(A), b(B)
{
}

/*
\brief Constructor
\param C box center
\param R radius
*/
inline Box::Box(const Vector3& C, float R)
{
	Vector3 RR = Vector3(R);
	a = C - RR;
	b = C + RR;
}

/*!
\brief Constructor from 2 boxes
\param b1 first box
\param b2 second box
*/
inline Box::Box(const Box& b1, const Box& b2)
{
	a = Vector3::Min(b1.a, b2.a);
	b = Vector3::Max(b1.b, b2.b);
}

/*
\brief Returns true if p is inside the box, false otherwise.
\param p world point
*/
inline bool Box::Contains(const Vector3& p) const
{
	return (p > a && p < b);
}

/*
\brief Returns the extended version of this box, without changing the instance.
\param r extending factor
*/
inline Box Box::Extended(const Vector3& r) const
{
	return Box(a - r, b + r);
}

/*!
\brief Creates a parallelepipedic box whose dimensions are integer
multiples of a given input reference size.
\param size Reference size, the dimension of the box will be a multiple of this size.
\param x, y ,z Three integers initialized in this function.
*/
inline void Box::SetParallelepipedic(float size, int& x, int& y, int& z)
{
	// Diagonal
	Vector3 d = (b - a);

	// Integer sizes
	x = int(d[0] / size);
	y = int(d[1] / size);
	z = int(d[2] / size);

	// Expand if necessary
	if (x == 0) { x++; }
	if (y == 0) { y++; }
	if (z == 0) { z++; }

	// Center
	Vector3 c = (a + b) * 0.5f;

	// Diagonal
	Vector3 e = Vector3(float(x), float(y), float(z)) * size / 2.0f;
	a = c - e;
	b = c + e;
}

/*!
\brief Inflates a box so that its dimensions should be a fraction of its maximum side length.
\param n Fraction.
\param x, y, z Three integers initialized in this function.
*/
inline void Box::SetParallelepipedic(int n, int& x, int& y, int& z)
{
	Vector3 d = (b - a); // Diagonal
	float e = d.Max(); // Maximum side length
	float size = e / n;
	SetParallelepipedic(size, x, y, z);
}

/*!
\brief Compute the distance between a point and the box.
\param p point
*/
inline float Box::Distance(const Vector3& p) const
{
	float r = 0.0;
	for (int i = 0; i < 3; i++)
	{
		if (p[i] < a[i])
		{
			float s = p[i] - a[i];
			r += s * s;
		}
		else if (p[i] > b[i])
		{
			float s = p[i] - b[i];
			r += s * s;
		}
	}
	return r;
}

/*!
\brief Compute a random point inside a box. Note that this
is not a uniform sampling if the box is not a regular box (width = height = length).

In practice, a uniform sampling doesn't give different results and is way slower, so
we avoid this.

\return a random point inside the box.
*/
inline Vector3 Box::RandomInside() const
{
	Vector3 s = b - a;
	float randw = Random::Uniform(-1.0f * s[0] / 2.0f, s[0] / 2.0f);
	float randh = Random::Uniform(-1.0f * s[1] / 2.0f, s[1] / 2.0f);
	float randl = Random::Uniform(-1.0f * s[2] / 2.0f, s[2] / 2.0f);
	return (a + b) / 2.0f + Vector3(randw, randh, randl);
}

/*
\brief Get one of the vertex of the box.
*/
inline Vector3 Box::Vertex(int i) const
{
	if (i == 0)
		return a;
	return b;
}

/*
\brief Get bottom left vertex in world coordinates
*/
inline Vector3 Box::BottomLeft() const
{
	return a;
}

/*
\brief Get top right vertex in world coordinates
*/
inline Vector3 Box::TopRight() const
{
	return b;
}

/*
\brief Access box vertex by reference
*/
inline Vector3& Box::operator[](int i)
{
	if (i == 0)
		return a;
	return b;
}

/*
\brief Access box vertex by const value
*/
inline Vector3 Box::operator[](int i) const
{
	if (i == 0)
		return a;
	return b;
}


// AABB 2D class.
class Box2D
{
protected:
	Vector2 a;
	Vector2 b;

public:
	explicit Box2D();
	explicit Box2D(const Vector2& A, const Vector2& B);
	explicit Box2D(const Vector2& C, float R);
	explicit Box2D(const Box& b);

	bool Contains(const Vector2&) const;
	bool Intersect(const Box2D& box) const;
	float Distance(const Vector2& p) const;

	Vector2 Vertex(int i) const;
	Vector2 Center() const;
	Vector2 BottomLeft() const;
	Vector2 TopRight() const;
	Box ToBox(float zMin, float zMax) const;
	Vector2& operator[](int i);
	Vector2 operator[](int i) const;
};

/*
\brief Default Constructor
*/
inline Box2D::Box2D()
{
	a = Vector2(0);
	b = Vector2(0);
}

/*
\brief Constructor
\param A lower left vertex in world coordinates
\param B upper right vertex in world coordinates
*/
inline Box2D::Box2D(const Vector2& A, const Vector2& B) : a(A), b(B)
{
}

/*
\brief Constructor
\param C box center
\param R radius
*/
inline Box2D::Box2D(const Vector2& C, float R)
{
	Vector2 RR = Vector2(R);
	a = C - RR;
	b = C + RR;
}

/*!
\brief Constructor from a 3D box.
\param box the box
*/
inline Box2D::Box2D(const Box& box)
{
	a = Vector2(box.Vertex(0));
	b = Vector2(box.Vertex(1));
}

/*
\brief Returns true if p is inside the box, false otherwise.
\param p world point
*/
inline bool Box2D::Contains(const Vector2& p) const
{
	return (p > a && p < b);
}

/*!
\brief Check if the 2D box intersects another 2D box.
\param box argument box.
*/
inline bool Box2D::Intersect(const Box2D& box) const
{
	if (((a[0] >= box.b[0]) || (a[1] >= box.b[1]) || (b[0] <= box.a[0]) || (b[1] <= box.a[1])))
		return false;
	else
		return true;
}

/*:
\brief Compute the distance between a point and the box.
\param p point
*/
inline float Box2D::Distance(const Vector2 & p) const
{
	float r = 0.0;
	for (int i = 0; i < 2; i++)
	{
		if (p[i] < a[i])
		{
			float s = p[i] - a[i];
			r += s * s;
		}
		else if (p[i] > b[i])
		{
			float s = p[i] - b[i];
			r += s * s;
		}
	}
	return r;
}

/*
\brief Get one of the vertex of the box.
*/
inline Vector2 Box2D::Vertex(int i) const
{
	if (i == 0)
		return a;
	return b;
}

/*
\brief Compute the box center.
*/
inline Vector2 Box2D::Center() const
{
	return (a + b) / 2;
}

/*
\brief Get bottom left vertex in world coordinates
*/
inline Vector2 Box2D::BottomLeft() const
{
	return a;
}

/*
\brief Get top right vertex in world coordinates
*/
inline Vector2 Box2D::TopRight() const
{
	return b;
}

/*
\brief Transform a Box2 in a Box.
\param yMin altitude of the first vertex for the new Box
\param yMax altitude of the second vertex for the new Box
*/
inline Box Box2D::ToBox(float yMin, float yMax) const
{
	return Box(a.ToVector3(yMin), b.ToVector3(yMax));
}

/*
\brief Access box vertex by reference
*/
inline Vector2& Box2D::operator[](int i)
{
	if (i == 0)
		return a;
	return b;
}

/*
\brief Access box vertex by const value
*/
inline Vector2 Box2D::operator[](int i) const
{
	if (i == 0)
		return a;
	return b;
}


// Circle2. A Circle geometric element.
class Circle2
{
protected:
	Vector2 center;
	float radius;

public:
	Circle2(const Vector2& c, float r);

	Vector2 RandomOn() const;
	Vector2 Center() const;
	float Radius() const;
	bool Contains(const Vector2& p) const;
};

/*!
\brief Constructor
\param c center
\param r radius
*/
inline Circle2::Circle2(const Vector2& c, float r)
{
	center = c;
	radius = r;
}

/*!
\brief Compute a random point on the circle, uniformly.
*/
inline Vector2 Circle2::RandomOn() const
{
	float u = Random::Uniform(-radius, radius);
	float v = Random::Uniform(-radius, radius);
	float s = u * u + v * v;

	float rx = (u * u - v * v) / s;
	float ry = 2.0f * u * v / s;
	return center + Vector2(rx, ry) * radius;
}

/*!
\brief Returns the circle center.
*/
inline Vector2 Circle2::Center() const
{
	return center;
}

/*!
\brief Returns the circle radius.
*/
inline float Circle2::Radius() const
{
	return radius;
}

/*!
\brief Check if a given point lies inside the circle.
*/
inline bool Circle2::Contains(const Vector2& p) const
{
	return (Magnitude(p - center) < radius);
}


// Sphere. Spherical geometric element.
class Sphere
{
protected:
	Vector3 center;
	float radius;

public:
	Sphere(const Vector3& c, float r);

	float Distance(const Vector3& p) const;
	bool Contains(const Vector3& p) const;
	Vector3 RandomInside() const;
	Vector3 Center() const;
	float Radius() const;
};

/*!
\brief Constructor.
\param c center
\param r radius
*/
inline Sphere::Sphere(const Vector3& c, float r) : center(c), radius(r)
{

}

/*!
\brief Compute the distance between a point and a sphere.
If the point lies inside the sphere, the distance is considered to be 0.
\param p world point
*/
inline float Sphere::Distance(const Vector3& p) const
{
	float d = Magnitude(p - center);
	return d < radius ? 0 : d;
}

/*!
\brief Returns true of the point lies inside the sphere (ie. Distance(p) == 0)
\param p world point
*/
inline bool Sphere::Contains(const Vector3& p) const
{
	return Distance(p) == 0;
}

/*!
\brief Compute a random point inside a sphere, uniformly.
*/
inline Vector3 Sphere::RandomInside() const
{
	return center + Vector3(Random::Uniform(-radius, radius), Random::Uniform(-radius, radius), Random::Uniform(-radius, radius));
}

/*!
\brief Returns the sphere center.
*/
inline Vector3 Sphere::Center() const
{
	return center;
}

/*!
\brief Returns the sphere radius.
*/
inline float Sphere::Radius() const
{
	return radius;
}


// ScalarField2D. Represents a 2D field (nx * ny) of scalar values bounded in world space. Can represent a heightfield.
class ScalarField2D
{
protected:
	Box2D box;
	int nx, ny;
	std::vector<float> values;

public:
	/*
	\brief Default Constructor
	*/
	inline ScalarField2D() : nx(0), ny(0)
	{
		// Empty
	}

	/*
	\brief Constructor
	\param nx size in x axis
	\param ny size in z axis
	\param bbox bounding box of the domain in world coordinates
	*/
	inline ScalarField2D(int nx, int ny, const Box2D& bbox) : box(bbox), nx(nx), ny(ny)
	{
		values.resize(size_t(nx * ny));
	}

	/*
	\brief Constructor
	\param nx size in x axis
	\param ny size in y axis
	\param bbox bounding box of the domain
	\param value default value of the field
	*/
	inline ScalarField2D(int nx, int ny, const Box2D& bbox, float value) : box(bbox), nx(nx), ny(ny)
	{
		values.resize(nx * ny);
		Fill(value);
	}

	/*
	\brief copy constructor
	\param field Scalarfield2D to copy
	*/
	inline ScalarField2D(const ScalarField2D& field) : ScalarField2D(field.nx, field.ny, field.box)
	{
		for (unsigned int i = 0; i < values.size(); i++)
			values[i] = field.values[i];
	}

	/*
	\brief Destructor
	*/
	inline ~ScalarField2D()
	{
	}

	/*
	\brief Compute the gradient for the vertex (i, j)
	*/
	inline Vector2 Gradient(int i, int j) const
	{
		Vector2 ret;
		float cellSizeX = (box.Vertex(1).x - box.Vertex(0).x) / (nx - 1);
		float cellSizeY = (box.Vertex(1).y - box.Vertex(0).y) / (ny - 1);

		// X Gradient
		if (i == 0)
			ret.x = (Get(i + 1, j) - Get(i, j)) / cellSizeX;
		else if (i == ny - 1)
			ret.x = (Get(i, j) - Get(i - 1, j)) / cellSizeX;
		else
			ret.x = (Get(i + 1, j) - Get(i - 1, j)) / (2.0f * cellSizeX);

		// Y Gradient
		if (j == 0)
			ret.y = (Get(i, j + 1) - Get(i, j)) / cellSizeY;
		else if (j == nx - 1)
			ret.y = (Get(i, j) - Get(i, j - 1)) / cellSizeY;
		else
			ret.y = (Get(i, j + 1) - Get(i, j - 1)) / (2.0f * cellSizeY);

		return ret;
	}

	/*
	\brief Normalize this field
	*/
	inline void NormalizeField()
	{
		float min = Min();
		float max = Max();
		for (int i = 0; i < ny * nx; i++)
			values[i] = (values[i] - min) / (max - min);
	}

	/*
	\brief Return the normalized version of this field
	*/
	inline ScalarField2D Normalized() const
	{
		ScalarField2D ret(*this);
		float min = Min();
		float max = Max();
		for (int i = 0; i < ny * nx; i++)
			ret.values[i] = (ret.values[i] - min) / (max - min);
		return ret;
	}

	/*!
	\brief Computes and returns the square root of the ScalarField.
	*/
	inline ScalarField2D Sqrt() const
	{
		ScalarField2D ret(*this);
		for (int i = 0; i < values.size(); i++)
			ret.values[i] = sqrt(ret.values[i]);
		return ret;
	}

	/*
	\brief Compute a vertex world position including his height.
	*/
	inline Vector3 Vertex(int i, int j) const
	{
		float x = box.Vertex(0).x + i * (box.Vertex(1).x - box.Vertex(0).x) / (nx - 1);
		float y = Get(i, j);
		float z = box.Vertex(0).y + j * (box.Vertex(1).y - box.Vertex(0).y) / (ny - 1);
		return Vector3(z, y, x);
	}

	/*
	\brief Compute a vertex world position including his height.
	*/
	inline Vector3 Vertex(const Vector2i& v) const
	{
		float x = box.Vertex(0).x + v.x * (box.Vertex(1).x - box.Vertex(0).x) / (nx - 1);
		float y = Get(v.x, v.y);
		float z = box.Vertex(0).y + v.y * (box.Vertex(1).y - box.Vertex(0).y) / (ny - 1);
		return Vector3(z, y, x);
	}

	/*
	\brief Get Vertex world position by performing bilinear interpolation.
	\param v world position in 2D
	*/
	inline Vector3 Vertex(const Vector2& v) const
	{
		return Vector3(v.x, GetValueBilinear(v), v.y);
	}

	/*!
	\brief Check if a point lies inside the bounding box of the field.
	*/
	inline bool Inside(const Vector2& p) const
	{
		Vector2 q = p - box.Vertex(0);
		Vector2 d = box.Vertex(1) - box.Vertex(0);

		float u = q[0] / d[0];
		float v = q[1] / d[1];

		int j = int(u * (nx - 1));
		int i = int(v * (ny - 1));

		return Inside(i, j);
	}

	/*!
	\brief Check if a point lies inside the bounding box of the field.
	*/
	inline bool Inside(int i, int j) const
	{
		if (i < 0 || i >= nx || j < 0 || j >= ny)
			return false;
		return true;
	}

	/*!
	\brief Check if a point lies inside the bounding box of the field.
	*/
	inline bool Inside(const Vector2i& v) const
	{
		if (v.x < 0 || v.x >= nx || v.y < 0 || v.y >= ny)
			return false;
		return true;
	}

	/*!
	\brief Utility.
	*/
	inline Vector2i ToIndex2D(const Vector2& p) const
	{
		Vector2 q = p - box.Vertex(0);
		Vector2 d = box.Vertex(1) - box.Vertex(0);

		float u = q[0] / d[0];
		float v = q[1] / d[1];

		int j = int(u * (nx - 1));
		int i = int(v * (ny - 1));

		return Vector2i(i, j);
	}

	/*!
	\brief Utility.
	*/
	inline void ToIndex2D(int index, int& i, int& j) const
	{
		i = index / nx;
		j = index % nx;
	}

	/*!
	\brief Utility.
	*/
	inline Vector2i ToIndex2D(int index) const
	{
		return Vector2i(index / nx, index % nx);
	}

	/*!
	\brief Utility.
	*/
	inline int ToIndex1D(const Vector2i& v) const
	{
		return v.x * nx + v.y;
	}

	/*!
	\brief Utility.
	*/
	inline int ToIndex1D(int i, int j) const
	{
		return i * nx + j;
	}

	/*!
	\brief Returns the value of the field at a given coordinate.
	*/
	inline float Get(int row, int column) const
	{
		int index = ToIndex1D(row, column);
		return values[index];
	}

	/*!
	\brief Returns the value of the field at a given coordinate.
	*/
	inline float Get(int index) const
	{
		return values[index];
	}

	/*!
	\brief Returns the value of the field at a given coordinate.
	*/
	inline float Get(const Vector2i& v) const
	{
		int index = ToIndex1D(v);
		return values[index];
	}

	/*!
	\brief Todo
	*/
	void Add(int i, int j, float v)
	{
		values[ToIndex1D(i, j)] += v;
	}

	/*!
	\brief Todo
	*/
	void Remove(int i, int j, float v)
	{
		values[ToIndex1D(i, j)] -= v;
	}

	/*!
	\brief Todo
	*/
	void Add(const ScalarField2D& field)
	{
		for (int i = 0; i < values.size(); i++)
			values[i] += field.values[i];
	}

	/*!
	\brief Todo
	*/
	void Remove(const ScalarField2D& field)
	{
		for (int i = 0; i < values.size(); i++)
			values[i] -= field.values[i];
	}

	/*!
	\brief Compute the bilinear interpolation at a given world point.
	\param p world point.
	*/
	inline float GetValueBilinear(const Vector2& p) const
	{
		Vector2 q = p - box.Vertex(0);
		Vector2 d = box.Vertex(1) - box.Vertex(0);

		float texelX = 1.0f / float(nx - 1);
		float texelY = 1.0f / float(ny - 1);

		float u = q[0] / d[0];
		float v = q[1] / d[1];

		int i = int(v * (ny - 1));
		int j = int(u * (nx - 1));

		if (!Inside(i, j) || !Inside(i + 1, j + 1))
			return -1.0f;

		float anchorU = j * texelX;
		float anchorV = i * texelY;

		float localU = (u - anchorU) / texelX;
		float localV = (v - anchorV) / texelY;

		float v1 = Get(i, j);
		float v2 = Get(i + 1, j);
		float v3 = Get(i + 1, j + 1);
		float v4 = Get(i, j + 1);

		return (1 - localU) * (1 - localV) * v1
			+ (1 - localU) * localV * v2
			+ localU * (1 - localV) * v4
			+ localU * localV * v3;
	}

	/*!
	\brief Fill all the field with a given value.
	*/
	inline void Fill(float v)
	{
		std::fill(values.begin(), values.end(), v);
	}

	/*!
	\brief Set a given value at a given coordinate.
	*/
	inline void Set(int row, int column, float v)
	{
		values[ToIndex1D(row, column)] = v;
	}

	/*!
	\brief Set a given value at a given coordinate.
	*/
	inline void Set(const Vector2i& coord, float v)
	{
		values[ToIndex1D(coord)] = v;
	}

	/*!
	\brief Set a given value at a given coordinate.
	*/
	inline void Set(int index, float v)
	{
		values[index] = v;
	}

	/*!
	\brief Todo
	*/
	inline void ThresholdInferior(float t, float v)
	{
		for (int i = 0; i < values.size(); i++)
		{
			if (values[i] <= t)
				values[i] = v;
		}
	}

	/*!
	\brief Compute the maximum of the field.
	*/
	inline float Max() const
	{
		if (values.size() == 0)
			return 0.0f;
		float max = values[0];
		for (int i = 1; i < values.size(); i++)
		{
			if (values[i] > max)
				max = values[i];
		}
		return max;
	}

	/*!
	\brief Compute the minimum of the field.
	*/
	inline float Min() const
	{
		if (values.size() == 0)
			return 0.0f;
		float min = values[0];
		for (int i = 1; i < values.size(); i++)
		{
			if (values[i] < min)
				min = values[i];
		}
		return min;
	}

	/*!
	\brief Compute the average value of the scalarfield.
	*/
	inline float Average() const
	{
		float sum = 0.0f;
		for (int i = 0; i < values.size(); i++)
			sum += values[i];
		return sum / values.size();
	}

	/*!
	\brief Returns the size of the array.
	*/
	inline Vector2i Size() const
	{
		return Vector2i(nx, ny);
	}

	/*!
	\brief Returns the size of x-axis of the array.
	*/
	inline int SizeX() const
	{
		return nx;
	}

	/*!
	\brief Returns the size of y-axis of the array.
	*/
	inline int SizeY() const
	{
		return ny;
	}

	/*!
	\brief Returns the bottom left corner of the bounding box.
	*/
	inline Vector2 BottomLeft() const
	{
		return box.Vertex(0);
	}

	/*!
	\brief Returns the top right corner of the bounding box.
	*/
	inline Vector2 TopRight() const
	{
		return box.Vertex(1);
	}

	/*!
	\brief Returns the bounding box of the field.
	*/
	inline Box2D GetBox() const
	{
		return box;
	}

	/*!
	\brief Compute the memory used by the field.
	*/
	inline int Memory() const
	{
		return sizeof(ScalarField2D) + sizeof(float) * int(values.size());
	}
};
