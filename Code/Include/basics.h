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
	static inline double Uniform(double a, double b)
	{
		return a + (b - a) * Uniform();
	}

	/*!
	\brief Compute a uniform random number in [0, 1]
	*/
	static inline double Uniform()
	{
		return double(rand()) / RAND_MAX;
	}

	/*!
	\brief Compute a random positive integer.
	*/
	static inline int Integer()
	{
		return rand();
	}
};


// 3D Ray
class Ray
{
public:
	Vector3 o;
	Vector3 d;

	Ray();
	Ray(const Vector3& oo, const Vector3& dd);
	Vector3 operator()(double t) const;
};

inline Ray::Ray()
{

}

inline Ray::Ray(const Vector3& oo, const Vector3& dd)
{
	o = oo;
	d = dd;
}

inline Vector3 Ray::operator()(double t) const
{
	return o + t * d;
}


// Plane
class Plane
{
protected:
	Vector3 p;
	Vector3 n;
	double c;

public:
	Plane();
	Plane(const Vector3& p, const Vector3& n);

	Vector3 Normal() const;
	Vector3 Point() const;
	int Side(const Vector3& p) const;
	double Signed(const Vector3& p) const;
	static bool Intersection(const Plane& a, const Plane& b, const Plane& c, Vector3& p);
	static std::vector<Vector3> ConvexPoints(const std::vector<Plane>& planes);
};

/*!
\brief
*/
inline Plane::Plane()
{
	p = Vector3(0);
	n = Vector3(0);
	c = 0.0;
}

/*!
\brief Constructor from a point and a normal.
\param pp point on the plane
\param plane normal (should be normalized)
*/
inline Plane::Plane(const Vector3& pp, const Vector3& nn)
{
	p = pp;
	n = nn;
	c = Dot(p, n);
}

/*!
\brief Returns the normal vector of the plane.
*/
inline Vector3 Plane::Normal() const
{
	return n;
}

/*!
\brief Computes the signed distance to the plane.
\param p point
*/
inline double Plane::Signed(const Vector3& pp) const
{
	return Dot(n, pp) - c;
}

/*!
\brief Returns a point on the plane.
*/
inline Vector3 Plane::Point() const
{
	return p;
}

/*!
\brief Check if a point lies inside, outside or even on the plane.
The epsilon tolerance used for testing if the point is on the plane
is 10<SUP>-6</SUP>.
\param p Point.
*/
inline int Plane::Side(const Vector3& pp) const
{
	double r = Dot(n, pp) - c;

	// Epsilon test
	if (r > 1e-6)
		return 1;
	else if (r < -1e-6)
		return -1;
	else
		return 0;
}

/*!
\brief
*/
inline bool Plane::Intersection(const Plane& a, const Plane& b, const Plane& c, Vector3& p)
{
	double e = Matrix4(Matrix3(a.Normal(), b.Normal(), c.Normal())).Determinant();
	if (e < 1e-06)
		return false;
	p = (Dot(a.Point(), a.Normal()) * (Cross(b.Normal(), c.Normal()))) +
		(Dot(b.Point(), b.Normal()) * (Cross(c.Normal(), a.Normal()))) +
		(Dot(c.Point(), c.Normal()) * (Cross(a.Normal(), b.Normal())));
	p = p / (-e);
	return true;
}

/*!
\brief Compute the intersection of a set of half spaces.
This is a O(n^4) algorithm.
A better version in O(n^3) could be implemented: see Finding the Intersection
of Half-Spaces in Time O(n ln n). Preparata and Muller, Theoretical Computer Science 8, 45-55, 1979.
Returns a set of point representing the minimal convex polygon embedding all half spaces.
The function doesn't check if the intersection is bounded or not.
\param planes Set of planes.
*/
inline std::vector<Vector3> Plane::ConvexPoints(const std::vector<Plane>& planes)
{
	std::vector<Vector3> pts;
	for (int i = 0; i < planes.size(); i++)
	{
		for (int j = i + 1; j < planes.size(); j++)
		{
			for (int k = j + 1; k < planes.size(); k++)
			{
				Vector3 p;
				bool intersect = Intersection(planes[i], planes[j], planes[k], p);
				if (intersect)
				{
					bool isInside = true;
					for (int l = 0; l < planes.size(); l++)
					{
						// Do not check point ijk with one of its generating plane
						if (l == i || l == j || l == k)
							continue;
						int s = planes[l].Side(p);
						if (s > 0)
						{
							isInside = false;
							break;
						}
					}
					if (isInside)
						pts.push_back(p);
				}
			}
		}
	}
	return pts;
}


// 3D Triangle
class Triangle
{
private:
	Vector3 pts[3];

public:
	Triangle();
	Triangle(const Vector3& a, const Vector3& b, const Vector3& c);

	Vector3 Center() const;
	Vector3 Normal() const;
	Vector3 Point(int i) const;
};

/*!
\brief Default Constructor.
*/
inline Triangle::Triangle()
{
}

/*!
\brief Constructor from three points.
\param a, b, c points.
*/
inline Triangle::Triangle(const Vector3& a, const Vector3& b, const Vector3& c)
{
	pts[0] = a;
	pts[1] = b;
	pts[2] = c;
}

/*!
\brief Computes and returns the center of the triangle.
*/
inline Vector3 Triangle::Center() const
{
	return (pts[0] + pts[1] + pts[2]) / 3.0f;
}

/*!
\brief Computes and returns the normal of the triangle.
*/
inline Vector3 Triangle::Normal() const
{
	return Normalize(Cross(pts[1] - pts[0], pts[2] - pts[0]));
}

/*!
\brief Returns one of the point of the triangle.
\param i point index
*/
inline Vector3 Triangle::Point(int i) const
{
	return pts[i];
}


// AABB 3D
class Box
{
protected:
	Vector3 a;
	Vector3 b;

public:
	inline Box() { }
	explicit Box(const Vector3& A, const Vector3& B);
	explicit Box(const Vector3& C, double R);
	explicit Box(const Box& b1, const Box& b2);
	explicit Box(const std::vector<Vector3>& pts);

	bool Contains(const Vector3&) const;
	Box Extended(const Vector3&) const;
	Vector3 Center() const;
	double Distance(const Vector3& p) const;
	Vector3 Diagonal() const;
	Vector3 Size() const;
	Vector3 RandomInside() const;
	void SetParallelepipedic(double size, int& x, int& y, int& z);
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
inline Box::Box(const Vector3& C, double R)
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

/*!
\brief Constructor from a point set.
\param pts set of point
*/
inline Box::Box(const std::vector<Vector3>& pts)
{
	for (int j = 0; j < 3; j++)
	{
		a[j] = pts.at(0)[j];
		b[j] = pts.at(0)[j];
		for (int i = 1; i < pts.size(); i++)
		{
			if (pts.at(i)[j] < a[j])
			{
				a[j] = pts.at(i)[j];
			}
			if (pts.at(i)[j] > b[j])
			{
				b[j] = pts.at(i)[j];
			}
		}
	}
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
\brief Computes and returns the center of the box.
*/
inline Vector3 Box::Center() const
{
	return (a + b) / 2.0f;
}

/*!
\brief Returns the diagonal of the box.
*/
inline Vector3 Box::Diagonal() const
{
	return (b - a);
}

/*!
\brief Returns the size of the box.
*/
inline Vector3 Box::Size() const
{
	return (b - a);
}

/*!
\brief Creates a parallelepipedic box whose dimensions are integer
multiples of a given input reference size.
\param size Reference size, the dimension of the box will be a multiple of this size.
\param x, y ,z Three integers initialized in this function.
*/
inline void Box::SetParallelepipedic(double size, int& x, int& y, int& z)
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
	Vector3 e = Vector3(double(x), double(y), double(z)) * size / 2.0f;
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
	double e = d.Max(); // Maximum side length
	double size = e / n;
	SetParallelepipedic(size, x, y, z);
}

/*!
\brief Compute the distance between a point and the box.
\param p point
*/
inline double Box::Distance(const Vector3& p) const
{
	double r = 0.0;
	for (int i = 0; i < 3; i++)
	{
		if (p[i] < a[i])
		{
			double s = p[i] - a[i];
			r += s * s;
		}
		else if (p[i] > b[i])
		{
			double s = p[i] - b[i];
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
	double randw = Random::Uniform(-1.0f * s[0] / 2.0f, s[0] / 2.0f);
	double randh = Random::Uniform(-1.0f * s[1] / 2.0f, s[1] / 2.0f);
	double randl = Random::Uniform(-1.0f * s[2] / 2.0f, s[2] / 2.0f);
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


// AABB 2D
class Box2D
{
protected:
	Vector2 a;
	Vector2 b;

public:
	explicit Box2D();
	explicit Box2D(const Vector2& A, const Vector2& B);
	explicit Box2D(const Vector2& C, double R);
	explicit Box2D(const Box& b);

	bool Contains(const Vector2&) const;
	bool Intersect(const Box2D& box) const;
	double Distance(const Vector2& p) const;

	Vector2 Vertex(int i) const;
	Vector2 Center() const;
	Vector2 BottomLeft() const;
	Vector2 TopRight() const;
	Box ToBox(double zMin, double zMax) const;
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
inline Box2D::Box2D(const Vector2& C, double R)
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
inline double Box2D::Distance(const Vector2 & p) const
{
	double r = 0.0;
	for (int i = 0; i < 2; i++)
	{
		if (p[i] < a[i])
		{
			double s = p[i] - a[i];
			r += s * s;
		}
		else if (p[i] > b[i])
		{
			double s = p[i] - b[i];
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
inline Box Box2D::ToBox(double yMin, double yMax) const
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


// Circle2. A 2D Circle geometric element.
class Circle2
{
protected:
	Vector2 center;
	double radius;

public:
	Circle2(const Vector2& c, double r);

	Vector2 RandomOn() const;
	Vector2 Center() const;
	double Radius() const;
	bool Contains(const Vector2& p) const;
};

/*!
\brief Constructor
\param c center
\param r radius
*/
inline Circle2::Circle2(const Vector2& c, double r)
{
	center = c;
	radius = r;
}

/*!
\brief Compute a random point on the circle, uniformly.
*/
inline Vector2 Circle2::RandomOn() const
{
	double u = Random::Uniform(-radius, radius);
	double v = Random::Uniform(-radius, radius);
	double s = u * u + v * v;

	double rx = (u * u - v * v) / s;
	double ry = 2.0f * u * v / s;
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
inline double Circle2::Radius() const
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


// Circle. A 3D Circle geometric element.
class Circle
{
protected:
	Vector3 center;
	Vector3 normal;
	double radius;

public:
	Circle(const Vector3& c, const Vector3& n, double r);

	Vector3 Center() const;
	Vector3 Normal() const;
	double Radius() const;
	bool Intersect(const Ray& r, double& t) const;
};

/*
\brief
*/
inline Circle::Circle(const Vector3& c, const Vector3& n, double r)
{
	center = c;
	normal = n;
	radius = r;
}

/*
\brief
*/
inline Vector3 Circle::Center() const
{
	return center;
}

/*
\brief
*/
inline Vector3 Circle::Normal() const
{
	return normal;
}

/*
\brief
*/
inline double Circle::Radius() const
{
	return radius;
}

/*!
\brief Compute the ray-disc intersection.
\param ray The ray.
\param t Intersection depth.
*/
inline bool Circle::Intersect(const Ray& ray, double& t) const
{
	double e = Dot(normal, ray.d);
	if (fabs(e) < 1e-6f)
		return false;
	t = Dot(center - ray.o, normal) / e;
	if (t < 0.0f)
		return false;
	Vector3 p = ray(t) - center;
	if (Dot(p, p) > radius * radius)
		return false;
	return true;
}


// Sphere. Spherical geometric element.
class Sphere
{
protected:
	Vector3 center;
	double radius;

public:
	Sphere(const Vector3& c, double r);

	double Distance(const Vector3& p) const;
	bool Contains(const Vector3& p) const;
	Vector3 RandomInside() const;
	Vector3 RandomSurface() const;
	Vector3 Center() const;
	double Radius() const;
};

/*!
\brief Constructor.
\param c center
\param r radius
*/
inline Sphere::Sphere(const Vector3& c, double r) : center(c), radius(r)
{

}

/*!
\brief Compute the distance between a point and a sphere.
If the point lies inside the sphere, the distance is considered to be 0.
\param p world point
*/
inline double Sphere::Distance(const Vector3& p) const
{
	double a = Dot((p - center), (p - center));
	if (a < radius * radius)
	{
		return 0.0;
	}
	a = sqrt(a) - radius;
	a *= a;
	return a;
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
\brief
*/
inline Vector3 Sphere::RandomSurface() const
{
	return Vector3(0);
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
inline double Sphere::Radius() const
{
	return radius;
}


// ScalarField2D. Represents a 2D field (nx * ny) of scalar values bounded in world space. Can represent a heightfield.
class ScalarField2D
{
protected:
	Box2D box;
	int nx, ny;
	std::vector<double> values;

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
	inline ScalarField2D(int nx, int ny, const Box2D& bbox, double value) : box(bbox), nx(nx), ny(ny)
	{
		values.resize(nx * ny);
		Fill(value);
	}

	/*!
	\brief Constructor
	\param nx size in x axis
	\param ny size in y axis
	\param bbox bounding box of the domain
	\param value all values for the field
	*/
	inline ScalarField2D(int nx, int ny, const Box2D& bbox, const std::vector<double>& vals) : ScalarField2D(nx, ny, bbox)
	{
		for (int i = 0; i < vals.size(); i++)
			values[i] = vals[i];
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
		double cellSizeX = (box.Vertex(1).x - box.Vertex(0).x) / (nx - 1);
		double cellSizeY = (box.Vertex(1).y - box.Vertex(0).y) / (ny - 1);

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
		double min = Min();
		double max = Max();
		for (int i = 0; i < ny * nx; i++)
			values[i] = (values[i] - min) / (max - min);
	}

	/*
	\brief Return the normalized version of this field
	*/
	inline ScalarField2D Normalized() const
	{
		ScalarField2D ret(*this);
		double min = Min();
		double max = Max();
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
		double x = box.Vertex(0).x + i * (box.Vertex(1).x - box.Vertex(0).x) / (nx - 1);
		double y = Get(i, j);
		double z = box.Vertex(0).y + j * (box.Vertex(1).y - box.Vertex(0).y) / (ny - 1);
		return Vector3(z, y, x);
	}

	/*
	\brief Compute a vertex world position including his height.
	*/
	inline Vector3 Vertex(const Vector2i& v) const
	{
		double x = box.Vertex(0).x + v.x * (box.Vertex(1).x - box.Vertex(0).x) / (nx - 1);
		double y = Get(v.x, v.y);
		double z = box.Vertex(0).y + v.y * (box.Vertex(1).y - box.Vertex(0).y) / (ny - 1);
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

		double u = q[0] / d[0];
		double v = q[1] / d[1];

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

		double u = q[0] / d[0];
		double v = q[1] / d[1];

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
	inline double Get(int row, int column) const
	{
		int index = ToIndex1D(row, column);
		return values[index];
	}

	/*!
	\brief Returns the value of the field at a given coordinate.
	*/
	inline double Get(int index) const
	{
		return values[index];
	}

	/*!
	\brief Returns the value of the field at a given coordinate.
	*/
	inline double Get(const Vector2i& v) const
	{
		int index = ToIndex1D(v);
		return values[index];
	}

	/*!
	\brief Todo
	*/
	void Add(int i, int j, double v)
	{
		values[ToIndex1D(i, j)] += v;
	}

	/*!
	\brief Todo
	*/
	void Remove(int i, int j, double v)
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
	inline double GetValueBilinear(const Vector2& p) const
	{
		Vector2 q = p - box.Vertex(0);
		Vector2 d = box.Vertex(1) - box.Vertex(0);

		double texelX = 1.0f / double(nx - 1);
		double texelY = 1.0f / double(ny - 1);

		double u = q[0] / d[0];
		double v = q[1] / d[1];

		int i = int(v * (ny - 1));
		int j = int(u * (nx - 1));

		if (!Inside(i, j) || !Inside(i + 1, j + 1))
			return -1.0f;

		double anchorU = j * texelX;
		double anchorV = i * texelY;

		double localU = (u - anchorU) / texelX;
		double localV = (v - anchorV) / texelY;

		double v1 = Get(i, j);
		double v2 = Get(i + 1, j);
		double v3 = Get(i + 1, j + 1);
		double v4 = Get(i, j + 1);

		return (1 - localU) * (1 - localV) * v1
			+ (1 - localU) * localV * v2
			+ localU * (1 - localV) * v4
			+ localU * localV * v3;
	}

	/*!
	\brief Fill all the field with a given value.
	*/
	inline void Fill(double v)
	{
		std::fill(values.begin(), values.end(), v);
	}

	/*!
	\brief Set a given value at a given coordinate.
	*/
	inline void Set(int row, int column, double v)
	{
		values[ToIndex1D(row, column)] = v;
	}

	/*!
	\brief Set a given value at a given coordinate.
	*/
	inline void Set(const Vector2i& coord, double v)
	{
		values[ToIndex1D(coord)] = v;
	}

	/*!
	\brief Set a given value at a given coordinate.
	*/
	inline void Set(int index, double v)
	{
		values[index] = v;
	}

	/*!
	\brief Todo
	*/
	inline void ThresholdInferior(double t, double v)
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
	inline double Max() const
	{
		if (values.size() == 0)
			return 0.0f;
		double max = values[0];
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
	inline double Min() const
	{
		if (values.size() == 0)
			return 0.0f;
		double min = values[0];
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
	inline double Average() const
	{
		double sum = 0.0f;
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
		return sizeof(ScalarField2D) + sizeof(double) * int(values.size());
	}
};
