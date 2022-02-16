#pragma once

#include <cmath>
#include <iostream>

/* Forward Declarations */
struct Vector2;
struct Vector3;
struct Vector4;

// Maths utility
namespace Math
{
	/*!
	\brief
	*/
	inline double Clamp(double x, double a = 0.0f, double b = 1.0f)
	{
		return x < a ? a : x > b ? b : x;
	}

	/*!
	\brief Create a linear step.
	\param x Value
	\param a, b Interval values.
	\return Real in unit inverval.
	*/
	inline double Step(double x, double a, double b)
	{
		if (x < a)
		{
			return 0.0f;
		}
		else if (x > b)
		{
			return 1.0f;
		}
		else
		{
			return (x - a) / (b - a);
		}
	}

	template<typename T>
	inline T Min(T a, T b)
	{
		return a < b ? a : b;
	}

	template<typename T>
	inline T Max(T a, T b)
	{
		return a > b ? a : b;
	}

	template<typename T>
	inline T Lerp(T a, T b, double t)
	{
		return (a * (1.0f - t)) + (b * t);
	}

	inline double Abs(double a)
	{
		return a < 0 ? -a : a;
	}

	inline double QuinticSmooth(double t)
	{
		return pow(t, 3.0f) * (t * (t * 6.0f - 15.0f) + 10.0f);
	}
}


/* Vector3 */
struct Vector3
{
public:
	double x, y, z;

	explicit Vector3() : x(0.0f), y(0.0f), z(0.0f) { }
	explicit Vector3(double n) : x(n), y(n), z(n) {}
	explicit Vector3(double x, double y, double z) : x(x), y(y), z(z) {}

	friend bool operator> (const Vector3&, const Vector3&);
	friend bool operator< (const Vector3&, const Vector3&);
	friend bool operator>= (const Vector3&, const Vector3&);
	friend bool operator<= (const Vector3&, const Vector3&);

	Vector3& operator+= (const Vector3& v)
	{
		x += v.x;
		y += v.y;
		z += v.z;
		return *this;
	}
	Vector3 operator-= (const Vector3& v)
	{
		return Vector3(x - v.x, y - v.y, z - v.z);
	}
	Vector3 operator*= (double f)
	{
		return Vector3(x * f, y * f, z * f);
	}
	Vector3 operator/= (double f)
	{
		return Vector3(x / f, y / f, z / f);
	}
	Vector3 operator*(const Vector3& u) const
	{
		return Vector3(x * u.x, y * u.y, z * u.z);
	}
	Vector3 operator*(double k) const
	{
		return Vector3(x * k, y * k, z * k);
	}
	Vector3 operator/(double k) const
	{
		return Vector3(x / k, y / k, z / k);
	}
	bool operator==(const Vector3& u) const
	{
		return (x == u.x && y == u.y && z == u.z);
	}
	bool operator!=(const Vector3& u) const
	{
		return (x != u.x || y != u.y || z != u.z);
	}
	Vector3 operator-(const Vector3& u) const
	{
		return Vector3(x - u.x, y - u.y, z - u.z);
	}
	Vector3 operator+(const Vector3& u) const
	{
		return Vector3(x + u.x, y + u.y, z + u.z);
	}
	Vector3 operator+(double k) const
	{
		return Vector3(x + k, y + k, z + k);
	}
	double operator[](int i) const
	{
		if (i == 0)
			return x;
		else if (i == 1)
			return y;
		return z;
	}
	double& operator[](int i)
	{
		if (i == 0)
			return x;
		else if (i == 1)
			return y;
		return z;
	}
	friend std::ostream& operator<<(std::ostream& stream, const Vector3& u);
	inline double Max() const
	{
		return Math::Max(Math::Max(x, y), z);
	}
	inline double Min() const
	{
		return Math::Min(Math::Min(x, y), z);
	}
	inline int MaxIndex() const
	{
		if (x >= y)
		{
			if (x >= z)
				return 0;
			else
				return 2;
		}
		else
		{
			if (y >= z)
				return 1;
			else
				return 2;
		}
	}
	static inline Vector3 Min(const Vector3& a, const Vector3& b)
	{
		return Vector3(Math::Min(a.x, b.x), Math::Min(a.y, b.y), Math::Min(a.z, b.z));
	}
	static inline Vector3 Max(const Vector3& a, const Vector3& b)
	{
		return Vector3(Math::Max(a.x, b.x), Math::Max(a.y, b.y), Math::Max(a.z, b.z));
	}
};
inline std::ostream& operator<<(std::ostream& stream, const Vector3& u)
{
	stream << "(" << u.x << ", " << u.y << ", " << u.z << ");";
	return stream;
}
inline Vector3 Cross(const Vector3& u, const Vector3& v)
{
	return Vector3((u.y * v.z) - (u.z * v.y), (u.z * v.x) - (u.x * v.z), (u.x * v.y) - (u.y * v.x));
}
inline double Dot(const Vector3& u, const Vector3& v)
{
	return u.x * v.x + u.y * v.y + u.z * v.z;
}
inline double Magnitude(const Vector3& u)
{
	return sqrt(u.x * u.x + u.y * u.y + u.z * u.z);
}
inline double SquaredMagnitude(const Vector3& u)
{
	return u.x * u.x + u.y * u.y + u.z * u.z;
}
inline Vector3 Normalize(const Vector3& v)
{
	double kk = 1.0f / Magnitude(v);
	return v * kk;
}
inline Vector3 operator-(const Vector3& v)
{
	return Vector3(-v.x, -v.y, -v.z);
}
inline bool operator>(const Vector3& u, const Vector3& v)
{
	return (u.x > v.x) && (u.y > v.y) && (u.z > v.z);
}
inline bool operator<(const Vector3& u, const Vector3& v)
{
	return (u.x < v.x) && (u.y < v.y) && (u.z < v.z);
}
inline bool operator>=(const Vector3& u, const Vector3& v)
{
	return (u.x >= v.x) && (u.y >= v.y) && (u.z >= v.z);
}
inline bool operator<=(const Vector3& u, const Vector3& v)
{
	return (u.x <= v.x) && (u.y <= v.y) && (u.z <= v.z);
}
inline Vector3 operator*(double a, const Vector3& v)
{
	return v * a;
}
inline Vector3 Abs(const Vector3& u)
{
	return Vector3(u[0] > 0.0 ? u[0] : -u[0], u[1] > 0.0 ? u[1] : -u[1], u[2] > 0.0 ? u[2] : -u[2]);
}

/* Vector2 */
struct Vector2
{
public:
	double x, y;

	explicit Vector2() : x(0.0f), y(0.0f) { }
	explicit Vector2(double n) : x(n), y(n) { }
	explicit Vector2(double x, double y) : x(x), y(y) { }
	explicit Vector2(const Vector3& v) : x(v.x), y(v.z) { }

	friend bool operator> (const Vector2&, const Vector2&);
	friend bool operator< (const Vector2&, const Vector2&);
	friend bool operator>= (const Vector2&, const Vector2&);
	friend bool operator<= (const Vector2&, const Vector2&);

	Vector2 operator+= (const Vector2& v)
	{
		return Vector2(x + v.x, y + v.y);
	}
	Vector2 operator-= (const Vector2& v)
	{
		return Vector2(x - v.x, y - v.y);
	}
	Vector2 operator*= (double f)
	{
		return Vector2(x * f, y * f);
	}
	Vector2 operator/= (double f)
	{
		return Vector2(x / f, y / f);
	}
	Vector2 operator*(const Vector2& v) const
	{
		return Vector2(x * v.x, y * v.y);
	}
	Vector2 operator*(double k) const
	{
		return Vector2(x * k, y * k);
	}
	Vector2 operator/(double k) const
	{
		return Vector2(x / k, y / k);
	}
	bool operator==(const Vector2& u) const
	{
		return (x == u.x && y == u.y);
	}
	Vector2 operator-(const Vector2& u) const
	{
		return Vector2(x - u.x, y - u.y);
	}
	Vector2 operator+(const Vector2& u) const
	{
		return Vector2(x + u.x, y + u.y);
	}
	Vector2 operator+(double k) const
	{
		return Vector2(x + k, y + k);
	}
	Vector2 operator-(double k) const
	{
		return Vector2(x - k, y - k);
	}
	double operator[](int i) const
	{
		if (i == 0)
			return x;
		return y;
	}
	double& operator[](int i)
	{
		if (i == 0)
			return x;
		return y;
	}
	friend std::ostream& operator<< (std::ostream& stream, const Vector2& u);
	inline Vector3 ToVector3(double yy) const
	{
		return Vector3(x, yy, y);
	}
	inline double Max() const
	{
		return Math::Max(x, y);
	}
	inline double Min() const
	{
		return Math::Min(x, y);
	}
};
inline std::ostream& operator<<(std::ostream& stream, const Vector2& u)
{
	stream << "(" << u.x << ", " << u.y << ");";
	return stream;
}
inline double Dot(const Vector2& u, const Vector2& v)
{
	return u.x * v.x + u.y * v.y;
}
inline double Magnitude(const Vector2& u)
{
	return sqrt(u.x * u.x + u.y * u.y);
}
inline double SquaredMagnitude(const Vector2& u)
{
	return u.x * u.x + u.y * u.y;
}
inline Vector2 Normalize(const Vector2& v)
{
	double kk = 1.0f / Magnitude(v);
	return v * kk;
}
inline Vector2 operator-(const Vector2& v)
{
	return Vector2(-v.x, -v.y);
}
inline bool operator>(const Vector2& u, const Vector2& v)
{
	return (u.x > v.x) && (u.y > v.y);
}
inline bool operator<(const Vector2& u, const Vector2& v)
{
	return (u.x < v.x) && (u.y < v.y);
}
inline bool operator>=(const Vector2& u, const Vector2& v)
{
	return (u.x >= v.x) && (u.y >= v.y);
}
inline bool operator<=(const Vector2& u, const Vector2& v)
{
	return (u.x <= v.x) && (u.y <= v.y);
}
inline Vector2 Abs(const Vector2& u)
{
	return Vector2(u[0] > 0.0 ? u[0] : -u[0], u[1] > 0.0 ? u[1] : -u[1]);
}

/* Matrix3 */
struct Matrix3
{
public:
	double r[9];

	Matrix3();
	Matrix3(const Vector3& a, const Vector3& b, const Vector3& c);
	Matrix3(double a00, double a01, double a02, double a10, double a11, double a12, double a20, double a21, double a22);
	double Determinant() const;
	double& operator() (int, int);
	double operator() (int, int) const;
};

inline Matrix3::Matrix3()
{
	for (int i = 0; i < 9; i++)
		r[i] = 0.0f;
}

inline Matrix3::Matrix3(const Vector3& a, const Vector3& b, const Vector3& c)
{
	r[0] = a[0];
	r[1] = a[1];
	r[2] = a[2];

	r[3] = b[0];
	r[4] = b[1];
	r[5] = b[2];

	r[6] = c[0];
	r[7] = c[1];
	r[8] = c[2];
}

inline Matrix3::Matrix3(double a00, double a01, double a02, double a10, double a11, double a12, double a20, double a21, double a22)
{
	r[0] = a00;
	r[1] = a01;
	r[2] = a02;

	r[3] = a10;
	r[4] = a11;
	r[5] = a12;

	r[6] = a20;
	r[7] = a21;
	r[8] = a22;
}

inline double Matrix3::Determinant() const
{
	return r[0] * r[4] * r[8] + r[1] * r[5] * r[6] + r[2] * r[3] * r[7] - r[2] * r[4] * r[6] - r[1] * r[3] * r[8] - r[0] * r[5] * r[7];
}

inline double& Matrix3::operator() (int i, int j)
{
	return r[i + j + j + j];
}

inline double Matrix3::operator() (int i, int j) const
{
	return r[i + j + j + j];
}


/* Matrix4 */
struct Matrix4
{
public:
	double r[16];

	Matrix4();
	Matrix4(const Matrix3& m3);
	double& operator() (int, int);
	double operator() (int, int) const;
	double Determinant() const;
};

inline Matrix4::Matrix4()
{
	for (int i = 0; i < 16; i++)
		r[i] = 0.0f;
}

/*!
\brief Create an homogeneous matrix from a simple Matrix.
The translation and shear coefficients are set to 0.0.
\param a Matrix.
*/
inline Matrix4::Matrix4(const Matrix3& a)
{
	// Rotation and scale
	r[0] = a.r[0];
	r[1] = a.r[1];
	r[2] = a.r[2];
	r[4] = a.r[3];
	r[5] = a.r[4];
	r[6] = a.r[5];
	r[8] = a.r[6];
	r[9] = a.r[7];
	r[10] = a.r[8];

	// Translation
	r[3] = r[7] = r[11] = 0.0;

	// Shear
	r[12] = r[13] = r[14] = 0.0;

	// Scale
	r[15] = 1.0;
}

inline double& Matrix4::operator() (int i, int j)
{
	return r[i + j + j + j];
}

inline double Matrix4::operator() (int i, int j) const
{
	return r[i + j + j + j];
}

inline double Matrix4::Determinant() const
{
	const Matrix4& M = *this;
	return M(0, 0) * Matrix3(M(1, 1), M(1, 2), M(1, 3), M(2, 1), M(2, 2), M(2, 3), M(3, 1), M(3, 2), M(3, 3)).Determinant()
		- M(1, 0) * Matrix3(M(0, 1), M(0, 2), M(0, 3), M(2, 1), M(2, 2), M(2, 3), M(3, 1), M(3, 2), M(3, 3)).Determinant()
		+ M(2, 0) * Matrix3(M(0, 1), M(0, 2), M(0, 3), M(1, 1), M(1, 2), M(1, 3), M(3, 1), M(3, 2), M(3, 3)).Determinant()
		- M(3, 0) * Matrix3(M(0, 1), M(0, 2), M(0, 3), M(1, 1), M(1, 2), M(1, 3), M(2, 1), M(2, 2), M(2, 3)).Determinant();
}
