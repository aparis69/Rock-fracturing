#pragma once

#include <cmath>
#include <iostream>

/* Forward Declarations */
struct Vector2i;
struct Vector2;
struct Vector3;
struct Vector4;

// Maths utility
namespace Math
{
	/*!
	\brief
	*/
	inline float Clamp(float x, float a = 0.0f, float b = 1.0f)
	{
		return x < a ? a : x > b ? b : x;
	}

	/*!
	\brief Create a linear step.
	\param x Value
	\param a, b Interval values.
	\return Real in unit inverval.
	*/
	inline float Step(float x, float a, float b)
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
	inline T Lerp(T a, T b, float t)
	{
		return (a * (1.0f - t)) + (b * t);
	}

	inline float Abs(float a)
	{
		return a < 0 ? -a : a;
	}

	inline float CubicSmoothCompact(float x, float r)
	{
		return (x > r) ? 0.0f : (1.0f - x / r)*(1.0f - x / r)*(1.0f - x / r);
	}

	inline float CubicSigmoid(float x, float r, float t)
	{
		if (x > 0.0f)
		{
			if (x < r)
			{
				return x * (1.0f + x * ((3.0f*t - 2.0f*r) / (r*r) + x * (r - 2.0f*t) / (r*r*r)));
			}
			else
			{
				return t;
			}
		}
		else
		{
			if (x > -r)
			{
				// Use symmetric
				float y = -x;
				return  -(y * (1.0f + y * ((3.0f*t - 2.0f*r) / (r*r) + y * (r - 2.0f*t) / (r*r*r))));
			}
			else
			{
				return -t;
			}
		}
	}

	inline float CubicSmooth(float x)
	{
		return x * x * (3.0f - 2.0f * x);
	}

	inline float CubicSmooth(float x, float r)
	{
		return (1.0f - x / r) * (1.0f - x / r) * (1.0f - x / r);
	}

	inline float CubicSmoothStep(float x, float a, float b)
	{
		if (x < a)
			return 0.0f;
		else if (x > b)
			return 1.0f;
		else
			return 1.0f - CubicSmooth((x - a) * (x - a), (b - a) * (b - a));
	}

	inline float QuinticSmooth(float t)
	{
		return pow(t, 3.0f) * (t * (t * 6.0f - 15.0f) + 10.0f);
	}
}


/* Vector2i */
struct Vector2i
{
public:
	int x, y;

	explicit Vector2i() : x(0), y(0) {}
	explicit Vector2i(int n) : x(n), y(n) {}
	explicit Vector2i(int x, int y) : x(x), y(y) {}

	Vector2i operator-(const Vector2i& u) const
	{
		return Vector2i(x - u.x, y - u.y);
	}
	Vector2i operator+(const Vector2i& u) const
	{
		return Vector2i(x + u.x, y + u.y);
	}
};


/* Vector3 */
struct Vector3
{
public:
	float x, y, z;

	explicit Vector3() : x(0.0f), y(0.0f), z(0.0f) { }
	explicit Vector3(float n) : x(n), y(n), z(n) {}
	explicit Vector3(float x, float y, float z) : x(x), y(y), z(z) {}

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
	Vector3 operator*= (float f)
	{
		return Vector3(x * f, y * f, z * f);
	}
	Vector3 operator/= (float f)
	{
		return Vector3(x / f, y / f, z / f);
	}
	Vector3 operator*(const Vector3& u) const
	{
		return Vector3(x * u.x, y * u.y, z * u.z);
	}
	Vector3 operator*(float k) const
	{
		return Vector3(x * k, y * k, z * k);
	}
	Vector3 operator/(float k) const
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
	Vector3 operator+(float k) const
	{
		return Vector3(x + k, y + k, z + k);
	}
	float operator[](int i) const
	{
		if (i == 0)
			return x;
		else if (i == 1)
			return y;
		return z;
	}
	float& operator[](int i)
	{
		if (i == 0)
			return x;
		else if (i == 1)
			return y;
		return z;
	}
	friend std::ostream& operator<<(std::ostream& stream, const Vector3& u);
	inline float Max() const
	{
		return Math::Max(Math::Max(x, y), z);
	}
	inline float Min() const
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
inline float Dot(const Vector3& u, const Vector3& v)
{
	return u.x * v.x + u.y * v.y + u.z * v.z;
}
inline float Magnitude(const Vector3& u)
{
	return sqrt(u.x * u.x + u.y * u.y + u.z * u.z);
}
inline float SquaredMagnitude(const Vector3& u)
{
	return u.x * u.x + u.y * u.y + u.z * u.z;
}
inline Vector3 Normalize(const Vector3& v)
{
	float kk = 1.0f / Magnitude(v);
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


/* Vector2 */
struct Vector2
{
public:
	float x, y;

	explicit Vector2() : x(0.0f), y(0.0f) { }
	explicit Vector2(const Vector2i& v) : x(float(v.x)), y(float(v.y)) { }
	explicit Vector2(float n) : x(n), y(n) { }
	explicit Vector2(float x, float y) : x(x), y(y) { }
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
	Vector2 operator*= (float f)
	{
		return Vector2(x * f, y * f);
	}
	Vector2 operator/= (float f)
	{
		return Vector2(x / f, y / f);
	}
	Vector2 operator*(const Vector2& v) const
	{
		return Vector2(x * v.x, y * v.y);
	}
	Vector2 operator*(float k) const
	{
		return Vector2(x * k, y * k);
	}
	Vector2 operator/(float k) const
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
	Vector2 operator+(float k) const
	{
		return Vector2(x + k, y + k);
	}
	Vector2 operator-(float k) const
	{
		return Vector2(x - k, y - k);
	}
	float operator[](int i) const
	{
		if (i == 0)
			return x;
		return y;
	}
	float& operator[](int i)
	{
		if (i == 0)
			return x;
		return y;
	}
	friend std::ostream& operator<< (std::ostream& stream, const Vector2& u);
	inline Vector3 ToVector3(float yy) const
	{
		return Vector3(x, yy, y);
	}
	inline float Max() const
	{
		return Math::Max(x, y);
	}
	inline float Min() const
	{
		return Math::Min(x, y);
	}
};
inline std::ostream& operator<<(std::ostream& stream, const Vector2& u)
{
	stream << "(" << u.x << ", " << u.y << ");";
	return stream;
}
inline float Dot(const Vector2& u, const Vector2& v)
{
	return u.x * v.x + u.y * v.y;
}
inline float Magnitude(const Vector2& u)
{
	return sqrt(u.x * u.x + u.y * u.y);
}
inline float SquaredMagnitude(const Vector2& u)
{
	return u.x * u.x + u.y * u.y;
}
inline Vector2 Normalize(const Vector2& v)
{
	float kk = 1.0f / Magnitude(v);
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
