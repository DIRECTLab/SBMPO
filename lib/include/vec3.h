/**
* @file vec3.h
*
* @brief Header file to vec3: Defining vector to set position values
*
**/

#ifndef VEC3_HPP_
#define VEC3_HPP_

/// 3d vector class
#include <cmath>
typedef double Real;

/// @brief defining vector to set position values
///
/// @class vec3
class vec3 {
private:
	Real data[3];
public:
	vec3();
	vec3(Real x, Real y, Real z);

	vec3& operator+= (const vec3& v);
	vec3& operator-= (const vec3& v);

	vec3& operator*= (Real s);
	vec3& operator/= (Real s);

	operator Real* ();
	operator const Real* () const;
};

/// initialization to (0,0,0)
inline vec3::vec3() {
	for (int i = 0; i < 3; ++i)
		data[i] = 0;
}

/// initialization to (x,y,z)
inline vec3::vec3(Real x, Real y, Real z) {
	data[0] = x;
	data[1] = y;
	data[2] = z;
}

/// Vector arithmetic operators
inline vec3& vec3::operator+= (const vec3& v) {
	for ( int i = 0; i < 3; ++i )
		data[i] += v.data[i];

	return *this;
}

inline vec3& vec3::operator-= (const vec3& v) {
	for ( int i = 0; i < 3; ++i )
		data[i] -= v.data[i];

	return *this;
}

inline vec3& vec3::operator*= (Real s) {
	for ( int i = 0; i < 3; ++i )
		data[i] *= s;

	return *this;
}

inline vec3& vec3::operator/= (Real s) {
	for ( int i = 0; i < 3; ++i )
		data[i] /= s;

	return *this;
}

/// Conversion operator for subscripting.
/// You can write v[2] = value; or value = v[2].
inline vec3::operator Real* () {
	return data;
}

/// Conversion operator for subscripting (const version).
/// You can write value = v[2].
inline vec3::operator const Real* () const {
	return data;
}

/******************************************************************************/
// non-member operators

/// @relates vec3
/// Vector addition
inline vec3 operator+(const vec3& a, const vec3& b) {
	vec3 r = a;
	return r += b;
}

/// @relates vec3
/// Vector subtraction
inline vec3 operator-(const vec3& a, const vec3& b) {
	vec3 r = a;
	return r -= b;
}

/// @relates vec3
/// Vector multiplication by a scalar
inline vec3 operator*(Real s, const vec3& v) {
	vec3 r = v;
	return r *= s;
}

/// @relates vec3
/// Divides each vector component by a scalar value
inline vec3 operator/(const vec3& v, Real s) {
	vec3 r = v;
	return r /= s;
}

/// @relates vec3
/// Unary minus
inline vec3 operator-(const vec3& v) {
	vec3 r = v;
	for (int i = 0; i < 3; ++i)
		r[i] = -r[i];

	return r;
}

/// @relates vec3
/// Comparison operators
inline bool operator==(const vec3& a, const vec3& b) {
	for (int i = 0; i < 3; ++i) {
#ifndef USE_FEQUAL_COMPARE
		if (a[i] != b[i]) return false;
#else
		if (!fequal(a[i], b[i])) return false;
#endif
	}

	return true;
}

/// @relates vec3
/// Vector comparison
inline bool operator!=(const vec3& a, const vec3& b) {
	return !operator==(a, b);
}

/// @relates vec3
/// Vector dot product
inline Real operator*(const vec3& a, const vec3& b) {
	Real dotprod = 0.0;
	for (int i = 0; i < 3; ++i)
		dotprod += a[i] * b[i];

	return dotprod;
}

/// @relates vec3
/// Vector dot product
inline Real dot(const vec3& a, const vec3& b) {
	return a * b;
}

/// @relates vec3
/// Returns the length^2
inline Real length2(const vec3& v) {
	Real l = 0.0;
	for (int i = 0; i < 3; ++i)
		l += v[i] * v[i];

	return l;
}

/// @relates vec3
/// Returns the vector length
inline Real length(const vec3& v) {
	return sqrt(length2(v));
}

/// @relates vec3
/// Vector cross product only for 3d vectors
inline vec3 cross(const vec3& a, const vec3& b) {
	vec3 r;
	r[0] = a[1] * b[2] - b[1] * a[2];
	r[1] = a[2] * b[0] - b[2] * a[0];
	r[2] = a[0] * b[1] - b[0] * a[1];
	return r;
}

/// @relates vec3
/// Returns the vector scaled to unit length
inline vec3 normalized(const vec3& v) {
	Real l = length(v);

	if (l != 0.0) {
		vec3 result(v[0] / l, v[1] / l, v[2] / l);
		return result;
	}

	return v;
}
#endif /* VEC3_HPP_ */
