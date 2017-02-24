#include "3dmath.h"

/** Take a pointer to a quaternion and normalize it (length == 1).
  *
  * @param *q:	Address to store the quaternion being normalized.
  *
  * @return Void.
***/
void quaternion_normalize(quaternion *q) {
	float magnitude = quaternion_get_magnitude(*q);

	q->w /= magnitude;
	q->x /= magnitude;
	q->y /= magnitude;
	q->z /= magnitude;
}

/** Find the magnitude (length) of a given quaternion.
  *
  * @param q:	The quaternion that is being measured.
  *
  * @return The magnitude of the quaternion.
***/
float quaternion_get_magnitude(quaternion q) {
	return sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
}

/** Find the product of multiplying one quaternion by another.
  *
  * @param q1:	Quaternion 1.
  * @param q2:	Quaternion 2.
  *
  * @return Result of quaternion multiplication.
***/
quaternion quaternion_get_product(quaternion q1, quaternion q2) {
	quaternion r;

	r.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
	r.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
	r.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
	r.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;

	return r;
}

/** Find the conjugate (inverse) of a quaternion.
  *
  * @param q:	The quaternion to find the conjugate of.
  *
  * @return The conjugate of the given quaternion.
***/
quaternion quaternion_get_conjugate(quaternion q) {
	quaternion r;

	r.w = q.w;
	r.x = -q.x;
	r.y = -q.y;
	r.z = -q.z;

	return r;
}

/** Resets a quaternion back to (1.0f, 0.0f, 0.0f, 0.0f).
  *
  * @param *q:	Address to the quaternion that will be zeroed.
  *
  * @return Void.
***/
void quaternion_zero(quaternion *q) {
	q->w = 1.0f;
	q->x = 0.0f;
	q->y = 0.0f;
	q->z = 0.0f;
}

/** Take a pointer to a vector and normalize it so that its length is 1.
  *
  * @param *v:	Address to the vector to be normalized.
  *
  * @return Void.
***/
void vector_normalize(vector *v) {
	float magnitude = vector_get_magnitude(*v);

	v->x /= magnitude;
	v->y /= magnitude;
	v->z /= magnitude;
}

/** Find the magnitude (length) of a given vector.
  *
  * @param v:	The vector to be measured.
  *
  * @return The value of the magnitude of the vector.
***/
float vector_get_magnitude(vector v) {
	return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

/** Resets a vector back to (0.0f, 0.0f, 0.0f).
  *
  * @param *v:	Address to the vector to be zeroed.
  *
  * @return Void.
***/
void vector_zero(vector *v) {
	v->x = 0.0f;
	v->y = 0.0f;
	v->z = 0.0f;
}

/** Rotate a vector by a given quaternion.
  *
  * @param v:	The original vector to be rotated.
  * @param q:	The quaternion representing the rotation to be done.
  *
  * @return The vector that has been rotated.
***/
vector vector_rotate(vector v, quaternion q) {
	vector u;
	vector result;

	float s = q.w;
	u.x = q.x;
	u.y = q.y;
	u.z = q.z;

	vector term_1 = vector_scale(2.0f * vector_dot_product(u, v), u);
	vector term_2 = vector_scale(s * s - vector_dot_product(u, u), v);
	vector term_3 = vector_scale(2.0f * s, vector_cross_product(u, v));

	result = vector_add(vector_add(term_1, term_2), term_3);

	return result;
}

/** Compute the dot product of two vectors.
  *
  * @param u:	Vector 1.
  * @param v:	Vector 2.
  *
  * @return The value of the vector dot product.
***/
float vector_dot_product(vector u, vector v) {
	return u.x * v.x + u.y * v.y + u.z * v.z;
}

/** Compute the cross product of two vectors.
  *
  * @param u:	Vector 1.
  * @param v:	Vector 2.
  *
  * @return The result of the vector cross product.
***/
vector vector_cross_product(vector u, vector v) {
	vector result;

	result.x = u.y * v.z - u.z * v.y;
	result.y = u.z * v.x - u.x * v.z;
	result.z = u.x * v.y - u.y * v.x;

	return result;
}

/** Scale a vector by a given scalar.
  *
  * @param scale:	The scalar to multiply the vector components by.
  * @param v:		The vector to be scaled.
  *
  * @return The scaled vector.
***/
vector vector_scale(float scale, vector v) {
	vector result = { scale * v.x, scale * v.y, scale * v.z };

	return result;
}

/** Compute the sum of two vectors.
  *
  * @param u:	Vector 1.
  * @param v:	Vector 2.
  *
  * @return the sum of two vectors.
***/
vector vector_add(vector u, vector v) {
	vector result = { u.x + v.x, u.y + v.y, u.z + v.z };

	return result;
}
