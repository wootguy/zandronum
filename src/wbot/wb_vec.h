#pragma once
#include <string>

// everybody's got their own vector class... here's mine!!!
// not using zandronum's for ease of porting wbots to other engines

struct vec3;

struct vec2
{
	float x, y;
	vec2() : x(), y() {}
	vec2(float x, float y) : x(x), y(y) {}
	vec2(const vec3& v);
	vec2 normalize(float length = 1.0f);
	float length();
	float lengthSquared(); // faster than length

	void operator-=(vec2 v);
	void operator+=(vec2 v);
	void operator*=(vec2 v);
	void operator/=(vec2 v);

	void operator-=(float f);
	void operator+=(float f);
	void operator*=(float f);
	void operator/=(float f);
};

vec2 operator-(vec2 v1, vec2 v2);
vec2 operator+(vec2 v1, vec2 v2);
vec2 operator*(vec2 v1, vec2 v2);
vec2 operator/(vec2 v1, vec2 v2);

vec2 operator+(vec2 v, float f);
vec2 operator-(vec2 v, float f);
vec2 operator*(vec2 v, float f);
vec2 operator/(vec2 v, float f);

bool operator==(vec2 v1, vec2 v2);
bool operator!=(vec2 v1, vec2 v2);

float dotProduct(vec2 v1, vec2 v2);
float crossProduct(vec2 v1, vec2 v2);

struct vec3
{
	float x, y, z;
	vec3() : x(), y(), z() {}
	vec3( float x, float y, float z ) : x( x ), y( y ), z( z ) {}
	vec3(vec2 v, float z) : x(v.x), y(v.y), z(z) {}
	vec3 normalize(float length=1.0f) const;
	float length();
	float lengthSquared(); // faster than length
	vec3 invert();
	vec2 xy() { return vec2(x, y); }

	void operator-=(vec3 v);
	void operator+=(vec3 v);
	void operator*=(vec3 v);
	void operator/=(vec3 v);

	void operator-=(float f);
	void operator+=(float f);
	void operator*=(float f);
	void operator/=(float f);
};

vec3 operator-(vec3 v1, vec3 v2);
vec3 operator+(vec3 v1, vec3 v2);
vec3 operator*(vec3 v1, vec3 v2);
vec3 operator/(vec3 v1, vec3 v2);

vec3 operator+(vec3 v, float f);
vec3 operator-(vec3 v, float f);
vec3 operator*(vec3 v, float f);
vec3 operator/(vec3 v, float f);

vec3 crossProduct(vec3 v1, vec3 v2);
float dotProduct(vec3 v1, vec3 v2);

bool operator==(vec3 v1, vec3 v2);
bool operator!=(vec3 v1, vec3 v2);

struct vec4
{
	float x, y, z, w;

	vec4() : x(0), y(0), z(0), w(0) {}
	vec4(float x, float y, float z) : x(x), y(y), z(z), w(1) {}
	vec4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
	vec4(vec3 v, float a) : x(v.x), y(v.y), z(v.z), w(a) {}
	vec3 xyz();
	vec2 xy();
};

vec4 operator-(vec4 v1, vec4 v2);
vec4 operator+(vec4 v1, vec4 v2);
vec4 operator*(vec4 v1, vec4 v2);
vec4 operator/(vec4 v1, vec4 v2);

vec4 operator+(vec4 v, float f);
vec4 operator-(vec4 v, float f);
vec4 operator*(vec4 v, float f);
vec4 operator/(vec4 v, float f);

bool operator==(vec4 v1, vec4 v2);
bool operator!=(vec4 v1, vec4 v2);