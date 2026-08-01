#include "wb_util.h"
#include "wb_bot.h"
#include "wb_map.h"
#include "wb_nav.h"

#include <chrono>
#include <float.h>
#include <stdarg.h>
#include <algorithm>
#include <limits.h>
#include <cmath>

using namespace std;
using namespace std::chrono;
using namespace wbot;

char* VarArgs(const char* format, ...) {
	va_list		argptr;
	static char		string[1024];

	va_start(argptr, format);
	vsnprintf(string, 1024, format, argptr);
	va_end(argptr);

	return string;
}

player_t* getAnyPlayer() {
	for (int i = 0; i < MAXPLAYERS; i++) {
		player_t* plr = get_player_for_index(i);
		CWootBot* bot = get_bot_for_index(i);

		if (plr && !bot)
			return plr;
	}

	return NULL;
}

bool onSegment(const vec2& p, const vec2& q, const vec2& r) {
	return (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) &&
		    q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y));
}

int orientation(const vec2& p, const vec2& q, const vec2& r) {
	float val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);

	if (val == 0.0)
		return 0;  // Collinear

	return (val > 0.0) ? 1 : 2;  // Clockwise or counterclockwise
}

bool DoLinesIntersect(const vec2& la1, const vec2& la2, const vec2& lb1, const vec2& lb2) {
	const vec2& A = la1;
	const vec2& B = la2;
	const vec2& C = lb1;
	const vec2& D = lb2;

	int o1 = orientation(A, B, C);
	int o2 = orientation(A, B, D);
	int o3 = orientation(C, D, A);
	int o4 = orientation(C, D, B);

	if (o1 != o2 && o3 != o4)
		return true;  // They intersect

	if (o1 == 0 && onSegment(A, C, B)) return true;
	if (o2 == 0 && onSegment(A, D, B)) return true;
	if (o3 == 0 && onSegment(C, A, D)) return true;
	if (o4 == 0 && onSegment(C, B, D)) return true;

	return false;  // Doesn't intersect
}

vec2 LineIntersect(const vec2& la1, const vec2& la2, const vec2& lb1, const vec2& lb2) {
	const vec2& A = la1;
	const vec2& B = la2;
	const vec2& C = lb1;
	const vec2& D = lb2;

	float a1 = B.y - A.y;
	float b1 = A.x - B.x;
	float c1 = a1 * A.x + b1 * A.y;

	float a2 = D.y - C.y;
	float b2 = C.x - D.x;
	float c2 = a2 * C.x + b2 * C.y;

	float determinant = a1 * b2 - a2 * b1;

	if (determinant == 0.0) {
		return vec2(0, 0);
	}

	float x = (b2 * c1 - b1 * c2) / determinant;
	float y = (a1 * c2 - a2 * c1) / determinant;
	return vec2(x, y);
}

bool LineIntersectsZRange(const vec3& a, const vec3& b, float zMin, float zMax, vec3& enter, vec3& exit) {
	float dz = b.z - a.z;

	// Horizontal line
	const float eps = 0.01f;
	if (dz < eps) {
		if (a.z >= zMin && a.z <= zMax) {
			enter = a;
			exit = b;
			return true;
		}
		return false;
	}

	float t0 = (zMin - a.z) / dz;
	float t1 = (zMax - a.z) / dz;

	if (t0 > t1)
		std::swap(t0, t1);

	t0 = std::max(0.0f, t0);
	t1 = std::min(1.0f, t1);

	if (t0 > t1)
		return false;

	enter = a + (b - a) * t0;
	exit = a + (b - a) * t1;
	return true;
}

vec2 ClosestPointOnSegment(const vec2& p, const vec2& a, const vec2& b) {
	vec2 ab = b - a;
	float lenSq = dotProduct(ab, ab);

	if (lenSq <= 0.0f)
		return a; // Segment is a point

	float t = dotProduct(p - a, ab) / lenSq;
	
	if (t < 0.0f) t = 0.0f;
	else if (t > 1.0f) t = 1.0f;

	return a + ab * t;
}

FSegment2 LineSegmentOverlap(const vec2& a1, const vec2& a2, const vec2& b1, const vec2& b2)
{
	FSegment2 result;

	const float eps = 2.0f; // TODO: why are clipped polygon edges this bad?

	// colinear tests
	if (fabs(DistanceToLine(a1, b1, b2)) >= eps || fabs(DistanceToLine(a2, b1, b2)) >= eps) {
		return result;
	}
	if (fabs(DistanceToLine(b1, a1, a2)) >= eps || fabs(DistanceToLine(b2, a1, a2)) >= eps) {
		return result;
	}

	if (fabs(a2.x - a1.x) >= fabs(a2.y - a1.y))
	{
		bool aForward = a1.x < a2.x;
		bool bForward = b1.x < b2.x;

		float start = std::max(std::min(a1.x, a2.x), std::min(b1.x, b2.x));
		float end = std::min(std::max(a1.x, a2.x), std::max(b1.x, b2.x));

		if (end - start < eps)
			return result;

		float dx = a2.x - a1.x;
		float dy = a2.y - a1.y;

		float t1 = (start - a1.x) / dx;
		float t2 = (end - a1.x) / dx;

		result.a = vec2(start, a1.y + t1 * dy);
		result.b = vec2(end, a1.y + t2 * dy);
	}
	else
	{
		float start = std::max(std::min(a1.y, a2.y), std::min(b1.y, b2.y));
		float end = std::min(std::max(a1.y, a2.y), std::max(b1.y, b2.y));

		if (end - start < eps)
			return result;

		float dx = a2.x - a1.x;
		float dy = a2.y - a1.y;

		float t1 = (start - a1.y) / dy;
		float t2 = (end - a1.y) / dy;

		result.a = vec2(a1.x + t1 * dx, start);
		result.b = vec2(a1.x + t2 * dx, end);
	}

	return result;
}

float DistanceToLine(const vec2& p, const vec2& a, const vec2& b) {
	vec2 ab = b - a;
	vec2 ap = p - a;
    return crossProduct(ab, ap) / ab.length();
}

float DistanceToLine(const vec2& p, wbot::MapLine* line) {
	return DistanceToLine(p, line->v1, line->v2);
}

float BoxRadiusForDir(const vec2& dir, float radius)
{
	float tx = (dir.x != 0) ? radius / fabs(dir.x) : FLT_MAX;
	float ty = (dir.y != 0) ? radius / fabs(dir.y) : FLT_MAX;
	return std::min(tx, ty);
}

bool IsBoxWallClipped(const vec2& pos, float radius) {
	for (MapLine* ld : get_crossed_lines(pos, radius)) {
		if (ld->isImpassable())
			return true;  // crosses an impassable line
	}

	return false;
}

vec2 GetFloorPosition(const vec3& pos, float radius) {
	float bestDist = FLT_MAX;

	vec2 bestFloorPoint = pos;

	for (MapLine* ld : get_crossed_lines(pos, radius)) {
		if (ld->isImpassable())
			return pos; // crosses an impassable line. stuck.

		float frontFloor = ld->frontsector->getFloorZ();
		float backFloor = ld->backsector->getFloorZ();

		if (pos.z == backFloor || pos.z == frontFloor) {			
			// find a position just beyond this line
			vec2 moveDir = pos.z == backFloor ? ld->normal() * -1 : ld->normal();
			vec2 floorPoint = ld->v1 + moveDir;
			float dist = fabs(DistanceToLine(pos, floorPoint, floorPoint + ld->dir()));

			if (dist < bestDist) {
				bestDist = dist;
				bestFloorPoint = pos + moveDir * dist;
			}
		}
	}

	return bestFloorPoint;
}

bool IsBoxClipped(const vec3& pos, float radius, float height) {
	float z = pos.z;
	float topZ = z + height;

	for (MapLine* ld : get_crossed_lines(pos, radius)) {
		if (ld->isImpassable())
			return true; // crosses an impassable line
		
		float frontCeil = ld->frontsector->getCeilZ();
		float frontFloor = ld->frontsector->getFloorZ();
		float backCeil = ld->backsector->getCeilZ();
		float backFloor = ld->backsector->getFloorZ();

		if (z < backFloor || z < frontFloor)
			return true; // clipped into the floor

		if (topZ > frontCeil || topZ > backCeil)
			return true; // clipped into the ceiling
	}

	return false;
}

std::vector<MapSector*> GetBoxClipSectors(const vec3& pos, float radius, float height) {
	std::vector<MapSector*> clipSectors;

	float z = pos.z;
	float topZ = z + height;

	for (MapLine* ld : get_crossed_lines(pos, radius)) {
		if (ld->isImpassable())
			continue; // crosses an impassable line, no sector to trigger

		float frontCeil = ld->frontsector->getCeilZ();
		float frontFloor = ld->frontsector->getFloorZ();
		float backCeil = ld->backsector->getCeilZ();
		float backFloor = ld->backsector->getFloorZ();

		if (z < backFloor || topZ > backCeil)
			clipSectors.push_back(ld->backsector);

		if (z < frontFloor || topZ > frontCeil)
			clipSectors.push_back(ld->frontsector);
	}

	return clipSectors;
}

void ClipPoly(std::vector<vec2>& poly, vec2 lp, vec2 ldir, bool front) {
	std::vector<vec2> out;

	for (int i = 0; i < poly.size(); i++)
	{
		const vec2& A = poly[i];
		const vec2& B = poly[(i + 1) % poly.size()];

		float da = ((A.y - (float)lp.y) * ldir.x + ((float)lp.x - A.x) * ldir.y);
		float db = ((B.y - (float)lp.y) * ldir.x + ((float)lp.x - B.x) * ldir.y);

		bool insideA = front ? (da >= 0) : (da <= 0);
		bool insideB = front ? (db >= 0) : (db <= 0);

		if (insideA && insideB) {
			// keep endpoint
			out.push_back(B);
		}
		else if (insideA && !insideB) {
			// leaving half-plane
			float t = (float)da / (float)(da - db);
			out.push_back(A + (B - A) * t);
		}
		else if (!insideA && insideB) {
			// entering half-plane
			float t = (float)da / (float)(da - db);
			out.push_back(A + (B - A) * t);
			out.push_back(B);
		}
		// else: both outside, keep nothing
	}

	poly.clear();

	for (int i = 0; i < out.size(); i++) {
		vec2 v = out[i];
		vec2 vNext = out[(i+1) % out.size()];

		if ((vNext - v).length() > 0.1f) {
			poly.push_back(vNext);
		}
	}
}

void ExtendSegment(vec2& a, vec2& b, float amount) {
	vec2 dir = b - a;

	if (dir.lengthSquared() == 0)
		return;

	dir = dir.normalize();

	a -= dir * amount;
	b += dir * amount;
}

bool PointAlignedSegment(const vec2& p, const vec2& a, const vec2& b) {
	vec2 delta = b - a;
	float len2 = delta.lengthSquared();

	if (len2 == 0)
		return false;

	float t = dotProduct(p - a, delta) / len2;
	return t >= 0 && t <= 1;
}

bool CircleIntersectsSegment(const vec2& center, float radius, const vec2& a, const vec2& b) {
	vec2 ab = b - a;
	float len2 = dotProduct(ab, ab);
	float r2 = radius * radius;

	if (len2 == 0.0f)
		return (center - a).lengthSquared() <= r2;

	float t = dotProduct(center - a, ab) / len2;
	if (t < 0) t = 0;
	if (t > 1) t = 1;

	vec2 closest = a + ab * t;
	return (center - closest).lengthSquared() <= r2;
}

uint64_t getEpochMillis() {
	return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

int draw_debug_line(vec3 start, vec3 end, AActor* actor) {
	// this sucks and the railgun effect crashes the client so can't use that

	vec3 delta = end - start;
	float len = std::min(delta.length(), 4096.0f);
	uint32_t ilen = (uint32_t)len;
	int spacing = 4;

	if (ilen > 1600)
		spacing = 64;
	else if (ilen > 800)
		spacing = 32;
	else if (ilen > 400)
		spacing = 16;
	else if (ilen > 200)
		spacing = 8;

	vec3 dir = delta.normalize(spacing);
	int spawns = len / spacing;
	int i = 0;
	for (i = 0; i < spawns; i++) {
		vec3 pos = start + (dir * i);
		if (i == spawns - 1) {
			pos = end;
		}
		SpawnBlood(pos, 1, actor);
	}

	return i;
}

bool TraceRadius(vec3 start, vec3 end, float radius, bool ignoreMonsters, AActor* ignoreEnt, TraceResult* tr) {
	vec2 dir = (end - start).normalize();
	vec3 rightDir(dir.y, -dir.x, 0);
	float minFrac = 1.0f;
	int rightStep = radius / 2;

	static TraceResult dummy;
	TraceResult* out = tr ? tr : &dummy;

	for (int i = -2; i <= 2; i++) {
		TraceResult temp;
		TraceLine(start + rightDir * i * rightStep, end + rightDir * i * rightStep, ignoreMonsters, ignoreEnt, &temp);
		
		if (temp.frac < minFrac) {
			minFrac = temp.frac;
			*out = temp;
		}
	}

	return minFrac < 1.0f;
}

