#include "wb_util.h"
#include "wb_bot.h"
#include "wb_map.h"
#include "wb_nav.h"

#include <chrono>
#include <float.h>
#include <stdarg.h>
#include <algorithm>
#include <limits.h>

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

		if (plr)
			return plr;
	}

	return NULL;
}

bool onSegment(const FVector2& p, const FVector2& q, const FVector2& r) {
	return (q.X <= max(p.X, r.X) && q.X >= min(p.X, r.X) &&
		    q.Y <= max(p.Y, r.Y) && q.Y >= min(p.Y, r.Y));
}

int orientation(const FVector2& p, const FVector2& q, const FVector2& r) {
	float val = (q.Y - p.Y) * (r.X - q.X) - (q.X - p.X) * (r.Y - q.Y);

	if (val == 0.0)
		return 0;  // Collinear

	return (val > 0.0) ? 1 : 2;  // Clockwise or counterclockwise
}

bool DoLinesIntersect(const FVector2& la1, const FVector2& la2, const FVector2& lb1, const FVector2& lb2) {
	const FVector2& A = la1;
	const FVector2& B = la2;
	const FVector2& C = lb1;
	const FVector2& D = lb2;

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

FVector2 LineIntersect(const FVector2& la1, const FVector2& la2, const FVector2& lb1, const FVector2& lb2) {
	const FVector2& A = la1;
	const FVector2& B = la2;
	const FVector2& C = lb1;
	const FVector2& D = lb2;

	float a1 = B.Y - A.Y;
	float b1 = A.X - B.X;
	float c1 = a1 * A.X + b1 * A.Y;

	float a2 = D.Y - C.Y;
	float b2 = C.X - D.X;
	float c2 = a2 * C.X + b2 * C.Y;

	float determinant = a1 * b2 - a2 * b1;

	if (determinant == 0.0) {
		return FVector2(0, 0);
	}

	float x = (b2 * c1 - b1 * c2) / determinant;
	float y = (a1 * c2 - a2 * c1) / determinant;
	return FVector2(x, y);
}

bool LineIntersectsZRange(const FVector3& a, const FVector3& b, float zMin, float zMax, FVector3& enter, FVector3& exit) {
	float dz = b.Z - a.Z;

	// Horizontal line
	const fixed_t eps = 1 << (FRACBITS / 2);
	if (dz < eps) {
		if (a.Z >= zMin && a.Z <= zMax) {
			enter = a;
			exit = b;
			return true;
		}
		return false;
	}

	float t0 = (zMin - a.Z) / dz;
	float t1 = (zMax - a.Z) / dz;

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

FVector2 ClosestPointOnSegment(const FVector2& p, const FVector2& a, const FVector2& b) {
	FVector2 ab = b - a;
	float lenSq = DotProduct(ab, ab);

	if (lenSq <= 0.0f)
		return a; // Segment is a point

	float t = DotProduct(p - a, ab) / lenSq;
	
	if (t < 0.0f) t = 0.0f;
	else if (t > 1.0f) t = 1.0f;

	return a + ab * t;
}

FSegment2 LineSegmentOverlap(const FVector2& a1, const FVector2& a2, const FVector2& b1, const FVector2& b2)
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

	if (fabs(a2.X - a1.X) >= fabs(a2.Y - a1.Y))
	{
		bool aForward = a1.X < a2.X;
		bool bForward = b1.X < b2.X;

		float start = std::max(std::min(a1.X, a2.X), std::min(b1.X, b2.X));
		float end = std::min(std::max(a1.X, a2.X), std::max(b1.X, b2.X));

		if (end - start < eps)
			return result;

		float dx = a2.X - a1.X;
		float dy = a2.Y - a1.Y;

		float t1 = (start - a1.X) / dx;
		float t2 = (end - a1.X) / dx;

		result.a = FVector2(start, a1.Y + t1 * dy);
		result.b = FVector2(end, a1.Y + t2 * dy);
	}
	else
	{
		float start = std::max(std::min(a1.Y, a2.Y), std::min(b1.Y, b2.Y));
		float end = std::min(std::max(a1.Y, a2.Y), std::max(b1.Y, b2.Y));

		if (end - start < eps)
			return result;

		float dx = a2.X - a1.X;
		float dy = a2.Y - a1.Y;

		float t1 = (start - a1.Y) / dy;
		float t2 = (end - a1.Y) / dy;

		result.a = FVector2(a1.X + t1 * dx, start);
		result.b = FVector2(a1.X + t2 * dx, end);
	}

	return result;
}

float DistanceToLine(const FVector2& p, const FVector2& a, const FVector2& b) {
	FVector2 ab = b - a;
    FVector2 ap = p - a;
    return CrossProduct(ab, ap) / ab.Length();
}

float DistanceToLine(const FVector2& p, wbot::MapLine* line) {
	return DistanceToLine(p, line->start(), line->end());
}

fixed_t BoxRadiusForDir(const FVector2& dir, fixed_t radius)
{
	float tx = (dir.X != 0) ? radius / fabs(dir.X) : FLT_MAX;
	float ty = (dir.Y != 0) ? radius / fabs(dir.Y) : FLT_MAX;
	return std::min(tx, ty);
}

bool IsBoxWallClipped(const FVector2& pos, fixed_t radius) {
	for (MapLine* ld : get_crossed_lines(pos, radius)) {
		if (ld->isImpassable())
			return true;  // crosses an impassable line
	}

	return false;
}

FVector2 GetFloorPosition(const FVector3& pos, fixed_t radius) {
	fixed_t bestDist = INT_MAX;

	FVector2 bestFloorPoint = pos;

	for (MapLine* ld : get_crossed_lines(pos, radius)) {
		if (ld->isImpassable())
			return pos; // crosses an impassable line. stuck.

		fixed_t frontFloor = ld->frontsector->getFloorZ();
		fixed_t backFloor = ld->backsector->getFloorZ();

		if (pos.Z == backFloor || pos.Z == frontFloor) {			
			// find a position just beyond this line
			FVector2 moveDir = pos.Z == backFloor ? ld->normal() * -1 : ld->normal();
			FVector2 floorPoint = ld->start() + moveDir * FRACUNIT;
			fixed_t dist = fabs(DistanceToLine(pos, floorPoint, floorPoint + ld->dir() * FRACUNIT));

			if (dist < bestDist) {
				fixed_t asd = fabs(DistanceToLine(pos, floorPoint, floorPoint + ld->dir() * FRACUNIT));
				bestDist = dist;
				bestFloorPoint = pos + moveDir * dist;
			}
		}
	}

	return bestFloorPoint;
}

bool IsBoxClipped(const FVector3& pos, fixed_t radius, fixed_t height) {
	fixed_t x = pos.X;
	fixed_t y = pos.Y;
	fixed_t z = pos.Z;
	fixed_t topZ = z + height;

	for (MapLine* ld : get_crossed_lines(pos, radius)) {
		if (ld->isImpassable())
			return true; // crosses an impassable line
		
		fixed_t frontCeil = ld->frontsector->getCeilZ();
		fixed_t frontFloor = ld->frontsector->getFloorZ();
		fixed_t backCeil = ld->backsector->getCeilZ();
		fixed_t backFloor = ld->backsector->getFloorZ();

		if (z < backFloor || z < frontFloor)
			return true; // clipped into the floor

		if (topZ > frontCeil || topZ > backCeil)
			return true; // clipped into the ceiling
	}

	return false;
}

std::vector<MapSector*> GetBoxClipSectors(const FVector3& pos, fixed_t radius, fixed_t height) {
	std::vector<MapSector*> clipSectors;

	fixed_t x = pos.X;
	fixed_t y = pos.Y;
	fixed_t z = pos.Z;
	fixed_t topZ = z + height;

	for (MapLine* ld : get_crossed_lines(pos, radius)) {
		if (ld->isImpassable())
			continue; // crosses an impassable line, no sector to trigger

		fixed_t frontCeil = ld->frontsector->getCeilZ();
		fixed_t frontFloor = ld->frontsector->getFloorZ();
		fixed_t backCeil = ld->backsector->getCeilZ();
		fixed_t backFloor = ld->backsector->getFloorZ();

		if (z < backFloor || topZ > backCeil)
			clipSectors.push_back(ld->backsector);

		if (z < frontFloor || topZ > frontCeil)
			clipSectors.push_back(ld->frontsector);
	}

	return clipSectors;
}

void ClipPoly(std::vector<FVector2>& poly, FVector2 lp, FVector2 ldir, bool front) {
	std::vector<FVector2> out;

	for (int i = 0; i < poly.size(); i++)
	{
		const FVector2& A = poly[i];
		const FVector2& B = poly[(i + 1) % poly.size()];

		float da = ((A.Y - (float)lp.Y) * ldir.X + ((float)lp.X - A.X) * ldir.Y);
		float db = ((B.Y - (float)lp.Y) * ldir.X + ((float)lp.X - B.X) * ldir.Y);

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
		FVector2 v = out[i];
		FVector2 vNext = out[(i+1) % out.size()];

		if ((vNext - v).Length() > 0.1f) {
			poly.push_back(vNext);
		}
	}
}

void ExtendSegment(FVector2& a, FVector2& b, float amount) {
	FVector2 dir = b - a;

	if (dir.LengthSquared() == 0)
		return;

	dir.MakeUnit();

	a -= dir * amount;
	b += dir * amount;
}

bool PointAlignedSegment(const FVector2& p, const FVector2& a, const FVector2& b) {
	FVector2 delta = b - a;
	float len2 = delta.LengthSquared();

	if (len2 == 0)
		return false;

	float t = DotProduct(p - a, delta) / len2;
	return t >= 0 && t <= 1;
}

bool CircleIntersectsSegment(const FVector2& center, float radius, const FVector2& a, const FVector2& b) {
	FVector2 ab = b - a;
	float len2 = DotProduct(ab, ab);
	float r2 = radius * radius;

	if (len2 == 0.0f)
		return (center - a).LengthSquared() <= r2;

	float t = DotProduct(center - a, ab) / len2;
	if (t < 0) t = 0;
	if (t > 1) t = 1;

	FVector2 closest = a + ab * t;
	return (center - closest).LengthSquared() <= r2;
}

float DotProduct(const FVector2& a, const FVector2& b)
{
	return a.X * b.X + a.Y * b.Y;
}

float CrossProduct(const FVector2& a, const FVector2& b) {
	return a.X * b.Y - a.Y * b.X;
}

uint64_t getEpochMillis() {
	return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

int draw_debug_line(FVector3 start, FVector3 end, AActor* actor) {
	// this sucks and the railgun effect crashes the client so can't use that

	FVector3 delta = end - start;
	float len = std::min(delta.Length(), (double)(4096 << FRACBITS));
	uint32_t ilen = (uint32_t)len >> FRACBITS;
	int spacing = 4;

	if (ilen > 1600)
		spacing = 64;
	else if (ilen > 800)
		spacing = 32;
	else if (ilen > 400)
		spacing = 16;
	else if (ilen > 200)
		spacing = 8;

	FVector3 dir = delta.Resize(spacing << FRACBITS);
	int spawns = len / (spacing << FRACBITS);
	int i = 0;
	for (i = 0; i < spawns; i++) {
		FVector3 pos = start + (dir * i);
		if (i == spawns - 1) {
			pos = end;
		}
		SpawnBlood(pos, 1, actor);
	}

	return i;
}

bool TraceRadius(FVector3 start, FVector3 end, fixed_t radius, bool ignoreMonsters, AActor* ignoreEnt, TraceResult* tr) {
	FVector2 dir = (end - start).Unit() * FRACUNIT;
	FVector3 rightDir(dir.Y, -dir.X, 0);
	fixed_t minFrac = 1.0f;
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

std::vector<int> debugv2(const FVector2& v) {
	return {
		((int)v.X) >> FRACBITS, ((int)v.Y) >> FRACBITS
	};
}

std::vector<int> debugv3(const FVector3& v) {
	return {
		((int)v.X) >> FRACBITS, ((int)v.Y) >> FRACBITS, ((int)v.Z) >> FRACBITS
	};
}
