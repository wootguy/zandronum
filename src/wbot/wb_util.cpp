#include "wb_util.h"
#include "wb_bot.h"
#include "wb_map.h"
#include "wb_nav.h"
#include "sv_commands.h"
#include "m_bbox.h"

#include <chrono>
#include <float.h>

using namespace std;
using namespace std::chrono;
using namespace wbot;

char* VarArgs(const char* format, ...)
{
	va_list		argptr;
	static char		string[1024];

	va_start(argptr, format);
	vsnprintf(string, 1024, format, argptr);
	va_end(argptr);

	return string;
}

AActor* getAnyPlayer() {
	AActor* player = NULL;
	for (int i = 0; i < MAXPLAYERS; i++)
	{
		if (!playeringame[i])
			continue;

		AActor* actor = players[i].mo;
		if (!actor || actor->player->bIsBot)
			continue;

		return actor;
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

void set_ori(AActor* actor, int x, int y, angle_t angle) {
	fixed_t fx = x << FRACBITS;
	fixed_t fy = y << FRACBITS;
	fixed_t z = g_map.GetSector(fx, fy)->getFloorZ();
	P_Teleport(actor, fx, fy, ONFLOORZ, angle, false, false, false, true, false);
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
	FBoundingBox box(pos.X, pos.Y, radius);
	FBlockLinesIterator it(box);
	line_t* ld;

	while ((ld = it.Next())) {
		if (box.Right() <= ld->bbox[BOXLEFT]
			|| box.Left() >= ld->bbox[BOXRIGHT]
			|| box.Top() <= ld->bbox[BOXBOTTOM]
			|| box.Bottom() >= ld->bbox[BOXTOP]) {
			continue;
		}

		if (box.BoxOnLineSide(ld) == -1 && (!ld->backsector || (ld->flags & ML_BLOCKING)))
			return true;  // crosses an impassable line
	}

	return false;
}

bool IsBoxClipped(const FVector3& pos, fixed_t radius, fixed_t height) {
	FBoundingBox box(pos.X, pos.Y, radius);
	FBlockLinesIterator it(box);
	line_t* ld;

	fixed_t x = pos.X;
	fixed_t y = pos.Y;
	fixed_t z = pos.Z;
	fixed_t topZ = z + height;

	while ((ld = it.Next())) {
		if (box.Right() <= ld->bbox[BOXLEFT]
			|| box.Left() >= ld->bbox[BOXRIGHT]
			|| box.Top() <= ld->bbox[BOXBOTTOM]
			|| box.Bottom() >= ld->bbox[BOXTOP]) {
			continue;
		}

		if (box.BoxOnLineSide(ld) != -1)
			continue; // doesn't cross the line

		if (!ld->backsector || (ld->flags & ML_BLOCKING))
			return true; // crosses an impassable line
		
		fixed_t frontCeil = ld->frontsector->ceilingplane.ZatPoint(x, y);
		fixed_t frontFloor = ld->frontsector->floorplane.ZatPoint(x, y);
		fixed_t backCeil = ld->backsector->ceilingplane.ZatPoint(x, y);
		fixed_t backFloor = ld->backsector->floorplane.ZatPoint(x, y);

		if (z < backFloor || z < frontFloor)
			return true; // clipped into the floor

		if (topZ > frontCeil || topZ > backCeil)
			return true; // clipped into the ceiling
	}

	return false;
}

std::vector<MapSector*> GetBoxClipSectors(const FVector3& pos, fixed_t radius, fixed_t height) {
	std::vector<MapSector*> clipSectors;
	
	FBoundingBox box(pos.X, pos.Y, radius);
	FBlockLinesIterator it(box);
	line_t* ld;

	fixed_t x = pos.X;
	fixed_t y = pos.Y;
	fixed_t z = pos.Z;
	fixed_t topZ = z + height;

	while ((ld = it.Next())) {

		if (box.Right() <= ld->bbox[BOXLEFT]
			|| box.Left() >= ld->bbox[BOXRIGHT]
			|| box.Top() <= ld->bbox[BOXBOTTOM]
			|| box.Bottom() >= ld->bbox[BOXTOP]) {
			continue;
		}

		if (box.BoxOnLineSide(ld) != -1)
			continue; // doesn't cross the line

		if (!ld->backsector || (ld->flags & ML_BLOCKING))
			continue; // crosses an impassable line, no sector to trigger

		fixed_t frontCeil = ld->frontsector->ceilingplane.ZatPoint(x, y);
		fixed_t frontFloor = ld->frontsector->floorplane.ZatPoint(x, y);
		fixed_t backCeil = ld->backsector->ceilingplane.ZatPoint(x, y);
		fixed_t backFloor = ld->backsector->floorplane.ZatPoint(x, y);

		if (z < backFloor || topZ > backCeil)
			clipSectors.push_back(&g_map.sectors[ld->backsector - sectors]);

		if (z < frontFloor || topZ > frontCeil)
			clipSectors.push_back(&g_map.sectors[ld->frontsector - sectors]);
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
	t = clamp(t, 0.0f, 1.0f);

	FVector2 closest = a + ab * t;
	return (center - closest).LengthSquared() <= r2;
}

void MakeVectors(angle_t angle, FVector3& forward, FVector3& right) {
	fixed_t fsine = finesine[angle >> ANGLETOFINESHIFT];
	fixed_t fcosine = finecosine[angle >> ANGLETOFINESHIFT];
	forward = FVector3(fcosine, fsine, 0);
	right = FVector3(fsine, -fcosine, 0);
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
		SERVERCOMMANDS_SpawnBlood((fixed_t)pos.X, (fixed_t)pos.Y, (fixed_t)pos.Z, 0, 1, actor);
	}

	return i;
}

bool TraceLine(FVector3 start, FVector3 end, bool ignoreMonsters, AActor* ignoreEnt, TraceResult* tr) {
	return g_map.Trace(start, end, ignoreMonsters ? 0 : 0xffffffff,
		ML_BLOCKEVERYTHING | ML_BLOCKHITSCAN, ignoreEnt, tr);
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

vector<TraceIsect> TraceIntersections(FVector2 start, FVector2 end) {
	FVector2 delta = end - start;
	fixed_t maxDist = delta.Length();
	delta = delta.Unit() * FRACUNIT;

	fixed_t StartX = start.X;
	fixed_t StartY = start.Y;
	fixed_t Vx = delta.X;
	fixed_t Vy = delta.Y;

	FPathTraverse path(start.X, start.Y, end.X, end.Y, PT_ADDLINES);
	intercept_t* in;

	vector<TraceIsect> intersections;

	while ((in = path.Next())) {
		line_t* wall = in->d.line;
		fixed_t dist = FixedMul(maxDist, in->frac);

		TraceIsect isect;
		isect.line = &g_map.lines[wall - lines];
		isect.pos = FVector2(StartX + FixedMul(Vx, dist), StartY + FixedMul(Vy, dist));
		isect.fraction = in->frac;

		if (wall->backsector == NULL) {
			isect.sector = &g_map.sectors[in->d.line->frontsector - sectors];
		}
		else {
			int lineside = P_PointOnLineSide(StartX, StartY, in->d.line);
			sector_t* sec = lineside ? wall->backsector : wall->frontsector;
			isect.sector = &g_map.sectors[sec - sectors];
		}

		intersections.push_back(isect);
	}

	return intersections;
}

bool TraceImpassable(FVector2 start, FVector2 end) {
	FPathTraverse path(start.X, start.Y, end.X, end.Y, PT_ADDLINES);
	intercept_t* in;

	FVector2 delta = end - start;
	fixed_t maxDist = delta.Length();
	delta = delta.Unit() * FRACUNIT;

	fixed_t StartX = start.X;
	fixed_t StartY = start.Y;
	fixed_t Vx = delta.X;
	fixed_t Vy = delta.Y;

	while ((in = path.Next())) {
		line_t* line = in->d.line;

		if (!line->backsector || (line->flags & ML_BLOCKING)) {
			fixed_t dist = FixedMul(maxDist, in->frac);
			FVector2 pos = FVector2(StartX + FixedMul(Vx, dist), StartY + FixedMul(Vy, dist));
			sector_t* hitSector = R_PointInSubsector(pos.X, pos.Y)->sector;

			if (hitSector != line->frontsector && hitSector != line->backsector)
				continue; // bug in trace where lines in other sectors nowhere near the impact point are hit

			return true;
		}
	}

	return false;
}

bool TraceSectorEdge(FVector2 start, FVector2 end, FVector2& edge, MapLine** line) {
	FPathTraverse path(start.X, start.Y, end.X, end.Y, PT_ADDLINES);
	intercept_t* in = path.Next();

	if (in) {
		FVector2 delta = end - start;
		fixed_t dist = FixedMul((fixed_t)delta.Length(), in->frac);
		FVector2 dir = delta.Unit() * FRACUNIT;

		fixed_t StartX = start.X;
		fixed_t StartY = start.Y;
		fixed_t Vx = dir.X;
		fixed_t Vy = dir.Y;

		if (line) {
			*line = &g_map.lines[in->d.line - ::lines];
		}

		edge = FVector2(StartX + FixedMul(Vx, dist), StartY + FixedMul(Vy, dist));
		return true;
	}
	else if (line) {
		*line = NULL;
	}

	edge = end;
	return false;
}

bool IsImpassable(MapLine* line) {
	return !line->backsector || (line->flags & ML_BLOCKING);
}

bool IsPropBlocker(AActor* actor) {
	if (!(actor->flags & (MF_SOLID)))
		return false;

	if (actor->player || (actor->flags3 & (MF3_ISMONSTER | MF_SHOOTABLE)))
		return false;

	return true;
}

void SpawnBlood(FVector3 pos, int damage, AActor* owner) {
	SERVERCOMMANDS_SpawnBlood(pos.X, pos.Y, pos.Z, 0, damage, owner);
}

void PrintNotification(const char* msg) {
	SERVERCOMMANDS_Print(msg, PRINT_CHAT);
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
