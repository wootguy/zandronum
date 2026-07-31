#pragma once
#include "wb_map.h"
#include <vector>

struct FTraceResults;

struct TraceIsect {
	wbot::MapLine* line;
	wbot::MapSector* sector;
	FVector2 pos;
	fixed_t fraction;
};

char* VarArgs(const char* format, ...);

player_t* getAnyPlayer();

bool DoLinesIntersect(const FVector2& la1, const FVector2& la2, const FVector2& lb1, const FVector2& lb2);

FVector2 LineIntersect(const FVector2& la1, const FVector2& la2, const FVector2& lb1, const FVector2& lb2);

bool LineIntersectsZRange(const FVector3& a, const FVector3& b, float zMin, float zMax, FVector3& enter, FVector3& exit);

FVector2 ClosestPointOnSegment(const FVector2& p, const FVector2& a, const FVector2& b);

// returns an empty segment if no overlap
wbot::FSegment2 LineSegmentOverlap(const FVector2& a1, const FVector2& a2, const FVector2& b1, const FVector2& b2);

void set_ori(AActor* actor, int x, int y, uint32_t angle);

float DistanceToLine(const FVector2& p, const FVector2& a, const FVector2& b);

float DistanceToLine(const FVector2& p, wbot::MapLine* line);

// get the distance from the inside to the outside of a box in the given direction (longer at the corners)
fixed_t BoxRadiusForDir(const FVector2& dir, fixed_t radius);

// true if the box is stuck inside a wall, ceiling, or floor
bool IsBoxClipped(const FVector3& pos, fixed_t radius, fixed_t height);

// true if the box is stuck inside an impassable wall
bool IsBoxWallClipped(const FVector2& pos, fixed_t radius);

// returns the position inside the box which is preventing the box from falling
FVector2 GetFloorPosition(const FVector3& pos, fixed_t radius);

// get sectors the box is clipping into
std::vector<wbot::MapSector*> GetBoxClipSectors(const FVector3& pos, fixed_t radius, fixed_t height);

// clip the polygon against the given line.
// lp = point on the line
// ldir = direction of the line
// front = clip in front of the line, else behind
void ClipPoly(std::vector<FVector2>& poly, FVector2 lp, FVector2 ldir, bool front);

// true if the point is in front of or behind the segment, not to the side
bool PointAlignedSegment(const FVector2& p, const FVector2& a, const FVector2& b);

bool CircleIntersectsSegment(const FVector2& center, float radius, const FVector2& a, const FVector2& b);

void ExtendSegment(FVector2& a, FVector2& b, float amount);

void MakeVectors(uint32_t angle, FVector3& forward, FVector3& right);

float DotProduct(const FVector2& a, const FVector2& b);

float CrossProduct(const FVector2& a, const FVector2& b);

uint64_t getEpochMillis();

int draw_debug_line(FVector3 start, FVector3 end, AActor* actor);

bool TraceRadius(FVector3 start, FVector3 end, fixed_t radius, bool ignoreMonsters=true, AActor* ignoreEnt=NULL, wbot::TraceResult* tr=NULL);

// returns all walls/sectors intersected by the given line
std::vector<TraceIsect> TraceIntersections(FVector2 start, FVector2 end);

// returns true if the trace intersects any impassable walls
bool TraceImpassable(FVector2 start, FVector2 end);

// trace until the first intersected sector edge
bool TraceSectorEdge(FVector2 start, FVector2 end, FVector2& edge, wbot::MapLine** line);

bool IsImpassable(wbot::MapLine* line);

bool IsPropBlocker(AActor* actor);

void SpawnBlood(FVector3 pos, int damage, AActor* owner);

void PrintNotification(const char* msg);

std::vector<int> debugv2(const FVector2& v);

std::vector<int> debugv3(const FVector2& v);
