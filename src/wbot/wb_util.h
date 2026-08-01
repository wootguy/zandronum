#pragma once
#include "wb_map.h"
#include "wb_vec.h"
#include <vector>

char* VarArgs(const char* format, ...);

player_t* getAnyPlayer();

bool DoLinesIntersect(const vec2& la1, const vec2& la2, const vec2& lb1, const vec2& lb2);

vec2 LineIntersect(const vec2& la1, const vec2& la2, const vec2& lb1, const vec2& lb2);

bool LineIntersectsZRange(const vec3& a, const vec3& b, float zMin, float zMax, vec3& enter, vec3& exit);

vec2 ClosestPointOnSegment(const vec2& p, const vec2& a, const vec2& b);

// returns an empty segment if no overlap
wbot::FSegment2 LineSegmentOverlap(const vec2& a1, const vec2& a2, const vec2& b1, const vec2& b2);

float DistanceToLine(const vec2& p, const vec2& a, const vec2& b);

float DistanceToLine(const vec2& p, wbot::MapLine* line);

// get the distance from the inside to the outside of a box in the given direction (longer at the corners)
float BoxRadiusForDir(const vec2& dir, float radius);

// true if the box is stuck inside a wall, ceiling, or floor
bool IsBoxClipped(const vec3& pos, float radius, float height);

// true if the box is stuck inside an impassable wall
bool IsBoxWallClipped(const vec2& pos, float radius);

// returns the position inside the box which is preventing the box from falling
vec2 GetFloorPosition(const vec3& pos, float radius);

// get sectors the box is clipping into
std::vector<wbot::MapSector*> GetBoxClipSectors(const vec3& pos, float radius, float height);

// clip the polygon against the given line.
// lp = point on the line
// ldir = direction of the line
// front = clip in front of the line, else behind
void ClipPoly(std::vector<vec2>& poly, vec2 lp, vec2 ldir, bool front);

// true if the point is in front of or behind the segment, not to the side
bool PointAlignedSegment(const vec2& p, const vec2& a, const vec2& b);

bool CircleIntersectsSegment(const vec2& center, float radius, const vec2& a, const vec2& b);

void ExtendSegment(vec2& a, vec2& b, float amount);

uint64_t getEpochMillis();

int draw_debug_line(vec3 start, vec3 end, AActor* actor);

bool TraceRadius(vec3 start, vec3 end, float radius, bool ignoreMonsters=true, AActor* ignoreEnt=NULL, wbot::TraceResult* tr=NULL);
