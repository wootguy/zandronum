#pragma once
#include "bots.h"
#include <unordered_map>
#include <string>

struct FTraceResults;

struct TraceIsect {
	line_t* line;
	sector_t* sector;
};

char* VarArgs(const char* format, ...);

AActor* getAnyPlayer();

// get direction pointing behind the given linedef
FVector2 getLineBackDir(line_t* line);

FVector2 getLineCenter(line_t* line);

bool DoLinesIntersect(const FVector2& la1, const FVector2& la2, const FVector2& lb1, const FVector2& lb2);

FVector2 LineIntersect(const FVector2& la1, const FVector2& la2, const FVector2& lb1, const FVector2& lb2);

FVector2 ClosestPointOnSegment(const FVector2& p, const FVector2& a, const FVector2& b);

void set_ori(AActor* actor, int x, int y, angle_t angle);

int getLineLength(line_t* line);

fixed_t DistanceToLine(const FVector2& p, const FVector2& a, const FVector2& b);

fixed_t DistanceToLine(const FVector2& p, line_t* line);

// true if the point is in front of or behind the segment, not to the side
bool PointAlignedSegment(const FVector2& p, const FVector2& a, const FVector2& b);

bool CircleIntersectsSegment(const FVector2& center, float radius, const FVector2& a, const FVector2& b);

void ExtendSegment(FVector2& a, FVector2& b, float amount);

void MakeVectors(angle_t angle, FVector3& forward, FVector3& right);

float DotProduct(const FVector2& a, const FVector2& b);

float CrossProduct(const FVector2& a, const FVector2& b);

uint64_t getEpochMillis();

int draw_debug_line(FVector3 start, FVector3 end, AActor* actor);

bool TraceLine(FVector3 start, FVector3 end, bool ignoreMonsters=true, AActor* ignoreEnt=NULL, FTraceResults* tr=NULL);

bool TraceRadius(FVector3 start, FVector3 end, fixed_t radius, bool ignoreMonsters=true, AActor* ignoreEnt=NULL, FTraceResults* tr=NULL);

// returns all walls/sectors intersected by the given line
std::vector<TraceIsect> TraceIntersections(FVector2 start, FVector2 end);

// returns true if the trace intersects any impassable walls
bool TraceImpassable(FVector2 start, FVector2 end);

bool IsPropBlocker(AActor* actor);

// Hooks
void wbot_handle_chat_command(ULONG ulPlayer, const char* msg);
void wbot_handle_line_activation(line_t* line, AActor* activator);
void wbot_map_init();
void wbot_tick();