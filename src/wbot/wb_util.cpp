#include "wb_util.h"
#include "wb_bot.h"
#include "wb_map.h"
#include "r_utility.h"
#include "p_spec.h"
#include "p_local.h"
#include "c_dispatch.h"
#include "m_cheat.h"
#include "network.h"
#include "sv_commands.h"
#include "p_trace.h"
#include "wbot/wb_nav.h"

#include <chrono>

using namespace std;
using namespace std::chrono;

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

// get direction pointing behind the given linedef
FVector2 getLineBackDir(line_t* line) {
	fixed_t dx = line->v2->x - line->v1->x;
	fixed_t dy = line->v2->y - line->v1->y;
	double len = sqrt((double)dx * dx + (double)dy * dy);
	return FVector2(-(double)dy / len, (double)dx / len);
}

FVector2 getLineCenter(line_t* line) {
	return FVector2((line->v1->x + line->v2->x) * 0.5, (line->v1->y + line->v2->y) * 0.5);
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

void set_ori(AActor* actor, int x, int y, angle_t angle) {
	fixed_t fx = x << FRACBITS;
	fixed_t fy = y << FRACBITS;
	fixed_t z = R_PointInSubsector(fx, fy)->sector->floorplane.ZatPoint(fx, fy);
	P_Teleport(actor, fx, fy, ONFLOORZ, angle, false, false, false, true, false);
}

int getLineLength(line_t* line) {
	return (int)(FVector2(line->v1->x, line->v1->y) - FVector2(line->v2->x, line->v2->y)).Length() >> FRACBITS;
}

fixed_t DistanceToLine(const FVector2& p, const FVector2& a, const FVector2& b) {
	FVector2 ab = b - a;
    FVector2 ap = p - a;
    return CrossProduct(ab, ap) / ab.Length();
}

fixed_t DistanceToLine(const FVector2& p, line_t* line) {
	return DistanceToLine(p, FVector2(line->v1->x, line->v1->y), FVector2(line->v2->x, line->v2->y));
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
	float len = delta.Length();
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

bool TraceLine(FVector3 start, FVector3 end, bool ignoreMonsters, AActor* ignoreEnt, FTraceResults* tr) {
	FVector3 delta = end - start;
	fixed_t dist = delta.Length();
	delta.MakeUnit();
	delta *= FRACUNIT;

	sector_t* sector = P_PointInSector(start.X, start.Y);

	static FTraceResults dummy;
	FTraceResults* out = tr ? tr : &dummy;

	return Trace((fixed_t)start.X, (fixed_t)start.Y, (fixed_t)start.Z, sector,
		(fixed_t)delta.X, (fixed_t)delta.Y, (fixed_t)delta.Z, dist, ignoreMonsters ? 0 : 0xffffffff,
		ML_BLOCKEVERYTHING | ML_BLOCKHITSCAN, ignoreEnt, *out);
}

bool TraceRadius(FVector3 start, FVector3 end, fixed_t radius, bool ignoreMonsters, AActor* ignoreEnt, FTraceResults* tr) {
	FVector2 dir = (end - start).Unit() * FRACUNIT;
	FVector3 rightDir(dir.Y, -dir.X, 0);
	fixed_t minFrac = FRACUNIT;
	int rightStep = radius / 2;

	static FTraceResults dummy;
	FTraceResults* out = tr ? tr : &dummy;

	for (int i = -2; i <= 2; i++) {
		FTraceResults temp;
		TraceLine(start + rightDir * i * rightStep, end + rightDir * i * rightStep, ignoreMonsters, ignoreEnt, &temp);
		
		if (temp.Fraction < minFrac) {
			minFrac = temp.Fraction;
			*out = temp;
		}
	}

	return minFrac < FRACUNIT;
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
		isect.line = wall;
		isect.pos = FVector2(StartX + FixedMul(Vx, dist), StartY + FixedMul(Vy, dist));

		if (wall->backsector == NULL) {
			isect.sector = in->d.line->frontsector;
		}
		else {
			int lineside = P_PointOnLineSide(StartX, StartY, in->d.line);
			isect.sector = lineside ? wall->backsector : wall->frontsector;
		}

		intersections.push_back(isect);
	}

	return intersections;
}

bool TraceImpassable(FVector2 start, FVector2 end) {
	FPathTraverse path(start.X, start.Y, end.X, end.Y, PT_ADDLINES);
	intercept_t* in;

	while ((in = path.Next())) {
		if (IsImpassable(in->d.line))
			return true;
	}

	return false;
}

bool TraceSectorEdge(FVector2 start, FVector2 end, FVector2& edge) {
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

		edge = FVector2(StartX + FixedMul(Vx, dist), StartY + FixedMul(Vy, dist));
		return true;
	}

	edge = end;
	return false;
}

bool IsImpassable(line_t* line) {
	return !line->backsector || (line->flags & ML_BLOCKING);
}

bool IsPropBlocker(AActor* actor) {
	if (!(actor->flags & (MF_SOLID)))
		return false;

	if (actor->player || (actor->flags3 & (MF3_ISMONSTER | MF_SHOOTABLE)))
		return false;

	return true;
}

void wbot_handle_line_activation(line_t* line, AActor* activator) {
	if (!line->special) {
		// line can no longer be activated. Remove trigger from possibly affected sectors
		int tag = line->args[0];
		for (int i = 0; i < numsectors; i++) {
			if (sectors[i].tag == tag)
				g_wb_mapinfo.remove_invalid_goals(i);
		}
		
		if (line->backsector)
			g_wb_mapinfo.remove_invalid_goals(line->backsector - sectors);
		if (line->frontsector)
			g_wb_mapinfo.remove_invalid_goals(line->frontsector - sectors);
	}

	for (int i = 0; i < MAXPLAYERS; i++) {
		AActor* player = players[i].mo;
		if (!playeringame[i] || !player || !player->player->bIsBot)
			continue;

		CWootBot* bot = (CWootBot*)player->player->pSkullBot;
		bot->HandleLineActivation(line, activator);
	}
}

void kill_all_shootables() {
	TThinkerIterator<AActor> it;
	AActor* actor;
	while ((actor = it.Next())) {
		if ((actor->flags & MF_SHOOTABLE) && !actor->player) {
			P_DamageMobj(actor, actor, actor, actor->health * 2, FName());
		}
	}
}

void wbot_map_init() {
	srand((unsigned int)time(NULL));
	g_wb_nav.init();

	for (int i = 0; i < MAXPLAYERS; i++) {
		AActor* player = players[i].mo;
		if (!playeringame[i] || !player || !player->player->bIsBot)
			continue;

		CWootBot* bot = (CWootBot*)player->player->pSkullBot;
		bot->Reset();
	}
}

void wbot_handle_chat_command(ULONG ulPlayer, const char* msg) {
	// test a specific route
	if (!strcmp(msg, "r")) {
		kill_all_shootables();

		for (int i = 0; i < MAXPLAYERS; i++) {
			AActor* player = players[i].mo;
			if (!playeringame[i] || !player)
				continue;

			if (player->player->bIsBot)	set_ori(player, 2641, 4735, ANGLE_1 * 0);
			else						set_ori(player, 2608, 4545, ANGLE_1 * 40);

			if (player->player->bIsBot) {
				CWootBot* bot = (CWootBot*)player->player->pSkullBot;
				bot->Reset();
				player->player->cheats &= ~CF_FROZEN;
				//bot->m_freezeOnRouteChange = true;
				//bot->m_speedMult = 0.6f;
			}
		}
	}

	// clear all enemies for general pathfinding tests
	if (!strcmp(msg, "y")) {
		kill_all_shootables();
	}

	if (!strcmp(msg, "reset")) {
		for (int i = 0; i < MAXPLAYERS; i++) {
			AActor* player = players[i].mo;
			if (!playeringame[i] || !player || !player->player->bIsBot)
				continue;

			CWootBot* bot = (CWootBot*)player->player->pSkullBot;
			bot->Reset();
		}
	}

	if (strstr(msg, "goto ") == msg) {
		int id = atoi(msg + 5);
		if (id >= 0 && id < numsubsectors) {
			NavSector& nav = g_wb_nav.mesh[id];
			FVector3 pos = nav.pos3D();
			P_Teleport(players[ulPlayer].mo, pos.X, pos.Y, pos.Z, 0, true, true, true);
		}
	}

	// give weapons/ammo for combat testing
	if (!strcmp(msg, "x")) {
		for (int i = 0; i < MAXPLAYERS; i++) {
			AActor* player = players[i].mo;
			if (!playeringame[i] || !player)
				continue;

			cht_Give(player->player, "backpack");
			cht_Give(player->player, "weapons");
			cht_Give(player->player, "ammo");
			cht_Give(player->player, "keys");
			cht_Give(player->player, "armor");
		}
	}

	// slowmotion bots
	if (!strcmp(msg, "f") || !strcmp(msg, "s")) {
		bool slowMotion = !strcmp(msg, "s");
		for (int i = 0; i < MAXPLAYERS; i++) {
			AActor* player = players[i].mo;
			if (!playeringame[i] || !player || !player->player->bIsBot)
				continue;

			CWootBot* bot = (CWootBot*)player->player->pSkullBot;
			bot->m_speedMult = slowMotion ? 0.1f : 1.0f;
		}
	}
}

void wbot_tick() {
	static bool testModo = true;
	if (testModo) {
		testModo = false;
		new CWootBot(NULL, NULL, BOTS_FindFreePlayerSlot());
	}

	g_wb_nav.relink_pending_sector();
}

CCMD(addbotw)
{
	if (gamestate != GS_LEVEL)
		return;

	// Don't allow bots in network mode, unless we're the host.
	if (NETWORK_InClientMode())
	{
		Printf("Only the host can add bots!\n");
		return;
	}

	ULONG ulPlayerIdx = BOTS_FindFreePlayerSlot();
	if (ulPlayerIdx == MAXPLAYERS)
	{
		Printf("The maximum number of players/bots has been reached.\n");
		return;
	}

	new CWootBot(NULL, NULL, ulPlayerIdx);
}
