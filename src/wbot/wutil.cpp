#include "wutil.h"
#include "wbot.h"
#include "r_utility.h"
#include "p_spec.h"
#include "p_local.h"
#include "c_dispatch.h"
#include "m_cheat.h"
#include "network.h"
#include "sv_commands.h"
#include "p_trace.h"

#include <chrono>

using namespace std;
using namespace std::chrono;

unordered_map<string, WeaponInfo> g_wbot_weapon_info = {
	{"Fist",			{0,  0,   0,   64}},
	{"Chainsaw",		{1,  0,   0,   64}},
	{"Pistol",			{2,  0,   200, 4000}},
	{"Shotgun",			{3,  0,   200, 2000}},
	{"Chaingun",		{4,  0,   400, 4000}},
	{"Minigun",			{4,  0,   400, 4000}},
	{"GrenadeLauncher",	{5,  200, 400, 2000}},
	{"RocketLauncher",	{5,  200, 500, 4000}},
	{"SuperShotgun",	{6,  0,   200, 2000}},
	{"PlasmaRifle",		{7,  0,   400, 4000}},
	{"Railgun",			{7,	 0,   400, 4000}},
	{"BFG9000",			{8,  0,   200, 2000}},
	{"BFG10K",			{8,  0,   200, 2000}},
};

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
FVector3 getLineBackDir(line_t* line) {
	fixed_t dx = line->v2->x - line->v1->x;
	fixed_t dy = line->v2->y - line->v1->y;
	double len = sqrt((double)dx * dx + (double)dy * dy);
	return FVector3(-(double)dy / len, (double)dx / len, 0);
}

FVector3 getLineCenter(line_t* line) {
	return FVector3((line->v1->x + line->v2->x) * 0.5, (line->v1->y + line->v2->y) * 0.5, 0);
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
	delta *= 1 << FRACBITS;

	sector_t* sector = P_PointInSector(start.X, start.Y);

	static FTraceResults dummy;
	FTraceResults* out = tr ? tr : &dummy;

	return Trace((fixed_t)start.X, (fixed_t)start.Y, (fixed_t)start.Z, sector,
		(fixed_t)delta.X, (fixed_t)delta.Y, (fixed_t)delta.Z, dist, ignoreMonsters ? 0 : 0xffffffff,
		ML_BLOCKEVERYTHING | ML_BLOCKHITSCAN, ignoreEnt, *out);
}

void wbot_handle_chat_command(ULONG ulPlayer, const char* msg) {
	// clear all enemies for general pathfinding tests
	if (!strcmp(msg, "y")) {
		TThinkerIterator<AActor> it;
		AActor* actor;
		while ((actor = it.Next())) {
			if (actor->flags3 & MF3_ISMONSTER) {
				P_DamageMobj(actor, actor, actor, actor->health * 2, FName());
			}
		}

		for (int i = 0; i < MAXPLAYERS; i++) {
			AActor* player = players[i].mo;
			if (!playeringame[i] || !player)
				continue;
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

	// test a specific route
	if (!strcmp(msg, "r")) {
		TThinkerIterator<AActor> it;
		AActor* actor;
		while ((actor = it.Next())) {
			if (actor->flags3 & MF3_ISMONSTER) {
				P_DamageMobj(actor, actor, actor, actor->health * 2, FName());
			}
		}

		for (int i = 0; i < MAXPLAYERS; i++) {
			AActor* player = players[i].mo;
			if (!playeringame[i] || !player)
				continue;

			if (player->player->bIsBot)	set_ori(player, 1022, 1237, ANGLE_1 * 0);
			else						set_ori(player, 1565, 1433, ANGLE_1 * 110);

			if (player->player->bIsBot) {
				CWootBot* bot = (CWootBot*)player->player->pSkullBot;
				bot->Reset();
				player->player->cheats &= ~CF_FROZEN;
				//bot->m_freezeOnRouteChange = true;
				//bot->m_speedMult = 0.6f;
			}
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
