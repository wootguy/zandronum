#include "wbot.h"
#include "botcommands.h"
#include "sv_commands.h"
#include "c_dispatch.h"
#include "network.h"
#include "d_event.h"
#include "p_enemy.h"
#include "p_local.h"
#include "po_man.h"
#include "wnav.h"
#include "p_trace.h"

#include <stdlib.h>
#include <time.h>
#include <unordered_set>
#include <unordered_map>
#include <string>

using namespace std;

#define RUN_SPEED 100 // max move speed allowed before the server kicks you
#define MAX_NODE_LINKS 16

extern FRandom g_RandomBotAimSeed;

char* VarArgs(const char* format, ...)
{
	va_list		argptr;
	static char		string[1024];

	va_start(argptr, format);
	vsnprintf(string, 1024, format, argptr);
	va_end(argptr);

	return string;
}

void init_wootbots() {
	static int lastInit;
	static int lastInitTime;
	if (lastInit != level.levelnum || lastInitTime == 0 || lastInitTime > level.time) {
		lastInit = level.levelnum;
		lastInitTime = level.time;

		srand((unsigned int)time(NULL));

		g_wbot_nav.generate_node_graph();
	}
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

CWootBot::CWootBot(const char* pszName, const char* pszTeamName, ULONG ulPlayerNum)
	: CSkullBot(pszName, pszTeamName, ulPlayerNum) {
	
	m_fov = ANGLE_180;
	m_bForwardMovePersist = true;
	m_bSideMovePersist = true;
	m_debug = true;
}
CWootBot::~CWootBot() {}

void CWootBot::ParseScript() {
	init_wootbots();

	// level changed
	if (lastInit != level.levelnum) {
		lastInit = level.levelnum;
		CancelRoute();
	}

	if (m_debug) {
		ShowDebugInfo();
	}

	if (m_pPlayer->health <= 0) {
		DeadThink();
		return;
	}

	m_lButtons = 0;

	FindEnemy();

	if (m_pPlayer->mo->target) {
		CombatThink();
	}
	else {
		if (m_route.empty()) {
			FindMoveGoal();
		}

		if (m_route.empty()) {
			IdleThink();
		}
		else {
			RouteThink();
		}
	}

	m_lForwardMove = static_cast<LONG>(0x32 * (m_forwardMove / 100.0f));
	m_lSideMove = static_cast<LONG>(0x32 * (m_sideMove / 100.0f));
}

void CWootBot::ShowDebugInfo() {
	g_wbot_nav.draw_nodes(m_pPlayer->mo);

	int thisSubId = g_wbot_nav.get_nav_id(m_pPlayer->mo);

	string routeStr = "Route: " + to_string(thisSubId);
	if (pretendRouteSector >= 0) {
		routeStr += " (pretend " + to_string(pretendRouteSector) + " )";
	}
	routeStr += " -> ";
	for (int i = 0; i < m_route.size(); i++) {
		if (i != 0)
			routeStr += " ";
		routeStr += to_string(m_route[i]);
	}
	
	AActor* player = getAnyPlayer();
	string navInfo = "Your sector: ";

	if (player) {
		int plrnavid = g_wbot_nav.get_nav_id(player);
		navInfo += to_string(plrnavid);
		NavSector& nav = g_wbot_nav.nav_sectors[plrnavid];
		navInfo += " (" + to_string(nav.links.size()) + " links)";
	}

	string stuckStr = "Stuck: " + to_string(stuckCounter);

	string enemyStr = "Enemy: <none>";
	if (m_pPlayer->mo->target)
		enemyStr = string("Enemy: ") + m_pPlayer->mo->target->GetClass()->TypeName.GetChars();

	string debugStr = navInfo + "\n\n" + enemyStr + "\n" + routeStr + "\n" + stuckStr;

	SERVERCOMMANDS_PrintHUDMessage(debugStr.c_str(), 0, 0.3f, 0, 0, 0, CR_RED, 1.0f, 0, 0, "SmallFont", MAKE_ID('W', 'B', 'O', 'T'));
}

void CWootBot::DeadThink() {
	// tap a button to respawn
	m_lForwardMove = 0;
	m_lSideMove = 0;
	m_lButtons ^= BT_ATTACK;
}

void CWootBot::IdleThink() {
	m_forwardMove = 0;
	m_sideMove = 0;
	m_pPlayer->mo->pitch = 0;
	m_lButtons |= BT_CROUCH;

	if (rand() % 20 == 0) {
		m_pPlayer->mo->angle = (rand() % 360) * ANGLE_1;
	}
}

void CWootBot::CancelRoute() {
	m_route.clear();
	pretendRouteSector = -1;
	stuckCounter = 0;
}

void CWootBot::RouteThink() {
	int thisSubId = g_wbot_nav.get_nav_id(m_pPlayer->mo);
	const int routeSpeed = 100;

	m_forwardMove = 0;
	m_sideMove = 0;

	if (m_route.size() > 1) {
		if (thisSubId == m_route[1]) {
			// inside the target sector. Advance the route.
			m_route.erase(m_route.begin());
			pretendRouteSector = -1;
		}
		else {
			FVector3 center = g_wbot_nav.nav_sectors[m_route[1]].pos();
			fixed_t dist = P_AproxDistance(m_pPlayer->mo->x - center.X, m_pPlayer->mo->y - center.Y);
			if (dist < (16 << FRACBITS)) {
				// already very close to the center, so this is probably a tiny polygon jammed
				// up against a wall. The bot can't get close enough in this case, so advance
				// the route now and pretend the bot is inside the target sector.
				pretendRouteSector = m_route[1];
				m_route.erase(m_route.begin());
			}
		}
	}

	if (pretendRouteSector >= 0) {
		thisSubId = pretendRouteSector;
	}

	if (m_route.size() > 1) {	
		if (thisSubId == m_route[0]) {
			NavSectorLink* link = g_wbot_nav.nav_sectors[thisSubId].getLink(m_route[1]);

			if (!link || link->blocked()) {
				SERVERCOMMANDS_Print("Link got blocked!\n", PRINT_CHAT);
				CancelRoute();
				return;
			}

			if (MoveTo(link->pos(), 32, routeSpeed)) {
				// now move towards the center until we end up inside the target sector
				FVector3 centerGoal = g_wbot_nav.nav_sectors[m_route[1]].pos();
				MoveTo(centerGoal, 16, routeSpeed);
			}
		}
		else {
			SERVERCOMMANDS_Print(VarArgs("Fell off the route (expected %d but got %d)\n", m_route[0], thisSubId), PRINT_CHAT);
			CancelRoute();
		}
	}
	else if (m_route.size() == 1) {
		FVector3 centerGoal = g_wbot_nav.nav_sectors[m_route[0]].pos();
		if (MoveTo(centerGoal, 32, routeSpeed)) {
			CancelRoute();
			SERVERCOMMANDS_Print("Finished route\n", PRINT_CHAT);
		}
	}

	if (StuckThink(500)) {
		CancelRoute();
		SERVERCOMMANDS_Print("I got stuck! Cancelling route\n", PRINT_CHAT);
	}
}

bool CWootBot::StuckThink(int maxStuck) {
	FVector2 curPos = FVector2(m_pPlayer->mo->x, m_pPlayer->mo->y);
	int movedDist = (int)(curPos - lastPos).Length() >> FRACBITS;
	lastPos = curPos;

	if ((m_forwardMove || m_sideMove) && movedDist <= 1) {
		stuckCounter += 10;
		if (stuckCounter > maxStuck) {
			return true;
		}
	}
	else {
		stuckCounter--;
		if (stuckCounter < 0)
			stuckCounter = 0;
	}

	return false;
}

void CWootBot::AimAtPos(FVector3 pos) {
	POS_t lookPos = { pos.X, pos.Y, pos.Z };
	fixed_t viewZ = m_pPlayer->mo->z + m_pPlayer->viewheight;
	fixed_t dist = P_AproxDistance(m_pPlayer->mo->x - lookPos.x, m_pPlayer->mo->y - lookPos.y);
	m_pPlayer->mo->pitch = -(SDWORD)R_PointToAngle2(0, viewZ, dist, lookPos.z);
	m_pPlayer->mo->angle = R_PointToAngle2(m_pPlayer->mo->x, m_pPlayer->mo->y, lookPos.x, lookPos.y);
}

bool CWootBot::MoveTo(FVector3 pos, int radius, int speed) {
	m_forwardMove = speed;
	m_sideMove = 0;
	pos.Z = m_pPlayer->mo->z + m_pPlayer->viewheight;
	AimAtPos(pos);

	// check for walls to jump over
	angle_t angle = m_pPlayer->mo->angle;
	fixed_t dx = finecosine[angle >> ANGLETOFINESHIFT];
	fixed_t dy = finesine[angle >> ANGLETOFINESHIFT];
	FVector3 start(m_pPlayer->mo->x, m_pPlayer->mo->y, m_pPlayer->mo->z + STEP_HEIGHT);
	fixed_t testDist = 32 << FRACBITS;
	sector_t* sector = R_PointInSubsector(start.X, start.Y)->sector;

	FTraceResults tr;
	if (Trace(start.X, start.Y, start.Z, sector, dx, dy, 0, testDist, 0, ML_BLOCKEVERYTHING | ML_BLOCKHITSCAN, NULL, tr)) {
		m_lButtons |= BT_JUMP;
	}

	fixed_t dist = P_AproxDistance(m_pPlayer->mo->x - pos.X, m_pPlayer->mo->y - pos.Y);
	return dist < (radius << FRACBITS);
}

void CWootBot::CombatThink() {
	AActor* targ = m_pPlayer->mo->target;

	if (!targ)
		return;

	// aim at enemy
	AimAtPos(FVector3(targ->x, targ->y, targ->z + targ->height / 2));

	m_forwardMove = 0;
	m_sideMove = 0;

	fixed_t dist = P_AproxDistance(m_pPlayer->mo->x - targ->x, m_pPlayer->mo->y - targ->y);
	fixed_t minChaseDist = 200 << FRACBITS;
	fixed_t maxChaseDist = 500 << FRACBITS;

	// don't get too close/far
	if (dist < minChaseDist) {
		m_forwardMove = -RUN_SPEED;
	}
	else if (dist > maxChaseDist) {
		m_forwardMove = RUN_SPEED;
	}

	// randomly strafe around the target
	int r = rand() % 10;
	if (r < 4) {
		m_sideMove = -RUN_SPEED;
	}
	else if (r < 8) {
		m_sideMove = RUN_SPEED;
	}
	else {
		m_sideMove = 0;
	}

	if (P_CheckSight(m_pPlayer->mo, targ, SF_SEEPASTSHOOTABLELINES)) {
		m_lButtons |= BT_ATTACK;
	}
}

AActor* wbot_LookForEnemiesInBlock(AActor* lookee, int index, void* extparam)
{
	FBlockNode* block;
	AActor* link;
	CWootBot* pbot = (CWootBot*)extparam;
	angle_t fov = pbot->m_fov;

	for (block = blocklinks[index]; block != NULL; block = block->NextActor)
	{
		link = block->Me;

		if (!(link->flags & MF_SHOOTABLE))
			continue;			// not shootable (observer or dead)

		if (link == lookee)
			continue;

		if (link->health <= 0)
			continue;			// dead

		if (link->flags2 & MF2_DORMANT)
			continue;			// don't target dormant things

		if (!(link->flags3 & MF3_ISMONSTER))
			continue;			// don't target it if it isn't a monster (could be a barrel)

		if (link->flags7 & MF7_NEVERTARGET)
			continue;

		if (lookee->IsFriend(link))
			continue;

		if (fov && fov < ANGLE_MAX)
		{
			angle_t an = R_PointToAngle2(lookee->x, lookee->y, link->x, link->y) - lookee->angle;

			if (an > (fov / 2) && an < (ANGLE_MAX - (fov / 2))) {
				continue;	// outside of fov
			}
		}

		// P_CheckSight is by far the most expensive operation in here so let's do it last.
		if (!P_CheckSight(lookee, link, SF_SEEPASTSHOOTABLELINES)) {
			continue;
		}

		return link;
	}
	return NULL;
}

FVector3 CWootBot::GetViewPos() {
	return FVector3(m_pPlayer->mo->x, m_pPlayer->mo->y, m_pPlayer->mo->z + m_pPlayer->mo->ViewHeight);
}

void CWootBot::FindEnemy() {
	if (m_pPlayer->mo->target && m_pPlayer->mo->target->health > 0) {
		if (P_CheckSight(m_pPlayer->mo, m_pPlayer->mo->target, SF_SEEPASTSHOOTABLELINES)) {
			m_targetLastSeenTic = level.maptime;
		}

		if (level.maptime - m_targetLastSeenTic < 35) {
			// don't forget about the target until some time has passed
			return;
		}
	}

	m_pPlayer->mo->target = P_BlockmapSearch(m_pPlayer->mo, 10, wbot_LookForEnemiesInBlock, this);
}

bool CWootBot::FindMoveGoal() {
	int thisSubId = g_wbot_nav.get_nav_id(m_pPlayer->mo);

	AActor* player = NULL;
	for (int i = 0; i < MAXPLAYERS; i++)
	{
		if (!playeringame[i])
			continue;

		AActor* actor = players[i].mo;
		if (!actor || actor->player->bIsBot)
			continue;

		int plrSubId = g_wbot_nav.get_nav_id(actor);

		m_route = g_wbot_nav.get_astar_route(thisSubId, plrSubId);

		if (m_route.size() > 1) {
			SERVERCOMMANDS_Print(VarArgs("Found a player to route to! Route from %d to %d (size %d)\n",
				m_route[0], m_route[m_route.size() - 1], m_route.size()), PRINT_CHAT);
			break;
		}
		else {
			m_route.clear();
		}
	}

	return m_route.size();
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