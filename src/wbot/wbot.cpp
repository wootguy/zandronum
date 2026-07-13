#include "wbot.h"
#include "botcommands.h"
#include "sv_commands.h"
#include "c_dispatch.h"
#include "network.h"
#include "d_event.h"
#include "p_enemy.h"
#include "p_local.h"

#include <stdlib.h>
#include <time.h>

#define RUN_SPEED 100 // max move speed allowed before the server kicks you

extern FRandom g_RandomBotAimSeed;

void CWootBot::draw_debug_line(FVector3 start, FVector3 end) {
	// this sucks and the railgun effect crashes the client so can't use that
	FVector3 dir = (end - start).Resize(10 << FRACBITS);
	int spawns = (end - start).Length() / (10 << FRACBITS);
	for (int i = 0; i < spawns; i++) {
		FVector3 pos = start + (dir * i);
		if (i == spawns - 1) {
			pos = end;
		}
		SERVERCOMMANDS_SpawnBlood(pos.X, pos.Y, pos.Z, 0, 1, m_pPlayer->mo);
	}
}

CWootBot::CWootBot(const char* pszName, const char* pszTeamName, ULONG ulPlayerNum)
	: CSkullBot(pszName, pszTeamName, ulPlayerNum) {

	static bool seedRand = false;
	if (!seedRand) {
		seedRand = true;
		srand((unsigned int)time(NULL));
	}

	m_fov = ANGLE_180;

	m_bForwardMovePersist = true;
	m_bSideMovePersist = true;
}
CWootBot::~CWootBot() {}

void CWootBot::ParseScript() {
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
		RoamThink();
	}

	m_lForwardMove = static_cast<LONG>(0x32 * (m_forwardMove / 100.0f));
	m_lSideMove = static_cast<LONG>(0x32 * (m_sideMove / 100.0f));
}
void CWootBot::DeadThink() {
	// tap a button to respawn
	m_lForwardMove = 0;
	m_lSideMove = 0;
	m_lButtons ^= BT_ATTACK;
}

void CWootBot::RoamThink() {
	m_forwardMove = 100;

	if (rand() % 10 == 0) {
		m_pPlayer->mo->angle = (rand() % 360) * ANGLE_1;
	}
}

void CWootBot::CombatThink() {
	AActor* targ = m_pPlayer->mo->target;

	if (!targ)
		return;

	// aim at enemy
	POS_t enemyPos = { targ->x, targ->y, targ->z };
	fixed_t shootz = m_pPlayer->mo->z - m_pPlayer->mo->floorclip + (targ->height >> 1) + (8 * FRACUNIT);
	fixed_t dist = P_AproxDistance(m_pPlayer->mo->x - enemyPos.x, m_pPlayer->mo->y - enemyPos.y);
	m_pPlayer->mo->pitch = -(SDWORD)R_PointToAngle2(0, shootz, dist, enemyPos.z + targ->height / 2);
	m_pPlayer->mo->angle = R_PointToAngle2(m_pPlayer->mo->x, m_pPlayer->mo->y, enemyPos.x, enemyPos.y);

	m_forwardMove = 0;
	m_sideMove = 0;

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