#include "wb_combat.h"
#include "wb_bot.h"
#include "wb_util.h"
#include "wb_eiface.h"

#include <algorithm>
#include <cstring>

using namespace std;
using namespace wbot;

unordered_map<string, WeaponInfo> g_wbot_weapon_info = {
	{"Fist",			{0,  0,   0,   64,		0}},
	{"Chainsaw",		{1,  0,   0,   64,		0}},
	{"Pistol",			{2,  0,   200, 4000,	1}},
	{"Shotgun",			{3,  0,   200, 2000,	1}},
	{"Chaingun",		{4,  0,   400, 4000,	1}},
	{"Minigun",			{4,  0,   400, 4000,	1}},
	{"GrenadeLauncher",	{5,  200, 400, 2000,	1}},
	{"RocketLauncher",	{5,  200, 500, 4000,	1}},
	{"SuperShotgun",	{6,  0,   200, 2000,	2}},
	{"PlasmaRifle",		{7,  0,   400, 4000,	1}},
	{"Railgun",			{7,	 0,   400, 4000,	1}},
	{"BFG9000",			{8,  0,   200, 2000,	40}},
	{"BFG10K",			{8,  0,   200, 2000,	40}},
};

CBotCombatController::CBotCombatController(CWootBot* pBot)
	: pBot(pBot), pActor(pBot->pActor), pPlayer(pBot->m_pPlayer) {}

void CBotCombatController::Think() {
	AActor* bestEnemy = BestEnemy();

	if (bestEnemy) {
		pBot->target = bestEnemy;
	}

	AActor* targ = pBot->target;

	if (!targ || get_actor_health(targ) <= 0) {
		pBot->target = NULL;
		return;
	}

	SelectBestWeapon();

	vec2 targPos = get_actor_pos(targ);

	float dist = (targPos - pBot->m_origin).length();
	float minChaseDist = 200;
	float maxChaseDist = 500;
	float maxRange = 2000;
	float minRange = 0;
	bool isMeleeWeapon = false;

	if (pBot->m_weaponName) {
		WeaponInfo& info = g_wbot_weapon_info[pBot->m_weaponName];
		minChaseDist = info.minRange + 64;
		maxChaseDist = std::max(minChaseDist, (float)info.idealRange);
		minRange = info.minRange;
		maxRange = info.maxRange;
		isMeleeWeapon = info.maxRange < 200;
	}

	MapSector* botSector = get_actor_sector((AActor*)pActor);
	MapSector* targSector = get_actor_sector(targ);
	if (isMeleeWeapon && dist > maxRange && targSector != botSector) {
		pBot->target = NULL;
		return; // ignore enemies not close enough to punch
	}

	bool hasLineOfSight = check_line_of_sight((AActor*)pActor, targ);

	if (!hasLineOfSight) {
		// forget about the target if not seen for a while
		if (get_game_tics() - m_targetLastSeenTic < 35) {
			pBot->target = NULL;
			return;
		}
	}

	m_targetLastSeenTic = get_game_tics();

	// aim at enemy
	pBot->AimAtPos(get_actor_pos(targ) + vec3(0, 0, get_actor_height(targ) / 2));

	pBot->m_forwardMove = 0;
	pBot->m_sideMove = 0;

	// don't get too close/far
	if (dist < minChaseDist) {
		pBot->m_forwardMove = -RUN_SPEED;
	}
	else if (dist > maxChaseDist) {
		pBot->m_forwardMove = RUN_SPEED;
	}

	// randomly strafe around the target
	int r = rand() % 10;
	if (r < 5) {
		pBot->m_sideMove = -RUN_SPEED;
	}
	else {
		pBot->m_sideMove = RUN_SPEED;
	}

	if (hasLineOfSight && dist > minRange && dist < maxRange) {
		pBot->Attack();
	}
}

void CBotCombatController::SelectBestWeapon() {
	AActor* bestWeapon = NULL;
	int bestPriority = -1;
	for (AActor* weapon : get_player_weapons(pBot->pActor, true)) {
		WeaponInfo& info = g_wbot_weapon_info[get_actor_type_name(weapon)];
		int prio = info.priority;
		if (prio > bestPriority) {
			bestPriority = prio;
			bestWeapon = weapon;
		}
	}

	player_select_weapon(pBot->m_pPlayer, bestWeapon);
}

AActor* CBotCombatController::GetWeaponByName(const char* selname) {

	for (AActor* weapon : get_player_weapons(pBot->pActor, true)) {
		if (!strcmp(get_actor_type_name(weapon), selname))
			return weapon;
	}

	return NULL;
}

bool CBotCombatController::SelectWeapon(const char* selname) {
	AActor* weapon = GetWeaponByName(selname);

	if (weapon) {
		player_select_weapon(pBot->m_pPlayer, weapon);
		return true;
	}

	return false;
}

AActor* CBotCombatController::BestEnemy() {
	return find_enemy(pBot);
}

void CBotCombatController::DebugPrint(const char* msg) {
	pBot->DebugPrint(msg);
}