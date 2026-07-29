#include "wb_combat.h"
#include "wb_bot.h"
#include "wb_util.h"
#include "p_local.h"
#include <algorithm>

using namespace std;

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
	: pBot(pBot), pActor(pBot->pActor), pPlayer(pBot->GetPlayer()) {}

void CBotCombatController::Think() {
	AActor* bestEnemy = BestEnemy();

	if (bestEnemy) {
		pActor->target = bestEnemy;
	}

	AActor* targ = pActor->target;

	if (!targ || targ->health <= 0) {
		pActor->target = NULL;
		return;
	}

	SelectBestWeapon();

	fixed_t dist = P_AproxDistance(pActor->x - targ->x, pActor->y - targ->y);
	fixed_t minChaseDist = 200 << FRACBITS;
	fixed_t maxChaseDist = 500 << FRACBITS;
	fixed_t maxRange = 2000 << FRACBITS;
	fixed_t minRange = 0;
	bool isMeleeWeapon = false;

	if (pPlayer->ReadyWeapon) {
		WeaponInfo& info = g_wbot_weapon_info[pPlayer->ReadyWeapon->GetClass()->TypeName.GetChars()];
		minChaseDist = (info.minRange << FRACBITS) + 64;
		maxChaseDist = std::max(minChaseDist, (info.idealRange << FRACBITS));
		minRange = info.minRange << FRACBITS;
		maxRange = info.maxRange << FRACBITS;
		isMeleeWeapon = info.maxRange < 200;
	}

	if (isMeleeWeapon && dist > maxRange && targ->Sector != pActor->Sector) {
		pActor->target = NULL;
		return; // ignore enemies not close enough to punch
	}

	bool hasLineOfSight = P_CheckSight(pActor, targ, SF_SEEPASTSHOOTABLELINES);

	if (!hasLineOfSight) {
		// forget about the target if not seen for a while
		if (level.maptime - m_targetLastSeenTic < 35) {
			pActor->target = NULL;
			return;
		}
	}

	m_targetLastSeenTic = level.maptime;

	// aim at enemy
	pBot->AimAtPos(FVector3(targ->x, targ->y, targ->z + targ->height / 2));

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
	AWeapon* bestWeapon = NULL;
	int bestPriority = -1;
	for (AInventory* item = pActor->Inventory; item != NULL; item = item->Inventory) {
		if (item->IsKindOf(RUNTIME_CLASS(AWeapon))) {
			AWeapon* weapon = static_cast<AWeapon*>(item);

			WeaponInfo& info = g_wbot_weapon_info[weapon->GetClass()->TypeName.GetChars()];
			int prio = info.priority;
			bool hasAmmo = !weapon->Ammo1 || weapon->Ammo1->Amount >= info.minAmmo;
			if (hasAmmo && prio > bestPriority) {
				bestPriority = prio;
				bestWeapon = weapon;
			}
		}
	}

	SelectWeapon(bestWeapon);
}

void CBotCombatController::SelectWeapon(AWeapon* weapon) {
	if (weapon && pPlayer->ReadyWeapon != weapon && pPlayer->PendingWeapon != weapon) {
		pPlayer->PendingWeapon = weapon;
		if (pPlayer->ReadyWeapon != NULL) {
			P_DropWeapon(pPlayer);
		}
		else if (pPlayer->PendingWeapon != WP_NOCHANGE) {
			P_BringUpWeapon(pPlayer);
		}
	}
}

AWeapon* CBotCombatController::GetWeaponByName(const char* selname) {
	for (AInventory* item = pActor->Inventory; item != NULL; item = item->Inventory) {
		if (item->IsKindOf(RUNTIME_CLASS(AWeapon))) {
			AWeapon* weapon = static_cast<AWeapon*>(item);
			const char* wepname = weapon->GetClass()->TypeName.GetChars();

			if (!strcmp(wepname, selname))
				return weapon;
		}
	}

	return NULL;
}

bool CBotCombatController::SelectWeapon(const char* selname) {
	AWeapon* weapon = GetWeaponByName(selname);

	if (weapon) {
		WeaponInfo& info = g_wbot_weapon_info[selname];
		bool hasAmmo = !weapon->Ammo1 || weapon->Ammo1->Amount >= info.minAmmo;
		if (hasAmmo) {
			SelectWeapon(weapon);
			return true;
		}
	}

	return false;
}

AActor* wbot_LookForEnemiesInBlock(AActor* lookee, int index, void* extparam)
{
	FBlockNode* block;
	AActor* link;
	CWootBot* pbot = (CWootBot*)extparam;
	angle_t fov = pbot->m_fov;
	AActor* plr = pbot->pActor;
	AActor* oldTarget = plr->target;
	fixed_t oldDist = oldTarget ? P_AproxDistance(oldTarget->x - plr->x, oldTarget->y - plr->y) : 0;

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

		// only retarget to closer enemies
		if (oldTarget && P_AproxDistance(plr->x - link->x, plr->y - link->y) >= oldDist) {
			continue;
		}

		return link;
	}

	return NULL;
}

AActor* CBotCombatController::BestEnemy() {
	return P_BlockmapSearch(pActor, 10, wbot_LookForEnemiesInBlock, pBot);
}

void CBotCombatController::DebugPrint(const char* msg) {
	pBot->DebugPrint(msg);
}