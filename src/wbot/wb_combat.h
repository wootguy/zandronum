#pragma once
#include "bots.h"
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <string>

class CWootBot;

struct WeaponInfo {
	int priority;
	int minRange;
	int idealRange;
	int maxRange;
};

extern std::unordered_map<std::string, WeaponInfo> g_wbot_weapon_info;


class CBotCombatController {
public:
	int m_targetLastSeenTic = 0;	// last tick the current target was visible
	int m_lastAttack;				// last tic the player attacked

	CBotCombatController(CWootBot* pBot);

	void Think();

private:
	CWootBot* pBot = NULL;
	APlayerPawn* pActor = NULL;
	player_t* pPlayer = NULL;

	void SelectBestWeapon();
	AActor* BestEnemy();

	void DebugPrint(const char* msg);
};