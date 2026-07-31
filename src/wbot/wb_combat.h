#pragma once
#include <unordered_map>
#include <string>

class CWootBot;
class APlayerPawn;
class player_t;
class AWeapon;
class AActor;

struct WeaponInfo {
	int priority;
	int minRange;
	int idealRange;
	int maxRange;
	int minAmmo;
};

extern std::unordered_map<std::string, WeaponInfo> g_wbot_weapon_info;


class CBotCombatController {
public:
	int m_targetLastSeenTic = 0;	// last tick the current target was visible
	int m_lastAttack;				// last tic the player attacked

	CBotCombatController(CWootBot* pBot);

	void Think();

	AActor* GetWeaponByName(const char* name);
	bool SelectWeapon(const char* name); // false if not in inventory or no ammo

private:
	CWootBot* pBot = NULL;
	APlayerPawn* pActor = NULL;
	player_t* pPlayer = NULL;

	void SelectBestWeapon();
	AActor* BestEnemy();

	void DebugPrint(const char* msg);
};