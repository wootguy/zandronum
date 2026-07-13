#pragma once
#include "bots.h"

class CWootBot : public CSkullBot {
public:
	int m_forwardMove = 0; // range of +/-100
	int m_sideMove = 0;
	int m_targetLastSeenTic = 0; // last tick the current target was visible
	angle_t m_fov = 0;
	std::vector<int> m_route; // current route
	bool m_debug = false;
	int lastInit = 0; // last time the bot was initialized (for level change detection)

	CWootBot(const char* pszName, const char* pszTeamName, ULONG ulPlayerNum);
	~CWootBot();

	// all thinking logic happens here
	void ParseScript(void) override;

	bool FindMoveGoal(); // find something to move towards in the map
	void FindEnemy();
	void AimAtPos(FVector3 pos);
	bool MoveTo(FVector3 pos, int radius=32, int speed=100);
	
	void DeadThink();	// dead
	void IdleThink();	// nothing to do
	void RouteThink();	// following a route somewhere
	void CombatThink();	// attacking an enemy

	void ShowDebugInfo();

	FVector3 GetViewPos();
};

AActor* getAnyPlayer();