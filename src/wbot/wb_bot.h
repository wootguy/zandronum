#pragma once
#include "bots.h"
#include "wb_route.h"
#include "wb_combat.h"
#include "wb_goal.h"
#include <unordered_set>

struct FTraceResults;
struct NavSectorLink;

#define FL_WBOT_WAIT_ELEV	(1<<0) // bot is waiting for an elevator to lift/descend
#define FL_WBOT_WAIT_DOOR	(1<<1) // bot is waiting for a door or platform to move out of the way
#define FL_WBOT_FLYING		(1<<2) // bot is jumping, falling, or above a gap in the floor
#define FL_WBOT_RUSHING		(1<<3) // bot is hurrying to reach a sector before a timed event ends
#define FL_WBOT_ON_ELEV		(1<<4) // bot is standing on a platform that is moving or about to move
#define FL_WBOT_SLOW_DOWN	(1<<5) // bot is trying to reduce their speed

class CWootBot : public CSkullBot {
public:
	APlayerPawn* pActor = NULL;
	std::vector<BotGoal> m_goals;	// stack of goals
	int m_forwardMove = 0;		// range of +/-100
	int m_sideMove = 0;			// range of +/-100
	int stateFlags = 0;			// FL_WBOT_*
	int m_lastUse;				// last tic the player used (for preventing sound spam)
	int m_nextThink;			// for cooling down failures
	angle_t m_fov = 0;			// field of view
	int stuckCounter = 0;				// increases while trying to move with nothing happening
	int m_cliffDist = 9999;
	FVector2 lastPos = FVector2(0, 0);	// used to detect being stuck
	bool m_wasDead = false;
	int rushNav = -1;				// subsector/nav ID that the bot is rushing for
	BotGoal rushTrigger;		// goal which caused the bot to start rushing
	bool m_followPlayer = false; // set true to follow player automatically

	// debug state
	bool m_debug = false;		// print thoughts to chat
	float m_speedMult = 1.0f;	// scale movement speed

	CBotRouteController m_routeController;
	CBotCombatController m_combatController;

	CWootBot(const char* pszName, const char* pszTeamName, ULONG ulPlayerNum);
	~CWootBot() {}

	void ParseScript(void) override { Think(); } // called by skullbot tick

	void Reset(); // clear all memory and restart the bot

	// goal management
	bool FindGoal(); // find something to do in the map
	bool PushLevelEndGoal(); // try to beat the map
	bool PushGoal(BotGoal& goal, NavSectorLink* purposeLink); // returns false if this is already the current goal
	bool PushKeyGoals(line_t* line);
	bool SelectGoal(std::vector<BotGoal>& goals, NavSectorLink* purposeLink);
	void CompleteGoal();
	void FailGoal(); // aborts the current goal and moves its blocked paths to the parent goal
	inline bool HasGoal() { return m_goals.size(); };
	inline BotGoal* CurrentGoal() { return m_goals.size() ? &m_goals[m_goals.size() - 1] : NULL; }
	
	void AimAtPos(FVector3 pos);
	bool MoveTo(FVector2 pos, int radius=32, int speed=100);
	bool StopMoving(); // try to stop movement. Returns true if stationary.
	FVector2 AvoidCornersVector(FVector2 wantDir); // direction to move to avoid hitting corners
	FVector2 AvoidLedges(AActor* actor, int& cliffDist); // direction to move to avoid falling off a ledge
	void UpdatePositionFlags();

	void Think();
	void DeadThink();	// dead
	void IdleThink();	// nothing to do	
	bool StuckThink(int maxStuck=1000);	// true if stuck longer than the given time
	bool HandleStuckPath();
	void GoalActionThink(); // do something with the goal object, after routing to it

	bool TraceAhead(int dist, FVector3 offset, bool ignoreMonsters, FTraceResults* tr);

	void DebugPrint(const char* msg);

	void Use(int ticsBetweenUses=7); // anti-spam use pressing
	void Attack();

	FVector3 GetViewPos();
	fixed_t GetDistance(FVector2 p);
	FVector3 GetVelocity();
	int GetSpeed2D();

	// something somewhere triggered a line
	void HandleLineActivation(line_t* line, AActor* activator);
};
