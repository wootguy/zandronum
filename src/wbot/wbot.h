#pragma once
#include "bots.h"

struct FTraceResults;

enum BotGoalAction {
	WBOT_GOAL_ACTION_MOVE_TO,	// move to the goal and do nothing
	WBOT_GOAL_ACTION_USE,		// use the given linedef
};

struct BotGoal {
	int action = -1; // WBOT_GOAL_ACTION_*
	int lineid = -1; // lindef id to interact with
	TObjPtr<AActor> actor = NULL; // actor to interact with

	std::string desc();
	int getNavId();

	bool matches(BotGoal& other) {
		return action == other.action && lineid == other.lineid && actor == other.actor;
	}
};

#define FL_WBOT_WAITING_ELEV 1 // bot is waiting for an elevator to lift/descend before continuing their route
#define FL_WBOT_WAITING_DOOR 2 // bot is waiting for a door or platform to move out of the way before continuing their route

class CWootBot : public CSkullBot {
public:
	int m_forwardMove = 0; // range of +/-100
	int m_sideMove = 0;
	int m_targetLastSeenTic = 0; // last tick the current target was visible
	angle_t m_fov = 0;
	std::vector<int> m_route; // current route
	std::vector<BotGoal> m_goals; // stack of goals
	bool m_debug = false;
	int lastInit = 0; // last time the bot was initialized (for level change detection)
	int pretendRouteSector = -1; // pretend we're in this sector for now
	int stuckCounter = 0; // increases while trying to move with nothing happening
	int stateFlags = 0; // FL_WBOT_*
	FVector2 lastPos;

	CWootBot(const char* pszName, const char* pszTeamName, ULONG ulPlayerNum);
	~CWootBot();

	// all thinking logic happens here
	void ParseScript(void) override;

	bool FindGoal(); // find something to move towards in the map
	void PushGoal(BotGoal& goal);
	void PopGoal();
	void RouteToGoal();
	void FindEnemy();
	void AimAtPos(FVector3 pos);
	bool MoveTo(FVector3 pos, int radius=32, int speed=100);
	
	void DeadThink();	// dead
	void IdleThink();	// nothing to do
	void GoalActionThink(); // do something with the goal object, after routing to it
	void RouteThink();	// following a route somewhere
	void CombatThink();	// attacking an enemy
	bool StuckThink(int maxStuck=1000);	// check if stuck and return true if been stuck long enough to cancel whatever the bot is doing

	bool TraceAhead(int dist, fixed_t height, FTraceResults* tr);

	void CancelRoute();

	void ShowDebugInfo();
	void DebugPrint(const char* msg);

	FVector3 GetViewPos();
};

AActor* getAnyPlayer();