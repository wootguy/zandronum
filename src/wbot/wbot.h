#pragma once
#include "bots.h"
#include <unordered_set>

struct FTraceResults;
struct NavSectorLink;

enum BotGoalAction {
	WBOT_GOAL_ACTION_MOVE_TO,	// move to the goal and do nothing
	WBOT_GOAL_ACTION_USE,		// use the given linedef
	WBOT_GOAL_ACTION_TOUCH,		// touch the given actor
};

struct BotGoal {
	int action = -1; // WBOT_GOAL_ACTION_*
	int lineid = -1; // lindef id to interact with
	std::unordered_set<int> blockers; // path IDs that block A* from reaching routing to this goal
	TObjPtr<AActor> actor = NULL; // actor to interact with

	BotGoal(int action, int lineid) : action(action), lineid(lineid) {}
	BotGoal(int action, AActor* actor) : action(action), actor(actor) {}

	std::string desc();
	int getNavId();
	FVector3 pos();
	int touchDistance(AActor* toucher); // how close the player needs to be to consider this goal as touched

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
	int m_navid; // current navigation node/subsector id
	FVector2 lastPos;

	CWootBot(const char* pszName, const char* pszTeamName, ULONG ulPlayerNum);
	~CWootBot();

	// all thinking logic happens here
	void ParseScript(void) override;

	bool FindGoal(); // find something to do in the map
	bool PushGoal(BotGoal& goal); // returns false if this is already the current goal
	void PopGoal();
	void RouteToGoal();
	void FindEnemy();
	void AimAtPos(FVector3 pos);
	bool MoveTo(FVector3 pos, int radius=32, int speed=100);
	
	void DeadThink();	// dead
	void IdleThink();	// nothing to do
	void GoalActionThink(); // do something with the goal object, after routing to it
	void RouteThink();	// following a route somewhere
	void BlockedPathThink(NavSectorLink* link); // a path in the route is blocked
	void CombatThink();	// attacking an enemy
	bool StuckThink(int maxStuck=1000);	// true if stuck longer than the given time

	bool TraceAhead(int dist, FVector3 offset, bool ignoreMonsters, FTraceResults* tr);

	void CancelRoute();

	void ShowDebugInfo();
	void DebugPrint(const char* msg);

	FVector3 GetViewPos();
};

AActor* getAnyPlayer();