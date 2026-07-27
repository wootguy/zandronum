#pragma once
#include "bots.h"
#include "wb_goal.h"
#include "wb_nav.h"
#include <unordered_set>
#include <vector>

#define RUN_SPEED 100 // max move speed allowed before the server kicks you

class CWootBot;
struct NavSector;
struct NavSectorLink;

enum BotJumpState {
	WBOT_JUMP_NONE, // not jumping
	WBOT_JUMP_PREP, // carefully move to the starting position for a running start
	WBOT_JUMP_RUN,	// run towards the jumpoff point
	WBOT_JUMP_FLY,	// flying through the air
};

enum BotWalkNodeState {
	WBOT_WALK_NODE_EDGE, // walking to the edge of the next node
	WBOT_WALK_NODE_CENTER,	// walking to the center of the next node
};

class CBotRouteController {
public:
	BotRoute m_route;					// current route
	int m_routeSpeed = 0;				// how fast to run between nodes
	int m_nodeRadius = 0;				// node touch distance
	int m_navid = -1;					// current navigation node/subsector id
	int pretendRouteSector = -1;		// pretend we're in this sector for now
	bool m_freezeOnRouteChange = false; // set to true to stop moving when the route changes. For debugging
	bool m_freezeOnGoalFail = false;	// set to true to stop moving when a goal fails.
	fixed_t m_lastElevZ = 0;			// last height of the elevator we've been standing on
	NavSector* m_navCur = NULL;			// current physical node (or "pretend" node)
	NavSector* m_navIdeal = NULL;		// current node in the route
	NavSector* m_navTarget = NULL;		// next node in the route
	NavSectorLink* m_navLink = NULL;	// link from the ideal nav to the target

	FVector2 jumpBackupPos = FVector2(0, 0);		// position to move to for a running start
	FVector2 jumpStartPos = FVector2(0,0);			// position where the jump begins
	FVector2 jumpEndPos = FVector2(0,0);			// position where the jump ends
	int jumpState = WBOT_JUMP_NONE;
	int walkNodeState = WBOT_WALK_NODE_EDGE;

	CBotRouteController(CWootBot* pBot);

	void Think();

	void CancelRoute();							// abort the current route
	bool RouteToGoal();
	inline bool HasRoute() { return m_route.route.size(); };

	std::unordered_set<int> GetBlockedPaths();	// paths blocked during path to current goal
	BotRoute RouteToSector(int subid, int blockSector=-1);

private:
	CWootBot* pBot = NULL;
	APlayerPawn* pActor = NULL;
	player_t* pPlayer = NULL;

	// think logic
	void UpdateRoute();							// setup and advance route nodes
	void JumpThink();							// execute a jump link
	void MoveThruLink();						// move through walkable route links
	bool HandleBlockedPaths();					// returns true if move should be cancelled
	bool ElevatorThink(bool linkBlocked);		// returns true if the bot is waiting on an elevator
	void RouteSlipThink();						// bot slipped off its route
	bool BeCareful();							// slow down around cliffs, true if should abort movement
	void BlockedPathThink(NavSectorLink* link, int blockReason);	// a path in the route is blocked
	void HandleStuckPath();						// bot failed to move thru a path

	void DebugPrint(const char* msg);
};