#pragma once
#include "bots.h"
#include "wb_goal.h"
#include <unordered_set>
#include <vector>

#define RUN_SPEED 100 // max move speed allowed before the server kicks you

class CWootBot;
struct NavSector;
struct NavSectorLink;


class CBotRouteController {
public:
	std::vector<int> m_route;			// current route
	int m_routeSpeed = 0;				// how fast to run between nodes
	int m_nodeRadius = 0;				// node touch distance
	int m_navid = -1;					// current navigation node/subsector id
	int pretendRouteSector = -1;		// pretend we're in this sector for now
	int stuckPath = -1;					// path the bot got stuck at
	bool m_freezeOnRouteChange = false; // set to true to stop moving when the route changes. For debugging
	fixed_t m_lastElevZ = 0;			// last height of the elevator we've been standing on
	NavSector* m_navCur = NULL;			// current physical node (or "pretend" node)
	NavSector* m_navIdeal = NULL;		// current node in the route
	NavSector* m_navTarget = NULL;		// next node in the route
	NavSectorLink* m_navLink = NULL;	// link from the ideal nav to the target

	CBotRouteController(CWootBot* pBot);

	void Think();

	void CancelRoute();							// abort the current route
	bool RouteToGoal();
	inline bool HasRoute() { return m_route.size(); };

	std::unordered_set<int> GetBlockedPaths();	// paths blocked during path to current goal
	std::vector<int> RouteToSector(int subid);

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
	void BlockedPathThink(NavSectorLink* link);	// a path in the route is blocked

	void DebugPrint(const char* msg);
};