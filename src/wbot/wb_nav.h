#pragma once
#include "bots.h"
#include "wb_goal.h"
#include <unordered_map>
#include <unordered_set>
#include <vector>

#define STEP_HEIGHT 24
#define JUMP_HEIGHT 56
#define STAND_HEIGHT 56
#define VIEW_HEIGHT 41
#define DUCK_HEIGHT 28
#define PLAYER_WIDTH 32
#define PLAYER_RADIUS 16
#define SAFE_CLIFF_DIST 80 // don't reduce movement speed when this far away from any cliff
#define JUMP_DIST 200 // max jump distance on flat ground

struct NavSector;

enum LinkBlockReason {
	LINK_BLOCK_CLEAR,		// can walk into the next sector
	LINK_BLOCK_TOO_HIGH,	// too high to jump up to the next sector
	LINK_BLOCK_TOO_LOW,		// next sector doesn't have enough vertical space
	LINK_BLOCK_TOO_NARROW,	// the link doesn't have enough space between walls
	LINK_BLOCK_CLIPPED,		// the player would be clipped inside walls/floor/ceil if moving here
	LINK_BLOCK_CANT_JUMP,	// the jump is too high/far or blocked
};

struct NavSectorLink {
	int id = -1;		// link ID, not associated with any bsp structure
	NavSector* parent = NULL;
	NavSector* target = NULL;
	NavSector* jumpNeighbor = NULL; // node that must be partially run into to begin the jump
	FVector2 overlapCenter;
	int linkWidth = 0;
	bool isTeleport = false;
	bool isCliff = false; // crossing this segs drops you down to a floor so low that you can't get back
	bool isJump = false; // jump required to reach the target sector
	fixed_t jumpDist = 0; // distance to the nearest landing point in the target sector
	seg_t* seg = NULL;

	FVector2 pos();
	FVector3 pos3D();
	fixed_t GetJumpBackupSpaceNeeded(FVector3 start, FVector3 end); // calculate how much of a running start is needed to complete the jump
	FVector2 GetJumpBackupPos(FVector2 targetPos, AActor* jumper); // find the best place to begin running for a jump
	NavSector* GetJumpBackupBlocker(FVector2 targetPos); // get the movable sector that blocks the backup position, if any
	FVector2 GetJumpStartPos(FVector2 targetPos); // find the best place to begin the jump
	FVector2 GetJumpEndPos(FVector2 targetPos); // find the closest point to land
	
	int blocked(AActor* actor, bool recurse=true); // returns LinkBlockReason
	std::vector<sector_t*> getClippedSectors(AActor* actor);
	bool walkable();
	bool jumpable();
	bool isJumpValid(); // checks if jump height and distance is valid currently
};

struct NavSector {
	FVector2 center;
	int id = -1; // also index into nav array
	bool hasCliffs = false; // bot should be careful here
	bool doesDamage = false; // bot should try to route around this
	std::vector<NavSectorLink> links;

	FVector2 pos();
	FVector3 pos3D();

	// get link by id
	NavSectorLink* getLink(int subSectorId);

	bool touches(AActor* actor);

	int getMoveFlags();
	bool isMoving();
	bool isFloorMoving();
	bool isCeilMoving();
	std::vector<BotGoal>& getTriggers();

	sector_t* sector();

	// get vertical space between the floor and ceiling at the center point of the subsector
	fixed_t getHeight();
	fixed_t getFloorZ();
	fixed_t getCeilZ();
};

enum RouteBlockHandling {
	WBOT_ROUTE_BLOCK_IGNORE,	// don't test if routes are blocked (best performance)
	WBOT_ROUTE_BLOCK_EXPENSIVE,	// increase the cost of blocked routes to strongly prefer unblocked routes, but use them anyway if there's no other path
	WBOT_ROUTE_BLOCK_FORBID,	// forbid routing thru a blocked path
};

struct BotRouteLink {
	int node = -1;
	float dist = 0;
	float cost = 0;

	BotRouteLink() {}
	BotRouteLink(int node, int dist, float cost) : node(node), dist(dist), cost(cost) {}
};

struct BotRoute {
	std::vector<int> route;
	float cost = 0;	// cost to run this path (e.g. running thru lava and unblocking paths is more expensive)
	int dist = 0;	// distance in map units
};

struct RouteOpts {
	int start = -1;					// starting subsector id
	int end = -1;					// ending subsector id
	bool timeSensitive = false;		// prefer running thru lava and jumping if the route would be shorter
	int blockedPathHandling = WBOT_ROUTE_BLOCK_IGNORE;
	AActor* actor = NULL;
	std::unordered_set<int> blockedPaths;		// disallowed paths
	std::unordered_set<int> blockedSubSectors;	// disallowd subsectors
};

class SectorNavMesh {
public:
	NavSector* mesh = NULL;
	std::vector<AActor*> propBlockers;
	std::vector<sector_t*> pending_sector_relinks; // sectors that will be relinked as soon as they stop moving

	int pathTests; // number of blocked path checks
	bool verbose;

	void init(); // generate mesh

	void draw_nodes(AActor* actor);

	// set timeSensitive=true if the most direct route should be preferred, even if the bot has
	// to walk through lava or jump somewhere
	BotRoute get_astar_route(const RouteOpts& opts);
	
	// get key(s) required to use the linedef. Returns false if keys don't exist anywhere in the map.
	bool get_key_goals_for_line(AActor* actor, line_t* linedef, std::vector<BotGoal>& keyGoals, std::unordered_set<int>* blockedPaths);

	// find all weapons in the map with the given name
	std::vector<BotGoal> get_weapon_goals(const char* wepname);

	// find all ammo in the map with the given name(s)
	std::vector<BotGoal> get_ammo_goals(const char* ammoname, const char* ammoname2=NULL);

	int get_nav_id(fixed_t x, fixed_t y);
	int get_nav_id(AActor* actor);

	// call to create or remove links on nodes that no longer move
	void relink_sector(sector_t* sec);
	void relink_pending_sector();

private:
	float node_heuristic(int a, int b); // how "close" node a is to b (may not be physical distance)
	float path_dist(NavSectorLink& link);
	float path_cost(NavSectorLink& link, float dist, const RouteOpts& opts);

	bool create_jump_link(NavSector& fromNav, NavSectorLink& fromLink, NavSector& toNav, NavSectorLink& toLink);
};

extern SectorNavMesh g_wb_nav;