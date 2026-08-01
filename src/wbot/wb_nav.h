#pragma once
#include "wb_goal.h"
#include "wb_map.h"
#include <vector>

#define STEP_HEIGHT 24
#define JUMP_HEIGHT 56
#define STAND_HEIGHT 56
#define VIEW_HEIGHT 41
#define DUCK_HEIGHT 28
#define PLAYER_WIDTH 32
#define PLAYER_RADIUS 16
#define PLAYER_USE_DIST 64
#define SAFE_CLIFF_DIST 80 // don't reduce movement speed when this far away from any cliff
#define JUMP_DIST 200 // max jump distance on flat ground

#define MAX_MESH_LINKS 8192

struct NavSector;
struct wbot::MapSeg;
struct wbot::MapSector;
class player_t;

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
	wbot::MapSector* sector = NULL; // sector that the bot must enter to cross this link (may be different than the target sector in the case of teleports)
	vec2 movePos; // position to move to for crossing the link
	int linkWidth = 0;
	bool isTeleport = false;
	bool isCliff = false; // crossing this segs drops you down to a floor so low that you can't get back
	bool isJump = false; // jump required to reach the target sector
	float jumpDist = 0; // distance to the nearest landing point in the target sector
	wbot::MapLine* linedef = NULL; // linedef this link crosses
	wbot::FSegment2 seg; // overlapping region between the source and target nodes
	
	int routeNumIgnore; // ignore this path for the next route if set to the current route call num

	vec2 pos();
	vec3 pos3D();
	float GetJumpBackupSpaceNeeded(vec3 start, vec3 end); // calculate how much of a running start is needed to complete the jump
	vec2 GetJumpBackupPos(vec2 targetPos, AActor* jumper); // find the best place to begin running for a jump
	NavSector* GetJumpBackupBlocker(vec2 targetPos); // get the movable sector that blocks the backup position, if any
	vec2 GetJumpStartPos(vec2 targetPos); // find the best place to begin the jump
	vec2 GetJumpEndPos(vec2 targetPos); // find the closest point to land

	int blocked(AActor* actor, bool recurse=true); // returns LinkBlockReason
	std::vector<wbot::MapSector*> getClippedSectors(AActor* actor);
	bool walkable();
	bool jumpable();
	bool isJumpValid(); // checks if jump height and distance is valid currently
	void updateFlags();
};

struct NavSector {
	vec2 center;
	int id = -1; // also index into nav array
	bool hasCliffs = false; // bot should be careful here
	bool doesDamage = false; // bot should try to route around this
	std::vector<NavSectorLink*> links;
	wbot::MapSector* sector;

	int routeNumIgnore; // ignore this node for the next route if set to the current route call num

	vec2 pos();
	vec3 pos3D();

	// get link by id
	NavSectorLink* getLink(int subSectorId);

	bool touches(AActor* actor);
};

struct BotMeshData {
	NavSector* nodes;
	NavSectorLink* links;
	int numLinks;

	BotMeshData() {}
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
};

struct AstarNode {
	float gScore;
	float fScore;
	float cameFromCost;

	bool closed;
	bool pathed;
	uint16_t cameFromNode;
	uint16_t cameFromDist;
};

extern int g_route_ignore_num; // incremented every time a route is calculated

class SectorNavMesh {
public:
	BotMeshData mesh;
	std::vector<AActor*> propBlockers;
	std::vector<wbot::MapSector*> pending_sector_relinks; // sectors that will be relinked as soon as they stop moving
	AstarNode* astarNodes; // temp data for astar routing

	int pathTests; // number of blocked path checks
	bool verbose;

	void init(); // generate mesh

	// set timeSensitive=true if the most direct route should be preferred, even if the bot has
	// to walk through lava or jump somewhere
	BotRoute get_astar_route(const RouteOpts& opts);
	
	// get key(s) required to use the linedef. Returns false if keys don't exist anywhere in the map.
	bool get_key_goals_for_line(AActor* actor, wbot::MapLine* linedef, std::vector<BotGoal>& keyGoals);

	// find all weapons in the map with the given name
	std::vector<BotGoal> get_weapon_goals(const char* wepname);

	// find all ammo in the map with the given name(s)
	std::vector<BotGoal> get_ammo_goals(const char* ammoname, const char* ammoname2=NULL);

	int get_nav_id(vec2 pos);
	int get_nav_id(AActor* actor); // gets the node directly below the actor center
	int get_nav_id(player_t* plr); // gets the node the player is standing on, which may be uncentered

	// call to create or remove links on nodes that no longer move
	void relink_sector(wbot::MapSector* sec);
	void relink_pending_sector();

	float node_heuristic(int a, int b); // how "close" node a is to b (may not be physical distance)
	float path_dist(NavSectorLink& link);
	float path_cost(NavSectorLink& link, float dist, const RouteOpts& opts);
};

extern SectorNavMesh g_wb_nav;