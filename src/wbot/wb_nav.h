#pragma once
#include "bots.h"
#include "wb_bot.h"
#include <unordered_map>
#include <unordered_set>

struct NavSector;

struct NavSectorLink {
	int id;		// link ID, not associated with any bsp structure
	NavSector* parent;
	NavSector* target;
	FVector2 overlapCenter;
	int linkWidth;
	int leftSector; // setor on the left side of the target sector, relative to the border segment
	int rightSector; // setor on the right side of the target sector, relative to the border segment
	bool isTeleport;
	bool isCliff; // crossing this segs drops you down to a floor so low that you can't get back
	bool isJump; // jump required to reach the target sector
	seg_t* seg;

	FVector2 pos();
	FVector3 pos3D();
	FVector2 GetJumpBackupPos(); // find the best place to begin running for a jump
	FVector2 GetJumpStartPos(); // find the best place to begin the jump
	
	// true if this link can be moved thru, else it is too high or blocked.
	bool blocked(AActor* actor, bool recurse=true);
	bool walkable();
	bool jumpable();
	bool isJumpHeightValid(); // checks if jump height is valid currently
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
	std::vector<BotGoal>& getTriggers();

	sector_t* sector();

	// get vertical space between the floor and ceiling at the center point of the subsector
	fixed_t getHeight();
	fixed_t getFloorZ();
	fixed_t getCeilZ();
};

class SectorNavMesh {
public:
	NavSector* mesh = NULL;

	int pathTests; // number of blocked path checks
	bool verbose;

	void init(); // generate mesh

	void draw_nodes(AActor* actor);

	// set timeSensitive=true if the most direct route should be preferred, even if the bot has
	// to walk through lava or jump somewhere
	std::vector<int> get_astar_route(int startSubSectorId, int endSubSectorId, std::unordered_set<int>* blockedPaths=NULL, bool timeSensitive=false);
	
	// get key required to use the linedef. Returns false if keys don't exist anywhere in the map.
	bool get_key_goals_for_line(AActor* actor, line_t* linedef, std::vector<BotGoal>& keyGoals, std::unordered_set<int>* blockedPaths);

	int get_nav_id(fixed_t x, fixed_t y);
	int get_nav_id(AActor* actor); // get key required to use the linedef. Returns false if keys don't exist anywhere in the map.
	int get_route_distance(std::vector<int>& route);

private:
	float path_cost(int a, int b);
	float path_cost(NavSectorLink& link, bool timeSensitive);

	bool create_jump_link(NavSector& fromNav, NavSectorLink& fromLink, NavSector& toNav, NavSectorLink& toLink);
};

extern SectorNavMesh g_wb_nav;