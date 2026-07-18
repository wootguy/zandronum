#pragma once
#include "bots.h"
#include "wbot.h"
#include <unordered_map>
#include <unordered_set>

#define STEP_HEIGHT 24
#define JUMP_HEIGHT 56
#define STAND_HEIGHT 56
#define DUCK_HEIGHT 28
#define PLAYER_WIDTH 32
#define PLAYER_RADIUS 16
#define SAFE_CLIFF_DIST 80 // don't reduce movement speed when this far away from any cliff

#define FL_SECTOR_MOVE_FLOOR_DOWN	1
#define FL_SECTOR_MOVE_FLOOR_UP		2
#define FL_SECTOR_MOVE_FLOOR_ANY	4
#define FL_SECTOR_MOVE_CEIL_UP		8

struct NavSector;

struct LinkSeg {
	fixed_t x1, y1;
	fixed_t x2, y2;
	int otherSub; // other subsector being linked to
	int idx; // index into linked sector's lines

	float length() {
		return (float)(FVector2(x1, y1) - FVector2(x2, y2)).Length();
	}
};

struct NavSectorLink {
	int target; // subsector ID
	int id;		// link ID, not associated with any bsp structure
	int parent; // parent nav id
	FVector3 overlapCenter;
	int linkWidth;
	int leftSector; // setor on the left side of the target sector, relative to the border segment
	int rightSector; // setor on the right side of the target sector, relative to the border segment
	bool isTeleport;
	bool isCliff; // crossing this segs drops you down to a floor so low that you can't get back
	bool isJump; // jump required to reach the target sector
	seg_t* seg;

	FVector3 pos() {
		return overlapCenter;
	}
	
	// true if this link can be moved thru, else it is too high or blocked.
	bool blocked(AActor* actor, bool recurse=true);
	bool walkable();
	bool isJumpHeightValid(); // checks if jump height is valid currently

	NavSector* getParent();
	NavSector* getTarget();
};

struct NavSector {
	int x, y, z; // center point of the sector
	int id = -1; // also index into nav array
	bool hasCliffs = false; // bot should be careful here
	bool doesDamage = false; // bot should try to route around this
	std::vector<BotGoal> triggers; // things which can trigger this sector to move up/down
	std::vector<NavSectorLink> links;

	FVector3 pos() {
		return FVector3(x << FRACBITS, y << FRACBITS, z << FRACBITS);
	}

	// get link by id
	NavSectorLink* getLink(int subSectorId);

	bool touches(AActor* actor);

	int getMoveFlags();

	sector_t* sector();

	// get vertical space between the floor and ceiling at the center point of the subsector
	fixed_t getHeight();
	fixed_t getFloorZ();
	fixed_t getCeilZ();
};

class SectorNavMesh {
	friend class NavSector;
	friend class NavSectorLink;

public:
	NavSector* nav_sectors = NULL;
	int* line_subsectors = NULL; // maps a linedef to the subsector in front of it
	int* sector_move_flags = NULL;

	int pathTests; // number of blocked path checks
	bool verbose;

	void generate_node_graph();
	void draw_nodes(AActor* actor);
	std::vector<int> get_astar_route(int startSubSectorId, int endSubSectorId, std::unordered_set<int>* blockedPaths=NULL);
	int get_nav_id(fixed_t x, fixed_t y);
	int get_nav_id(AActor* actor);
	bool get_key_goals_for_line(AActor* actor, line_t* linedef, std::vector<BotGoal>& keyGoals, std::unordered_set<int>* blockedPaths); // get key required to use the linedef. Returns false if keys don't exist anywhere in the map.
	std::vector<int> GetTouchedSubsectors(AActor* actor);
	int get_route_distance(std::vector<int>& route);

private:
	LinkSeg GetSegmentOverlap(seg_t* a, seg_t* b);
	bool is_seg_potentially_crossable(seg_t* seg);
	float path_cost(int a, int b);
	float path_cost(NavSectorLink& link);
	LinkSeg get_neighbor_subsector(subsector_t* ignoreSector, seg_t* borderSeg);

	void add_stair_sector_move_flags();
	void find_linedef_sectors();
	void add_sector_move_flags();
	void calc_nav_centers();
	void add_sector_trigger_goals();
	void add_jump_links();
	bool is_link_bordered_by_walls(subsector_t& sub, int segIdx, int& leftSubId, int& rightSubId);
	bool can_cross_seg_now(seg_t* seg);
	int get_linedef_move_flag(line_t* line); // returns SectorMoveMode
	int get_linedef_goal_action(line_t* line);
	bool subsector_does_damage(subsector_t* sec);
	bool create_jump_link(NavSector& fromNav, NavSectorLink& fromLink, NavSector& toNav, NavSectorLink& toLink);
};

extern SectorNavMesh g_wbot_nav;