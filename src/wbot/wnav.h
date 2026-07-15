#pragma once
#include "bots.h"
#include "wbot.h"
#include <unordered_map>
#include <unordered_set>

#define STEP_HEIGHT 24
#define JUMP_HEIGHT 48
#define STAND_HEIGHT 56
#define DUCK_HEIGHT 28
#define PLAYER_WIDTH 32

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
	seg_t* seg;

	FVector3 pos() {
		return overlapCenter;
	}
	
	// true if this link can be moved thru, else it is too high or blocked.
	bool blocked(bool recurse=true);
};

struct TagTriggerGoal {
	bool canTrigger = false;
	BotGoal goal;
};

struct NavSector {
	int x, y, z; // center point of the sector
	int id; // also index into nav array
	std::vector<BotGoal> triggers; // things which can trigger this sector to move up/down
	std::vector<NavSectorLink> links;

	FVector3 pos() {
		return FVector3(x << FRACBITS, y << FRACBITS, z << FRACBITS);
	}

	// get link by id
	NavSectorLink* getLink(int subSectorId);

	// get vertical space between the floor and ceiling at the center point of the subsector
	fixed_t getHeight();
	fixed_t getFloorZ();
	fixed_t getCeilZ();
};

class SectorNavMesh {
	friend class NavSector;
	friend class NavSectorLink;

public:
	std::vector<NavSector> nav_sectors;
	std::unordered_map<int, int> line_subsectors; // maps a linedef to the subsector in front of it

	int pathTests; // number of blocked path checks
	bool verbose;

	void generate_node_graph();
	void draw_nodes(AActor* actor);
	std::vector<int> get_astar_route(int startSubSectorId, int endSubSectorId, std::unordered_set<int>* blockedPaths=NULL);
	int get_nav_id(fixed_t x, fixed_t y);
	int get_nav_id(AActor* actor);

private:
	int draw_debug_line(FVector3 start, FVector3 end, AActor* actor); // returns number of sprites drawn
	LinkSeg GetSegmentOverlap(seg_t* a, seg_t* b);
	bool is_seg_potentially_crossable(seg_t* seg);
	float path_cost(int a, int b);
	LinkSeg get_neighbor_subsector(subsector_t* ignoreSector, seg_t* borderSeg);

	bool can_cross_seg_now(seg_t* seg);
	bool does_linedef_move_tag(line_t* line, short tag); // true if this linedef moves the given sector tag
};

extern SectorNavMesh g_wbot_nav;