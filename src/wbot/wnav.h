#pragma once
#include "bots.h"
#include "wbot.h"
#include <unordered_map>
#include <unordered_set>

#define STEP_HEIGHT 24
#define JUMP_HEIGHT 48

struct LinkSeg {
	fixed_t x1, y1;
	fixed_t x2, y2;

	float length() {
		return (float)(FVector2(x1, y1) - FVector2(x2, y2)).Length();
	}
};

struct NavSectorLink {
	int target; // subsector ID
	int id;		// link ID, not associated with any bsp structure
	LinkSeg overlap; // overlapping region of the segments joining these sectors
	FVector3 overlapCenter;
	seg_t* seg;

	FVector3 pos() {
		return overlapCenter;
	}
	
	// true if this link can be moved thru, else it is too high or blocked.
	// unblocker is filled if something can be done by the actor to unblock the link
	bool blocked(AActor* actor, BotGoal* unblockGoal);
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
};

class SectorNavMesh {
public:
	std::vector<NavSector> nav_sectors;
	std::unordered_map<int, int> line_subsectors; // maps a linedef to the subsector in front of it
	std::unordered_set<int> blocked_link_cache; // prevents infinite loops when creating subgoals (e.g to unlock this door you must first open this door then press the button beyond it! oh wait...)

	void generate_node_graph();
	void draw_nodes(AActor* actor);
	std::vector<int> get_astar_route(AActor* actor, int startSubSectorId, int endSubSectorId);
	int get_nav_id(fixed_t x, fixed_t y);
	int get_nav_id(AActor* actor);
	bool can_cross_seg_now(seg_t* seg);
	bool does_linedef_move_tag(line_t* line, short tag); // true if this linedef moves the given sector tag

private:
	void draw_debug_line(FVector3 start, FVector3 end, AActor* actor);
	LinkSeg GetSegmentOverlap(seg_t* a, seg_t* b);
	bool is_seg_potentially_crossable(seg_t* seg);
	float path_cost(int a, int b);
	subsector_t* get_neighbor_subsector(subsector_t* ignoreSector, seg_t* borderSeg, LinkSeg& linkSeg);
};

extern SectorNavMesh g_wbot_nav;