#pragma once
#include "bots.h"
#include "wb_goal.h"
#include <unordered_set>
#include <vector>

#define FL_SECTOR_MOVE_FLOOR_DOWN	1
#define FL_SECTOR_MOVE_FLOOR_UP		2
#define FL_SECTOR_MOVE_FLOOR_ANY	4
#define FL_SECTOR_MOVE_CEIL_UP		8
#define FL_SECTOR_MOVE_TIMED		16	// sector resets to its old position shortly after being activated

struct LinkSeg {
	fixed_t x1, y1;
	fixed_t x2, y2;
	int otherSub; // other subsector being linked to
	int idx; // index into linked sector's lines

	float length() {
		return (float)(FVector2(x1, y1) - FVector2(x2, y2)).Length();
	}
};

// extra sector metadata for bots
struct BotSectorInfo {
	int moveFlags = 0;
	std::vector<BotGoal> triggers;
};

class BotMapInfo {
public:
	int* line_subsectors = NULL; // maps a linedef to the subsector in front of it
	BotSectorInfo* sector_info = NULL;

	// collect sector/line info from the loaded map
	void init();

	std::vector<int> GetTouchedSubsectors(AActor* actor);
	int get_linedef_move_flag(line_t* line); // FL_SECTOR_MOVE_*
	LinkSeg GetSegmentOverlap(seg_t* a, seg_t* b);
	LinkSeg get_neighbor_subsector(subsector_t* ignoreSector, seg_t* borderSeg);
	bool is_link_bordered_by_walls(subsector_t& sub, int segIdx, int& leftSubId, int& rightSubId);
	bool is_seg_potentially_crossable(seg_t* seg);
	int segment_walkability(seg_t* seg); // returns LinkBlockReason
	int get_linedef_goal_action(line_t* line);
	bool subsector_does_damage(subsector_t* sec);

	// remove trigger goals for single-use lines that have been used
	void remove_invalid_goals(int secid);

private:
	void add_stair_sector_info();
	void find_linedef_sectors();
	void add_sector_info();
};

extern BotMapInfo g_wb_mapinfo;