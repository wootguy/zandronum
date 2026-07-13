#pragma once
#include "bots.h"

#define STEP_HEIGHT 24
#define JUMP_HEIGHT 48

struct LinkSeg
{
	fixed_t x1, y1;
	fixed_t x2, y2;

	float length() {
		return (FVector2(x1, y1) - FVector2(x2, y2)).Length();
	}
};

struct NavSectorLink {
	int target; // subsector ID
	LinkSeg overlap; // overlapping region of the segments joining these sectors
	FVector3 overlapCenter;

	FVector3 pos() {
		return overlapCenter;
	}
};

struct NavSector {
	int x, y, z; // center point of the sector
	int id; // also index into nav array
	std::vector<NavSectorLink> links;

	FVector3 pos() {
		return FVector3(x << FRACBITS, y << FRACBITS, z << FRACBITS);
	}

	// get movement goal to reach a linked subsector
	FVector3 getLinkPos(int subSectorId);
};

class SectorNavMesh {
public:
	std::vector<NavSector> nav_sectors;
	void generate_node_graph();
	void draw_nodes(AActor* actor);
	std::vector<int> get_astar_route(int startSubSectorId, int endSubSectorId);
	int get_nav_id(fixed_t x, fixed_t y);
	int get_nav_id(AActor* actor);

private:
	void draw_debug_line(FVector3 start, FVector3 end, AActor* actor);
	LinkSeg GetSegmentOverlap(seg_t* a, seg_t* b);
	bool can_cross_seg(seg_t* seg);
	float path_cost(int a, int b);
};

extern SectorNavMesh g_wbot_nav;