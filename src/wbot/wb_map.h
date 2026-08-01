#pragma once
#include "wb_goal.h"
#include <unordered_set>
#include <vector>

struct LumpSubSector;
struct LumpSeg;
struct MapData;

namespace wbot {
	struct MapSubsector;

	struct FSegment2 {
		vec2 a = vec2(0, 0);
		vec2 b = vec2(0, 0);
		inline float length() { return (b - a).length(); }
		inline vec2 center() { return (a + b) * 0.5f; }
		inline vec2 normal() { vec2 dir = (b - a).normalize(); return vec2(dir.y, -dir.x); }
	};

	struct MapNode {
		int16_t x, y;
		int16_t dx, dy;
		int16_t bbox[2][4];
		uint16_t children[2];
	};

	struct MapSector {
		int id; // index into sectors array
		int16_t tag;

		int moveFlags = 0;
		std::vector<BotGoal> triggers;
		std::vector<MapSubsector*> subsectors;

		float getHeight();
		float getFloorZ();
		float getCeilZ();
		bool isMoving();
		bool isFloorMoving();
		bool isCeilMoving();
		int special();
	};

	struct MapLine {
		int id;
		vec2 v1, v2;
		uint16_t tag;
		uint16_t flags;
		MapSector* frontsector;
		MapSector* backsector;

		vec2 dir() { return (v2 - v1).normalize(); }
		vec2 center();
		vec2 normal();
		int length();
		int special();
		int getArg(int idx);
		bool isTeleport();
		bool isLockedDoor();
		bool isLevelExit();
		bool isImpassable();
		bool canPlayerActivate();
		vec2 getTeleportDest();
	};

	struct MapSeg {
		vec2 v1, v2; // may be off grid for implicit segs
		std::vector<MapLine*> lines; // linedefs that this segment overlaps

		float length();
		vec2 center();
		vec2 normal();
	};

	struct MapSubsector {
		int id; // index into subsectors array
		int firstseg = 0;
		int numsegs = 0;
		MapSector* sector = NULL;
		vec2 mins, maxs; // bounding box
	};

	struct LinkSeg {
		FSegment2 overlap;
		MapLine* line = NULL;
		int otherSub = -1; // other subsector being linked to
	};

	struct BspClip {
		vec2 linePoint;
		vec2 lineDir;
		bool front;
	};

	// query map data in an engine-agnostic way
	class BotMapInfo {
	public:
		int* line_subsectors = NULL; // maps a linedef to the subsector in front of it

		MapLine* lines = NULL;
		int numlines = 0;

		MapSeg* segs = NULL;
		int numsegs = 0;
		
		MapSector* sectors = NULL;
		int numsectors = 0;

		MapSubsector* subsectors = NULL;
		int numsubsectors = 0;

		MapNode* nodes = NULL;
		int numnodes = 0;

		// collect sector/line info from the loaded map
		void init();

		// load lumps into engine-agnostic structs
		void load_lumps();

		// create the implicit segs for subsectors
		void build_subsectors_recurse(int nodeid, std::vector<BspClip>& clips,
			std::vector<MapSeg>& totalSegs, MapLumps& lumps);

		void build_subsector(int subid, std::vector<BspClip>& clips,
			std::vector<MapSeg>& totalSegs, MapLumps& lumps);

		// get subsector for position
		MapSubsector* GetSubsector(int x, int y);

		// get sector for position
		MapSector* GetSector(int x, int y);

		MapSector* GetSector(AActor* actor);

		// find all subsectors within the actor's radius
		std::vector<int> GetTouchedSubsectors(AActor* actor);

		// get subsector touching the the given borderSeg
		std::vector<LinkSeg> get_neighbor_subsectors(MapSubsector* rootSub, MapSeg* borderSeg, std::unordered_set<MapSector*>& checkSectors);

		// true if the target sector can ever be reached fromt the source
		bool is_sector_border_potentially_crossable(MapSector* from, MapSector* to);

		// returns LinkBlockReason if the target sector can't be entered from the source
		int sector_border_walkability(MapSector* from, MapSector* to);

		// true if lava/slime
		bool subsector_does_damage(MapSubsector* sec);

		// remove trigger goals for single-use lines that have been used
		void remove_invalid_goals(int secid);

	private:
		void find_linedef_sectors();
		void add_sector_info();
		int PointOnNodeSide(int32_t x, int32_t y, const MapNode* node);
	};

	extern BotMapInfo g_map;
};