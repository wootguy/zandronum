#include "wb_map.h"
#include "wb_util.h"
#include "wb_nav.h"

#include <algorithm>
#include <float.h>
#include <cstring>

using namespace std;
using namespace wbot;

BotMapInfo wbot::g_map;

float MapSector::getHeight() {
	return get_sector_ceil_z(id) - get_sector_floor_z(id);
}

float MapSector::getFloorZ() {
	return get_sector_floor_z(id);
}

float MapSector::getCeilZ() {
	return get_sector_ceil_z(id);
}

bool MapSector::isMoving() {
	return is_sector_ceil_moving(id) || is_sector_floor_moving(id);
}

bool MapSector::isFloorMoving() {
	return is_sector_floor_moving(id);
}

bool MapSector::isCeilMoving() {
	return is_sector_ceil_moving(id);
}

int MapSector::special() {
	return get_sector_special(id);
}

vec2 MapLine::center() {
	return (v1 + v2) * 0.5f;
}

vec2 MapLine::normal() {
	vec2 dir = (v2 - v1).normalize();
	return vec2(dir.y, -dir.x);
}


int MapLine::length() {
	return (v2 - v1).length();
}

int MapLine::activation() {
	return get_line_activation(id);
}

int MapLine::special() {
	return get_line_special(id);
}

int MapLine::getArg(int idx) {
	return get_line_arg(this - g_map.lines, idx);
}

bool MapLine::isTeleport() {
	return special_is_teleport(special());
}

bool MapLine::isLockedDoor() {
	return special_is_locked_door(special());
}

bool MapLine::isLevelExit() {
	return special_is_level_exit(special());
}

bool MapLine::isImpassable() {
	return is_impassable_line(id);
}

bool MapLine::canPlayerActivate() {
	return can_player_activate_line(id);
}

vec2 MapLine::getTeleportDest() {
	return get_tele_dest(id);
}


vec2 MapSeg::center() {
	return (v1 + v2) * 0.5f;
}

vec2 MapSeg::normal() {
	vec2 dir = (v2 - v1).normalize();
	return vec2(dir.y, -dir.x);
}

float MapSeg::length() {
	return (v2 - v1).length();
}

void BotMapInfo::init() {
	if (line_subsectors) {
		delete[] line_subsectors;
		line_subsectors = NULL;
	}

	load_lumps();

	line_subsectors = new int[numlines];
	memset(line_subsectors, -1, sizeof(int) * numlines);

	find_linedef_sectors();
	add_sector_info();
}

void BotMapInfo::load_lumps() {
	if (nodes) {
		delete[] nodes;
		delete[] segs;
		delete[] subsectors;
		delete[] sectors;
		delete[] lines;
		nodes = NULL;
		segs = NULL;
		subsectors = NULL;
		sectors = NULL;
		lines = NULL;
	}

	MapLumps lumps = load_wad_lump_data();

	if (!lumps.numsubsectors) {
		gprintf("[wbot] failed to load map data\n");
		return;
	}

	for (int i = 0; i < lumps.numsides; i++) {
		if (lumps.sides[i].sector >= lumps.numsectors) {
			gprintf("[wbot] Bad lump side %d\n", i);
			return;
		}
	}

	// sectors, lines, and nodes are copied from lumps as is
	{
		numsectors = lumps.numsectors;
		sectors = new MapSector[numsectors];
		for (int i = 0; i < numsectors; i++) {
			MapSector& dst = sectors[i];
			LumpSector& src = lumps.sectors[i];

			dst.id = i;
			dst.tag = src.tag;
		}

		numlines = lumps.numlines;
		lines = new MapLine[numlines];
		for (int i = 0; i < numlines; i++) {
			MapLine& dst = lines[i];
			LumpLine& src = lumps.lines[i];
			int16_t frontside = src.sidenum[0];
			int16_t backside = src.sidenum[1];

			if (src.v1 >= lumps.numverts || src.v2 >= lumps.numverts)
				continue;
			if (frontside >= lumps.numsides || backside >= lumps.numsides)
				continue;

			dst.id = i;
			dst.v1 = vec2(lumps.verts[src.v1].x, lumps.verts[src.v1].y);
			dst.v2 = vec2(lumps.verts[src.v2].x, lumps.verts[src.v2].y);
			dst.tag = src.tag;
			dst.flags = src.flags;
			dst.frontsector = frontside >= 0 ? &sectors[lumps.sides[frontside].sector] : NULL;
			dst.backsector = backside >= 0 ? &sectors[lumps.sides[backside].sector] : NULL;
		}

		numnodes = lumps.numnodes;
		nodes = new MapNode[numnodes];
		for (int i = 0; i < numnodes; i++) {
			MapNode& dst = nodes[i];
			LumpNode& src = lumps.nodes[i];

			dst.x = src.x;
			dst.y = src.y;
			dst.dx = src.dx;
			dst.dy = src.dy;
			for (int k = 0; k < 2; k++) {
				for (int j = 0; j < 4; j++) {
					dst.bbox[k][j] = src.bbox[k][j];
				}
				dst.children[k] = src.children[k];
			}
		}
	}

	// build subsector boundaries from nodes and segments
	// the segments stored in the wad don't often create a closed polygon by themselves
	numsubsectors = lumps.numsubsectors;
	subsectors = new MapSubsector[numsubsectors];
	for (int i = 0; i < numsubsectors; i++) {
		LumpSeg& seg = lumps.segs[lumps.subsectors[i].firstseg];
		int16_t side = lumps.lines[seg.linedef].sidenum[seg.side];
		subsectors[i].id = i;
		subsectors[i].sector = &sectors[lumps.sides[side].sector];
		subsectors[i].sector->subsectors.push_back(&subsectors[i]);
	}

	std::vector<MapSeg> totalSegs;
	std::vector<BspClip> clips;
	build_subsectors_recurse(numnodes - 1, clips, totalSegs, lumps);

	segs = new MapSeg[totalSegs.size()];
	numsegs = totalSegs.size();
	for (int i = 0; i < numsegs; i++)
		segs[i] = totalSegs[i];

	delete[] lumps.lines;
	delete[] lumps.nodes;
	delete[] lumps.sectors;
	delete[] lumps.segs;
	delete[] lumps.sides;
	delete[] lumps.subsectors;
	delete[] lumps.verts;
}

int BotMapInfo::PointOnNodeSide(int32_t x, int32_t y, const MapNode* node) {
	return ((y - (int32_t)node->y) * node->dx + ((int32_t)node->x - x) * node->dy) > 0;
}

void BotMapInfo::build_subsectors_recurse(int nodeid, std::vector<BspClip>& clips,
	std::vector<MapSeg>& totalSegs, MapLumps& lumps) {
	MapNode& node = nodes[nodeid];

	for (int i = 0; i < 2; i++) {
		clips.push_back({ vec2(node.x, node.y), vec2(node.dx, node.dy), i == 1});

		uint16_t child = node.children[i];

		if (child & 0x8000) {
			build_subsector(child & 0x7FFF, clips, totalSegs, lumps);
		}
		else {
			build_subsectors_recurse(child, clips, totalSegs, lumps);
		}

		clips.pop_back();
	}
}

void BotMapInfo::build_subsector(int subid, std::vector<BspClip>& clips,
	std::vector<MapSeg>& totalSegs, MapLumps& lumps) {
	MapSubsector& sub = subsectors[subid];
	LumpSubSector& lumpSub = lumps.subsectors[subid];

	std::vector<vec2> poly = {
		vec2(INT16_MAX, INT16_MAX),
		vec2(INT16_MAX, INT16_MIN),
		vec2(INT16_MIN, INT16_MIN),
		vec2(INT16_MIN, INT16_MAX),
	};

	for (const BspClip& clip : clips) {
		ClipPoly(poly, clip.linePoint, clip.lineDir, clip.front);
	}

	for (int i = 0; i < lumpSub.numsegs; i++) {
		LumpSeg& seg = lumps.segs[lumpSub.firstseg + i];
		LumpVert v1 = lumps.verts[seg.v1];
		LumpVert v2 = lumps.verts[seg.v2];
		vec2 fv1(v1.x, v1.y);
		vec2 fv2(v2.x, v2.y);
		ClipPoly(poly, fv1, fv2 - fv1, false);
	}

	sub.firstseg = totalSegs.size();
	sub.numsegs = poly.size();
	sub.mins = vec2(FLT_MAX, FLT_MAX);
	sub.maxs = vec2(-FLT_MAX, -FLT_MAX);

	for (int i = 0; i < poly.size(); i++) {
		vec2& start = poly[i];
		vec2& end = poly[(i + 1) % poly.size()];
		
		if (start.x > sub.maxs.x) sub.maxs.x = start.x;
		if (start.y > sub.maxs.y) sub.maxs.y = start.y;
		
		if (start.x < sub.mins.x) sub.mins.x = start.x;
		if (start.y < sub.mins.y) sub.mins.y = start.y;

		MapSeg seg;
		seg.v1 = start;
		seg.v2 = end;

		for (int k = 0; k < lumpSub.numsegs; k++) {
			LumpSeg& lseg = lumps.segs[lumpSub.firstseg + k];
			MapLine& line = lines[lseg.linedef];

			FSegment2 overlap = LineSegmentOverlap(line.v1, line.v2, seg.v1, seg.v2);
			if (overlap.length() >= 1.0f) {
				seg.lines.push_back(&line);
			}
		}

		totalSegs.push_back(seg);
	}

	// to ensure boxes overlap
	const float eps = 1.0f;
	sub.mins.x -= eps;
	sub.mins.y -= eps;
	sub.maxs.x += eps;
	sub.maxs.y += eps;
}

MapSubsector* BotMapInfo::GetSubsector(int x, int y) {
	if (numnodes == 0)
		return subsectors;

	uint16_t nodenum = numnodes - 1;

	while (true) {
		MapNode* node = &nodes[nodenum];
		int side = PointOnNodeSide(x, y, node);

		uint16_t child = node->children[side];

		if (child & 0x8000) {
			return &subsectors[child & 0x7FFF];
		}

		nodenum = child;
	}
}

MapSector* BotMapInfo::GetSector(int x, int y) {
	return GetSubsector(x, y)->sector;
}

MapSector* BotMapInfo::GetSector(AActor* actor) {
	vec3 pos = get_actor_pos(actor);
	return GetSector(pos.x, pos.y);
}

std::vector<int> BotMapInfo::GetTouchedSubsectors(AActor* actor) {
	if (!actor)
		return vector<int>();
	
	unordered_set<int> subs;

	int r = get_actor_radius(actor);
	float d = (r * 0.7071f);
	vec3 pos = get_actor_pos(actor);
	int x = pos.x;
	int y = pos.y;

	subs.insert(GetSubsector(x, y) - subsectors);

	// axes
	subs.insert(GetSubsector(x + r, y) - subsectors);
	subs.insert(GetSubsector(x - r, y) - subsectors);
	subs.insert(GetSubsector(x, y + r) - subsectors);
	subs.insert(GetSubsector(x, y - r) - subsectors);

	// diagonals
	subs.insert(GetSubsector(x + d, y + d) - subsectors);
	subs.insert(GetSubsector(x + d, y - d) - subsectors);
	subs.insert(GetSubsector(x - d, y + d) - subsectors);
	subs.insert(GetSubsector(x - d, y - d) - subsectors);

	std::vector<int> ret;
	for (auto item : subs) {
		ret.push_back(item);
	}

	return ret;
}

bool BotMapInfo::is_sector_border_potentially_crossable(MapSector* from, MapSector* to) {
	if (!from || !to)
		return false;

	if (from == to)
		return true;

	int frontMovement = from->moveFlags;
	int backMovement = to->moveFlags;

	float frontFloor = from->getFloorZ();
	float backFloor = to->getFloorZ();

	const bool canJump = true;
	int jumpHeight = (canJump ? JUMP_HEIGHT : STEP_HEIGHT);

	if (backFloor - frontFloor > jumpHeight) {
		// too high to jump
		if (!(backMovement & FL_SECTOR_MOVE_FLOOR_DOWN) && !(frontMovement & FL_SECTOR_MOVE_FLOOR_UP))
			return false; // and neither of the sectors move in a way that would make the jump possible
	}

	float backCeil = to->getCeilZ();
	float frontCeil = from->getCeilZ();
	const float fduckHeight = DUCK_HEIGHT;

	if (backCeil - backFloor < fduckHeight) {
		// not enough space in target sector
		if (!(backMovement & (FL_SECTOR_MOVE_CEIL_UP | FL_SECTOR_MOVE_FLOOR_DOWN)))
			return false; // and the sector never expands
	}

	if (backCeil - frontFloor < fduckHeight) {
		// too low of a ceil in the target sector to duck thru from the starting floor
		if (!(backMovement & FL_SECTOR_MOVE_CEIL_UP) && !(frontMovement & FL_SECTOR_MOVE_FLOOR_DOWN))
			return false; // and the sectors don't move in a way that would make the duck possible
	}

	if (frontCeil - backFloor < fduckHeight) {
		// too low of a ceil in the start sector to duck thru to the target floor
		if (!(backMovement & FL_SECTOR_MOVE_FLOOR_DOWN) && !(frontMovement & FL_SECTOR_MOVE_CEIL_UP))
			return false; // and the sectors don't move in a way that would make the duck possible
	}

	return true;
}

int BotMapInfo::sector_border_walkability(MapSector* from, MapSector* to) {
	if (from == to)
		return LINK_BLOCK_CLEAR;

	float frontFloor = from->getFloorZ();
	float backFloor = to->getFloorZ();

	const bool canJump = true;
	int maxHeight = canJump ? JUMP_HEIGHT : STEP_HEIGHT;

	if ((backFloor - frontFloor) > maxHeight) {
		return LINK_BLOCK_TOO_HIGH; // too high to step
	}

	float backCeil = to->getCeilZ();
	float frontCeil = from->getCeilZ();
	const float fduckHeight = DUCK_HEIGHT;

	float backHeight = backCeil - backFloor;
	float borderHeight = std::min(backCeil - frontFloor, frontCeil - backFloor);

	if (backHeight < fduckHeight || borderHeight < fduckHeight) {
		return LINK_BLOCK_TOO_LOW; // not enough space in target sector or border
	}

	return LINK_BLOCK_CLEAR;
}

std::vector<LinkSeg> BotMapInfo::get_neighbor_subsectors(MapSubsector* rootSub, MapSeg* borderSeg, std::unordered_set<MapSector*>& checkSectors) {
	const float epsilonWidth = 2;

	std::vector<LinkSeg> links;

	float bestLen = epsilonWidth;
	bool foundSeg = false;
	vec2 borderNormal = borderSeg->normal();

	vec2& mins1 = rootSub->mins;
	vec2& maxs1 = rootSub->maxs;

	for (MapSector* sector : checkSectors) {
		for (MapSubsector* otherSub : sector->subsectors) {
			if (otherSub == rootSub)
				continue;

			vec2& mins2 = otherSub->mins;
			vec2& maxs2 = otherSub->maxs;
			if ((maxs1.x < mins2.x || mins1.x > maxs2.x) ||
				(maxs1.y < mins2.y || mins1.y > maxs2.y)) {
				continue;
			}

			for (int s = 0; s < otherSub->numsegs; s++) {
				MapSeg& tseg = segs[otherSub->firstseg + s];

				FSegment2 overlap = LineSegmentOverlap(tseg.v1, tseg.v2, borderSeg->v1, borderSeg->v2);
				if (overlap.length() >= epsilonWidth) {
					LinkSeg link;
					link.overlap.a = overlap.a;
					link.overlap.b = overlap.b;
					link.otherSub = otherSub->id;

					float bestOverlap = 0;
					for (MapLine* line : tseg.lines) {
						FSegment2 lineOverlap = LineSegmentOverlap(overlap.a, overlap.b, borderSeg->v1, borderSeg->v2);
						float overlapLen = lineOverlap.length();
						if (overlapLen > bestOverlap) {
							bestOverlap = overlapLen;
							link.line = line;
						}
					}

					if (dotProduct(link.overlap.normal(), borderNormal) < 0) {
						// segment normals should always point inward towards the subsector
						vec2 temp = link.overlap.a;
						link.overlap.a = link.overlap.b;
						link.overlap.b = temp;
					}

					links.push_back(link);
				}
			}
		}
	}

	return links;
}

bool BotMapInfo::subsector_does_damage(MapSubsector* sub) {
	return sector_special_is_damage(sub->sector->special());
}

void BotMapInfo::find_linedef_sectors() {
	// create linedef sector mapping
	for (int i = 0; i < numlines; i++) {
		MapLine& line = lines[i];
		if (!line.special()) {
			continue;
		}

		if (!line.canPlayerActivate())
			continue;

		int lineAction = get_linedef_goal_action(&line);
		bool doubleSidedCrossLine = is_double_sided_cross_line(i);

		vec2 center = line.center();
		vec2 normal = line.normal();

		// route to a nearby sector if the adjacent one is too small to fit a player inside
		float useDist = BoxRadiusForDir(normal, PLAYER_RADIUS) + 1;

		if (lineAction == WBOT_GOAL_ACTION_CROSS || lineAction == WBOT_GOAL_ACTION_TOUCH) {
			// need to be very close to the line
			useDist = 2;
		}

		vec2 frontPoint = center + normal * useDist;
		vec2 backPoint = center - normal * useDist;
		int frontSubId = GetSubsector(frontPoint.x, frontPoint.y)->id;
		int backSubId = GetSubsector(backPoint.x, backPoint.y)->id;
		int routeToId = frontSubId;

		if (line.backsector && frontSubId == backSubId)
			gprintf("Front/back subsectors of line %d are the same!\n", i);

		if (doubleSidedCrossLine) {
			// pick the side that allows crossing so that bot doesn't try to cross lines from the bottom of a cliff
			float frontZ = subsectors[frontSubId].sector->getFloorZ();
			float backZ = subsectors[backSubId].sector->getFloorZ();
			
			if (backZ > frontZ) {
				routeToId = backSubId;
			}
		}

		line_subsectors[i] = routeToId;
	}
}

void BotMapInfo::add_sector_info() {
	for (int i = 0; i < numsectors; i++) {
		MapSector& sec = sectors[i];

		// add flags for lines that trigger this sector by tag
		if (sec.tag) {
			for (int k = 0; k < numlines; k++) {
				MapLine& line = lines[k];

				if (!line.tag || line.tag != sec.tag || !line.canPlayerActivate())
					continue;

				int flags = get_linedef_move_flag(&line);

				if (flags) {
					sec.moveFlags |= flags;
					sec.triggers.push_back(BotGoal(get_linedef_goal_action(&line), k));
				}
			}

			if (sec.tag == 666) {
				// hardcoded doom1/2 logic
				// just assume the sector can be moved in some way that is helpful
				sec.moveFlags |= FL_SECTOR_MOVE_FLOOR_DOWN | FL_SECTOR_MOVE_FLOOR_UP |
					FL_SECTOR_MOVE_CEIL_UP;
			}
		}

		// lines that have this sector as a backsector also trigger it, if no tag
		for (int k = 0; k < numlines; k++) {
			MapLine& line = lines[k];

			if (line.tag || line.backsector != &sec || !line.canPlayerActivate())
				continue;

			int flags = get_linedef_move_flag(&line);

			if (flags) {
				sec.moveFlags |= flags;
				sec.triggers.push_back(BotGoal(get_linedef_goal_action(&line), &line - lines));
			}
		}
	}

	add_stair_sector_info();
}

void BotMapInfo::remove_invalid_goals(int secid) {
	MapSector& info = sectors[secid];

	if (!info.moveFlags)
		return; // doesn't move, which means nothing triggers it

	for (int i = 0; i < info.triggers.size(); i++) {
		BotGoal& goal = info.triggers[i];

		if (!goal.valid()) {
			info.triggers.erase(info.triggers.begin() + i);
			i--;
			//Printf("Removed invalid goal on sector %d (%d left)\n", secid, info.triggers.size());
		}
	}

	if (info.triggers.empty()) {
		// TODO: don't unlink perpetually moving sectors like crushers
		g_wb_nav.relink_sector(&g_map.sectors[secid]);
	}
}
