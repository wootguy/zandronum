#include "wb_map.h"
#include "wb_util.h"
#include "wb_nav.h"
#include <algorithm>
#include <float.h>

using namespace std;
using namespace wbot;

BotMapInfo wbot::g_map;

fixed_t MapSector::getHeight() {
	return get_sector_ceil_z(id) - get_sector_floor_z(id);
}

fixed_t MapSector::getFloorZ() {
	return get_sector_floor_z(id);
}

fixed_t MapSector::getCeilZ() {
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

FVector2 MapLine::center() {
	return (start() + end()) * 0.5f;
}

FVector2 MapLine::normal() {
	FVector2 dir = (v2 - v1).Unit();
	return FVector2(dir.Y, -dir.X);
}


int MapLine::length() {
	return (v2 - v1).Length();
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

bool MapLine::canPlayerActivate() {
	return can_player_activate_line(id);
}

FVector2 MapLine::getTeleportDest() {
	return get_tele_dest(id);
}


FVector2 MapSeg::center() {
	return (start() + end()) * 0.5f;
}

FVector2 MapSeg::normal() {
	FVector2 dir = (end() - start()).Unit();
	return FVector2(dir.Y, -dir.X);
}

float MapSeg::length() {
	return (v2 - v1).Length();
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
			dst.v1 = FVector2(lumps.verts[src.v1].x, lumps.verts[src.v1].y);
			dst.v2 = FVector2(lumps.verts[src.v2].x, lumps.verts[src.v2].y);
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
		clips.push_back({ FVector2(node.x, node.y), FVector2(node.dx, node.dy), i == 1});

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

	std::vector<FVector2> poly = {
		FVector2(INT16_MAX, INT16_MAX),
		FVector2(INT16_MAX, INT16_MIN),
		FVector2(INT16_MIN, INT16_MIN),
		FVector2(INT16_MIN, INT16_MAX),		
	};

	for (const BspClip& clip : clips) {
		ClipPoly(poly, clip.linePoint, clip.lineDir, clip.front);
	}

	for (int i = 0; i < lumpSub.numsegs; i++) {
		LumpSeg& seg = lumps.segs[lumpSub.firstseg + i];
		LumpVert v1 = lumps.verts[seg.v1];
		LumpVert v2 = lumps.verts[seg.v2];
		FVector2 fv1(v1.x, v1.y);
		FVector2 fv2(v2.x, v2.y);
		ClipPoly(poly, fv1, fv2 - fv1, false);
	}

	sub.firstseg = totalSegs.size();
	sub.numsegs = poly.size();
	sub.mins = FVector2(FLT_MAX, FLT_MAX);
	sub.maxs = FVector2(-FLT_MAX, -FLT_MAX);

	for (int i = 0; i < poly.size(); i++) {
		FVector2& start = poly[i];
		FVector2& end = poly[(i + 1) % poly.size()];
		
		if (start.X > sub.maxs.X) sub.maxs.X = start.X;
		if (start.Y > sub.maxs.Y) sub.maxs.Y = start.Y;
		
		if (start.X < sub.mins.X) sub.mins.X = start.X;
		if (start.Y < sub.mins.Y) sub.mins.Y = start.Y;

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
	sub.mins.X -= eps;
	sub.mins.Y -= eps;
	sub.maxs.X += eps;
	sub.maxs.Y += eps;
}

MapSubsector* BotMapInfo::GetSubsector(fixed_t x, fixed_t y) {
	if (numnodes == 0)
		return subsectors;

	uint16_t nodenum = numnodes - 1;

	x >>= FRACBITS;
	y >>= FRACBITS;

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

MapSector* BotMapInfo::GetSector(fixed_t x, fixed_t y) {
	return GetSubsector(x, y)->sector;
}

MapSector* BotMapInfo::GetSector(AActor* actor) {
	FVector3 pos = get_actor_pos(actor);
	return GetSector(pos.X, pos.Y);
}

std::vector<int> BotMapInfo::GetTouchedSubsectors(AActor* actor) {
	if (!actor)
		return vector<int>();
	
	unordered_set<int> subs;

	fixed_t r = get_actor_radius(actor);
	fixed_t d = (r * 46341) >> FRACBITS; // diagonal radius
	FVector3 pos = get_actor_pos(actor);
	fixed_t x = pos.X;
	fixed_t y = pos.Y;

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

	fixed_t frontFloor = from->getFloorZ();
	fixed_t backFloor = to->getFloorZ();

	const bool canJump = true;
	int jumpHeight = (canJump ? JUMP_HEIGHT : STEP_HEIGHT) << FRACBITS;

	if (backFloor - frontFloor > jumpHeight) {
		// too high to jump
		if (!(backMovement & FL_SECTOR_MOVE_FLOOR_DOWN) && !(frontMovement & FL_SECTOR_MOVE_FLOOR_UP))
			return false; // and neither of the sectors move in a way that would make the jump possible
	}

	fixed_t backCeil = to->getCeilZ();
	fixed_t frontCeil = from->getCeilZ();
	const fixed_t fduckHeight = DUCK_HEIGHT << FRACBITS;

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

	fixed_t frontFloor = from->getFloorZ();
	fixed_t backFloor = to->getFloorZ();

	const bool canJump = true;
	int maxHeight = canJump ? JUMP_HEIGHT : STEP_HEIGHT;

	if ((backFloor - frontFloor) > (maxHeight << FRACBITS)) {
		return LINK_BLOCK_TOO_HIGH; // too high to step
	}

	fixed_t backCeil = to->getCeilZ();
	fixed_t frontCeil = from->getCeilZ();
	const fixed_t fduckHeight = DUCK_HEIGHT << FRACBITS;

	fixed_t backHeight = backCeil - backFloor;
	fixed_t borderHeight = std::min(backCeil - frontFloor, frontCeil - backFloor);

	if (backHeight < fduckHeight || borderHeight < fduckHeight) {
		return LINK_BLOCK_TOO_LOW; // not enough space in target sector or border
	}

	return LINK_BLOCK_CLEAR;
}

std::vector<LinkSeg> BotMapInfo::get_neighbor_subsectors(MapSubsector* rootSub, MapSeg* borderSeg, std::unordered_set<MapSector*>& checkSectors) {
	const fixed_t epsilonWidth = 2;

	std::vector<LinkSeg> links;

	float bestLen = epsilonWidth;
	bool foundSeg = false;
	FVector2 borderNormal = borderSeg->normal();

	FVector2& mins1 = rootSub->mins;
	FVector2& maxs1 = rootSub->maxs;

	for (MapSector* sector : checkSectors) {
		for (MapSubsector* otherSub : sector->subsectors) {
			if (otherSub == rootSub)
				continue;

			FVector2& mins2 = otherSub->mins;
			FVector2& maxs2 = otherSub->maxs;
			if ((maxs1.X < mins2.X || mins1.X > maxs2.X) ||
				(maxs1.Y < mins2.Y || mins1.Y > maxs2.Y)) {
				continue;
			}

			for (int s = 0; s < otherSub->numsegs; s++) {
				MapSeg& tseg = segs[otherSub->firstseg + s];

				FSegment2 overlap = LineSegmentOverlap(tseg.v1, tseg.v2, borderSeg->v1, borderSeg->v2);
				if (overlap.length() >= epsilonWidth) {
					LinkSeg link;
					link.overlap.a = overlap.a * FRACUNIT;
					link.overlap.b = overlap.b * FRACUNIT;
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

					if (DotProduct(link.overlap.normal(), borderNormal) < 0) {
						// segment normals should always point inward towards the subsector
						FVector2 temp = link.overlap.a;
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

		FVector2 center = line.center();
		FVector2 normal = line.normal();

		// route to a nearby sector if the adjacent one is too small to fit a player inside
		fixed_t useDist = BoxRadiusForDir(normal, PLAYER_RADIUS << FRACBITS) + FRACUNIT;

		if (lineAction == WBOT_GOAL_ACTION_CROSS || lineAction == WBOT_GOAL_ACTION_TOUCH) {
			// need to be very close to the line
			useDist = FRACUNIT * 2;
		}

		FVector2 frontPoint = center + normal * useDist;
		FVector2 backPoint = center - normal * useDist;
		int frontSubId = GetSubsector(frontPoint.X, frontPoint.Y)->id;
		int backSubId = GetSubsector(backPoint.X, backPoint.Y)->id;
		int routeToId = frontSubId;

		if (line.backsector && frontSubId == backSubId)
			gprintf("Front/back subsectors of line %d are the same!\n", i);

		if (doubleSidedCrossLine) {
			// pick the side that allows crossing so that bot doesn't try to cross lines from the bottom of a cliff
			fixed_t frontZ = subsectors[frontSubId].sector->getFloorZ();
			fixed_t backZ = subsectors[backSubId].sector->getFloorZ();
			
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
