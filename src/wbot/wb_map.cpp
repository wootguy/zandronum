#include "wb_map.h"
#include "wb_util.h"
#include "wb_nav.h"
#include "r_utility.h"
#include "p_lnspec.h"
#include "p_spec.h"
#include "p_setup.h"
#include "p_trace.h"
#include "p_local.h"
#include "a_keys.h"
#include <algorithm>
#include <float.h>

using namespace std;
using namespace wbot;

BotMapInfo wbot::g_map;

fixed_t MapSector::getHeight() {
	sector_t& sec = ::sectors[id];
	return sec.ceilingplane.Zat0() - sec.floorplane.Zat0();
}

fixed_t MapSector::getFloorZ() {
	sector_t& sec = ::sectors[id];
	return sec.floorplane.Zat0();
}

fixed_t MapSector::getCeilZ() {
	sector_t& sec = ::sectors[id];
	return sec.ceilingplane.Zat0();
}

bool MapSector::isMoving() {
	sector_t& sec = ::sectors[id];
	return sec.floordata || sec.ceilingdata;
}

bool MapSector::isFloorMoving() {
	return ::sectors[id].floordata;
}

bool MapSector::isCeilMoving() {
	return ::sectors[id].ceilingdata;
}

int MapSector::special() {
	return ::sectors[id].special;
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
	return ::lines[id].activation;
}

int MapLine::special() {
	return ::lines[id].special;
}

int MapLine::getArg(int idx) {
	line_t& line = ::lines[this - g_map.lines];
	return line.args[idx];
}

bool MapLine::isTeleport() {
	return special() == Teleport;
}

FVector2 MapLine::getTeleportDest() {
	AActor* actor = SelectTeleDest(getArg(0), getArg(1));
	return actor ? FVector2(actor->x, actor->y) : FVector2(0,0);
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

void* BotMapInfo::load_wad_lump(MapData* map, int id, int& len, int structSize) {
	int dataLen = map->Size(id);
	uint8_t* data = new uint8_t[dataLen];
	map->Read(id, data);
	len = dataLen / structSize;
	return data;
}

MapLumps BotMapInfo::load_wad_lump_data() {
	MapLumps lumps;
	MapData* map = P_OpenMapData(level.mapname, true);

	if (!map) {
		Printf("[wbot] Failed to open map data\n");
		memset(&lumps, 0, sizeof(lumps));
		return lumps;
	}

	lumps.verts = (LumpVert*)load_wad_lump(map, ML_VERTEXES, lumps.numverts, sizeof(LumpVert));
	lumps.sides = (LumpSide*)load_wad_lump(map, ML_SIDEDEFS, lumps.numsides, sizeof(LumpSide));
	lumps.segs = (LumpSeg*)load_wad_lump(map, ML_SEGS, lumps.numsegs, sizeof(LumpSeg));
	lumps.subsectors = (LumpSubSector*)load_wad_lump(map, ML_SSECTORS, lumps.numsubsectors, sizeof(LumpSubSector));
	lumps.lines = (LumpLine*)load_wad_lump(map, ML_LINEDEFS, lumps.numlines, sizeof(LumpLine));
	lumps.sectors = (LumpSector*)load_wad_lump(map, ML_SECTORS, lumps.numsectors, sizeof(LumpSector));
	lumps.nodes = (LumpNode*)load_wad_lump(map, ML_NODES, lumps.numnodes, sizeof(LumpNode));

	return lumps;
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
		Printf("[wbot] failed to load map data\n");
		return;
	}

	for (int i = 0; i < lumps.numsides; i++) {
		if (lumps.sides[i].sector >= lumps.numsectors) {
			Printf("[wbot] Bad lump side %d\n", i);
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
			if (frontside >= numsides || backside >= numsides)
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
	return GetSector(actor->x, actor->y);
}

std::vector<int> BotMapInfo::GetTouchedSubsectors(AActor* actor) {
	unordered_set<int> subs;

	fixed_t r = actor->radius;
	fixed_t d = FixedMul(r, 46341); // diagonal radius

	subs.insert(GetSubsector(actor->x, actor->y) - subsectors);

	// axes
	subs.insert(GetSubsector(actor->x + r, actor->y) - subsectors);
	subs.insert(GetSubsector(actor->x - r, actor->y) - subsectors);
	subs.insert(GetSubsector(actor->x, actor->y + r) - subsectors);
	subs.insert(GetSubsector(actor->x, actor->y - r) - subsectors);

	// diagonals
	subs.insert(GetSubsector(actor->x + d, actor->y + d) - subsectors);
	subs.insert(GetSubsector(actor->x + d, actor->y - d) - subsectors);
	subs.insert(GetSubsector(actor->x - d, actor->y + d) - subsectors);
	subs.insert(GetSubsector(actor->x - d, actor->y - d) - subsectors);

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

int BotMapInfo::get_linedef_move_flag(MapLine* line) {
	int timingFlag = 0;

	switch (line->special()) {
	case Plat_UpWaitDownStay:
	case Plat_UpNearestWaitDownStay:
	case Plat_DownWaitUpStay:
	case Plat_DownWaitUpStayLip:
	case Ceiling_CrushRaiseAndStayA:
	case Ceiling_CrushAndRaiseA:
	case Ceiling_CrushAndRaiseSilentA:
		timingFlag = FL_SECTOR_MOVE_TIMED;
		break;
	}

	// only add specials here that could be potentially helpful for unblocking a path.
	// For instance, raising a door or elevator. A ceiling or door coming down lower 
	// will not help a bot pass the sector
	switch (line->special()) {
	case 0:
		return 0;
	case Door_Open:
	case Door_Raise:
	case Door_LockedRaise:
	case Ceiling_RaiseByValue:
	case Ceiling_RaiseToNearest:
	case Ceiling_RaiseInstant:
	case Ceiling_RaiseByValueTimes8:
	case Generic_Ceiling: // TODO: check if really does move up
	case Generic_Door:
		return timingFlag | FL_SECTOR_MOVE_CEIL_UP;

	case Plat_UpWaitDownStay:
	case Plat_UpByValue:
	case Plat_UpNearestWaitDownStay:
	case Plat_RaiseAndStayTx0:
	case Plat_UpByValueStayTx:
	case Floor_RaiseToHighest:
	case Floor_RaiseToNearest:
	case Floor_RaiseByValueTxTy:
	case Floor_RaiseToLowestCeiling:
	case Elevator_RaiseToNearest:
	case Stairs_BuildUp:
	case Stairs_BuildUpSync:
	case Stairs_BuildUpDoom:
		return timingFlag | FL_SECTOR_MOVE_FLOOR_UP;

	case Plat_PerpetualRaise:
	case Plat_PerpetualRaiseLip:
		return timingFlag | FL_SECTOR_MOVE_FLOOR_UP | FL_SECTOR_MOVE_FLOOR_DOWN;

	case Plat_DownWaitUpStay:
	case Plat_DownByValue:
	case Plat_DownWaitUpStayLip:
	case Floor_LowerToLowest:
	case Floor_LowerToNearest:
	case Floor_LowerToHighest:
	case Floor_LowerToLowestTxTy:
	case Elevator_LowerToNearest:
	case Stairs_BuildDown:
	case Stairs_BuildDownSync:
		return timingFlag | FL_SECTOR_MOVE_FLOOR_DOWN;

	case Generic_Floor:
	case Elevator_MoveToFloor:
	case Generic_Lift:
	case Generic_Stairs:
		return timingFlag | FL_SECTOR_MOVE_FLOOR_UP | FL_SECTOR_MOVE_FLOOR_DOWN; // TODO: can do both dirs?	

	case Ceiling_LowerToHighestFloor:
	case Ceiling_LowerInstant:
	case Ceiling_CrushRaiseAndStayA:
	case Ceiling_CrushAndRaiseA:
	case Ceiling_CrushAndRaiseSilentA:
	case Ceiling_LowerByValueTimes8:
		return 0; // a ceiling getting lower is not helpful

	case Scroll_Texture_Left:
	case Scroll_Texture_Right:
	case Scroll_Texture_Up:
	case Scroll_Texture_Down:
	case Light_ForceLightning:
	case Light_RaiseByValue:
	case Light_LowerByValue:
	case Light_ChangeToValue:
	case Light_Fade:
	case Light_Glow:
	case Light_Flicker:
	case Light_Strobe:
	case Light_Stop:
		return 0; // visual-only specials

	case Teleport:
	case Plat_Stop:
		return 0; // does not cause sectors to move

	default:
		Printf("Unknown special %d for line %d\n", line->special(), line - lines);
		return 0;
	}
}

int BotMapInfo::get_linedef_goal_action(MapLine* line) {
	if (!line)
		return -1;

	line_t* eline = &::lines[line - lines];

	if (eline->activation & SPAC_Impact) {
		return WBOT_GOAL_ACTION_SHOOT;
	}
	if (eline->activation & (SPAC_Use | SPAC_UseThrough)) {
		return WBOT_GOAL_ACTION_USE;
	}
	if (eline->activation & (SPAC_Cross | SPAC_AnyCross)) {
		return WBOT_GOAL_ACTION_CROSS;
	}
	if (eline->activation & SPAC_Push) {
		return WBOT_GOAL_ACTION_TOUCH;
	}

	if (eline->activation)
		Printf("Don't know how to activate line %d\n", line - lines);
	
	return -1;
}

bool BotMapInfo::subsector_does_damage(MapSubsector* sub) {
	switch (sub->sector->special()) {
	case dDamage_Hellslime:
	case dDamage_LavaHefty:
	case dDamage_LavaWimpy:
	case dDamage_Nukage:
	case dDamage_SuperHellslime:
		return true;
	}

	return false;
}

void BotMapInfo::add_stair_sector_info() {
	for (int s = 0; s < numlines; s++) {
		MapLine& line = lines[s];
		line_t& eline = ::lines[s];

		bool isStairBuilder = false;
		int usespecials = 0;
		bool igntxt = false;
		int moveFlags = 0;

		switch (line.special()) {
		case Stairs_BuildDown:
		case Stairs_BuildUp:
			usespecials = 1;
			isStairBuilder = true;
			break;
		case Stairs_BuildDownSync:
		case Stairs_BuildUpSync:
			usespecials = 2;
			isStairBuilder = true;
			break;
		case Stairs_BuildUpDoom:
			isStairBuilder = true;
			break;
		case Generic_Stairs:
			isStairBuilder = true;
			igntxt = eline.args[3] & 2;
			break;
		}

		switch (line.special()) {
		case Stairs_BuildDown:
		case Stairs_BuildDownSync:
			moveFlags |= FL_SECTOR_MOVE_FLOOR_DOWN;
			break;
		case Stairs_BuildUp:
		case Stairs_BuildUpSync:
		case Stairs_BuildUpDoom:
		case Generic_Stairs:
			moveFlags |= FL_SECTOR_MOVE_FLOOR_UP;
			break;
		}

		if (!isStairBuilder)
			continue;

		int tag = line.tag;
		if (tag == 0)
			continue; // only back sector moves

		int i_compatflags = 0;
		int (*FindSector) (int tag, int start) =
			(i_compatflags & COMPATF_STAIRINDEX) ? P_FindSectorFromTagLinear : P_FindSectorFromTag;

		// The compatibility mode doesn't work with a hashing algorithm.
		// It needs the original linear search method. This was broken in Boom.

		BotGoal stairTrigger = BotGoal(get_linedef_goal_action(&line), s);

		int secnum = -1;
		int newsecnum = -1;
		sector_t* prev = NULL;
		while ((secnum = FindSector(tag, secnum)) >= 0) {
			sector_t* sec = &::sectors[secnum];

			// Find next sector to raise
			// 1. Find 2-sided line with same sector side[0] (lowest numbered)
			// 2. Other side is the next sector to raise
			// 3. Unless already moving, or different texture, then stop building
			bool ok;
			do
			{
				ok = false;
				sector_t* tsec = NULL;

				if (usespecials)
				{
					// [RH] Find the next sector by scanning for Stairs_Special?
					tsec = sec->NextSpecialSector(
						(sec->special & 0xff) == Stairs_Special1 ?
						Stairs_Special2 : Stairs_Special1, prev);

					ok = (tsec != NULL);
					newsecnum = (int)(tsec - ::sectors);
				}
				else
				{
					for (int i = 0; i < sec->linecount; i++)
					{
						if (!((sec->lines[i])->flags & ML_TWOSIDED))
							continue;

						tsec = (sec->lines[i])->frontsector;
						newsecnum = (int)(tsec - ::sectors);

						if (secnum != newsecnum)
							continue;

						tsec = (sec->lines[i])->backsector;
						if (!tsec) continue;	//jff 5/7/98 if no backside, continue
						newsecnum = (int)(tsec - ::sectors);

						FTextureID texture = sec->GetTexture(sector_t::floor);

						if (!igntxt && tsec->GetTexture(sector_t::floor) != texture)
							continue;

						ok = true;
						break;
					}
				}

				if (ok) {
					prev = sec;
					sec = tsec;
					secnum = newsecnum;

					MapSector& msec = g_map.sectors[tsec - ::sectors];
					msec.moveFlags |= moveFlags;
					msec.triggers.push_back(stairTrigger);
				}
			} while (ok);
		}
	}
}

void BotMapInfo::find_linedef_sectors() {
	// create linedef sector mapping
	for (int i = 0; i < numlines; i++) {
		MapLine& line = lines[i];
		if (!line.special()) {
			continue;
		}

		int lineAction = get_linedef_goal_action(&line);
		bool doubleSidedCrossLine = lineAction == WBOT_GOAL_ACTION_CROSS && (line.flags & ML_TWOSIDED);

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

		if (frontSubId == backSubId)
			Printf("Front/back subsectors of line %d are the same!\n", i);

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

				if (!line.tag || line.tag != sec.tag)
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

			if (line.tag || line.backsector != &sec)
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

bool BotMapInfo::Trace(FVector3 start, FVector3 end, uint32_t actorMask, uint32_t wallMask, AActor* ignore, TraceResult* tr) {
	FVector3 delta = end - start;
	fixed_t dist = delta.Length();
	delta = delta.Unit() * FRACUNIT;

	sector_t* sector = P_PointInSector(start.X, start.Y);

	FTraceResults trInternal;

	bool hit = ::Trace((fixed_t)start.X, (fixed_t)start.Y, (fixed_t)start.Z, sector,
		(fixed_t)delta.X, (fixed_t)delta.Y, (fixed_t)delta.Z, dist, actorMask,
		wallMask, ignore, trInternal);
	
	if (tr) {
		tr->endPos = FVector3(trInternal.X, trInternal.Y, trInternal.Z);
		tr->actor = trInternal.Actor;
		tr->frac = trInternal.Fraction / (float)FRACUNIT;
		tr->hitType = (TraceHitType)trInternal.HitType;
		tr->line = trInternal.Line ? &lines[trInternal.Line - ::lines] : NULL;
		tr->sector = trInternal.Sector ? &sectors[trInternal.Sector - ::sectors] : NULL;
	}

	return hit;
}

bool BotMapInfo::CheckKeys(AActor* activator, MapLine* line) {
	return P_CheckKeys(activator, line->getArg(3), false);
}