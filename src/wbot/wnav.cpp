#include "wnav.h"
#include "wbot.h"
#include "r_state.h"
#include "sv_commands.h"
#include "p_local.h"
#include "p_lnspec.h"
#include "p_trace.h"
#include "a_keys.h"
#include "wutil.h"

#include <algorithm>
#include <unordered_set>
#include <unordered_map>
#include <vector>

using namespace std;

SectorNavMesh g_wbot_nav;
int g_total_links;

bool NavSectorLink::blocked(AActor* actor, bool recurse) {
	g_wbot_nav.pathTests++;

	if (!g_wbot_nav.can_cross_seg_now(seg)) {
		line_t* line = seg->linedef;

		if (line && line->args[0] == 0 && g_wbot_nav.get_linedef_move_flag(line)) {
			if (line->special == Door_LockedRaise && !P_CheckKeys(actor, line->args[3], false)) {
				return true; // don't have the keys required to use this
			}

			// border is an untagged linedef, which means it moves the target sector,
			// which probably unblocks the path. TODO: not always!
			return false;
		}

		return true;
	}

	// check if jumps are valid for dynamic links that depend on from/to sector states
	if (isJump) {
		if (!isJumpHeightValid()) {
			return true;
		}

		NavSector* target = getTarget();
		FVector3 start = pos() + FVector3(0, 0, 56 << FRACBITS);
		FVector3 end = target->pos() + FVector3(0, 0, 56 << FRACBITS);

		FTraceResults tr;
		if (TraceLine(start, end, true, NULL, &tr)) {
			return true;
		}
	}
	
	if (recurse && linkWidth <= PLAYER_WIDTH) {
		// too narrow to move thru unless the neighbor sectors are passable
		NavSector& nav = g_wbot_nav.nav_sectors[parent];

		int expandedWidth = linkWidth;
		int neighbors[2] = {leftSector, rightSector};
		for (int i = 0; i < 2; i++) {
			if (neighbors[i] < 0)
				continue;

			NavSector& neighborNav = g_wbot_nav.nav_sectors[neighbors[i]];
			bool isWalkable = nav.getFloorZ() + (STEP_HEIGHT << FRACBITS) > neighborNav.getFloorZ();
			bool wontHitHead = neighborNav.getCeilZ() > nav.getFloorZ() + (DUCK_HEIGHT << FRACBITS);
			if (isWalkable && wontHitHead) {
				// the neighboring sector is not a wall and won't prevent movement on the target sector
				// TODO: need to check how thick the neighbor is too
				expandedWidth += PLAYER_WIDTH;
				continue;
			}

			NavSectorLink* link = nav.getLink(neighbors[i]);

			if (link && !link->blocked(actor, false)) {
				expandedWidth += link->linkWidth;
			}
		}
		
		if (expandedWidth > PLAYER_WIDTH) {
			// wide enough if you consider the neighbor sectors
			return false;
		}

		return true;
	}

	return false;
}

bool NavSectorLink::walkable() {
	return g_wbot_nav.can_cross_seg_now(seg);
}

bool NavSectorLink::isJumpHeightValid() {
	NavSector* parent = getParent();
	NavSector* target = getTarget();

	fixed_t jumpHeight = target->getFloorZ() - parent->getFloorZ();

	return jumpHeight < (JUMP_HEIGHT << FRACBITS);
}

NavSector* NavSectorLink::getParent() {
	return &g_wbot_nav.nav_sectors[parent];
}

NavSector* NavSectorLink::getTarget() {
	return &g_wbot_nav.nav_sectors[target];
}

NavSectorLink* NavSector::getLink(int subSectorId) {
	if (subSectorId < 0)
		return NULL;

	for (int i = 0; i < links.size(); i++) {
		if (links[i].target == subSectorId) {
			return &links[i];
		}
	}

	//Printf("Nav sector %d has no link to %d\n", id, subSectorId);
	return NULL;
}

bool NavSector::touches(AActor* actor) {
	if (!actor)
		return false;

	FVector2 actorPos = FVector2(actor->x, actor->y);

	int subid = R_PointInSubsector(actorPos.X, actorPos.Y) - subsectors;
	if (subid == id)
		return true;

	subsector_t& sub = subsectors[id];
	for (int i = 0; i < sub.numlines; i++) {
		seg_t& seg = sub.firstline[i];
		FVector2 v1(seg.v1->x, seg.v1->y);
		FVector2 v2(seg.v2->x, seg.v2->y);

		if (CircleIntersectsSegment(actorPos, actor->radius, v1, v2)) {
			return true;
		}
	}
	
	return false;
}

int NavSector::getMoveFlags() {
	return g_wbot_nav.sector_move_flags[subsectors[id].sector - sectors];
}

sector_t* NavSector::sector() {
	return subsectors[id].sector;
}

fixed_t NavSector::getHeight() {
	fixed_t fx = x << FRACBITS;
	fixed_t fy = y << FRACBITS;

	sector_t* sec = subsectors[id].sector;
	return sec->ceilingplane.ZatPoint(fx, fy) - sec->floorplane.ZatPoint(fx, fy);
}

fixed_t NavSector::getFloorZ() {
	return subsectors[id].sector->floorplane.ZatPoint(x << FRACBITS, y << FRACBITS);
}

fixed_t NavSector::getCeilZ() {
	return subsectors[id].sector->ceilingplane.ZatPoint(x << FRACBITS, y << FRACBITS);
}

LinkSeg SectorNavMesh::GetSegmentOverlap(seg_t* a, seg_t* b)
{
	LinkSeg result = { 0 };

	FVector2 a1(a->v1->x, a->v1->y);
	FVector2 a2(a->v2->x, a->v2->y);
	FVector2 b1(b->v1->x, b->v1->y);
	FVector2 b2(b->v2->x, b->v2->y);

	const fixed_t distEpsilon = 1 << FRACBITS;

	// colinear tests
	if (abs(DistanceToLine(a1, b1, b2)) > distEpsilon || abs(DistanceToLine(a2, b1, b2)) > distEpsilon) {
		return result;
	}
	if (abs(DistanceToLine(b1, a1, a2)) > distEpsilon || abs(DistanceToLine(b2, a1, a2)) > distEpsilon) {
		return result;
	}

	if (fabs(a2.X - a1.X) >= fabs(a2.Y - a1.Y))
	{
		bool aForward = a1.X < a2.X;
		bool bForward = b1.X < b2.X;

		float start = std::max(std::min(a1.X, a2.X), std::min(b1.X, b2.X));
		float end = std::min(std::max(a1.X, a2.X), std::max(b1.X, b2.X));

		if (end - start < (8 << FRACBITS))
			return result;

		float dx = a2.X - a1.X;
		float dy = a2.Y - a1.Y;

		float t1 = (start - a1.X) / dx;
		float t2 = (end - a1.X) / dx;

		result.x1 = (fixed_t)start;
		result.y1 = (fixed_t)(a1.Y + t1 * dy);

		result.x2 = (fixed_t)end;
		result.y2 = (fixed_t)(a1.Y + t2 * dy);
	}
	else
	{
		float start = std::max(std::min(a1.Y, a2.Y), std::min(b1.Y, b2.Y));
		float end = std::min(std::max(a1.Y, a2.Y), std::max(b1.Y, b2.Y));

		if (end - start < (8 << FRACBITS))
			return result;

		float dx = a2.X - a1.X;
		float dy = a2.Y - a1.Y;

		float t1 = (start - a1.Y) / dy;
		float t2 = (end - a1.Y) / dy;

		result.x1 = (fixed_t)(a1.X + t1 * dx);
		result.y1 = (fixed_t)start;

		result.x2 = (fixed_t)(a1.X + t2 * dx);
		result.y2 = (fixed_t)end;
	}

	return result;
}

bool SectorNavMesh::is_seg_potentially_crossable(seg_t* seg) {
	if (!seg->frontsector || !seg->backsector)
		return false;

	if (seg->linedef && (seg->linedef->flags & ML_BLOCKING))
		return false; // impassable

	int frontMovement = sector_move_flags[seg->frontsector - sectors];
	int backMovement = sector_move_flags[seg->backsector - sectors];

	fixed_t x = (seg->v1->x + seg->v2->x) / 2;
	fixed_t y = (seg->v1->y + seg->v2->y) / 2;

	fixed_t frontFloor = seg->frontsector->floorplane.ZatPoint(x, y);
	fixed_t backFloor = seg->backsector->floorplane.ZatPoint(x, y);

	const bool canJump = true;
	int jumpHeight = (canJump ? JUMP_HEIGHT : STEP_HEIGHT) << FRACBITS;

	if (backFloor - frontFloor > jumpHeight) {
		// too high to jump
		if (!(backMovement & FL_SECTOR_MOVE_FLOOR_DOWN) && !(frontMovement & FL_SECTOR_MOVE_FLOOR_UP))
			return false; // and neither of the sectors move in a way that would make the jump possible
	}

	fixed_t backCeil = seg->backsector->ceilingplane.ZatPoint(x, y);
	fixed_t fduckHeight = DUCK_HEIGHT << FRACBITS;

	if (backCeil - backFloor < fduckHeight || backCeil - frontFloor < fduckHeight) {
		// too low of a ceil in the target sector to duck thru from the starting floor
		if (!(backMovement & FL_SECTOR_MOVE_CEIL_UP) && !(frontMovement & FL_SECTOR_MOVE_FLOOR_DOWN))
			return false; // and the sectors don't move in a way that would make the duck possible
	}

	return true;
}

bool SectorNavMesh::can_cross_seg_now(seg_t* seg)
{
	fixed_t x = (seg->v1->x + seg->v2->x) / 2;
	fixed_t y = (seg->v1->y + seg->v2->y) / 2;

	fixed_t frontFloor = seg->frontsector->floorplane.ZatPoint(x, y);
	fixed_t backFloor = seg->backsector->floorplane.ZatPoint(x, y);

	const bool canJump = true;
	int maxHeight = canJump ? JUMP_HEIGHT : STEP_HEIGHT;

	if ((backFloor - frontFloor) > (maxHeight << FRACBITS)) {
		return false; // too high to step
	}

	fixed_t backCeil = seg->backsector->ceilingplane.ZatPoint(x, y);

	if ((backCeil - backFloor) < (DUCK_HEIGHT << FRACBITS)) {
		return false; // not enough space in target sector
	}
	if ((backCeil - frontFloor) < (DUCK_HEIGHT << FRACBITS)) {
		return false; // not enough space at the border segment (can't step down)
	}

	return true;
}

LinkSeg SectorNavMesh::get_neighbor_subsector(subsector_t* ignoreSector, seg_t* borderSeg) {
	const fixed_t epsilonWidth = 2 << FRACBITS;
	LinkSeg ret;
	memset(&ret, 0, sizeof(ret));

	for (int j = 0; j < numsubsectors; j++) {
		subsector_t& otherSub = subsectors[j];
		if (&otherSub == ignoreSector)
			continue;

		for (int s = 0; s < otherSub.numlines; s++) {
			seg_t& tseg = otherSub.firstline[s];

			if (!tseg.frontsector || !tseg.backsector)
				continue; // impassable wall

			// TODO: Allow thin segments for tightrope areas
			ret = GetSegmentOverlap(&tseg, borderSeg);
			if (ret.length() >= epsilonWidth) {
				ret.idx = s;
				ret.otherSub = j;
				return ret;
			}
		}
	}

	ret.otherSub = -1;
	return ret;
}

int SectorNavMesh::get_linedef_move_flag(line_t* line) {
	int timingFlag = 0;

	switch (line->special) {
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
	switch (line->special) {
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

	case Plat_PerpetualRaise:
	case Plat_UpWaitDownStay:
	case Plat_UpByValue:
	case Plat_UpNearestWaitDownStay:
	case Plat_PerpetualRaiseLip:
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
		return 0; // does not move sectors

	default:
		Printf("Unknown special %d for line %d\n", line->special, line - lines);
		return 0;
	}
}

int SectorNavMesh::get_linedef_goal_action(line_t* line) {
	if (line->activation & SPAC_Impact) {
		return WBOT_GOAL_ACTION_SHOOT;
	}
	if (line->activation & (SPAC_Use | SPAC_UseThrough)) {
		return WBOT_GOAL_ACTION_USE;
	}
	if (line->activation & (SPAC_Cross | SPAC_AnyCross)) {
		return WBOT_GOAL_ACTION_CROSS;
	}
	if (line->activation & SPAC_Push) {
		return WBOT_GOAL_ACTION_TOUCH;
	}

	Printf("Don't know how to activate line %d\n", line - lines);
	return WBOT_GOAL_ACTION_USE;
}

bool SectorNavMesh::subsector_does_damage(subsector_t* sub) {
	switch (sub->sector->special) {
	case dDamage_Hellslime:
	case dDamage_LavaHefty:
	case dDamage_LavaWimpy:
	case dDamage_Nukage:
	case dDamage_SuperHellslime:
		return true;
	}

	return sub->sector->damage > 0;
}

bool SectorNavMesh::create_jump_link(NavSector& fromNav, NavSectorLink& fromLink, NavSector& toNav, NavSectorLink& toLink) {
	fixed_t fromHeight = fromNav.getFloorZ();
	fixed_t toHeight = toNav.getFloorZ();
	fixed_t dropHeight = fromHeight - toHeight;

	if (dropHeight < -(JUMP_HEIGHT << FRACBITS)) {
		// too high to jump to
		if (!(fromNav.getMoveFlags() & FL_SECTOR_MOVE_FLOOR_UP) && !(toNav.getMoveFlags() & FL_SECTOR_MOVE_FLOOR_DOWN))
			return false; // and neither sector moves in a way that would make the jump possible later
	}

	// increase link distance for drops, decrease for jumps
	int maxDist = 200 << FRACBITS;
	maxDist += dropHeight;

	fixed_t linkDist = (fromLink.pos() - toLink.pos()).Length();
	if (linkDist > maxDist) {
		return false; // too far to link to
	}

	FVector2 e1v1(fromLink.seg->v1->x, fromLink.seg->v1->y);
	FVector2 e1v2(fromLink.seg->v2->x, fromLink.seg->v2->y);

	FVector2 e2v1(toLink.seg->v1->x, toLink.seg->v1->y);
	FVector2 e2v2(toLink.seg->v2->x, toLink.seg->v2->y);

	FVector2 edge1 = (e1v2 - e1v1);
	FVector2 edge2 = (e2v2 - e2v1);
	edge1.MakeUnit();
	edge2.MakeUnit();

	FVector2 normalA(edge1.Y, -edge1.X);
	FVector2 normalB(edge2.Y, -edge2.X);

	float dot = DotProduct(normalA, normalB);
	if (dot > -0.5f) {
		return false; // edges not facing each other enough
	}

	FVector2 delta = fromLink.pos() - toLink.pos();
	float planeDist = DotProduct(normalA, delta);
	if (planeDist < 0) {
		return false; // not in front of the segment
	}

	if (fromNav.getLink(toNav.id)) {
		return false; // already have a link to this sector
	}

	// check that the path is clear
	{
		FVector3 start = fromLink.pos() + FVector3(0, 0, 56 << FRACBITS);
		FVector3 end = toNav.pos() + FVector3(0, 0, 56 << FRACBITS);

		FVector2 delta = end - start;
		delta.MakeUnit();
		FVector3 rightDir(delta.Y, -delta.X, 0);
		int radius = PLAYER_WIDTH / 2;
		fixed_t rightStep = (radius / 2) << FRACBITS;

		for (int i = -2; i <= 2; i++) {
			FTraceResults tr;
			if (TraceLine(start + rightDir * i * rightStep, end + rightDir * i * rightStep, true, NULL, &tr)) {
				if (tr.Line && tr.Line->backsector) {
					int moveFlags = sector_move_flags[tr.Line->backsector - sectors];
					if (moveFlags) {
						continue; // wall may move out of the way in the future
					}
				}

				return false; // hit an immovable wall
			}
		}
	}

	NavSectorLink link;
	link.parent = fromLink.parent;
	link.overlapCenter = fromLink.overlapCenter;
	link.linkWidth = fromLink.linkWidth;
	link.leftSector = fromLink.leftSector;
	link.rightSector = fromLink.rightSector;
	link.isTeleport = fromLink.isTeleport;
	link.isCliff = fromLink.isCliff;
	link.seg = fromLink.seg;

	link.target = toNav.id;
	link.id = g_total_links++;
	link.isJump = true;

	//if (link.id == 2673 && toNav.id == 673) {
		//Printf("");
	//}

	fromNav.links.push_back(link);

	return true;
}

void SectorNavMesh::add_jump_links() {
	// add jump links between cliff segments
	for (int i = 0; i < numsubsectors; i++) {
		NavSector& nav = nav_sectors[i];
		bool allLinksCanBeCliffsLater = nav.getMoveFlags() & FL_SECTOR_MOVE_FLOOR_UP;

		for (int k = 0; k < nav.links.size(); k++) {
			NavSectorLink& link = nav.links[k];

			if (link.isJump)
				continue;

			if (!link.isCliff && !allLinksCanBeCliffsLater)
				continue;

			// find other other cliff segments to try linking to
			for (int j = 0; j < numsubsectors; j++) {
				NavSector& otherNav = nav_sectors[j];

				if (j == i)
					continue;

				for (int x = 0; x < otherNav.links.size(); x++) {
					NavSectorLink& otherLink = otherNav.links[x];
					if (!otherLink.isCliff || otherLink.isJump)
						continue;

					if (create_jump_link(nav, nav.links[k], otherNav, otherLink)) {
						break;
					}
				}
			}
		}
	}
}

void SectorNavMesh::add_stair_sector_move_flags() {
	for (int s = 0; s < numlines; s++) {
		line_t& line = lines[s];

		bool isStairBuilder = false;
		int usespecials = 0;
		bool igntxt = false;
		int moveFlags = 0;

		switch (line.special) {
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
			igntxt = line.args[3] & 2;
			break;
		}

		switch (line.special) {
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

		int tag = line.args[0];
		if (tag == 0)
			continue; // only back sector moves

		int i_compatflags = 0;
		int (*FindSector) (int tag, int start) =
			(i_compatflags & COMPATF_STAIRINDEX) ? P_FindSectorFromTagLinear : P_FindSectorFromTag;

		// The compatibility mode doesn't work with a hashing algorithm.
		// It needs the original linear search method. This was broken in Boom.

		int secnum = -1;
		int newsecnum = -1;
		sector_t* prev = NULL;
		while ((secnum = FindSector(tag, secnum)) >= 0) {
			sector_t* sec = &sectors[secnum];

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
					newsecnum = (int)(tsec - sectors);
				}
				else
				{
					for (int i = 0; i < sec->linecount; i++)
					{
						if (!((sec->lines[i])->flags & ML_TWOSIDED))
							continue;

						tsec = (sec->lines[i])->frontsector;
						newsecnum = (int)(tsec - sectors);

						if (secnum != newsecnum)
							continue;

						tsec = (sec->lines[i])->backsector;
						if (!tsec) continue;	//jff 5/7/98 if no backside, continue
						newsecnum = (int)(tsec - sectors);

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
					sector_move_flags[tsec - sectors] |= moveFlags;
				}
			} while (ok);
		}
	}
}

void SectorNavMesh::find_linedef_sectors() {
	// create linedef sector mapping
	for (int i = 0; i < numlines; i++) {
		line_t& line = lines[i];
		if (!line.special) {
			continue;
		}
		seg_t lineSeg; // fake seg for overlap test
		lineSeg.v1 = line.v1;
		lineSeg.v2 = line.v2;

		for (int s = 0; s < numsubsectors; s++) {
			subsector_t& sub = subsectors[s];

			float mostOverlap = 0;

			for (int k = 0; k < sub.numlines; k++) {
				seg_t& seg = sub.firstline[k];
				if (seg.linedef != &line) {
					continue;
				}
				if (seg.frontsector != line.frontsector) {
					continue; // only want the sector in front of the line
				}

				LinkSeg linkseg = GetSegmentOverlap(&seg, &lineSeg);
				float overlap = linkseg.length();
				if (overlap > mostOverlap) {
					mostOverlap = overlap;
					line_subsectors[i] = s;
				}
			}
		}
	}
}

void SectorNavMesh::add_sector_move_flags() {
	for (int i = 0; i < numsectors; i++) {
		sector_t& sec = sectors[i];

		int& flags = sector_move_flags[i];

		// add flags for lines that trigger this sector by tag
		if (sec.tag) {
			for (int k = 0; k < numlines; k++) {
				line_t& line = lines[k];

				if (line.args[0] != sec.tag)
					continue;

				flags |= get_linedef_move_flag(&line);
			}
		}

		// lines that have this sector as a backsector also trigger it, if no tag
		for (int i = 0; i < sec.linecount; i++) {
			line_t* line = sec.lines[i];

			if (line->args[0] != sec.tag || line->backsector != &sec)
				continue;

			flags |= get_linedef_move_flag(line);
		}
	}

	add_stair_sector_move_flags();
}

void SectorNavMesh::add_sector_trigger_goals() {
	// add sector triggers
	for (int i = 0; i < numsubsectors; i++) {
		subsector_t& sub = subsectors[i];
		sector_t* sec = sub.sector;
		NavSector& nav = nav_sectors[i];

		if (sec->tag) {
			// any line with this sector's tag can move the sector
			for (int k = 0; k < numlines; k++) {
				line_t& line = lines[k];

				if (line.args[0] != sec->tag)
					continue;

				if (get_linedef_move_flag(&line)) {
					nav.triggers.push_back(BotGoal(get_linedef_goal_action(&line), k));
				}
			}
		}
		else {
			// surrounding lines can move the sector if untagged
			for (int k = 0; k < sec->linecount; k++) {
				line_t* line = sec->lines[k];

				if (line->args[0] != 0)
					continue;

				if (line->backsector == sec && get_linedef_move_flag(line)) {
					nav.triggers.push_back(BotGoal(get_linedef_goal_action(line), line - lines));
				}
			}
		}

		// try closest goals first
		if (nav.triggers.size()) {
			for (int k = 0; k < nav.triggers.size(); k++) {
				BotGoal& goal = nav.triggers[k];
				goal.purposeSector = i;

				int goalNav = goal.getNavId();
				if (goalNav != -1) {
					goal.dist = get_route_distance(get_astar_route(goalNav, i));
				}
			}

			std::sort(nav.triggers.begin(), nav.triggers.end(), [](const BotGoal& a, const BotGoal& b) {
				return a.dist < b.dist;
				});
		}
	}
}

bool SectorNavMesh::is_link_bordered_by_walls(subsector_t& sub, int segIdx, int& leftSubId, int& rightSubId) {
	int leftIdx = (segIdx + sub.numlines - 1) % sub.numlines;
	int rightIdx = (segIdx + 1) % sub.numlines;
	LinkSeg leftSector = get_neighbor_subsector(&sub, &sub.firstline[leftIdx]);
	LinkSeg rightSector = get_neighbor_subsector(&sub, &sub.firstline[rightIdx]);
	int leftSub = leftSector.otherSub;
	int rightSub = rightSector.otherSub;

	bool leftSubIsWall = leftSub == -1 || !is_seg_potentially_crossable(&sub.firstline[leftIdx]);
	bool rightSubIsWall = rightSub == -1 || !is_seg_potentially_crossable(&sub.firstline[rightIdx]);

	leftSubId = leftSub;
	rightSubId = rightSub;

	return leftSubIsWall && rightSubIsWall;
}

void SectorNavMesh::calc_nav_centers() {
	for (int i = 0; i < numsubsectors; i++) {
		subsector_t& sub = subsectors[i];
		NavSector& nav = nav_sectors[i];

		int subCenterX = 0;
		int subCenterY = 0;

		for (int k = 0; k < sub.numlines; k++) {
			seg_t& seg = sub.firstline[k];
			subCenterX += seg.v1->x >> FRACBITS;
			subCenterY += seg.v1->y >> FRACBITS;
		}

		nav.x = subCenterX / (int)sub.numlines;
		nav.y = subCenterY / (int)sub.numlines;
		nav.z = sub.sector->floorplane.ZatPoint(nav.x << FRACBITS, nav.y << FRACBITS) >> FRACBITS;
	}
}

void SectorNavMesh::generate_node_graph() {
	uint64_t genStart = getEpochMillis();
	
	if (nav_sectors) {
		delete[] nav_sectors;
		nav_sectors = NULL;
	}
	if (sector_move_flags) {
		delete[] sector_move_flags;
		sector_move_flags = NULL;
	}
	if (line_subsectors) {
		delete[] line_subsectors;
		line_subsectors = NULL;
	}

	nav_sectors = new NavSector[numsubsectors];

	sector_move_flags = new int[numsectors];
	memset(sector_move_flags, 0, sizeof(int) * numsectors);

	line_subsectors = new int[numlines];
	memset(line_subsectors, -1, sizeof(int) * numlines);

	g_total_links = 0;

	find_linedef_sectors();
	calc_nav_centers();
	add_sector_move_flags();

	// link sectors as a nav mesh
	for (int i = 0; i < numsubsectors; i++) {
		subsector_t& sub = subsectors[i];
		NavSector& nav = nav_sectors[i];
		
		nav.id = i;
		nav.doesDamage = subsector_does_damage(&sub);

		for (int k = 0; k < sub.numlines; k++) {
			seg_t& seg = sub.firstline[k];

			if (!is_seg_potentially_crossable(&seg)) {
				continue;
			}

			LinkSeg linkseg = get_neighbor_subsector(&sub, &seg);

			if (linkseg.otherSub < 0)
				continue;

			fixed_t cx = (linkseg.x1 + linkseg.x2) / 2;
			fixed_t cy = (linkseg.y1 + linkseg.y2) / 2;
			float z = sub.sector->floorplane.ZatPoint(cx, cy);

			NavSectorLink link;
			link.target = linkseg.otherSub;
			link.overlapCenter = FVector3(cx, cy, z);
			link.seg = &seg;
			link.id = g_total_links++;
			link.parent = i;
			link.linkWidth = (int)linkseg.length() >> FRACBITS;
			link.isJump = false;
			link.isTeleport = false;
			link.leftSector = -1;
			link.rightSector = -1;

			subsector_t& neighbor = subsectors[linkseg.otherSub];

			if (link.linkWidth <= PLAYER_WIDTH) {
				int dummy;
				if (is_link_bordered_by_walls(sub, k, dummy, dummy))
					continue; // link is too narrow to enter and both sides of it are impassable walls
				if (is_link_bordered_by_walls(neighbor, linkseg.idx, link.leftSector, link.rightSector))
					continue; // link is too narrow to leave and both sides of it are impassable walls
			}

			link.isCliff = sub.sector->floorplane.ZatPoint(cx, cy)
				- neighbor.sector->floorplane.ZatPoint(cx, cy) > (JUMP_HEIGHT << FRACBITS);

			if (link.isCliff)
				nav.hasCliffs = true;

			// redirect target sector if this is the front side of a teleport
			line_t* line = seg.linedef;
			bool isPlayerTele = line && line->special == Teleport && (line->activation & SPAC_Cross);
			if (isPlayerTele && DistanceToLine(nav.pos(), line) < 0) {
				AActor* dest = SelectTeleDest(line->args[0], line->args[1]);
				if (dest) {
					subsector_t* destSub = R_PointInSubsector(dest->x, dest->y);
					link.target = destSub - subsectors;
					link.isTeleport = true;
				}
			}

			nav.links.push_back(link);
		}
	}

	add_jump_links();
	add_sector_trigger_goals();

	Printf("Generated %d nodes, %d links in %d ms\n", (int)numsubsectors, g_total_links, (int)(getEpochMillis() - genStart));
}

int SectorNavMesh::get_route_distance(std::vector<int>& route) {
	int totalDist = 0;
	for (int i = 1; i < route.size(); i++) {
		totalDist += (int)(nav_sectors[i].pos() - nav_sectors[i - 1].pos()).Length() >> FRACBITS;
	}
	return totalDist;
}

void SectorNavMesh::draw_nodes(AActor* actor) {
	static int lastDraw;

	if (level.time - lastDraw < 10 && lastDraw < level.time) {
		return;
	}

	lastDraw = level.time;

	AActor* player = getAnyPlayer();
	if (!player)
		return;

	subsector_t* sub = R_PointInSubsector(player->x, player->y);
	int subId = sub - subsectors;

	if (sub && subId < numsubsectors) {
		NavSector& nav = nav_sectors[subId];
		FVector3 pos(nav.x << FRACBITS, nav.y << FRACBITS, nav.z << FRACBITS);
		int spritesDrawn = 0;
		const int maxSprites = 1000;

		for (int k = 0; k < nav.links.size() && spritesDrawn < maxSprites; k++) {
			NavSectorLink& link = nav.links[k];
			NavSector& linkNav = nav_sectors[link.target];
			
			if (!link.blocked(player)) {
				FVector3 linkPos = link.pos();
				linkPos.Z = nav.getFloorZ();

				if (link.isJump) {
					FVector3 jumpPos = linkPos + FVector3(0, 0, 56 << FRACBITS);
					FVector3 landPos = linkNav.pos() + FVector3(0, 0, 56 << FRACBITS);

					spritesDrawn += draw_debug_line(nav.pos(), linkPos, actor);
					spritesDrawn += draw_debug_line(linkPos, jumpPos, actor);
					spritesDrawn += draw_debug_line(jumpPos, landPos, actor);
					spritesDrawn += draw_debug_line(landPos, linkNav.pos(), actor);
				}
				else {
					spritesDrawn += draw_debug_line(nav.pos(), linkPos, actor);
					spritesDrawn += draw_debug_line(linkPos, linkNav.pos(), actor);
				}
			}
		}

		fixed_t borderZ = nav.z << FRACBITS;
		for (int k = 0; k < sub->numlines && spritesDrawn < maxSprites; k++) {
			seg_t& seg = sub->firstline[k];

			FVector3 start(seg.v1->x, seg.v1->y, borderZ);
			FVector3 end(seg.v2->x, seg.v2->y, borderZ);
			spritesDrawn += draw_debug_line(start, end, actor);
		}

		if (spritesDrawn >= maxSprites) {
			Printf("Overflow sprites!\n");
		}
	}

	for (int i = 0; i < numsubsectors; i++) {
		NavSector& node = nav_sectors[i];

		if (P_AproxDistance((node.x << FRACBITS) - player->x, (node.y << FRACBITS) - player->y) > (1000 << FRACBITS)) {
			continue;
		}

		SERVERCOMMANDS_SpawnBlood(node.x << FRACBITS, node.y << FRACBITS, (node.z + 16) << FRACBITS, 0, 100, player);
	}
}

int SectorNavMesh::get_nav_id(fixed_t x, fixed_t y) {
	return R_PointInSubsector(x, y) - subsectors;
}

int SectorNavMesh::get_nav_id(AActor* actor) {
	return get_nav_id(actor->x, actor->y);
}

float SectorNavMesh::path_cost(int a, int b) {
	NavSector& nodea = nav_sectors[a];
	NavSector& nodeb = nav_sectors[b];
	FVector3 delta = nodea.pos() - nodeb.pos();
	return delta.Length();
}

float SectorNavMesh::path_cost(NavSectorLink& link, bool timeSensitive) {
	NavSector& parent = nav_sectors[link.parent];
	NavSector& target = nav_sectors[link.target];
	float cost = 0;

	if (link.isTeleport)
		cost += (link.pos() - parent.pos()).Length();
	else
		cost += (parent.pos() - target.pos()).Length();

	if (link.linkWidth < PLAYER_WIDTH) {
		// try to avoid tiny links. They gets the bot stuck on corners.
		cost += 200 << FRACBITS;
	}

	if (!timeSensitive) {
		// avoid things that are hard to navigate if speed isn't important

		if (target.doesDamage) {
			cost += 4000 << FRACBITS; // avoid damage sectors
		}

		if (link.isJump) {
			// jumps are error-prone
			cost += 1000 << FRACBITS;

			if (!link.isJumpHeightValid()) {
				// jumps that can't be made yet are even more error prone
				cost += 4000 << FRACBITS;
			}
		}
	}

	if (!can_cross_seg_now(link.seg)) {
		if (target.getFloorZ() > parent.getFloorZ() + (JUMP_HEIGHT << FRACBITS))
			cost += 4000 << FRACBITS; // avoid elevators
	}

	return cost;
}

vector<int> SectorNavMesh::get_astar_route(int startSubSectorId, int endSubSectorId, unordered_set<int>* blockedPaths, bool timeSensitive)
{
	unordered_set<int> closedSet;
	unordered_set<int> openSet;

	unordered_map<int, float> gScore;
	unordered_map<int, float> fScore;
	unordered_map<int, int> cameFrom;

	vector<int> emptyRoute;

	if (verbose) {
		Printf("START route from %d to %d\n", startSubSectorId, endSubSectorId);
	}

	if (startSubSectorId < 0 || endSubSectorId < 0 || startSubSectorId >= numsubsectors || endSubSectorId >= numsubsectors) {
		Printf("AStarRoute: invalid start/end nodes\n");
		return emptyRoute;
	}

	if (startSubSectorId == endSubSectorId) {
		emptyRoute.push_back(startSubSectorId);
		return emptyRoute;
	}

	NavSector& start = nav_sectors[startSubSectorId];
	NavSector& goal = nav_sectors[endSubSectorId];

	openSet.insert(startSubSectorId);
	gScore[startSubSectorId] = 0;
	fScore[startSubSectorId] = path_cost(start.id, goal.id);

	const int maxIter = 8192;
	int curIter = 0;
	while (!openSet.empty()) {

		if (++curIter > maxIter) {
			Printf("AStarRoute exceeded max iterations searching path (%d)", maxIter);
			break;
		}

		// get node in openset with lowest cost
		int current = -1;
		float bestScore = 9e99;
		for (int nodeId : openSet)
		{
			float score = fScore[nodeId];
			if (score < bestScore) {
				bestScore = score;
				current = nodeId;
			}
		}

		if (current == goal.id) {
			// goal reached, build the route
			vector<int> path;
			path.push_back(current);

			int maxPathLen = 1000;
			int i = 0;
			while (cameFrom.count(current)) {
				current = cameFrom[current];
				path.push_back(current);
				if (++i > maxPathLen) {
					Printf("AStarRoute exceeded max path length (%d)", maxPathLen);
					break;
				}
			}
			reverse(path.begin(), path.end());

			if (verbose) {
				Printf("FINISH route calculation from %d to %d. Size is %d. visited %d nodes\n", startSubSectorId, endSubSectorId, path.size(), closedSet.size());
			}

			return path;
		}

		openSet.erase(current);
		closedSet.insert(current);

		NavSector& currentNode = nav_sectors[current];

		for (int i = 0; i < currentNode.links.size(); i++) {
			NavSectorLink& link = currentNode.links[i];
			if (link.target == -1) {
				break;
			}

			int neighbor = link.target;
			if (neighbor < 0 || neighbor >= numsubsectors)
				continue;

			if (closedSet.count(neighbor))
				continue;

			if (blockedPaths && blockedPaths->count(link.id))
				continue;

			// discover a new node
			openSet.insert(neighbor);

			// The distance from start to a neighbor
			NavSector& neighborNode = nav_sectors[neighbor];

			float tentative_gScore = gScore[current];
			tentative_gScore += path_cost(link, timeSensitive);

			if (link.linkWidth < PLAYER_WIDTH) {
				// try to avoid tiny links. They gets the bot stuck on corners.
				tentative_gScore += 200 << FRACBITS;
			}

			float neighbor_gScore = 9e99;
			if (gScore.count(neighbor))
				neighbor_gScore = gScore[neighbor];

			if (tentative_gScore >= neighbor_gScore)
				continue; // not a better path

			// This path is the best until now. Record it!
			cameFrom[neighbor] = current;
			gScore[neighbor] = tentative_gScore;
			fScore[neighbor] = tentative_gScore + path_cost(neighborNode.id, goal.id);
		}
	}

	return emptyRoute;
}

bool SectorNavMesh::get_key_goals_for_line(AActor* actor, line_t* line, vector<BotGoal>& keyGoals, std::unordered_set<int>* blockedPaths) {
	if (line->special != Door_LockedRaise)
		return true; // not a locked door

	if (P_CheckKeys(actor, line->args[3], false))
		return true; // already have all the keys

	TArray<TArray<PClass*>> keyGroups = P_GetRequiredKeys(line->args[3]);

	if (keyGroups.Size() == 0)
		return true; // no keys required

	// Get routes to all keys in the map, so the closest one can be selected
	struct KeyRoute {
		AKey* key;
		int routeSize;
	};

	int actorNavId = get_nav_id(actor);
	unordered_map<PClass*, KeyRoute> mapKeys;
	TThinkerIterator<AKey> it;
	AKey* mapKey;
	while ((mapKey = it.Next()) != NULL) {
		if (mapKey->Owner)
			continue; // key in someone's inventory

		int keyNavId = get_nav_id(mapKey);

		KeyRoute keyRoute;
		keyRoute.key = mapKey;
		keyRoute.routeSize = get_route_distance(get_astar_route(actorNavId, keyNavId, blockedPaths));

		mapKeys[mapKey->GetClass()] = keyRoute;
	}

	for (int i = 0; i < keyGroups.Size(); i++) {
		TArray<PClass*>& group = keyGroups[i];

		int bestKeyDist = INT_MAX;
		AKey* bestKey = NULL;

		if (group.Size()) {
			for (int k = 0; k < group.Size(); k++) {
				auto key = mapKeys.find(group[k]);
				if (key != mapKeys.end() && key->second.routeSize < bestKeyDist) {
					bestKeyDist = key->second.routeSize;
					bestKey = key->second.key;
				}
			}
		}
		else {
			// empty group means any key will work
			for (auto item : mapKeys) {
				if (item.second.routeSize < bestKeyDist) {
					bestKeyDist = item.second.routeSize;
					bestKey = item.second.key;
				}
			}
		}

		if (!bestKey) {
			// no key satisfies the group requirement
			Printf("Impossible key requirements for line %d:\n", line - lines);

			for (int i = 0; i < keyGroups.Size(); i++) {
				TArray<PClass*>& group = keyGroups[i];
				for (int k = 0; k < group.Size(); k++) {
					Printf("  %s", group[k]->TypeName.GetChars());
				}
				Printf("\n");
			}
			Printf("Map keys:\n");
			for (auto item : mapKeys) {
				Printf("   %s", item.second.key->GetClass()->TypeName.GetChars());
			}
			Printf("\n");

			return false;
		}

		BotGoal goal = BotGoal(WBOT_GOAL_ACTION_TOUCH, bestKey);
		goal.dist = bestKeyDist;
		keyGoals.push_back(goal);
	}

	return true;
}

std::vector<int> SectorNavMesh::GetTouchedSubsectors(AActor* actor) {
	unordered_set<int> subs;

	fixed_t r = actor->radius;
	fixed_t d = FixedMul(r, 46341); // diagonal radius

	subs.insert(R_PointInSubsector(actor->x,		actor->y)		- subsectors);

	// axes
	subs.insert(R_PointInSubsector(actor->x + r,	actor->y	)	- subsectors);
	subs.insert(R_PointInSubsector(actor->x - r,	actor->y	)	- subsectors);
	subs.insert(R_PointInSubsector(actor->x,		actor->y + r)	- subsectors);
	subs.insert(R_PointInSubsector(actor->x,		actor->y - r)	- subsectors);

	// diagonals
	subs.insert(R_PointInSubsector(actor->x + d, actor->y + d) - subsectors);
	subs.insert(R_PointInSubsector(actor->x + d, actor->y - d) - subsectors);
	subs.insert(R_PointInSubsector(actor->x - d, actor->y + d) - subsectors);
	subs.insert(R_PointInSubsector(actor->x - d, actor->y - d) - subsectors);

	std::vector<int> ret;
	for (auto item : subs) {
		ret.push_back(item);
	}

	return ret;
}
