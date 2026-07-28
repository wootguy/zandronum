#include "wb_map.h"
#include "wb_util.h"
#include "wb_nav.h"
#include "r_utility.h"
#include "p_lnspec.h"
#include "p_spec.h"
#include <algorithm>

using namespace std;

BotMapInfo g_wb_mapinfo;

void BotMapInfo::init() {
	if (sector_info) {
		delete[] sector_info;
		sector_info = NULL;
	}
	if (line_subsectors) {
		delete[] line_subsectors;
		line_subsectors = NULL;
	}

	sector_info = new BotSectorInfo[numsectors];

	line_subsectors = new int[numlines];
	memset(line_subsectors, -1, sizeof(int) * numlines);

	find_linedef_sectors();
	add_sector_info();
}

std::vector<int> BotMapInfo::GetTouchedSubsectors(AActor* actor) {
	unordered_set<int> subs;

	fixed_t r = actor->radius;
	fixed_t d = FixedMul(r, 46341); // diagonal radius

	subs.insert(R_PointInSubsector(actor->x, actor->y) - subsectors);

	// axes
	subs.insert(R_PointInSubsector(actor->x + r, actor->y) - subsectors);
	subs.insert(R_PointInSubsector(actor->x - r, actor->y) - subsectors);
	subs.insert(R_PointInSubsector(actor->x, actor->y + r) - subsectors);
	subs.insert(R_PointInSubsector(actor->x, actor->y - r) - subsectors);

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

LinkSeg BotMapInfo::GetSegmentOverlap(seg_t* a, seg_t* b)
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

bool BotMapInfo::is_seg_potentially_crossable(seg_t* seg) {
	if (!seg->frontsector || !seg->backsector)
		return false;

	if (seg->linedef && (seg->linedef->flags & ML_BLOCKING))
		return false; // impassable

	int frontMovement = sector_info[seg->frontsector - sectors].moveFlags;
	int backMovement = sector_info[seg->backsector - sectors].moveFlags;

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
	fixed_t frontCeil = seg->frontsector->ceilingplane.ZatPoint(x, y);
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

int BotMapInfo::segment_walkability(seg_t* seg)
{
	fixed_t x = (seg->v1->x + seg->v2->x) / 2;
	fixed_t y = (seg->v1->y + seg->v2->y) / 2;

	fixed_t frontFloor = seg->frontsector->floorplane.ZatPoint(x, y);
	fixed_t backFloor = seg->backsector->floorplane.ZatPoint(x, y);

	const bool canJump = true;
	int maxHeight = canJump ? JUMP_HEIGHT : STEP_HEIGHT;

	if ((backFloor - frontFloor) > (maxHeight << FRACBITS)) {
		return LINK_BLOCK_TOO_HIGH; // too high to step
	}

	fixed_t backCeil = seg->backsector->ceilingplane.ZatPoint(x, y);
	fixed_t frontCeil = seg->frontsector->ceilingplane.ZatPoint(x, y);
	const fixed_t fduckHeight = DUCK_HEIGHT << FRACBITS;

	fixed_t backHeight = backCeil - backFloor;
	fixed_t borderHeight = std::min(backCeil - frontFloor, frontCeil - backFloor);

	if (backHeight < fduckHeight || borderHeight < fduckHeight) {
		return LINK_BLOCK_TOO_LOW; // not enough space in target sector or border
	}

	return LINK_BLOCK_CLEAR;
}

LinkSeg BotMapInfo::get_neighbor_subsector(subsector_t* ignoreSector, seg_t* borderSeg) {
	const fixed_t epsilonWidth = 2 << FRACBITS;
	LinkSeg ret;
	memset(&ret, 0, sizeof(ret));
	ret.otherSub = -1;

	float bestLen = epsilonWidth;
	bool foundSeg = false;

	for (int j = 0; j < numsubsectors; j++) {
		subsector_t& otherSub = subsectors[j];
		if (&otherSub == ignoreSector)
			continue;

		for (int s = 0; s < otherSub.numlines; s++) {
			seg_t& tseg = otherSub.firstline[s];

			if (!tseg.frontsector || !tseg.backsector)
				continue; // impassable wall

			// TODO: Allow thin segments for tightrope areas
			LinkSeg lseg = GetSegmentOverlap(&tseg, borderSeg);
			float len = lseg.length();
			if (len >= bestLen) {
				ret = lseg;
				ret.idx = s;
				ret.otherSub = j;
				bestLen = len;
			}
		}
	}

	return ret;
}

int BotMapInfo::get_linedef_move_flag(line_t* line) {
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
		Printf("Unknown special %d for line %d\n", line->special, line - lines);
		return 0;
	}
}

int BotMapInfo::get_linedef_goal_action(line_t* line) {
	if (!line)
		return -1;

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

	if (line->activation)
		Printf("Don't know how to activate line %d\n", line - lines);
	
	return -1;
}

bool BotMapInfo::subsector_does_damage(subsector_t* sub) {
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

void BotMapInfo::add_stair_sector_info() {
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

		BotGoal stairTrigger = BotGoal(get_linedef_goal_action(&line), s);

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

					BotSectorInfo& info = sector_info[tsec - sectors];
					info.moveFlags |= moveFlags;
					info.triggers.push_back(stairTrigger);
				}
			} while (ok);
		}
	}
}

void BotMapInfo::find_linedef_sectors() {
	// create linedef sector mapping
	for (int i = 0; i < numlines; i++) {
		line_t& line = lines[i];
		if (!line.special) {
			continue;
		}

		int lineAction = get_linedef_goal_action(&line);
		bool doubleSidedCrossLine = lineAction == WBOT_GOAL_ACTION_CROSS && (line.flags & ML_TWOSIDED);

		FVector2 center = getLineCenter(&line);
		FVector2 normal = getLineBackDir(&line) * -1;


		// route to a nearby sector if the adjacent one is too small to fit a player inside
		fixed_t useDist = BoxRadiusForDir(normal, PLAYER_RADIUS << FRACBITS) + FRACUNIT;

		if (lineAction == WBOT_GOAL_ACTION_CROSS || lineAction == WBOT_GOAL_ACTION_TOUCH) {
			// need to be very close to the line
			useDist = FRACUNIT;
		}

		FVector2 frontPoint = center + normal * useDist;
		FVector2 backPoint = center - normal * useDist;
		int frontSubId = R_PointInSubsector(frontPoint.X, frontPoint.Y) - subsectors;
		int backSubId = R_PointInSubsector(backPoint.X, backPoint.Y) - subsectors;
		int routeToId = frontSubId;

		if (doubleSidedCrossLine) {
			// pick the side that allows crossing so that bot doesn't try to cross lines from the bottom of a cliff
			fixed_t frontZ = subsectors[frontSubId].sector->floorplane.ZatPoint((fixed_t)frontPoint.X, (fixed_t)frontPoint.Y);
			fixed_t backZ = subsectors[backSubId].sector->floorplane.ZatPoint((fixed_t)frontPoint.X, (fixed_t)frontPoint.Y);
			
			if (backZ > frontZ) {
				routeToId = backSubId;
			}
		}

		line_subsectors[i] = routeToId;
	}
}

void BotMapInfo::add_sector_info() {
	for (int i = 0; i < numsectors; i++) {
		sector_t& sec = sectors[i];
		BotSectorInfo& info = sector_info[i];

		// add flags for lines that trigger this sector by tag
		if (sec.tag) {
			for (int k = 0; k < numlines; k++) {
				line_t& line = lines[k];

				if (!line.args[0] || line.args[0] != sec.tag)
					continue;

				int flags = get_linedef_move_flag(&line);

				if (flags) {
					info.moveFlags |= flags;
					info.triggers.push_back(BotGoal(get_linedef_goal_action(&line), k));
				}
			}
		}

		// lines that have this sector as a backsector also trigger it, if no tag
		for (int k = 0; k < sec.linecount; k++) {
			line_t* line = sec.lines[k];

			if (line->args[0] || line->backsector != &sec)
				continue;

			int flags = get_linedef_move_flag(line);

			if (flags) {
				info.moveFlags |= flags;
				info.triggers.push_back(BotGoal(get_linedef_goal_action(line), line - lines));
			}
		}
	}

	add_stair_sector_info();
}

void BotMapInfo::remove_invalid_goals(int secid) {
	BotSectorInfo& info = sector_info[secid];

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
		g_wb_nav.relink_sector(&sectors[secid]);
	}
}