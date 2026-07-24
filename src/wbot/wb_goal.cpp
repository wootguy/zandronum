#include "wb_goal.h"
#include "wb_nav.h"
#include "wb_map.h"
#include "wb_util.h"
#include "r_state.h"
#include "r_utility.h"
#include <string>

using namespace std;

std::string BotGoal::desc() {
	std::string thingName;
	if (actor) {
		thingName = actor->GetClass()->TypeName.GetChars();
	}
	else if (lineid) {
		thingName = "Line " + to_string(lineid);
	}

	thingName += " in " + to_string(getNavId());

	switch (action) {
	case WBOT_GOAL_ACTION_MOVE_TO:
		return "Move to " + thingName;
	case WBOT_GOAL_ACTION_USE:
		return "Use " + thingName;
	case WBOT_GOAL_ACTION_TOUCH:
		return "Touch " + thingName;
	case WBOT_GOAL_ACTION_CROSS:
		return "Cross " + thingName;
	case WBOT_GOAL_ACTION_SHOOT:
		return "Shoot " + thingName;
	}

	return "??? " + thingName;
}

std::string BotGoal::descLong() {
	string descStr = desc();

	string blockerStr;
	int numBlock = 0;
	for (const int& id : blockers) {
		blockerStr += " " + to_string(id);
		if (++numBlock >= 4) {
			break;
		}
	}
	if (numBlock < blockers.size()) {
		descStr += "\n        blockers: " + blockerStr + " (+" + to_string(blockers.size() - numBlock) + ")";
	}
	else if (numBlock > 0) {
		descStr += "\n        blockers: " + blockerStr;
	}

	string unblockStr;
	int numunBlock = 0;
	for (auto item : unblockAttempts) {
		//unblockStr += " [" + to_string(item.first) + "->" + to_string(item.second) + "]";
		unblockStr += " " + to_string(item.first);
		if (++numunBlock >= 4) {
			break;
		}
	}
	if (numunBlock < unblockAttempts.size()) {
		descStr += "\n        Used: " + unblockStr + " (+" + to_string(unblockStr.size() - numunBlock) + ")";
	}
	else if (numunBlock > 0) {
		descStr += "\n        Used: " + unblockStr;
	}

	return descStr;
}

int BotGoal::getNavId() {
	if (actor) {
		return g_wb_nav.get_nav_id(actor);
	}
	else if (lineid >= 0) {
		int ret = g_wb_mapinfo.line_subsectors[lineid];

		if (ret == -1)
			Printf("Failed to find subsector for line %d\n", lineid);
		else {
			NavSector& node = g_wb_nav.mesh[ret];
			if (node.links.empty() && (action == WBOT_GOAL_ACTION_USE || action == WBOT_GOAL_ACTION_SHOOT)) {
				// line is in an unreachable sector.
				// Try tracing in front of it to see if it can be activated from a sector nearby
				fixed_t use_dist = 64 << FRACBITS; // TODO: get actor use range
				line_t* line = &lines[lineid];
				FVector2 center = getLineCenter(line);
				FVector2 front = center + getLineBackDir(line) * -use_dist;
				center += (front - center).Unit() * FRACUNIT; // nudge to prevent collision with this line
				subsector_t* centersub = &subsectors[ret];
				subsector_t* frontsub = R_PointInSubsector(front.X, front.Y);
				fixed_t startZ = centersub->sector->floorplane.ZatPoint((fixed_t)center.X, (fixed_t)center.Y);
				fixed_t endZ = frontsub->sector->floorplane.ZatPoint((fixed_t)front.X, (fixed_t)front.Y);

				if (abs(startZ - endZ) < (JUMP_HEIGHT << FRACBITS) && !TraceImpassable(center, front)) {
					// no impassable walls between the line sector and the one in front
					// try routing to that subsector instead
					ret = frontsub - subsectors;
				}
			}
		}

		return ret;
	}
	else {
		Printf("Routing not implemented for this type of goal\n");
	}

	return -1;
}

FVector3 BotGoal::pos() {
	if (actor) {
		return FVector3(actor->x, actor->y, actor->Sector->floorplane.ZatPoint(actor->x, actor->y));
	}
	else if (lineid >= 0) {
		line_t& line = lines[lineid];
		fixed_t x = (line.v1->x + line.v2->x) / 2;
		fixed_t y = (line.v1->y + line.v2->y) / 2;
		fixed_t z = line.frontsector->floorplane.ZatPoint(x, y);
		return FVector3(x, y, z);
	}

	Printf("Goal has no actor nor lineid\n");
	return FVector3(0, 0, 0);
}

int BotGoal::touchDistance(AActor* toucher) {
	if (actor) {
		return ((actor->radius + toucher->radius) >> FRACBITS) - 1; // subtracted 1 unit just in case
	}
	else if (lineid >= 0) {
		return (toucher->radius >> FRACBITS) + 1; // added 1 in case wall is solid and you can't go inside it
	}

	return 0;
}

bool BotGoal::valid() {
	if (lineid >= 0) {
		return lines[lineid].special != 0;
	}
	return actor != NULL;
}
