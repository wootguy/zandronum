#include "wb_goal.h"
#include "wnav.h"
#include "r_state.h"
#include <string>

using namespace std;

std::string BotGoal::desc() {
	std::string thingName;
	if (actor) {
		thingName = actor->GetClass()->TypeName.GetChars();
	}
	else if (lineid) {
		thingName = "Linedef " + to_string(lineid);
	}

	thingName += " in sector " + to_string(getNavId());

	string blockerStr;
	for (const int& id : blockers) {
		blockerStr += " " + to_string(id);
	}
	if (blockerStr.size()) {
		thingName += "   (blocked at " + blockerStr + ")";
	}

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

int BotGoal::getNavId() {
	if (actor) {
		return g_wbot_nav.get_nav_id(actor);
	}
	else if (lineid >= 0) {
		int ret = g_wbot_nav.line_subsectors[lineid];

		if (ret == -1)
			Printf("Failed to find subsector for line %d\n", lineid);

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
}

bool BotGoal::valid() {
	if (lineid >= 0) {
		return lines[lineid].special != 0;
	}
	return actor != NULL;
}
