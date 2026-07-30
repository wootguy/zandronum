#include "wb_goal.h"
#include "wb_nav.h"
#include "wb_map.h"
#include "wb_util.h"
#include "actor.h"
#include <string>

using namespace std;
using namespace wbot;

#define ROCKET_EXPLODE_RADIUS 96 // reduced a bit, just in case
#define ROCKET_RADIUS 20

std::string BotGoal::desc() const {
	std::string thingName;
	if ((TObjPtr<AActor>)actor) {
		thingName = ((TObjPtr<AActor>)actor)->GetClass()->TypeName.GetChars();
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
	case WBOT_GOAL_ACTION_BOSS_BRAIN:
		return "Rocket " + thingName;
	}

	return "??? " + thingName;
}

std::string BotGoal::descLong() const {
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

int BotGoal::getNavId() const {
	if (const_cast<TObjPtr<AActor>&>(actor)) {
		if (shootAlignment.subid >= 0)
			return shootAlignment.subid;
		return g_wb_nav.get_nav_id(const_cast<TObjPtr<AActor>&>(actor));
	}
	else if (lineid >= 0) {
		int ret = g_map.line_subsectors[lineid];

		if (ret == -1)
			Printf("Failed to find subsector for line %d\n", lineid);
		else {
			NavSector& node = g_wb_nav.mesh.nodes[ret];
			if (node.links.empty() && (action == WBOT_GOAL_ACTION_USE || action == WBOT_GOAL_ACTION_SHOOT)) {
				// line is in an unreachable sector.
				// Try tracing in front of it to see if it can be activated from a sector nearby
				fixed_t use_dist = 64 << FRACBITS; // TODO: get actor use range
				MapLine& line = g_map.lines[lineid];
				FVector2 center = line.center();
				FVector2 front = center + line.normal() * use_dist;
				center += (front - center).Unit() * FRACUNIT; // nudge to prevent collision with this line
				MapSubsector* centersub = &g_map.subsectors[ret];
				MapSubsector* frontsub = g_map.GetSubsector(front.X, front.Y);
				fixed_t startZ = centersub->sector->getFloorZ();
				fixed_t endZ = frontsub->sector->getFloorZ();

				if (abs(startZ - endZ) < (JUMP_HEIGHT << FRACBITS) && !TraceImpassable(center, front)) {
					// no impassable walls between the line sector and the one in front
					// try routing to that subsector instead
					ret = frontsub - g_map.subsectors;
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
		return FVector3(actor->x, actor->y, g_map.GetSector(actor)->getFloorZ());
	}
	else if (lineid >= 0) {
		MapLine& line = g_map.lines[lineid];
		fixed_t z = line.frontsector->getFloorZ();
		return FVector3(line.center(), z);
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

bool BotGoal::valid() const {
	if (lineid >= 0) {
		return g_map.lines[lineid].special() != 0;
	}
	return const_cast<TObjPtr<AActor>&>(actor) != NULL;
}

void BotGoal::TestBossBrainShootRay(FVector3 brainPos, FVector3 rayStart, FVector3 rayDir,
	bool isCeilTrace, unordered_map<int, IndirectShootPos>& shootNodes)
{
	FVector3 impactPos = rayStart;

	if (!isCeilTrace) {
		FVector3 impactPos = rayStart + rayDir * ((ROCKET_EXPLODE_RADIUS + 64) << FRACBITS);
		if (!TraceLine(rayStart, impactPos, true)) {
			return; // no impact
		}
	}

	fixed_t maxDist = ((ROCKET_EXPLODE_RADIUS + ROCKET_RADIUS) << FRACBITS) + actor->radius;

	if ((impactPos - brainPos).Length() > maxDist) {
		return; // impact point not close enough to the target to do damage
	}

	// trace in the opposite direction to find a sector to shoot the impact point from
	TraceResult tr;
	FVector3 shootPos = impactPos - rayDir * (4000 << FRACBITS);
	TraceLine(impactPos, shootPos, true, NULL, &tr);
	shootPos = tr.endPos;
	FVector3 shootDelta = shootPos - impactPos;
	fixed_t shootLen = shootDelta.Length();

	// check vertical clearance for the rocket
	FVector3 lowerPos = impactPos - FVector3(0, 0, (ROCKET_RADIUS / 2) << FRACBITS);
	TraceLine(lowerPos, shootPos, true, NULL, &tr);
	if (tr.frac < 0.9f) {
		return; // not enough clearance
	}

	// find which sectors are interesected
	std::unordered_set<MapSector*> isectors;
	for (TraceIsect& isect : TraceIntersections(rayStart, shootPos)) {
		isectors.insert(isect.sector);
	}

	// do intersection tests against subsectors to find something to route to
	for (int k = 0; k < g_map.numsubsectors; k++) {
		MapSubsector& sub = g_map.subsectors[k];
		MapSector* sec = sub.sector;
		if (!isectors.count(sec))
			continue; // sector not intersected

		NavSector& nav = g_wb_nav.mesh.nodes[k];

		// test if the shoot line intersects this subsector
		int numisect = 0;
		FVector2 segsect[2];
		for (int s = 0; s < sub.numsegs; s++) {
			MapSeg& seg = g_map.segs[sub.firstseg + s];
			FVector2 v1 = seg.start();
			FVector2 v2 = seg.end();
			if (DoLinesIntersect(v1, v2, impactPos, shootPos)) {
				segsect[numisect++] = LineIntersect(v1, v2, impactPos, shootPos);
			}
		}

		if (numisect == 0)
			continue; // no intersection

		fixed_t floorZ = sec->getFloorZ();
		fixed_t maxZ = floorZ + ((VIEW_HEIGHT + 8) << FRACBITS);
		fixed_t minZ = floorZ + ((VIEW_HEIGHT - 8) << FRACBITS);

		float frac1 = FixedDiv((segsect[0] - impactPos).Length(), shootLen) / (float)FRACUNIT;
		fixed_t z1 = (shootPos + shootDelta * frac1).Z;

		IndirectShootPos shoot;
		shoot.subid = k;
		shoot.shootAt = impactPos;

		if (numisect == 2) {
			// line passes thru this subsector
			float frac2 = FixedDiv((segsect[1] - impactPos).Length(), shootLen) / (float)FRACUNIT;
			fixed_t z2 = (shootPos + shootDelta * frac2).Z;
			
			FVector3 start = impactPos + frac1 * shootDelta;
			FVector3 end = impactPos + frac2 * shootDelta;
			FVector3 goodStart, goodEnd;
			if (LineIntersectsZRange(start, end, minZ, maxZ, goodStart, goodEnd)) {
				// anywhere along this line is a good place to shoot from
				shoot.shootFrom = goodStart + (goodEnd - goodStart) * 0.5f;
				shootNodes[k] = shoot;
			}
		}
		else if (numisect == 1) {
			// line terminates in this sector
			int subid = g_map.GetSubsector(shootPos.X, shootPos.Y) - g_map.subsectors;

			if (subid == k) {
				// line hits the floor and an earlier point on the line is a good shooting height

				FVector3 start = impactPos + frac1 * shootDelta;
				FVector3 goodStart, goodEnd;
				if (LineIntersectsZRange(start, shootPos, minZ, maxZ, goodStart, goodEnd)) {
					// anywhere along this line is a good place to shoot from
					shoot.shootFrom = goodStart + (goodEnd - goodStart) * 0.5f;
					shootNodes[k] = shoot;
				}
			}
		}
	}
}

unordered_map<int, IndirectShootPos> BotGoal::FindBossBrainShootPositions() {
	// boss brain is normally unreachable, but can be damaged by rockets
	FVector3 actorPos(actor->x, actor->y, actor->z);
	const int maxPitch = 20;

	unordered_map<int, IndirectShootPos> shootFromNodes;

	// test impacts against nearby walls
	for (int h = 0; h < 128; h += 8) {
		FVector3 abovePos = actorPos + FVector3(0, 0, h << FRACBITS);

		for (int p = -maxPitch; p <= maxPitch; p++) {
			float pitchRad = p * (M_PI / 180.0f);
			float pitchCos = cosf(pitchRad);
			float pitchSin = sinf(pitchRad);

			for (int i = 0; i < 360; i += 90) {
				float rad = i * (M_PI / 180.0f);
				FVector3 dir(cosf(rad) * pitchCos, sinf(rad) * pitchCos, pitchSin);
				TestBossBrainShootRay(actorPos, abovePos, dir, false, shootFromNodes);
			}
		}
	}

	// test impacts against the ceiling
	FVector3 ceilPos = actorPos;
	ceilPos.Z = g_map.GetSector(actor)->getCeilZ();
	ceilPos.Z -= FRACUNIT;
	//ceilPos.Z -= (ROCKET_RADIUS / 2) << FRACBITS;

	for (int p = 1; p < maxPitch; p++) {
		float pitchRad = p * (M_PI / 180.0f);
		float pitchCos = cosf(pitchRad);
		float pitchSin = sinf(pitchRad);

		for (int i = 0; i < 360; i += 90) {
			float rad = i * (M_PI / 180.0f);
			FVector3 dir(cosf(rad) * pitchCos, sinf(rad) * pitchCos, pitchSin);
			TestBossBrainShootRay(actorPos, ceilPos, dir, true, shootFromNodes);
		}
	}


	return shootFromNodes;
}