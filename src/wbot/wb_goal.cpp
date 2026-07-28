#include "wb_goal.h"
#include "wb_nav.h"
#include "wb_map.h"
#include "wb_util.h"
#include "r_state.h"
#include "r_utility.h"
#include "p_trace.h"
#include <string>

using namespace std;

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
		int ret = g_wb_mapinfo.line_subsectors[lineid];

		if (ret == -1)
			Printf("Failed to find subsector for line %d\n", lineid);
		else {
			NavSector& node = g_wb_nav.mesh.nodes[ret];
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

bool BotGoal::valid() const {
	if (lineid >= 0) {
		return lines[lineid].special != 0;
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
	FTraceResults tr;
	FVector3 shootPos = impactPos - rayDir * (4000 << FRACBITS);
	TraceLine(impactPos, shootPos, true, NULL, &tr);
	shootPos = FVector3(tr.X, tr.Y, tr.Z);
	FVector3 shootDelta = shootPos - impactPos;
	fixed_t shootLen = shootDelta.Length();

	// find which sectors are interesected
	std::unordered_set<sector_t*> isectors;
	for (TraceIsect& isect : TraceIntersections(rayStart, shootPos)) {
		isectors.insert(isect.sector);
	}

	// do intersection tests against subsectors to find something to route to
	for (int k = 0; k < numsubsectors; k++) {
		subsector_t& sub = subsectors[k];
		sector_t* sec = sub.sector;
		if (!isectors.count(sec))
			continue; // sector not intersected

		NavSector& nav = g_wb_nav.mesh.nodes[k];

		// test if the shoot line intersects this subsector
		int numisect = 0;
		FVector2 segsect[2];
		for (int s = 0; s < sub.numlines; s++) {
			seg_t& seg = sub.firstline[s];
			FVector2 v1(seg.v1->x, seg.v1->y);
			FVector2 v2(seg.v2->x, seg.v2->y);
			if (DoLinesIntersect(v1, v2, impactPos, shootPos)) {
				segsect[numisect++] = LineIntersect(v1, v2, impactPos, shootPos);
			}
		}

		if (numisect == 0)
			continue; // no intersection

		fixed_t floorZ = sec->floorplane.ZatPoint((fixed_t)segsect[0].X, (fixed_t)segsect[0].Y);
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
			int subid = R_PointInSubsector(shootPos.X, shootPos.Y) - subsectors;

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
	ceilPos.Z = actor->Sector->ceilingplane.ZatPoint(actor->x, actor->y);
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