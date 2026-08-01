#include "wb_goal.h"
#include "wb_nav.h"
#include "wb_map.h"
#include "wb_util.h"

#include <string>
#include <cmath>

using namespace std;
using namespace wbot;

#define ROCKET_EXPLODE_RADIUS 96 // reduced a bit, just in case
#define ROCKET_RADIUS 20

std::string BotGoal::desc() const {
	std::string thingName;
	if (h_actor.get()) {
		thingName = g_engine.get_actor_state(h_actor.get()).name;
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
	if (h_actor.get()) {
		if (shootAlignment.subid >= 0)
			return shootAlignment.subid;
		return g_wb_nav.get_nav_id(h_actor.get());
	}
	else if (lineid >= 0) {
		int ret = g_map.line_subsectors[lineid];

		if (ret == -1)
			g_engine.printf("Failed to find subsector for line %d\n", lineid);
		else {
			NavSector& node = g_wb_nav.mesh.nodes[ret];
			if (node.links.empty() && (action == WBOT_GOAL_ACTION_USE || action == WBOT_GOAL_ACTION_SHOOT)) {
				// line is in an unreachable sector.
				// Try tracing in front of it to see if it can be activated from a sector nearby
				float use_dist = 64; // TODO: get actor use range
				MapLine& line = g_map.lines[lineid];
				vec2 center = line.center();
				vec2 front = center + line.normal() * use_dist;
				center += (front - center).normalize(); // nudge to prevent collision with this line
				MapSubsector* centersub = &g_map.subsectors[ret];
				MapSubsector* frontsub = g_map.GetSubsector(front.x, front.y);
				float startZ = centersub->sector->getFloorZ();
				float endZ = frontsub->sector->getFloorZ();

				if (fabs(startZ - endZ) < JUMP_HEIGHT && !g_engine.TraceImpassable(center, front)) {
					// no impassable walls between the line sector and the one in front
					// try routing to that subsector instead
					ret = frontsub - g_map.subsectors;
				}
			}
		}

		return ret;
	}
	else {
		g_engine.printf("Routing not implemented for this type of goal\n");
	}

	return -1;
}

vec3 BotGoal::pos() {
	if (h_actor.get()) {
		vec3 actorPos = g_engine.get_actor_state(h_actor.get()).origin;
		return vec3(actorPos.x, actorPos.y, g_map.GetSector(h_actor.get())->getFloorZ());
	}
	else if (lineid >= 0) {
		MapLine& line = g_map.lines[lineid];
		float z = line.frontsector->getFloorZ();
		return vec3(line.center(), z);
	}

	g_engine.printf("Goal has no actor nor lineid\n");
	return vec3(0, 0, 0);
}

int BotGoal::touchDistance(AActor* toucher) {
	if (h_actor.get()) {
		return (g_engine.get_actor_state(h_actor.get()).radius + g_engine.get_actor_state(toucher).radius) - 1; // subtracted 1 unit just in case
	}
	else if (lineid >= 0) {
		return g_engine.get_actor_state(toucher).radius + 1; // added 1 in case wall is solid and you can't go inside it
	}

	return 0;
}

bool BotGoal::matches(const BotGoal& other) {
	return action == other.action && lineid == other.lineid && h_actor.get() == other.h_actor.get();
}

bool BotGoal::valid() const {
	if (lineid >= 0) {
		return g_map.lines[lineid].special() != 0;
	}
	return h_actor.get() != NULL;
}

void BotGoal::TestBossBrainShootRay(vec3 brainPos, vec3 rayStart, vec3 rayDir,
	bool isCeilTrace, unordered_map<int, IndirectShootPos>& shootNodes)
{
	vec3 impactPos = rayStart;

	if (!isCeilTrace) {
		vec3 impactPos = rayStart + rayDir * (ROCKET_EXPLODE_RADIUS + 64);
		if (!g_engine.TraceLine(rayStart, impactPos, true, NULL, NULL)) {
			return; // no impact
		}
	}

	float maxDist = (ROCKET_EXPLODE_RADIUS + ROCKET_RADIUS) + g_engine.get_actor_state(h_actor.get()).radius;

	if ((impactPos - brainPos).length() > maxDist) {
		return; // impact point not close enough to the target to do damage
	}

	// trace in the opposite direction to find a sector to shoot the impact point from
	TraceResult tr;
	vec3 shootPos = impactPos - rayDir * 4000;
	g_engine.TraceLine(impactPos, shootPos, true, NULL, &tr);
	shootPos = tr.endPos;
	vec3 shootDelta = shootPos - impactPos;
	float shootLen = shootDelta.length();

	// check vertical clearance for the rocket
	vec3 lowerPos = impactPos - vec3(0, 0, ROCKET_RADIUS / 2);
	g_engine.TraceLine(lowerPos, shootPos, true, NULL, &tr);
	if (tr.frac < 0.9f) {
		return; // not enough clearance
	}

	// find which sectors are interesected
	std::unordered_set<MapSector*> isectors;
	for (TraceIsect& isect : g_engine.TraceIntersections(rayStart, shootPos)) {
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
		vec2 segsect[2];
		for (int s = 0; s < sub.numsegs; s++) {
			MapSeg& seg = g_map.segs[sub.firstseg + s];
			if (DoLinesIntersect(seg.v1, seg.v2, impactPos, shootPos)) {
				segsect[numisect++] = LineIntersect(seg.v1, seg.v2, impactPos, shootPos);
			}
		}

		if (numisect == 0)
			continue; // no intersection

		float floorZ = sec->getFloorZ();
		float maxZ = floorZ + (VIEW_HEIGHT + 8);
		float minZ = floorZ + (VIEW_HEIGHT - 8);

		float isectLen1 = (segsect[0] - impactPos).length();
		float fShootLen = shootLen;

		float frac1 = isectLen1 / fShootLen;

		IndirectShootPos shoot;
		shoot.subid = k;
		shoot.shootAt = impactPos;

		if (numisect == 2) {
			// line passes thru this subsector
			float isectLen2 = (segsect[1] - impactPos).length();
			float frac2 = isectLen2 / fShootLen;
			
			vec3 start = impactPos + shootDelta * frac1;
			vec3 end = impactPos + shootDelta * frac2;
			vec3 goodStart, goodEnd;
			if (LineIntersectsZRange(start, end, minZ, maxZ, goodStart, goodEnd)) {
				// anywhere along this line is a good place to shoot from
				shoot.shootFrom = goodStart + (goodEnd - goodStart) * 0.5f;
				shootNodes[k] = shoot;
			}
		}
		else if (numisect == 1) {
			// line terminates in this sector
			int subid = g_map.GetSubsector(shootPos.x, shootPos.y) - g_map.subsectors;

			if (subid == k) {
				// line hits the floor and an earlier point on the line is a good shooting height

				vec3 start = impactPos + shootDelta * frac1;
				vec3 goodStart, goodEnd;
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
	vec3 actorPos = g_engine.get_actor_state(h_actor.get()).origin;
	const int maxPitch = 20;

	unordered_map<int, IndirectShootPos> shootFromNodes;

	// test impacts against nearby walls
	for (int h = 0; h < 128; h += 8) {
		vec3 abovePos = actorPos + vec3(0, 0, h);

		for (int p = -maxPitch; p <= maxPitch; p++) {
			float pitchRad = p * (M_PI / 180.0f);
			float pitchCos = cosf(pitchRad);
			float pitchSin = sinf(pitchRad);

			for (int i = 0; i < 360; i += 90) {
				float rad = i * (M_PI / 180.0f);
				vec3 dir(cosf(rad) * pitchCos, sinf(rad) * pitchCos, pitchSin);
				TestBossBrainShootRay(actorPos, abovePos, dir, false, shootFromNodes);
			}
		}
	}

	// test impacts against the ceiling
	vec3 ceilPos = actorPos;
	ceilPos.z = g_map.GetSector(h_actor.get())->getCeilZ();
	ceilPos.z -= 1.0f;
	//ceilPos.Z -= (ROCKET_RADIUS / 2) << FRACBITS;

	for (int p = 1; p < maxPitch; p++) {
		float pitchRad = p * (M_PI / 180.0f);
		float pitchCos = cosf(pitchRad);
		float pitchSin = sinf(pitchRad);

		for (int i = 0; i < 360; i += 90) {
			float rad = i * (M_PI / 180.0f);
			vec3 dir(cosf(rad) * pitchCos, sinf(rad) * pitchCos, pitchSin);
			TestBossBrainShootRay(actorPos, ceilPos, dir, true, shootFromNodes);
		}
	}


	return shootFromNodes;
}