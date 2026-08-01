#include "wb_nav.h"
#include "wb_bot.h"
#include "wb_map.h"
#include "wb_nav_gen.h"
#include "wb_util.h"
#include "wb_hooks.h"
#include "wb_eiface.h"

#include <algorithm>
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <vector>
#include <string>
#include <limits.h>
#include <cstring>
#include <cmath>
#include <float.h>

using namespace std;
using namespace wbot;

SectorNavMesh g_wb_nav;
int g_route_ignore_num;

vec2 NavSectorLink::pos() {
	return movePos;
}

vec3 NavSectorLink::pos3D() {
	return vec3(movePos.x, movePos.y, parent->sector->getFloorZ());
}

int NavSectorLink::blocked(AActor* actor, bool recurse) {
	g_wb_nav.pathTests++;

	// TODO: only do this when sectors move
	updateFlags();

	int walkability = g_map.sector_border_walkability(parent->sector, sector);
	if (walkability != LINK_BLOCK_CLEAR) {
		MapLine* line = linedef;
		LineState lstate = g_engine.get_line_state(line->id);

		if (line && lstate.args[0] == 0 && lstate.moveFlags) {
			if (actor && (lstate.flags & FL_LINE_IS_LOCKED_DOOR) && !g_engine.can_unlock_door(actor, line)) {
				return walkability; // don't have the keys required to use this
			}

			// border is an untagged linedef, which means it moves the target sector,
			// which probably unblocks the path. TODO: not always!
			return LINK_BLOCK_CLEAR;
		}

		return walkability;
	}

	const float pradius = PLAYER_RADIUS;
	float parentFloorZ = parent->sector->getFloorZ();
	vec3 jumpOverPos = vec3(movePos, parentFloorZ + JUMP_HEIGHT);
	vec3 duckUnderPos = vec3(movePos, parentFloorZ + DUCK_HEIGHT);
	if (IsBoxClipped(jumpOverPos, pradius, 0) && IsBoxClipped(duckUnderPos, pradius, 0)) {
		return LINK_BLOCK_CLIPPED;
	}

	// check if jumps are valid for dynamic links that depend on from/to sector states
	if (!jumpable()) {
		return LINK_BLOCK_CANT_JUMP;
	}

	return LINK_BLOCK_CLEAR;
}

std::vector<MapSector*> NavSectorLink::getClippedSectors(AActor* actor) {
	float bottomZ = parent->sector->getFloorZ() + JUMP_HEIGHT;
	return GetBoxClipSectors(vec3(movePos, bottomZ), PLAYER_RADIUS, 0);
}

bool NavSectorLink::walkable() {
	return g_map.sector_border_walkability(parent->sector, sector) == LINK_BLOCK_CLEAR && jumpable();
}

bool NavSectorLink::jumpable() {
	if (!isJump)
		return true;

	if (!isJumpValid()) {
		return false;
	}

	vec3 start = pos3D() + vec3(0, 0, 56);
	vec3 end = target->pos3D() + vec3(0, 0, 56);

	const float pradius = PLAYER_RADIUS;
	float targetFloorZ = target->sector->getFloorZ();
	vec3 jumpOverPos = vec3(end, targetFloorZ + JUMP_HEIGHT);
	vec3 duckUnderPos = vec3(end, targetFloorZ + DUCK_HEIGHT);
	if (IsBoxClipped(jumpOverPos, pradius, 0) && IsBoxClipped(duckUnderPos, pradius, 0)) {
		return false;
	}

	if (g_engine.TraceLine(start, end, true, NULL, NULL)) {
		return false;
	}

	return true;
}

bool NavSectorLink::isJumpValid() {
	float floorZ = parent->sector->getFloorZ();
	float jumpHeight = target->sector->getFloorZ() - floorZ;
	if (jumpHeight >= JUMP_HEIGHT)
		return false;

	if (jumpDist > JUMP_DIST - jumpHeight)
		return false; // too far to make it

	if (jumpNeighbor->sector->getFloorZ() - floorZ == 0)
		return false; // not currently an edge that can help with jumping

	return true;
}

void NavSectorLink::updateFlags() {
	bool oldCliff = isCliff;
	isCliff = parent->sector->getFloorZ() - target->sector->getFloorZ() > JUMP_HEIGHT;

	if (isCliff && !oldCliff) {
		parent->hasCliffs = true;
	}
	else if (!isCliff && oldCliff) {
		parent->hasCliffs = false;
		for (NavSectorLink* link : parent->links) {
			if (link->isCliff) {
				parent->hasCliffs = true;
				break;
			}
		}
	}
}

vec2 NavSectorLink::GetJumpStartPos(vec2 targetPos) {
	// bring points inward a bit to avoid getting too close to a cliff or wall
	float dist = (seg.b - seg.a).length();
	vec2 dir = (seg.b - seg.a).normalize();
	float nudgeDist = std::min(dist / 2.0f, 32.0f);
	vec2 a = seg.a + dir * nudgeDist;
	vec2 b = seg.b - dir * nudgeDist;

	return ClosestPointOnSegment(targetPos, a, b);
}

vec2 NavSectorLink::GetJumpEndPos(vec2 targetPos) {
	vec2 jumpPos = GetJumpStartPos(targetPos);

	// move the landing point from the center to the nearest ledge
	vec2 ledgeDir = (jumpPos - targetPos).normalize();

	MapLine* line;
	vec2 edgePos;
	if (g_engine.TraceSectorEdge(targetPos, targetPos + ledgeDir * 1000, edgePos, &line)) {
		// move edge away from the line endings to avoid collision with a wall
		vec2 a = line->v1;
		vec2 b = line->v2;
		vec2 dir = (b - a).normalize();
		float nudgeDist = std::min((b - a).length() / 2, 32.0f);
		a += dir * nudgeDist;
		b -= dir * nudgeDist;

		return ClosestPointOnSegment(jumpPos, a, b);
	}

	return targetPos;
}

NavSector* NavSectorLink::GetJumpBackupBlocker(vec2 targetPos) {
	vec2 startPos = GetJumpStartPos(targetPos);

	// back up the starting position to get a running start for the jump
	vec2 jumpDir = (targetPos - startPos).normalize();

	float maxBackupDist = 256;
	vec2 backupStartPos = startPos - jumpDir; // avoid clipping against the backoff line
	vec2 backupPos = startPos - jumpDir * maxBackupDist;
	float startZ = parent->sector->getFloorZ();
	int isects = 0;

	vec2 edge;
	MapLine* line;
	if (g_engine.TraceSectorEdge(backupStartPos, backupPos, edge, &line)) {
		MapSubsector& sub = g_map.subsectors[parent->id];
		for (NavSectorLink* link : parent->links) {
			if (link->linedef == line) {
				bool canMoveAway = link->target->sector->moveFlags & (FL_SECTOR_MOVE_FLOOR_DOWN | FL_SECTOR_MOVE_CEIL_UP);
				return canMoveAway ? link->target : NULL;
			}
		}
	}

	return NULL;
}

float NavSectorLink::GetJumpBackupSpaceNeeded(vec3 start, vec3 end) {
	float gap = jumpDist - PLAYER_WIDTH;

	const float maxBackup = 256;

	vec3 dir = (end - start).normalize();
	float distScale = dir.z + 1; // increase gap for upward jumps, decrease for downward

	return std::min(maxBackup, gap * distScale);
}

vec2 NavSectorLink::GetJumpBackupPos(vec2 targetPos, AActor* jumper) {
	vec2 startPos = GetJumpStartPos(targetPos);
	vec2 endPos = GetJumpEndPos(targetPos);
	vec3 startPos3D(startPos, parent->sector->getFloorZ());
	vec3 endPos3D(endPos, target->sector->getFloorZ());

	// back up the starting position to get a running start for the jump
	vec2 jumpDir = (endPos - startPos).normalize();

	float maxBackupDist = GetJumpBackupSpaceNeeded(startPos3D, endPos3D);
	vec2 backupStartPos = startPos - jumpDir; // avoid clipping against the backoff line
	vec2 backupPos = startPos - jumpDir * maxBackupDist;
	float startZ = parent->sector->getFloorZ();
	int isects = 0;

	for (TraceIsect& isect : g_engine.TraceIntersections(backupStartPos, backupPos)) {
		float sectorZ = isect.sector->getFloorZ();
		int heightDiff = (sectorZ - startZ);

		if (isect.line->isImpassable() || heightDiff > STEP_HEIGHT) {
			if (isects++ == 0) {
				backupPos = isect.pos;
			}
			break;
		}

		backupPos = isect.pos;
	}

	// clip against walls for our radius
	TraceResult tr;
	vec2 backupDelta = backupPos - backupStartPos;
	float traceZ = startZ + STEP_HEIGHT;
	if (TraceRadius(vec3(backupStartPos, traceZ), vec3(backupPos, traceZ), PLAYER_WIDTH / 2, false, jumper, &tr)) {
		backupPos = backupStartPos + backupDelta * tr.frac;
	}

	float backupDist = (backupPos - backupStartPos).length();
	float nudgeDist = std::min(8.0f, backupDist);
	backupPos += jumpDir * nudgeDist;

	return backupPos;
}


vec3 NavSector::pos3D() {
	return vec3(center, sector->getFloorZ());
}

vec2 NavSector::pos() {
	return center;
}

NavSectorLink* NavSector::getLink(int subSectorId) {
	if (subSectorId < 0)
		return NULL;

	for (int i = 0; i < links.size(); i++) {
		if (links[i]->target->id == subSectorId) {
			return links[i];
		}
	}

	//Printf("Nav sector %d has no link to %d\n", id, subSectorId);
	return NULL;
}

bool NavSector::touches(AActor* actor) {
	if (!actor)
		return false;

	ActorState astate = g_engine.get_actor_state(actor);
	vec2 actorPos = astate.origin;

	int subid = g_map.GetSubsector(actorPos.x, actorPos.y)->id;
	if (subid == id)
		return true;

	MapSubsector& sub = g_map.subsectors[id];
	for (int i = 0; i < sub.numsegs; i++) {
		MapSeg& seg = g_map.segs[sub.firstseg + i];

		if (CircleIntersectsSegment(actorPos, astate.radius, seg.v1, seg.v2)) {
			return true;
		}
	}
	
	return false;
}


void SectorNavMesh::init() {
	pending_sector_relinks.clear();
	propBlockers = g_engine.find_prop_blockers();

	mesh = SectorNavMeshGenerator::generate(propBlockers);
	astarNodes = new AstarNode[g_map.numsubsectors];
	memset(astarNodes, 0, sizeof(AstarNode) * g_map.numsubsectors);
}

int SectorNavMesh::get_nav_id(vec2 pos) {
	return g_map.GetSubsector(pos.x, pos.y)->id;
}

int SectorNavMesh::get_nav_id(AActor* actor) {
	return get_nav_id(g_engine.get_actor_state(actor).origin);
}

int SectorNavMesh::get_nav_id(player_t* plr) {
	AActor* pActor = (AActor*)g_engine.get_player(plr);
	vec3 pos = g_engine.get_actor_state(pActor).origin;

	MapSubsector* centeredSub = g_map.GetSubsector(pos.x, pos.y);
	
	if (centeredSub->sector->getFloorZ() + STEP_HEIGHT < pos.z && g_engine.get_player_state(plr).onGround) {
		// above the centered subsector because of hanging over a ledge
		vec2 floorPoint = GetFloorPosition(pos, g_engine.get_actor_state(pActor).radius);
		return g_map.GetSubsector(floorPoint.x, floorPoint.y)->id;
	}

	return centeredSub->id;
}

float SectorNavMesh::node_heuristic(int a, int b) {
	NavSector& nodea = mesh.nodes[a];
	NavSector& nodeb = mesh.nodes[b];
	vec2 delta = nodea.pos() - nodeb.pos();
	return delta.length();
}

float SectorNavMesh::path_dist(NavSectorLink& link) {
	NavSector& parent = *link.parent;
	NavSector& target = *link.target;

	if (link.isTeleport)
		return (link.pos() - parent.pos()).length();

	return (parent.pos() - target.pos()).length();
}

float SectorNavMesh::path_cost(NavSectorLink& link, float dist, const RouteOpts& opts) {
	NavSector& parent = *link.parent;
	NavSector& target = *link.target;
	float cost = dist;

	if (link.linkWidth < PLAYER_WIDTH) {
		// try to avoid tiny links. They gets the bot stuck on corners.
		cost += 200;
	}

	if (!opts.timeSensitive) {
		// avoid things that are hard to navigate if speed isn't important

		if (target.doesDamage) {
			cost += dist * 8; // avoid damage sectors
		}

		if (link.isJump) {
			// jumps are error-prone. If you must take one, choose the shortest
			cost += dist * 4;
		}
	}
	
	if (link.isJump) {
		if (!link.isJumpValid()) {
			// jumps that can't be made yet are error prone
			cost += 4000;
		}
		if (link.jumpNeighbor->doesDamage) {
			// failing this jump will hurt
			cost += 2000;
		}
	} else if (link.isCliff) {
		// avoid dropping down cliffs
		cost += 2000;
	}

	if (g_map.sector_border_walkability(link.parent->sector, link.sector) == LINK_BLOCK_TOO_HIGH) {
		// avoid elevators, so that jumping into a pit to take an elevator isn't preferred
		// over walking around it
		cost += 2000;
	}	

	if (opts.blockedPathHandling == WBOT_ROUTE_BLOCK_EXPENSIVE && link.blocked(opts.actor))
		cost += 4000;

	return cost;
}

BotRoute SectorNavMesh::get_astar_route(const RouteOpts& opts)
{
	struct OpenNode {
		int id;
		float f;

		bool operator<(const OpenNode& other) const {
			return f > other.f; // reversed because priority_queue is a max heap
		}
	};
	std::priority_queue<OpenNode> openQueue;

	BotRoute emptyRoute;

	if (verbose) {
		g_engine.printf("START route from %d to %d\n", opts.start, opts.end);
	}

	if (opts.start < 0 || opts.end < 0 || opts.start >= g_map.numsubsectors || opts.end >= g_map.numsubsectors) {
		g_engine.printf("AStarRoute: invalid start/end nodes\n");
		g_route_ignore_num++;
		return emptyRoute;
	}

	if (opts.start == opts.end) {
		emptyRoute.route.push_back(opts.start);
		g_route_ignore_num++;
		return emptyRoute;
	}

	memset(astarNodes, 0, sizeof(AstarNode) * g_map.numsubsectors);

	NavSector& start = mesh.nodes[opts.start];
	NavSector& goal = mesh.nodes[opts.end];

	astarNodes[opts.start].gScore = 0;
	astarNodes[opts.start].fScore = node_heuristic(start.id, goal.id);

	openQueue.push({ opts.start, astarNodes[opts.start].fScore });

	const int maxIter = 8192;
	int curIter = 0;
	while (!openQueue.empty()) {

		if (++curIter > maxIter) {
			g_engine.printf("AStarRoute exceeded max iterations searching path (%d)", maxIter);
			break;
		}

		// get node in openset with lowest cost
		auto top = openQueue.top();
		openQueue.pop();
		AstarNode& curDat = astarNodes[top.id];

		if (curDat.closed)
			continue;
		if (top.f != curDat.fScore)
			continue; // stale entry

		int current = top.id;

		if (current == goal.id) {
			// goal reached, build the route
			BotRoute route;
			route.route.push_back(current);

			while (astarNodes[current].pathed) {
				const AstarNode& from = astarNodes[current];
				current = from.cameFromNode;
				route.cost += from.cameFromCost;
				route.dist += from.cameFromDist;
				route.route.push_back(current);
			}
			reverse(route.route.begin(), route.route.end());

			if (verbose) {
				g_engine.printf("FINISH route calculation from %d to %d. Size is %d.\n",
					opts.start, opts.end, route.route.size());
			}

			g_route_ignore_num++;
			return route;
		}

		curDat.closed = true;

		NavSector& currentNode = mesh.nodes[current];

		for (NavSectorLink* link : currentNode.links) {
			int neighbor = link->target->id;
			AstarNode& neighborDat = astarNodes[neighbor];

			if (neighborDat.closed)
				continue;

			if (link->routeNumIgnore == g_route_ignore_num)
				continue;

			if (link->target->routeNumIgnore == g_route_ignore_num)
				continue;

			if (opts.blockedPathHandling == WBOT_ROUTE_BLOCK_FORBID && link->blocked(opts.actor))
				continue;

			float linkDist = path_dist(*link);
			float linkCost = path_cost(*link, linkDist, opts);
			float tentative_gScore = curDat.gScore + linkCost;

			if (neighborDat.pathed && tentative_gScore >= neighborDat.gScore)
				continue; // not a better path

			// This path is the best until now. Record it!
			neighborDat.pathed = true;
			neighborDat.gScore = tentative_gScore;
			neighborDat.fScore = tentative_gScore + node_heuristic(mesh.nodes[neighbor].id, goal.id);
			neighborDat.cameFromNode = current;
			neighborDat.cameFromDist = linkDist;
			neighborDat.cameFromCost = linkCost;

			// discover a new node
			openQueue.push({ neighbor, neighborDat.fScore });
		}
	}

	g_route_ignore_num++;
	return emptyRoute;
}

class AKey;

bool SectorNavMesh::get_key_goals_for_line(AActor* actor, MapLine* line, vector<BotGoal>& keyGoals) {
	if (!line->isLockedDoor())
		return true; // not a locked door

	if (g_engine.can_unlock_door(actor, line))
		return true; // already have all the keys

	vector<vector<PClass*>> keyGroups = g_engine.get_required_key_types(line);

	if (keyGroups.size() == 0)
		return true; // no keys required

	// Get routes to all keys in the map, so the closest one can be selected
	struct KeyRoute {
		AActor* key;
		int routeSize;
	};

	int actorNavId = get_nav_id(actor);
	unordered_map<PClass*, KeyRoute> mapKeys;
	AKey* mapKey;
	
	RouteOpts opts;
	opts.start = actorNavId;

	int oldRouteIgnoreNum = g_route_ignore_num;

	for (AActor* mapKey : g_engine.find_map_keys()) {
		int keyNavId = get_nav_id(mapKey);
		opts.end = keyNavId;

		g_route_ignore_num = oldRouteIgnoreNum;

		KeyRoute keyRoute;
		keyRoute.key = mapKey;
		keyRoute.routeSize = get_astar_route(opts).dist;

		mapKeys[g_engine.get_actor_state(mapKey).pClass] = keyRoute;
	}

	for (int i = 0; i < keyGroups.size(); i++) {
		vector<PClass*>& group = keyGroups[i];

		int bestKeyDist = INT_MAX;
		AActor* bestKey = NULL;

		if (group.size()) {
			for (int k = 0; k < group.size(); k++) {
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
			g_engine.printf("Impossible key requirements for line %d:\n", line->id);

			for (int i = 0; i < keyGroups.size(); i++) {
				vector<PClass*>& group = keyGroups[i];
				for (int k = 0; k < group.size(); k++) {
					g_engine.printf("  %s", g_engine.get_class_type_name(group[k]));
				}
				g_engine.printf("\n");
			}
			g_engine.printf("Map keys:\n");
			for (auto item : mapKeys) {
				g_engine.printf("   %s", g_engine.get_actor_state(item.second.key).name);
			}
			g_engine.printf("\n");

			return false;
		}

		BotGoal goal = BotGoal(WBOT_GOAL_ACTION_TOUCH, bestKey);
		goal.dist = bestKeyDist;
		goal.required = true;
		keyGoals.push_back(goal);
	}

	return true;
}

vector<BotGoal> SectorNavMesh::get_weapon_goals(const char* wepname) {
	vector<BotGoal> goals;

	for (AActor* weapon : g_engine.find_map_weapons(wepname)) {
		goals.push_back(BotGoal(WBOT_GOAL_ACTION_TOUCH, weapon));
	}

	return goals;
}

vector<BotGoal> SectorNavMesh::get_ammo_goals(const char* ammoname, const char* ammoname2) {
	vector<BotGoal> goals;

	for (AActor* weapon : g_engine.find_map_ammo(ammoname, ammoname2)) {
		goals.push_back(BotGoal(WBOT_GOAL_ACTION_TOUCH, weapon));
	}

	return goals;
}

void SectorNavMesh::relink_sector(MapSector* sec) {
	for (MapSector* pend : pending_sector_relinks) {
		if (pend == sec) {
			return;
		}
	}

	pending_sector_relinks.push_back(sec);
}

void SectorNavMesh::relink_pending_sector() {
	if (pending_sector_relinks.empty())
		return;

	// iterate once per tick
	static int t = 0;
	t++;

	int idx = (t % pending_sector_relinks.size());
	MapSector* sec = pending_sector_relinks[idx];
	int secid = sec - g_map.sectors;

	if (sec->isMoving()) {
		return;
	}

	int linksAdded = 0;

	g_map.sectors[secid].moveFlags = 0;

	for (int i = 0; i < g_map.numsubsectors; i++) {
		if (g_map.subsectors[i].sector == sec) {
			linksAdded += SectorNavMeshGenerator::relink_node(mesh, i, propBlockers);
		}
	}
	
	if (!g_wbot_test_mode)
		g_engine.printf("Relinked sector %d (%+d links)\n", secid, linksAdded);

	pending_sector_relinks.erase(pending_sector_relinks.begin() + idx);
}

