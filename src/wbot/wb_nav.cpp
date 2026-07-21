#include "wb_nav.h"
#include "wb_bot.h"
#include "wb_map.h"
#include "wb_nav_gen.h"
#include "r_state.h"
#include "sv_commands.h"
#include "p_local.h"
#include "p_lnspec.h"
#include "p_trace.h"
#include "a_keys.h"
#include "wb_util.h"

#include <algorithm>
#include <unordered_set>
#include <unordered_map>
#include <vector>

using namespace std;

SectorNavMesh g_wb_nav;

FVector2 NavSectorLink::pos() {
	return overlapCenter;
}

FVector3 NavSectorLink::pos3D() {
	return FVector3(overlapCenter.X, overlapCenter.Y, parent->getFloorZ());
}

bool NavSectorLink::blocked(AActor* actor, bool recurse) {
	g_wb_nav.pathTests++;

	if (!g_wb_mapinfo.can_cross_seg_now(seg)) {
		line_t* line = seg->linedef;

		if (line && line->args[0] == 0 && g_wb_mapinfo.get_linedef_move_flag(line)) {
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
	if (!jumpable()) {
		return true;
	}
	
	if (recurse && linkWidth <= PLAYER_WIDTH) {
		// too narrow to move thru unless the neighbor sectors are passable
		int expandedWidth = linkWidth;
		int neighbors[2] = {leftSector, rightSector};
		for (int i = 0; i < 2; i++) {
			if (neighbors[i] < 0)
				continue;

			NavSector& neighborNav = g_wb_nav.mesh[neighbors[i]];
			bool isWalkable = parent->getFloorZ() + (STEP_HEIGHT << FRACBITS) > neighborNav.getFloorZ();
			bool wontHitHead = neighborNav.getCeilZ() > parent->getFloorZ() + (DUCK_HEIGHT << FRACBITS);
			if (isWalkable && wontHitHead) {
				// the neighboring sector is not a wall and won't prevent movement on the target sector
				// TODO: need to check how thick the neighbor is too
				expandedWidth += PLAYER_WIDTH;
				continue;
			}

			NavSectorLink* link = parent->getLink(neighbors[i]);

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
	return g_wb_mapinfo.can_cross_seg_now(seg) && jumpable();
}

bool NavSectorLink::jumpable() {
	if (!isJump)
		return true;

	if (!isJumpValid()) {
		return false;
	}

	FVector3 start = pos3D() + FVector3(0, 0, 56 << FRACBITS);
	FVector3 end = target->pos3D() + FVector3(0, 0, 56 << FRACBITS);

	FTraceResults tr;
	if (TraceLine(start, end, true, NULL, &tr)) {
		return false;
	}

	return true;
}

bool NavSectorLink::isJumpValid() {
	fixed_t jumpHeight = target->getFloorZ() - parent->getFloorZ();
	if (jumpHeight >= (JUMP_HEIGHT << FRACBITS))
		return false;

	if (jumpDist > (JUMP_DIST << FRACBITS) - jumpHeight)
		return false; // too far to make it

	return true;
}

FVector2 NavSectorLink::GetJumpStartPos() {
	FVector2 a(seg->v1->x, seg->v1->y);
	FVector2 b(seg->v2->x, seg->v2->y);
	
	// bring points inward a bit to avoid getting too close to a cliff or wall
	fixed_t dist = (b - a).Length();
	FVector2 dir = (b - a).Unit();
	fixed_t nudgeDist = std::min(dist / 2, 32 << FRACBITS);
	a += dir * nudgeDist;
	b -= dir * nudgeDist;

	return ClosestPointOnSegment(target->pos(), a, b);
}

FVector2 NavSectorLink::GetJumpEndPos() {
	FVector2 jumpPos = GetJumpStartPos();
	FVector2 center = target->pos();

	// move the landing point from the center to the nearest ledge
	FVector2 ledgeDir = (jumpPos - center).Unit();

	FVector2 edgePos;
	if (TraceSectorEdge(center, center + ledgeDir * (1000 << FRACBITS), edgePos)) {
		return edgePos;
	}

	return center;
}


FVector2 NavSectorLink::GetJumpBackupPos(AActor* jumper) {
	FVector2 startPos = GetJumpStartPos();

	// back up the starting position to get a running start for the jump
	FVector2 jumpDir = (target->pos() - startPos).Unit();

	fixed_t maxBackupDist = 256 << FRACBITS;
	FVector2 backupStartPos = startPos - (jumpDir * FRACUNIT); // avoid clipping against the backoff line
	FVector2 backupPos = startPos - jumpDir * maxBackupDist;
	fixed_t startZ = parent->getFloorZ();
	int isects = 0;

	for (TraceIsect& isect : TraceIntersections(backupStartPos, backupPos)) {
		fixed_t sectorZ = isect.sector->floorplane.ZatPoint((fixed_t)isect.pos.X, (fixed_t)isect.pos.Y);
		int heightDiff = (sectorZ - startZ) >> FRACBITS;

		if (IsImpassable(isect.line) || heightDiff > STEP_HEIGHT) {
			if (isects++ == 0) {
				backupPos = isect.pos;
			}
			break;
		}

		backupPos = isect.pos;
	}

	// clip against walls for our radius
	FTraceResults tr;
	FVector2 backupDelta = backupPos - backupStartPos;
	fixed_t traceZ = startZ + (STEP_HEIGHT << FRACBITS);
	if (TraceRadius(FVector3(backupStartPos, traceZ), FVector3(backupPos, traceZ), PLAYER_WIDTH / 2, false, jumper, &tr)) {
		float frac = tr.Fraction / (float)FRACUNIT;
		backupPos = backupStartPos + backupDelta * frac;
	}

	fixed_t backupDist = (backupPos - backupStartPos).Length();
	fixed_t nudgeDist = std::min(8 << FRACBITS, backupDist);
	backupPos += jumpDir * nudgeDist;

	return backupPos;
}

FVector3 NavSector::pos3D() {
	return FVector3(center.X, center.Y, getFloorZ());
}

FVector2 NavSector::pos() {
	return center;
}

NavSectorLink* NavSector::getLink(int subSectorId) {
	if (subSectorId < 0)
		return NULL;

	for (int i = 0; i < links.size(); i++) {
		if (links[i].target->id == subSectorId) {
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
	return g_wb_mapinfo.sector_info[subsectors[id].sector - sectors].moveFlags;
}

std::vector<BotGoal>& NavSector::getTriggers() {
	return g_wb_mapinfo.sector_info[subsectors[id].sector - sectors].triggers;
}

sector_t* NavSector::sector() {
	return subsectors[id].sector;
}

fixed_t NavSector::getHeight() {
	sector_t* sec = subsectors[id].sector;
	return sec->ceilingplane.ZatPoint((fixed_t)center.X, (fixed_t)center.Y)
		   - sec->floorplane.ZatPoint((fixed_t)center.X, (fixed_t)center.Y);
}

fixed_t NavSector::getFloorZ() {
	return subsectors[id].sector->floorplane.ZatPoint((fixed_t)center.X, (fixed_t)center.Y);
}

fixed_t NavSector::getCeilZ() {
	return subsectors[id].sector->ceilingplane.ZatPoint((fixed_t)center.X, (fixed_t)center.Y);
}

void SectorNavMesh::init() {
	propBlockers.clear();

	// find all immovable and invulnerable props
	TThinkerIterator<AActor> it;
	AActor* actor;
	while ((actor = it.Next())) {
		if (IsPropBlocker(actor))
			propBlockers.push_back(actor);
		//Printf("Prop '%s' %d\n", actor->GetClass()->TypeName.GetChars(), actor->health);
	}

	mesh = SectorNavMeshGenerator::generate(propBlockers);
	g_wb_mapinfo.sort_sector_trigger_goals();
}

int SectorNavMesh::get_route_distance(std::vector<int>& route) {
	int totalDist = 0;
	for (int i = 1; i < route.size(); i++) {
		totalDist += (int)(mesh[i].pos() - mesh[i - 1].pos()).Length() >> FRACBITS;
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
		NavSector& nav = mesh[subId];
		int spritesDrawn = 0;
		const int maxSprites = 1000;

		for (int k = 0; k < nav.links.size() && spritesDrawn < maxSprites; k++) {
			NavSectorLink& link = nav.links[k];
			
			if (!link.blocked(player)) {
				FVector3 linkPos = link.pos3D();

				if (link.isJump) {
					FVector3 jumpStart = FVector3(link.GetJumpStartPos(), 0);
					FVector3 jumpStartFloor = jumpStart;
					FVector3 jumpEnd = FVector3(link.GetJumpEndPos(), 0);
					FVector3 jumpEndFloor = jumpEnd;
					jumpEndFloor.Z = link.target->getFloorZ();
					jumpStartFloor.Z = link.parent->getFloorZ();
					jumpStart.Z = link.parent->getFloorZ() + (56 << FRACBITS);
					jumpEnd.Z = link.target->getFloorZ() + (56 << FRACBITS);
					
					FVector2 backupPos = link.GetJumpBackupPos(player);
					FVector3 backupEnd = FVector3(backupPos.X, backupPos.Y, jumpStart.Z);

					spritesDrawn += draw_debug_line(nav.pos3D(), jumpStartFloor, actor);
					spritesDrawn += draw_debug_line(jumpStartFloor, jumpStart, actor);
					spritesDrawn += draw_debug_line(jumpStart, jumpEnd, actor);
					spritesDrawn += draw_debug_line(jumpEnd, jumpEndFloor, actor);
					spritesDrawn += draw_debug_line(jumpEndFloor, link.target->pos3D(), actor);

					if (link.jumpDist > (PLAYER_WIDTH << FRACBITS))
						spritesDrawn += draw_debug_line(jumpStart, backupEnd, actor);
				}
				else {
					spritesDrawn += draw_debug_line(nav.pos3D(), linkPos, actor);
					spritesDrawn += draw_debug_line(linkPos, link.target->pos3D(), actor);
				}
			}
		}

		fixed_t borderZ = nav.getFloorZ();
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
		NavSector& node = mesh[i];
		FVector3 pos = node.pos3D();

		if (P_AproxDistance(pos.X - player->x, pos.Y - player->y) > (1000 << FRACBITS)) {
			continue;
		}

		SERVERCOMMANDS_SpawnBlood(pos.X, pos.Y, pos.Z + (16 << FRACBITS), 0, 100, player);
	}
}

int SectorNavMesh::get_nav_id(fixed_t x, fixed_t y) {
	return R_PointInSubsector(x, y) - subsectors;
}

int SectorNavMesh::get_nav_id(AActor* actor) {
	return get_nav_id(actor->x, actor->y);
}

float SectorNavMesh::path_cost(int a, int b) {
	NavSector& nodea = mesh[a];
	NavSector& nodeb = mesh[b];
	FVector2 delta = nodea.pos() - nodeb.pos();
	return delta.Length();
}

float SectorNavMesh::path_cost(NavSectorLink& link, bool timeSensitive) {
	NavSector& parent = *link.parent;
	NavSector& target = *link.target;
	float cost = 0;
	float dist = 0;

	if (link.isTeleport)
		dist = (link.pos() - parent.pos()).Length();
	else
		dist = (parent.pos() - target.pos()).Length();

	cost += dist;

	if (link.linkWidth < PLAYER_WIDTH) {
		// try to avoid tiny links. They gets the bot stuck on corners.
		cost += 200 << FRACBITS;
	}

	if (!timeSensitive) {
		// avoid things that are hard to navigate if speed isn't important

		if (target.doesDamage) {
			cost += dist * (4 << FRACBITS); // avoid damage sectors
		}

		if (link.isJump) {
			// jumps are error-prone. If you must take one, choose the shortest
			cost += dist * (4 << FRACBITS);

			if (!link.isJumpValid()) {
				// jumps that can't be made yet are even more error prone
				cost += 4000 << FRACBITS;
			}
		}
	}

	if (!g_wb_mapinfo.can_cross_seg_now(link.seg)) {
		fixed_t elevDist = fabs(target.getFloorZ() - parent.getFloorZ());
		if (elevDist > (JUMP_HEIGHT << FRACBITS)) {
			cost += elevDist * (4 << FRACBITS); // avoid elevators
		}
	}

	return cost;
}

vector<int> SectorNavMesh::get_astar_route(int startSubSectorId, int endSubSectorId, unordered_set<int>* blockedPaths, int blockedSector, bool timeSensitive)
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

	NavSector& start = mesh[startSubSectorId];
	NavSector& goal = mesh[endSubSectorId];

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

		NavSector& currentNode = mesh[current];

		for (int i = 0; i < currentNode.links.size(); i++) {
			NavSectorLink& link = currentNode.links[i];
			int neighbor = link.target->id;

			if (closedSet.count(neighbor))
				continue;

			if (neighbor == blockedSector)
				continue;

			if (blockedPaths && blockedPaths->count(link.id))
				continue;

			// discover a new node
			openSet.insert(neighbor);

			// The distance from start to a neighbor
			NavSector& neighborNode = mesh[neighbor];

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

void SectorNavMesh::relink_sector(sector_t* sec) {
	for (sector_t* pend : pending_sector_relinks) {
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
	sector_t* sec = pending_sector_relinks[idx];
	int secid = sec - sectors;

	if (sec->floordata || sec->ceilingdata) {
		return;
	}

	int linksAdded = 0;

	g_wb_mapinfo.sector_info[secid].moveFlags = 0;

	for (int i = 0; i < numsubsectors; i++) {
		if (subsectors[i].sector == sec) {
			linksAdded += SectorNavMeshGenerator::relink_node(mesh, i, propBlockers);
		}
	}
	
	Printf("Relinked sector %d (%+d links)\n", secid, linksAdded);

	pending_sector_relinks.erase(pending_sector_relinks.begin() + idx);
}

