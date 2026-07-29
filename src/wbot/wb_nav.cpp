#include "wb_nav.h"
#include "wb_bot.h"
#include "wb_map.h"
#include "wb_nav_gen.h"
#include "wb_util.h"
#include "wb_hooks.h"
#include "p_lnspec.h"
#include "a_keys.h"

#include <algorithm>
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <vector>

using namespace std;
using namespace wbot;

SectorNavMesh g_wb_nav;
int g_route_ignore_num;

FVector2 NavSectorLink::pos() {
	return movePos;
}

FVector3 NavSectorLink::pos3D() {
	return FVector3(movePos.X, movePos.Y, parent->getFloorZ());
}

int NavSectorLink::blocked(AActor* actor, bool recurse) {
	g_wb_nav.pathTests++;

	int walkability = g_map.sector_border_walkability(parent->sector, sector);
	if (walkability != LINK_BLOCK_CLEAR) {
		MapLine* line = linedef;

		if (line && line->getArg(0) == 0 && g_map.get_linedef_move_flag(line)) {
			if (actor && line->special() == Door_LockedRaise && !g_map.CheckKeys(actor, line)) {
				return walkability; // don't have the keys required to use this
			}

			// border is an untagged linedef, which means it moves the target sector,
			// which probably unblocks the path. TODO: not always!
			return LINK_BLOCK_CLEAR;
		}

		return walkability;
	}

	// check if jumps are valid for dynamic links that depend on from/to sector states
	if (!jumpable()) {
		return LINK_BLOCK_CANT_JUMP;
	}

	const fixed_t pradius = PLAYER_RADIUS << FRACBITS;
	fixed_t parentFloorZ = parent->getFloorZ();
	FVector3 jumpOverPos = FVector3(movePos, parentFloorZ + (JUMP_HEIGHT << FRACBITS));
	FVector3 duckUnderPos = FVector3(movePos, parentFloorZ + (DUCK_HEIGHT << FRACBITS));
	if (IsBoxClipped(jumpOverPos, pradius, 0) && IsBoxClipped(duckUnderPos, pradius, 0)) {
		return LINK_BLOCK_CLIPPED;
	}

	return LINK_BLOCK_CLEAR;
}

std::vector<MapSector*> NavSectorLink::getClippedSectors(AActor* actor) {
	fixed_t bottomZ = parent->getFloorZ() + (JUMP_HEIGHT << FRACBITS);
	return GetBoxClipSectors(FVector3(movePos, bottomZ), PLAYER_RADIUS << FRACBITS, 0);
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

	FVector3 start = pos3D() + FVector3(0, 0, 56 << FRACBITS);
	FVector3 end = target->pos3D() + FVector3(0, 0, 56 << FRACBITS);

	if (TraceLine(start, end, true, NULL, NULL)) {
		return false;
	}

	return true;
}

bool NavSectorLink::isJumpValid() {
	fixed_t floorZ = parent->getFloorZ();
	fixed_t jumpHeight = target->getFloorZ() - floorZ;
	if (jumpHeight >= (JUMP_HEIGHT << FRACBITS))
		return false;

	if (jumpDist > (JUMP_DIST << FRACBITS) - jumpHeight)
		return false; // too far to make it

	if (jumpNeighbor->getFloorZ() - floorZ == 0)
		return false; // not currently an edge that can help with jumping

	return true;
}

FVector2 NavSectorLink::GetJumpStartPos(FVector2 targetPos) {	
	// bring points inward a bit to avoid getting too close to a cliff or wall
	fixed_t dist = (seg.b - seg.a).Length();
	FVector2 dir = (seg.b - seg.a).Unit();
	fixed_t nudgeDist = std::min(dist / 2, 32 << FRACBITS);
	FVector2 a = seg.a + dir * nudgeDist;
	FVector2 b = seg.b - dir * nudgeDist;

	return ClosestPointOnSegment(targetPos, a, b);
}

FVector2 NavSectorLink::GetJumpEndPos(FVector2 targetPos) {
	FVector2 jumpPos = GetJumpStartPos(targetPos);

	// move the landing point from the center to the nearest ledge
	FVector2 ledgeDir = (jumpPos - targetPos).Unit();

	MapLine* line;
	FVector2 edgePos;
	if (TraceSectorEdge(targetPos, targetPos + ledgeDir * (1000 << FRACBITS), edgePos, &line)) {
		// move edge away from the line endings to avoid collision with a wall
		FVector2 a = line->start();
		FVector2 b = line->end();
		FVector2 dir = (b - a).Unit();
		fixed_t nudgeDist = std::min(line->length() / 2, 32 << FRACBITS);
		a += dir * nudgeDist;
		b -= dir * nudgeDist;

		edgePos = ClosestPointOnSegment(jumpPos, a, b);

		return edgePos;
	}

	return targetPos;
}

NavSector* NavSectorLink::GetJumpBackupBlocker(FVector2 targetPos) {
	FVector2 startPos = GetJumpStartPos(targetPos);

	// back up the starting position to get a running start for the jump
	FVector2 jumpDir = (targetPos - startPos).Unit();

	fixed_t maxBackupDist = 256 << FRACBITS;
	FVector2 backupStartPos = startPos - (jumpDir * FRACUNIT); // avoid clipping against the backoff line
	FVector2 backupPos = startPos - jumpDir * maxBackupDist;
	fixed_t startZ = parent->getFloorZ();
	int isects = 0;

	FVector2 edge;
	MapLine* line;
	if (TraceSectorEdge(backupStartPos, backupPos, edge, &line)) {
		MapSubsector& sub = g_map.subsectors[parent->id];
		for (NavSectorLink* link : parent->links) {
			if (link->linedef == line) {
				bool canMoveAway = link->target->getMoveFlags() & (FL_SECTOR_MOVE_FLOOR_DOWN | FL_SECTOR_MOVE_CEIL_UP);
				return canMoveAway ? link->target : NULL;
			}
		}
	}

	return NULL;
}

fixed_t NavSectorLink::GetJumpBackupSpaceNeeded(FVector3 start, FVector3 end) {
	fixed_t gap = jumpDist - (PLAYER_WIDTH << FRACBITS);

	const fixed_t maxBackup = 256 << FRACBITS;

	FVector3 dir = (end - start).Unit();
	float distScale = dir.Z + 1; // increase gap for upward jumps, decrease for downward

	return std::min(maxBackup, (fixed_t)(gap * distScale));
}

FVector2 NavSectorLink::GetJumpBackupPos(FVector2 targetPos, AActor* jumper) {
	FVector2 startPos = GetJumpStartPos(targetPos);
	FVector2 endPos = GetJumpEndPos(targetPos);
	FVector3 startPos3D(startPos, parent->getFloorZ());
	FVector3 endPos3D(endPos, target->getFloorZ());

	// back up the starting position to get a running start for the jump
	FVector2 jumpDir = (endPos - startPos).Unit();

	fixed_t maxBackupDist = GetJumpBackupSpaceNeeded(startPos3D, endPos3D);
	FVector2 backupStartPos = startPos - (jumpDir * FRACUNIT); // avoid clipping against the backoff line
	FVector2 backupPos = startPos - jumpDir * maxBackupDist;
	fixed_t startZ = parent->getFloorZ();
	int isects = 0;

	for (TraceIsect& isect : TraceIntersections(backupStartPos, backupPos)) {
		fixed_t sectorZ = isect.sector->getFloorZ();
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
	TraceResult tr;
	FVector2 backupDelta = backupPos - backupStartPos;
	fixed_t traceZ = startZ + (STEP_HEIGHT << FRACBITS);
	if (TraceRadius(FVector3(backupStartPos, traceZ), FVector3(backupPos, traceZ), PLAYER_WIDTH / 2, false, jumper, &tr)) {
		backupPos = backupStartPos + backupDelta * tr.frac;
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

	FVector2 actorPos = FVector2(actor->x, actor->y);

	int subid = g_map.GetSubsector(actorPos.X, actorPos.Y)->id;
	if (subid == id)
		return true;

	MapSubsector& sub = g_map.subsectors[id];
	for (int i = 0; i < sub.numsegs; i++) {
		MapSeg& seg = g_map.segs[sub.firstseg + i];

		if (CircleIntersectsSegment(actorPos, actor->radius, seg.start(), seg.end())) {
			return true;
		}
	}
	
	return false;
}

int NavSector::getMoveFlags() {
	return sector->moveFlags;
}

bool NavSector::isMoving() {
	return sector->isMoving() ;
}

bool NavSector::isFloorMoving() {
	return sector->isFloorMoving();
}

bool NavSector::isCeilMoving() {
	return sector->isCeilMoving();
}

std::vector<BotGoal>& NavSector::getTriggers() {
	return sector->triggers;
}

fixed_t NavSector::getHeight() {
	return sector->getHeight();
}

fixed_t NavSector::getFloorZ() {
	return sector->getFloorZ();
}

fixed_t NavSector::getCeilZ() {
	return sector->getCeilZ();
}

void SectorNavMesh::init() {
	propBlockers.clear();
	pending_sector_relinks.clear();

	// find all immovable and invulnerable props
	TThinkerIterator<AActor> it;
	AActor* actor;
	while ((actor = it.Next())) {
		if (IsPropBlocker(actor))
			propBlockers.push_back(actor);
		//Printf("Prop '%s' %d\n", actor->GetClass()->TypeName.GetChars(), actor->health);
	}

	mesh = SectorNavMeshGenerator::generate(propBlockers);
	astarNodes = new AstarNode[g_map.numsubsectors];
	memset(astarNodes, 0, sizeof(AstarNode) * g_map.numsubsectors);
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

	MapSubsector* sub = g_map.GetSubsector(player->x, player->y);

	if (sub && sub->id < g_map.numsubsectors) {
		NavSector& nav = mesh.nodes[sub->id];
		int spritesDrawn = 0;
		const int maxSprites = 1000;

		for (int k = 0; k < nav.links.size() && spritesDrawn < maxSprites; k++) {
			NavSectorLink& link = *nav.links[k];
			
			if (!link.blocked(player)) {
				FVector3 linkPos = link.pos3D();
				FVector2 targetPos = link.target->pos();

				if (link.isJump) {
					FVector3 jumpStart = FVector3(link.GetJumpStartPos(targetPos), 0);
					FVector3 jumpStartFloor = jumpStart;
					FVector3 jumpEnd = FVector3(link.GetJumpEndPos(targetPos), 0);
					FVector3 jumpEndFloor = jumpEnd;
					jumpEndFloor.Z = link.target->getFloorZ();
					jumpStartFloor.Z = link.parent->getFloorZ();
					jumpStart.Z = link.parent->getFloorZ() + (56 << FRACBITS);
					jumpEnd.Z = link.target->getFloorZ() + (56 << FRACBITS);
					
					FVector2 backupPos = link.GetJumpBackupPos(targetPos, player);
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
		for (int k = 0; k < sub->numsegs && spritesDrawn < maxSprites; k++) {
			MapSeg& seg = g_map.segs[sub->firstseg + k];
			FVector3 start(seg.start(), borderZ);
			FVector3 end(seg.end(), borderZ);
			spritesDrawn += draw_debug_line(start, end, actor);
		}

		FVector2 playerPos(player->x, player->y);
		int closestSeg = -1;
		fixed_t bestDist = INT_MAX;
		for (int i = 0; i < sub->numsegs; i++) {
			MapSeg& seg = g_map.segs[sub->firstseg + i];
			fixed_t dist = abs(DistanceToLine(playerPos, seg.start(), seg.end()));
			if (dist < bestDist) {
				bestDist = dist;
				closestSeg = sub->firstseg + i;
			}
		}
		if (closestSeg != -1) {
			MapSeg& seg = g_map.segs[closestSeg];
			fixed_t z = borderZ + FRACUNIT * 16;
			FVector3 normStart(seg.center(), z);
			FVector3 normEnd(seg.center() + seg.normal() * FRACUNIT * 32, z);
			spritesDrawn += draw_debug_line(normStart, normEnd, actor);
		}

		if (spritesDrawn >= maxSprites) {
			Printf("Overflow sprites!\n");
		}
	}

	for (int i = 0; i < g_map.numsubsectors; i++) {
		NavSector& node = mesh.nodes[i];
		FVector3 pos = node.pos3D();

		FVector2 delta = (FVector2)pos - FVector2(player->x, player->y);
		if (delta.Length() > (1000 << FRACBITS)) {
			continue;
		}

		SpawnBlood(pos + FVector3(0, 0, (16 << FRACBITS)), 100, player);
	}
}

int SectorNavMesh::get_nav_id(fixed_t x, fixed_t y) {
	return g_map.GetSubsector(x, y)->id;
}

int SectorNavMesh::get_nav_id(AActor* actor) {
	return get_nav_id(actor->x, actor->y);
}

float SectorNavMesh::node_heuristic(int a, int b) {
	NavSector& nodea = mesh.nodes[a];
	NavSector& nodeb = mesh.nodes[b];
	FVector2 delta = nodea.pos() - nodeb.pos();
	return delta.Length();
}

float SectorNavMesh::path_dist(NavSectorLink& link) {
	NavSector& parent = *link.parent;
	NavSector& target = *link.target;

	if (link.isTeleport)
		return (link.pos() - parent.pos()).Length();

	return (parent.pos() - target.pos()).Length();
}


float SectorNavMesh::path_cost(NavSectorLink& link, float dist, const RouteOpts& opts) {
	NavSector& parent = *link.parent;
	NavSector& target = *link.target;
	float cost = dist;

	if (link.linkWidth < PLAYER_WIDTH) {
		// try to avoid tiny links. They gets the bot stuck on corners.
		cost += 200 << FRACBITS;
	}

	if (!opts.timeSensitive) {
		// avoid things that are hard to navigate if speed isn't important

		if (target.doesDamage) {
			cost += dist * (8 << FRACBITS); // avoid damage sectors
		}

		if (link.isJump) {
			// jumps are error-prone. If you must take one, choose the shortest
			cost += dist * (4 << FRACBITS);
		}
	}
	
	if (link.isJump && !link.isJumpValid()) {
		// jumps that can't be made yet are error prone
		cost += 4000 << FRACBITS;
	}

	if (g_map.sector_border_walkability(link.parent->sector, link.sector) == LINK_BLOCK_TOO_HIGH) {
		// avoid elevators, so that jumping into a pit to take an elevator isn't preferred
		// over walking around it
		cost += (2000 << FRACBITS);
	}

	if (opts.blockedPathHandling == WBOT_ROUTE_BLOCK_EXPENSIVE && link.blocked(opts.actor))
		cost += 4000 << FRACBITS;

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
		Printf("START route from %d to %d\n", opts.start, opts.end);
	}

	if (opts.start < 0 || opts.end < 0 || opts.start >= g_map.numsubsectors || opts.end >= g_map.numsubsectors) {
		Printf("AStarRoute: invalid start/end nodes\n");
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
			Printf("AStarRoute exceeded max iterations searching path (%d)", maxIter);
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
				route.dist += ((int)from.cameFromDist) >> FRACBITS;
				route.route.push_back(current);
			}
			reverse(route.route.begin(), route.route.end());

			if (verbose) {
				Printf("FINISH route calculation from %d to %d. Size is %d.\n",
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

bool SectorNavMesh::get_key_goals_for_line(AActor* actor, MapLine* line, vector<BotGoal>& keyGoals) {
	if (line->special() != Door_LockedRaise)
		return true; // not a locked door

	if (g_map.CheckKeys(actor, line))
		return true; // already have all the keys

	TArray<TArray<PClass*>> keyGroups = P_GetRequiredKeys(line->getArg(3));

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
	
	RouteOpts opts;
	opts.start = actorNavId;

	int oldRouteIgnoreNum = g_route_ignore_num;

	while ((mapKey = it.Next()) != NULL) {
		if (mapKey->Owner)
			continue; // key in someone's inventory

		int keyNavId = get_nav_id(mapKey);
		opts.end = keyNavId;

		g_route_ignore_num = oldRouteIgnoreNum;

		KeyRoute keyRoute;
		keyRoute.key = mapKey;
		keyRoute.routeSize = get_astar_route(opts).dist;

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
			Printf("Impossible key requirements for line %d:\n", line->id);

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
		goal.required = true;
		keyGoals.push_back(goal);
	}

	return true;
}

vector<BotGoal> SectorNavMesh::get_weapon_goals(const char* wepname) {
	TThinkerIterator<AWeapon> it;
	AWeapon* weapon;

	vector<BotGoal> goals;

	while ((weapon = it.Next()) != NULL) {
		if (weapon->Owner)
			continue; // weapon in someone's inventory

		if (!strcmp(weapon->GetClass()->TypeName.GetChars(), wepname))
			goals.push_back(BotGoal(WBOT_GOAL_ACTION_TOUCH, weapon));
	}

	return goals;
}

vector<BotGoal> SectorNavMesh::get_ammo_goals(const char* ammoname, const char* ammoname2) {
	TThinkerIterator<AAmmo> it;
	AAmmo* ammo;

	vector<BotGoal> goals;

	while ((ammo = it.Next()) != NULL) {
		if (ammo->Owner)
			continue; // weapon in someone's inventory

		const char* name = ammo->GetClass()->TypeName.GetChars();
		if (!strcmp(name, ammoname) || (ammoname2 && !strcmp(name, ammoname2)))
			goals.push_back(BotGoal(WBOT_GOAL_ACTION_TOUCH, ammo));
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
		Printf("Relinked sector %d (%+d links)\n", secid, linksAdded);

	pending_sector_relinks.erase(pending_sector_relinks.begin() + idx);
}

