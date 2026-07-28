#include "wb_nav.h"
#include "wb_bot.h"
#include "wb_map.h"
#include "wb_nav_gen.h"
#include "wb_util.h"
#include "r_state.h"
#include "sv_commands.h"
#include "p_local.h"
#include "p_lnspec.h"
#include "p_trace.h"
#include "a_keys.h"

#include <algorithm>
#include <unordered_set>
#include <unordered_map>
#include <vector>

using namespace std;

SectorNavMesh g_wb_nav;

FVector2 NavSectorLink::pos() {
	return movePos;
}

FVector3 NavSectorLink::pos3D() {
	return FVector3(movePos.X, movePos.Y, parent->getFloorZ());
}

int NavSectorLink::blocked(AActor* actor, bool recurse) {
	g_wb_nav.pathTests++;

	int walkability = g_wb_mapinfo.segment_walkability(seg);
	if (walkability != LINK_BLOCK_CLEAR) {
		line_t* line = seg->linedef;

		if (line && line->args[0] == 0 && g_wb_mapinfo.get_linedef_move_flag(line)) {
			if (actor && line->special == Door_LockedRaise && !P_CheckKeys(actor, line->args[3], false)) {
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

std::vector<sector_t*> NavSectorLink::getClippedSectors(AActor* actor) {
	fixed_t bottomZ = parent->getFloorZ() + (JUMP_HEIGHT << FRACBITS);
	return GetBoxClipSectors(FVector3(movePos, bottomZ), PLAYER_RADIUS << FRACBITS, 0);
}

bool NavSectorLink::walkable() {
	return g_wb_mapinfo.segment_walkability(seg) == LINK_BLOCK_CLEAR && jumpable();
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
	FVector2 a(seg->v1->x, seg->v1->y);
	FVector2 b(seg->v2->x, seg->v2->y);
	
	// bring points inward a bit to avoid getting too close to a cliff or wall
	fixed_t dist = (b - a).Length();
	FVector2 dir = (b - a).Unit();
	fixed_t nudgeDist = std::min(dist / 2, 32 << FRACBITS);
	a += dir * nudgeDist;
	b -= dir * nudgeDist;

	return ClosestPointOnSegment(targetPos, a, b);
}

FVector2 NavSectorLink::GetJumpEndPos(FVector2 targetPos) {
	FVector2 jumpPos = GetJumpStartPos(targetPos);

	// move the landing point from the center to the nearest ledge
	FVector2 ledgeDir = (jumpPos - targetPos).Unit();

	line_t* line;
	FVector2 edgePos;
	if (TraceSectorEdge(targetPos, targetPos + ledgeDir * (1000 << FRACBITS), edgePos, &line)) {
		// move edge away from the line endings to avoid collision with a wall
		FVector2 a = FVector2(line->v1->x, line->v1->y);
		FVector2 b = FVector2(line->v2->x, line->v2->y);
		fixed_t dist = (b - a).Length();
		FVector2 dir = (b - a).Unit();
		fixed_t nudgeDist = std::min(dist / 2, 32 << FRACBITS);
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
	line_t* line;
	if (TraceSectorEdge(backupStartPos, backupPos, edge, &line)) {
		subsector_t& sub = subsectors[parent->id];
		for (NavSectorLink& link : parent->links) {
			if (link.seg->linedef == line) {
				bool canMoveAway = link.target->getMoveFlags() & (FL_SECTOR_MOVE_FLOOR_DOWN | FL_SECTOR_MOVE_CEIL_UP);
				return canMoveAway ? link.target : NULL;
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

bool NavSector::isMoving() {
	sector_t* sec = sector();
	return sec->floordata || sec->ceilingdata;
}

bool NavSector::isFloorMoving() {
	return sector()->floordata;
}

bool NavSector::isCeilMoving() {
	return sector()->ceilingdata;
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
	astarNodes = new AstarNode[numsubsectors];
	memset(astarNodes, 0, sizeof(AstarNode) * numsubsectors);
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

float SectorNavMesh::node_heuristic(int a, int b) {
	NavSector& nodea = mesh[a];
	NavSector& nodeb = mesh[b];
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

	if (g_wb_mapinfo.segment_walkability(link.seg) == LINK_BLOCK_TOO_HIGH) {
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

	if (opts.start < 0 || opts.end < 0 || opts.start >= numsubsectors || opts.end >= numsubsectors) {
		Printf("AStarRoute: invalid start/end nodes\n");
		return emptyRoute;
	}

	if (opts.start == opts.end) {
		emptyRoute.route.push_back(opts.start);
		return emptyRoute;
	}

	memset(astarNodes, 0, sizeof(AstarNode) * numsubsectors);

	NavSector& start = mesh[opts.start];
	NavSector& goal = mesh[opts.end];

	astarNodes[opts.start].gScore = 0;
	astarNodes[opts.start].fScore = node_heuristic(start.id, goal.id);

	openQueue.push({ opts.start, astarNodes[opts.start].fScore });

	bool hasBlockedPaths = opts.blockedPaths.size();
	bool hasBlockedSubSectors = opts.blockedSubSectors.size();

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

			return route;
		}

		curDat.closed = true;

		NavSector& currentNode = mesh[current];

		for (int i = 0; i < currentNode.links.size(); i++) {
			NavSectorLink& link = currentNode.links[i];
			int neighbor = link.target->id;
			AstarNode& neighborDat = astarNodes[neighbor];

			if (neighborDat.closed)
				continue;

			if (hasBlockedSubSectors && opts.blockedSubSectors.count(neighbor))
				continue;

			if (hasBlockedPaths && opts.blockedPaths.count(link.id))
				continue;

			if (opts.blockedPathHandling == WBOT_ROUTE_BLOCK_FORBID && link.blocked(opts.actor))
				continue;

			float linkDist = path_dist(link);
			float linkCost = path_cost(link, linkDist, opts);
			float tentative_gScore = curDat.gScore + linkCost;

			if (neighborDat.pathed && tentative_gScore >= neighborDat.gScore)
				continue; // not a better path

			// This path is the best until now. Record it!
			neighborDat.pathed = true;
			neighborDat.gScore = tentative_gScore;
			neighborDat.fScore = tentative_gScore + node_heuristic(mesh[neighbor].id, goal.id);
			neighborDat.cameFromNode = current;
			neighborDat.cameFromDist = linkDist;
			neighborDat.cameFromCost = linkCost;

			// discover a new node
			openQueue.push({ neighbor, neighborDat.fScore });
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
	
	RouteOpts opts;
	opts.start = actorNavId;
	if (blockedPaths)
		opts.blockedPaths = *blockedPaths;

	while ((mapKey = it.Next()) != NULL) {
		if (mapKey->Owner)
			continue; // key in someone's inventory

		int keyNavId = get_nav_id(mapKey);
		opts.end = keyNavId;

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
	
	if (!g_wbot_test_mode)
		Printf("Relinked sector %d (%+d links)\n", secid, linksAdded);

	pending_sector_relinks.erase(pending_sector_relinks.begin() + idx);
}

