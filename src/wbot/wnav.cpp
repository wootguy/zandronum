#include "wnav.h"
#include "wbot.h"
#include "r_state.h"
#include "sv_commands.h"
#include "p_local.h"
#include "p_lnspec.h"

#include <algorithm>
#include <unordered_set>
#include <unordered_map>
#include <vector>

using namespace std;

SectorNavMesh g_wbot_nav;

bool NavSectorLink::blocked(AActor* actor, BotGoal* unblockGoal) {
	if (!g_wbot_nav.can_cross_seg_now(seg)) {
		if (seg->linedef) {
			switch (seg->linedef->special) {
			case Door_Open:
			case Door_Raise:
				return false;
			default:
				break;
			}
		}

		if (actor) {
			subsector_t& targetSub = subsectors[target];
			if (targetSub.sector->tag && g_wbot_nav.can_trigger_tag(actor, targetSub.sector->tag, unblockGoal)) {
				// the target sector can be moved by a linedef that the actor can currently reach.
				return false;
			}
		}

		return true;
	}

	return false;
}

NavSectorLink* NavSector::getLink(int subSectorId) {
	for (int i = 0; i < links.size(); i++) {
		if (links[i].target == subSectorId) {
			return &links[i];
		}
	}

	Printf("Nav sector %d has no link to %d\n", id, subSectorId);
	return NULL;
}

LinkSeg SectorNavMesh::GetSegmentOverlap(seg_t* a, seg_t* b)
{
	LinkSeg result = { 0 };

	int64_t ax1 = a->v1->x;
	int64_t ay1 = a->v1->y;
	int64_t ax2 = a->v2->x;
	int64_t ay2 = a->v2->y;

	int64_t bx1 = b->v1->x;
	int64_t by1 = b->v1->y;
	int64_t bx2 = b->v2->x;
	int64_t by2 = b->v2->y;

	const int64_t EPS = 16;

	int64_t cross1 =
		(ax2 - ax1) * (by1 - ay1) -
		(ay2 - ay1) * (bx1 - ax1);

	int64_t cross2 =
		(ax2 - ax1) * (by2 - ay1) -
		(ay2 - ay1) * (bx2 - ax1);

	if (llabs(cross1) > EPS || llabs(cross2) > EPS)
		return result;

	if (llabs(ax2 - ax1) >= llabs(ay2 - ay1))
	{
		bool aForward = ax1 < ax2;
		bool bForward = bx1 < bx2;

		int64_t start = std::max(std::min(ax1, ax2), std::min(bx1, bx2));
		int64_t end = std::min(std::max(ax1, ax2), std::max(bx1, bx2));

		if (end - start < (8 << FRACBITS))
			return result;

		double dx = double(ax2 - ax1);
		double dy = double(ay2 - ay1);

		double t1 = (start - ax1) / dx;
		double t2 = (end - ax1) / dx;

		result.x1 = (fixed_t)start;
		result.y1 = (fixed_t)(ay1 + t1 * dy);

		result.x2 = (fixed_t)end;
		result.y2 = (fixed_t)(ay1 + t2 * dy);
	}
	else
	{
		int64_t start = std::max(std::min(ay1, ay2), std::min(by1, by2));
		int64_t end = std::min(std::max(ay1, ay2), std::max(by1, by2));

		if (end - start < (8 << FRACBITS))
			return result;

		double dx = double(ax2 - ax1);
		double dy = double(ay2 - ay1);

		double t1 = (start - ay1) / dy;
		double t2 = (end - ay1) / dy;

		result.x1 = (fixed_t)(ax1 + t1 * dx);
		result.y1 = (fixed_t)start;

		result.x2 = (fixed_t)(ax1 + t2 * dx);
		result.y2 = (fixed_t)end;
	}

	return result;
}

bool SectorNavMesh::is_seg_potentially_crossable(seg_t* seg) {
	if (!seg->frontsector || !seg->backsector)
		return false;

	if (seg->linedef && (seg->linedef->flags & ML_BLOCKING))
		return false; // impassable

	return true;
}

bool SectorNavMesh::can_cross_seg_now(seg_t* seg)
{
	fixed_t x = (seg->v1->x + seg->v2->x) / 2;
	fixed_t y = (seg->v1->y + seg->v2->y) / 2;

	fixed_t frontFloor = seg->frontsector->floorplane.ZatPoint(x, y);
	fixed_t backFloor = seg->backsector->floorplane.ZatPoint(x, y);

	const bool canJump = true;
	int maxHeight = canJump ? JUMP_HEIGHT : STEP_HEIGHT;

	if ((backFloor - frontFloor) > (maxHeight << FRACBITS)) {
		return false; // too high to step
	}

	fixed_t backCeil = seg->backsector->ceilingplane.ZatPoint(x, y);

	if ((backCeil - backFloor) < (56 << FRACBITS)) {
		return false; // not enough space
	}

	return true;
}

subsector_t* SectorNavMesh::get_neighbor_subsector(subsector_t* ignoreSector, seg_t* borderSeg, LinkSeg& linkseg) {
	const fixed_t playerWidth = 32 << FRACBITS;
	memset(&linkseg, 0, sizeof(linkseg));

	for (int j = 0; j < numsubsectors; j++) {
		subsector_t& otherSub = subsectors[j];
		if (&otherSub == ignoreSector)
			continue;

		for (int s = 0; s < otherSub.numlines; s++) {
			seg_t& tseg = otherSub.firstline[s];

			if (!tseg.frontsector || !tseg.backsector)
				continue; // impassable wall

			// TODO: Allow thin segments for tightrope areas
			linkseg = GetSegmentOverlap(&tseg, borderSeg);
			if (linkseg.length() >= playerWidth) {
				return &otherSub;
			}
		}
	}

	return NULL;
}

void SectorNavMesh::generate_node_graph() {
	nav_sectors.clear();
	nav_sectors.resize(numsubsectors);
	line_subsectors.clear();

	int totalLinks = 0;

	// link sectors as a nav mesh
	for (int i = 0; i < numsubsectors; i++) {
		subsector_t& sub = subsectors[i];
		NavSector& nav = nav_sectors[i];

		int subId = &sub - subsectors;
		int subCenterX = 0;
		int subCenterY = 0;

		for (int k = 0; k < sub.numlines; k++) {
			seg_t& seg = sub.firstline[k];

			subCenterX += seg.v1->x >> FRACBITS;
			subCenterY += seg.v1->y >> FRACBITS;

			if (!is_seg_potentially_crossable(&seg))
				continue;

			LinkSeg linkseg;
			subsector_t* neighborSector = get_neighbor_subsector(&sub, &seg, linkseg);

			if (!neighborSector)
				continue;

			fixed_t cx = (linkseg.x1 + linkseg.x2) / 2;
			fixed_t cy = (linkseg.y1 + linkseg.y2) / 2;
			subsector_t* csector = R_PointInSubsector(cx, cy);
			float z = csector->sector->floorplane.ZatPoint(cx, cy);

			NavSectorLink link;
			link.target = neighborSector - subsectors;
			link.overlap = linkseg;
			link.overlapCenter = FVector3(cx, cy, z);
			link.seg = &seg;
			nav.links.push_back(link);

			totalLinks++;
		}

		nav.id = i;
		nav.x = subCenterX / (int)sub.numlines;
		nav.y = subCenterY / (int)sub.numlines;
		nav.z = sub.sector->floorplane.ZatPoint(nav.x << FRACBITS, nav.y << FRACBITS) >> FRACBITS;
	}

	// create linedef sector mapping
	for (int i = 0; i < numlines; i++) {
		line_t& line = lines[i];
		if (!line.special) {
			continue;
		}

		for (int s = 0; s < numsubsectors; s++) {
			subsector_t& sub = subsectors[s];

			for (int k = 0; k < sub.numlines; k++) {
				if (sub.firstline[k].linedef != &line) {
					continue;
				}

				line_subsectors[i].push_back(s);
			}
		}
	}

	Printf("Nav mesh is %d sectors with %d links\n", (int)nav_sectors.size(), totalLinks);
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

	triggerable_tags.clear();

	subsector_t* sub = R_PointInSubsector(player->x, player->y);
	int subId = sub - subsectors;

	if (sub && subId < nav_sectors.size()) {
		NavSector& nav = nav_sectors[subId];
		FVector3 pos(nav.x << FRACBITS, nav.y << FRACBITS, nav.z << FRACBITS);
		FVector3 plr(player->x, player->y, player->z + 16);

		for (int k = 0; k < nav.links.size(); k++) {
			NavSectorLink& link = nav.links[k];
			NavSector& linkNav = nav_sectors[link.target];
			if (!link.blocked(actor, NULL)) {
				draw_debug_line(nav.pos(), link.pos(), actor);
				draw_debug_line(link.pos(), linkNav.pos(), actor);
			}
		}

		fixed_t borderZ = nav.z << FRACBITS;
		for (int k = 0; k < sub->numlines; k++) {
			seg_t& seg = sub->firstline[k];

			FVector3 start(seg.v1->x, seg.v1->y, borderZ);
			FVector3 end(seg.v2->x, seg.v2->y, borderZ);
			draw_debug_line(start, end, actor);
		}
	}

	for (int i = 0; i < nav_sectors.size(); i++) {
		NavSector& node = nav_sectors[i];
		SERVERCOMMANDS_SpawnBlood(node.x << FRACBITS, node.y << FRACBITS, (node.z + 16) << FRACBITS, 0, 100, player);
	}
}

void SectorNavMesh::draw_debug_line(FVector3 start, FVector3 end, AActor* actor) {
	// this sucks and the railgun effect crashes the client so can't use that
	FVector3 dir = (end - start).Resize(4 << FRACBITS);
	int spawns = (end - start).Length() / (4 << FRACBITS);
	for (int i = 0; i < spawns; i++) {
		FVector3 pos = start + (dir * i);
		if (i == spawns - 1) {
			pos = end;
		}
		SERVERCOMMANDS_SpawnBlood(pos.X, pos.Y, pos.Z, 0, 1, actor);
	}
}

int SectorNavMesh::get_nav_id(fixed_t x, fixed_t y) {
	return R_PointInSubsector(x, y) - subsectors;
}

int SectorNavMesh::get_nav_id(AActor* actor) {
	return get_nav_id(actor->x, actor->y);
}

float SectorNavMesh::path_cost(int a, int b) {
	NavSector& nodea = nav_sectors[a];
	NavSector& nodeb = nav_sectors[b];
	FVector3 delta = nodea.pos() - nodeb.pos();
	return delta.Length();
}

vector<int> SectorNavMesh::get_astar_route(AActor* actor, int startSubSectorId, int endSubSectorId)
{
	triggerable_tags.clear();

	unordered_set<int> closedSet;
	unordered_set<int> openSet;

	unordered_map<int, float> gScore;
	unordered_map<int, float> fScore;
	unordered_map<int, int> cameFrom;

	vector<int> emptyRoute;

	if (startSubSectorId < 0 || endSubSectorId < 0 || startSubSectorId > nav_sectors.size() || endSubSectorId > nav_sectors.size()) {
		Printf("AStarRoute: invalid start/end nodes\n");
		return emptyRoute;
	}

	if (startSubSectorId == endSubSectorId) {
		emptyRoute.push_back(startSubSectorId);
		return emptyRoute;
	}

	NavSector& start = nav_sectors[startSubSectorId];
	NavSector& goal = nav_sectors[endSubSectorId];

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

		//println("Current is " + current);

		if (current == goal.id) {
			//println("MAde it to the goal");
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

			return path;
		}

		openSet.erase(current);
		closedSet.insert(current);

		NavSector& currentNode = nav_sectors[current];

		for (int i = 0; i < currentNode.links.size(); i++) {
			NavSectorLink& link = currentNode.links[i];
			if (link.target == -1) {
				break;
			}

			int neighbor = link.target;
			if (neighbor < 0 || neighbor >= nav_sectors.size()) {
				continue;
			}
			if (closedSet.count(neighbor))
				continue;

			if (link.blocked(actor, NULL)) {
				continue;
			}
			//if (currentNode.blockers.size() > i and currentNode.blockers[i] & blockers != 0)
			//	continue; // blocked by something (monsterclip, normal clip, etc.). Don't route through this path.

			// discover a new node
			openSet.insert(neighbor);

			// The distance from start to a neighbor
			NavSector& neighborNode = nav_sectors[neighbor];

			float tentative_gScore = gScore[current];
			tentative_gScore += path_cost(currentNode.id, neighborNode.id);

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

bool SectorNavMesh::can_trigger_tag(AActor* actor, short tag, BotGoal* unblockGoal) {
	auto alreadyChecked = g_wbot_nav.triggerable_tags.find(tag);
	if (alreadyChecked != g_wbot_nav.triggerable_tags.end()) {
		if (unblockGoal)
			*unblockGoal = alreadyChecked->second.goal;
		return alreadyChecked->second.canTrigger;
	}

	TagTriggerGoal tgoal;
	tgoal.canTrigger = false;

	int actorNavId = get_nav_id(actor);

	int lineid = -1;
	for (int i = 0; i < numlines; i++) {
		line_t& line = lines[i];

		if (line.args[0] == tag) {
			lineid = i;
			break;
		}
	}

	if (lineid == -1) {
		triggerable_tags[tag] = tgoal;
		return false;
	}

	auto lineSectors = line_subsectors.find(lineid);
	if (lineSectors == line_subsectors.end()) {
		triggerable_tags[tag] = tgoal;
		return false;
	}

	for (int subid : lineSectors->second) {
		// not passing an actor to prevent infinite loops
		if (subid == actorNavId || get_astar_route(NULL, actorNavId, subid).size()) {
			// actor can route to a subsector which touches the linedef which activates the tag
			tgoal.canTrigger = true;
			tgoal.goal.action = WBOT_GOAL_ACTION_USE;
			tgoal.goal.lineid = lineid;
			triggerable_tags[tag] = tgoal;
			if (unblockGoal)
				*unblockGoal = tgoal.goal;
			//Printf("Line %d targets tag %d and can be routed to\n", subid, (int)tag);
			return true;
		}
	}

	triggerable_tags[tag] = tgoal;
	return false;
}