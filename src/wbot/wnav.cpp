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

bool NavSectorLink::blocked(bool recurse) {
	g_wbot_nav.pathTests++;

	if (!g_wbot_nav.can_cross_seg_now(seg)) {
		if (seg->linedef && g_wbot_nav.does_linedef_move_tag(seg->linedef, 0)) {
			// border is an untagged linedef, which means it moves the target sector,
			// which probably unblocks the path. TODO: not always!
			return false;
		}

		return true;
	}
	
	if (recurse && linkWidth <= PLAYER_WIDTH) {
		// too narrow to move thru unless the neighbor sectors are passable
		NavSector& nav = g_wbot_nav.nav_sectors[parent];

		int expandedWidth = linkWidth;
		int neighbors[2] = {leftSector, rightSector};
		for (int i = 0; i < 2; i++) {
			if (neighbors[i] < 0)
				continue;

			NavSector& neighborNav = g_wbot_nav.nav_sectors[neighbors[i]];
			bool isWalkable = nav.getFloorZ() + (STEP_HEIGHT << FRACBITS) > neighborNav.getFloorZ();
			bool wontHitHead = neighborNav.getCeilZ() > nav.getFloorZ() + (DUCK_HEIGHT << FRACBITS);
			if (isWalkable && wontHitHead) {
				// the neighboring sector is not a wall and won't prevent movement on the target sector
				// TODO: need to check how thick the neighbor is too
				expandedWidth += PLAYER_WIDTH;
				continue;
			}

			NavSectorLink* link = nav.getLink(neighbors[i]);

			if (link && !link->blocked(false)) {
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

NavSectorLink* NavSector::getLink(int subSectorId) {
	if (subSectorId < 0)
		return NULL;

	for (int i = 0; i < links.size(); i++) {
		if (links[i].target == subSectorId) {
			return &links[i];
		}
	}

	//Printf("Nav sector %d has no link to %d\n", id, subSectorId);
	return NULL;
}

fixed_t NavSector::getHeight() {
	fixed_t fx = x << FRACBITS;
	fixed_t fy = y << FRACBITS;

	sector_t* sec = subsectors[id].sector;
	if (!sec) {
		Printf("Nav %d has no sector!\n", id);
		return 0;
	}

	return sec->ceilingplane.ZatPoint(fx, fy) - sec->floorplane.ZatPoint(fx, fy);
}

fixed_t NavSector::getFloorZ() {
	fixed_t fx = x << FRACBITS;
	fixed_t fy = y << FRACBITS;

	sector_t* sec = subsectors[id].sector;
	if (!sec) {
		Printf("Nav %d has no sector!\n", id);
		return 0;
	}

	return sec->floorplane.ZatPoint(fx, fy);
}

fixed_t NavSector::getCeilZ() {
	fixed_t fx = x << FRACBITS;
	fixed_t fy = y << FRACBITS;

	sector_t* sec = subsectors[id].sector;
	if (!sec) {
		Printf("Nav %d has no sector!\n", id);
		return 0;
	}

	return sec->ceilingplane.ZatPoint(fx, fy);
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

	if ((backCeil - backFloor) < (DUCK_HEIGHT << FRACBITS)) {
		return false; // not enough space
	}

	return true;
}

LinkSeg SectorNavMesh::get_neighbor_subsector(subsector_t* ignoreSector, seg_t* borderSeg) {
	const fixed_t epsilonWidth = 1 << FRACBITS;
	LinkSeg ret;
	memset(&ret, 0, sizeof(ret));

	for (int j = 0; j < numsubsectors; j++) {
		subsector_t& otherSub = subsectors[j];
		if (&otherSub == ignoreSector)
			continue;

		for (int s = 0; s < otherSub.numlines; s++) {
			seg_t& tseg = otherSub.firstline[s];

			if (!tseg.frontsector || !tseg.backsector)
				continue; // impassable wall

			// TODO: Allow thin segments for tightrope areas
			ret = GetSegmentOverlap(&tseg, borderSeg);
			if (ret.length() >= epsilonWidth) {
				ret.idx = s;
				ret.otherSub = j;
				return ret;
			}
		}
	}

	ret.otherSub = -1;
	return ret;
}

bool SectorNavMesh::does_linedef_move_tag(line_t* line, short tag) {
	// only add specials here that could be potentially helpful for unblocking a path.
	// For instance, raising a door or elevator. A ceiling or door coming down lower 
	// will not help a bot pass the sector
	switch (line->special) {
	case Door_Open:
	case Door_Raise:
	case Door_LockedRaise:
	case Plat_PerpetualRaise:
	case Plat_DownWaitUpStay:
	case Plat_DownByValue:
	case Plat_UpWaitDownStay:
	case Plat_UpByValue:
	case Plat_UpNearestWaitDownStay:
	case Plat_DownWaitUpStayLip:
	case Plat_PerpetualRaiseLip:
	case Plat_RaiseAndStayTx0:
	case Plat_UpByValueStayTx:
	case Floor_LowerToLowest:
	case Floor_LowerToNearest:
	case Floor_LowerToHighest:
	case Floor_RaiseToHighest:
	case Floor_RaiseToNearest:
	case Floor_RaiseByValueTxTy:
	case Floor_LowerToLowestTxTy:
	case Floor_RaiseToLowestCeiling:
	case Elevator_RaiseToNearest:
	case Elevator_MoveToFloor:
	case Elevator_LowerToNearest:
	case Ceiling_RaiseByValue:
	case Ceiling_RaiseToNearest:
	case Generic_Floor:
	case Generic_Ceiling:
	case Generic_Door:
	case Generic_Lift:
	case Generic_Stairs:
	case Stairs_BuildDown:
	case Stairs_BuildUp:
	case Stairs_BuildDownSync:
	case Stairs_BuildUpSync:
	case Stairs_BuildUpDoom:
		return line->args[0] == tag;
	default:
		if (tag != 0 && line->args[0] == tag) {
			Printf("Line %d targets tag %d but don't know what special %d is\n", line - lines, tag, line->special);
		}

		return false;
	}
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

			LinkSeg linkseg = get_neighbor_subsector(&sub, &seg);

			if (linkseg.otherSub < 0)
				continue;

			fixed_t cx = (linkseg.x1 + linkseg.x2) / 2;
			fixed_t cy = (linkseg.y1 + linkseg.y2) / 2;
			subsector_t* csector = R_PointInSubsector(cx, cy);
			float z = csector->sector->floorplane.ZatPoint(cx, cy);

			NavSectorLink link;
			link.target = linkseg.otherSub;
			link.overlapCenter = FVector3(cx, cy, z);
			link.seg = &seg;
			link.id = totalLinks++;
			link.parent = i;
			link.linkWidth = (int)linkseg.length() >> FRACBITS;

			if (i == 87 && linkseg.otherSub == 94) {
				Printf("");
			}

			subsector_t& neighbor = subsectors[linkseg.otherSub];
			int leftIdx = (linkseg.idx + neighbor.numlines - 1) % neighbor.numlines;
			int rightIdx = (linkseg.idx + 1) % neighbor.numlines;
			LinkSeg leftSector = get_neighbor_subsector(&neighbor, &neighbor.firstline[leftIdx]);
			LinkSeg rightSector = get_neighbor_subsector(&neighbor, &neighbor.firstline[rightIdx]);
			link.leftSector = leftSector.otherSub;
			link.rightSector = rightSector.otherSub;

			if (link.linkWidth < PLAYER_WIDTH && link.leftSector == -1 && link.rightSector == -1) {
				// link is too narrow to enter and both sides of it are impassable walls
				continue;
			}

			nav.links.push_back(link);
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
		seg_t lineSeg; // fake seg for overlap test
		lineSeg.v1 = line.v1;
		lineSeg.v2 = line.v2;

		for (int s = 0; s < numsubsectors; s++) {
			subsector_t& sub = subsectors[s];

			float mostOverlap = 0;

			for (int k = 0; k < sub.numlines; k++) {
				seg_t& seg = sub.firstline[k];
				if (seg.linedef != &line) {
					continue;
				}
				if (seg.frontsector != line.frontsector) {
					continue; // only want the sector in front of the line
				}

				LinkSeg linkseg = GetSegmentOverlap(&seg, &lineSeg);
				float overlap = linkseg.length();
				if (overlap > mostOverlap) {
					mostOverlap = overlap;
					line_subsectors[i] = s;
				}
			}
		}
	}

	// add sector triggers
	for (int i = 0; i < numsubsectors; i++) {
		subsector_t& sub = subsectors[i];
		sector_t* sec = sub.sector;
		NavSector& nav = nav_sectors[i];

		if (sec->tag) {
			// any line with this sector's tag can move the sector
			for (int k = 0; k < numlines; k++) {
				line_t& line = lines[k];

				if (does_linedef_move_tag(&line, sec->tag)) {
					BotGoal trig;
					trig.action = WBOT_GOAL_ACTION_USE;
					trig.lineid = k;
					nav.triggers.push_back(trig);
				}
			}
		}
		else {
			// surrounding lines can move the sector if untagged
			for (int k = 0; k < sec->linecount; k++) {
				line_t* line = sec->lines[k];

				if (line->backsector == sec && does_linedef_move_tag(line, 0)) {
					BotGoal trig;
					trig.action = WBOT_GOAL_ACTION_USE;
					trig.lineid = line - lines;
					nav.triggers.push_back(trig);
				}
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

	subsector_t* sub = R_PointInSubsector(player->x, player->y);
	int subId = sub - subsectors;

	if (sub && subId < nav_sectors.size()) {
		NavSector& nav = nav_sectors[subId];
		FVector3 pos(nav.x << FRACBITS, nav.y << FRACBITS, nav.z << FRACBITS);
		int spritesDrawn = 0;
		const int maxSprites = 1000;

		for (int k = 0; k < nav.links.size() && spritesDrawn < maxSprites; k++) {
			NavSectorLink& link = nav.links[k];
			NavSector& linkNav = nav_sectors[link.target];
			
			if (!link.blocked()) {
				spritesDrawn += draw_debug_line(nav.pos(), link.pos(), actor);
				spritesDrawn += draw_debug_line(link.pos(), linkNav.pos(), actor);
			}
		}

		fixed_t borderZ = nav.z << FRACBITS;
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

	for (int i = 0; i < nav_sectors.size(); i++) {
		NavSector& node = nav_sectors[i];

		if (P_AproxDistance((node.x << FRACBITS) - player->x, (node.y << FRACBITS) - player->y) > (1000 << FRACBITS)) {
			continue;
		}

		SERVERCOMMANDS_SpawnBlood(node.x << FRACBITS, node.y << FRACBITS, (node.z + 16) << FRACBITS, 0, 100, player);
	}
}

int SectorNavMesh::draw_debug_line(FVector3 start, FVector3 end, AActor* actor) {
	// this sucks and the railgun effect crashes the client so can't use that
	
	FVector3 delta = end - start;
	float len = delta.Length();
	int ilen = (int)len >> FRACBITS;
	int spacing = 4;
	
	if (ilen > 1600)
		spacing = 64;
	else if (ilen > 800)
		spacing = 32;
	else if (ilen > 400)
		spacing = 16;
	else if (ilen > 200)
		spacing = 8;

	FVector3 dir = delta.Resize(spacing << FRACBITS);
	int spawns = len / (spacing << FRACBITS);
	int i = 0;
	for (i = 0; i < spawns; i++) {
		FVector3 pos = start + (dir * i);
		if (i == spawns - 1) {
			pos = end;
		}
		SERVERCOMMANDS_SpawnBlood((fixed_t)pos.X, (fixed_t)pos.Y, (fixed_t)pos.Z, 0, 1, actor);
	}

	return i;
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

vector<int> SectorNavMesh::get_astar_route(int startSubSectorId, int endSubSectorId, unordered_set<int>* blockedPaths)
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

		NavSector& currentNode = nav_sectors[current];

		for (int i = 0; i < currentNode.links.size(); i++) {
			NavSectorLink& link = currentNode.links[i];
			if (link.target == -1) {
				break;
			}

			int neighbor = link.target;
			if (neighbor < 0 || neighbor >= nav_sectors.size())
				continue;

			if (closedSet.count(neighbor))
				continue;

			if (blockedPaths && blockedPaths->count(link.id))
				continue;

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