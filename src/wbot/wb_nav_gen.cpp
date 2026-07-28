#include "wb_nav_gen.h"
#include "wb_map.h"
#include "wb_util.h"
#include "p_trace.h"
#include "p_local.h"
#include "r_state.h"
#include "r_utility.h"
#include "p_lnspec.h"
#include "p_spec.h"

using namespace std;

int g_total_links;

bool SectorNavMeshGenerator::trace_jump(FVector3 start, FVector3 end, int fromMovement, int toMovement) {
	const fixed_t rightStep = ((PLAYER_WIDTH / 2) / 2) << FRACBITS;
	FVector2 start2d = FVector2(start.X, start.Y);
	FVector2 end2d = FVector2(end.X, end.Y);
	FVector2 jumpDir = (end2d - start2d).Unit();
	FVector3 rightDir(jumpDir.Y, -jumpDir.X, 0);
	bool fromNavMovesDown = fromMovement & FL_SECTOR_MOVE_FLOOR_DOWN;
	FTraceResults tr;

	for (int i = -2; i <= 2; i++) {
		if (TraceLine(start + rightDir * i * rightStep, end + rightDir * i * rightStep, true, NULL, &tr)) {
			if (tr.HitType == TRACE_HitWall && tr.Line && tr.Line->backsector) {
				if (g_wb_mapinfo.sector_info[tr.Line->backsector - sectors].moveFlags) {
					continue; // wall may move out of the way in the future
				}
			}
			else if (tr.HitType == TRACE_HitCeiling && fromNavMovesDown && start.Z > tr.Z) {
				// sector may be fully lifted to the ceiling and so can't connect yet
				continue; // may move out of the way in the future
			}
			else if (tr.HitType == TRACE_HitFloor) {
				if (g_wb_mapinfo.sector_info[tr.Sector - sectors].moveFlags & FL_SECTOR_MOVE_FLOOR_DOWN) {
					continue; // hit a floor that may move out of the way later
				}
			}

			FVector3 impact(tr.X, tr.Y, tr.Z);
			if (tr.HitType == TRACE_HitWall && tr.Line) {
				// trace has a bug where you can impact a wall in a different sector
				sector_t* impactSector = R_PointInSubsector(tr.X, tr.Y)->sector;
				if (tr.Line->frontsector != impactSector && tr.Line->backsector != impactSector) {
					continue; // impossible to have hit this line
				}
			}

 			//TraceLine(start + rightDir * i * rightStep, end + rightDir * i * rightStep, true, NULL, &tr); // debug
			return false; // hit an immovable wall or ceiling/floor
		}
	}

	return true;
}

bool SectorNavMeshGenerator::is_potential_jump_link(NavSector& fromNav, NavSectorLink& fromLink, NavSector& toNav, NavSectorLink& toLink) {
	fixed_t fromFloorZ = fromNav.getFloorZ();
	fixed_t toFloorZ = toNav.getFloorZ();
	fixed_t jumpHeight = toFloorZ - fromFloorZ;

	int fromMovement = fromNav.getMoveFlags();
	int toMovement = toNav.getMoveFlags();
	bool jumpMayBeLowerLater = (toMovement & FL_SECTOR_MOVE_FLOOR_DOWN) || (fromMovement & FL_SECTOR_MOVE_FLOOR_UP);

	if (jumpHeight >= (JUMP_HEIGHT << FRACBITS) && !jumpMayBeLowerLater) {
		// too high to jump to
		return false;
	}

	fixed_t linkDist = (fromLink.pos() - toLink.pos()).Length();

	// increase link distance for drops, decrease for jumps
	int maxJumpDist = (JUMP_DIST << FRACBITS) - jumpHeight;
	bool jumpTooFarCurrently = linkDist > maxJumpDist;
	if (jumpTooFarCurrently && !jumpMayBeLowerLater) {
		return false; // too far to link to, and the target won't ever lower
	}

	FVector2 e1v1(fromLink.seg->v1->x, fromLink.seg->v1->y);
	FVector2 e1v2(fromLink.seg->v2->x, fromLink.seg->v2->y);

	FVector2 e2v1(toLink.seg->v1->x, toLink.seg->v1->y);
	FVector2 e2v2(toLink.seg->v2->x, toLink.seg->v2->y);

	FVector2 dir1 = (e1v2 - e1v1).Unit();
	FVector2 dir2 = (e2v2 - e2v1).Unit();

	FVector2 normalA(dir1.Y, -dir1.X);
	FVector2 normalB(dir2.Y, -dir2.X);

	// flip normals if pointing inward
	if (DotProduct(normalA, (e1v1 - fromNav.center).Unit()) < 0) {
		normalA *= -1;
	}
	if (DotProduct(normalB, (e2v1 - toNav.center).Unit()) < 0) {
		normalB *= -1;
	}

	float normalDot = DotProduct(normalA, normalB);
	if (normalDot > -0.5f) {
		return false; // edges not facing each other enough
	}

	if (fromNav.getLink(toNav.id)) {
		return false; // already have a link to this sector
	}

	FVector2 targetPos = toNav.pos();
	FVector2 jumpStart = fromLink.GetJumpStartPos(targetPos);
	FVector2 jumpEnd = fromLink.GetJumpEndPos(targetPos);
	FVector2 jumpDir = (jumpEnd - jumpStart).Unit();

	// move inward a bit to avoid collision with the start/end lines
	jumpStart += jumpDir * FRACUNIT;
	jumpEnd -= jumpDir * FRACUNIT;

	float jumpDirDot = DotProduct(normalA, jumpDir);
	if (jumpDirDot < 0.5f) {
		return false; // awkward jump direction
	}

	if (IsBoxWallClipped(jumpStart, (PLAYER_RADIUS << FRACBITS))) {
		return false; // jump position clipped inside a wall
	}

	for (NavSectorLink& link : fromNav.links) {
		NavSector* neighbor = link.target;
		if (link.isJump || link.isCliff)
			continue;

		for (NavSectorLink& neighborLink : neighbor->links) {
			if (neighborLink.isJump || neighborLink.isCliff)
				continue;

			if (neighborLink.target->getMoveFlags() & FL_SECTOR_MOVE_FLOOR_DOWN)
				continue; // may not be level in the future

			if (neighborLink.target->id == toNav.id && !neighborLink.isJump) {
				// a neighbor has a walkable link to the jump target.
				// This jump would hardly be a shortcut.
				return false;
			}
		}
	}

	// check that the path is clear
	FVector3 start = FVector3(jumpStart, fromFloorZ + (56 << FRACBITS));
	FVector3 end = FVector3(jumpEnd, toFloorZ + (56 << FRACBITS));

	bool fromNavMovesUp = fromMovement & FL_SECTOR_MOVE_FLOOR_UP;
	bool toNavMovesDown = toMovement & FL_SECTOR_MOVE_FLOOR_DOWN;

	// First test if any impassable walls are intersected.
	if (TraceImpassable(jumpStart, jumpEnd))
		return false;

	if (jumpTooFarCurrently && jumpMayBeLowerLater) {
		// One or both sectors must move for the jump to be possible.
		// Test ideal elevations instead of current.
		fixed_t movementNeeded = abs(linkDist - (JUMP_DIST << FRACBITS));
		FVector3 moved(0, 0, movementNeeded);
		FVector3 halfMoved(0, 0, movementNeeded / 2);

		bool canRaiseStart = true;
		bool canRaiseStartHalf = true;
		if (!(fromMovement & FL_SECTOR_MOVE_CEIL_UP)) {
			// if the ceiling in the start sector doesn't move, then it can only be lifted so high
			// before jumping is impossible
			fixed_t maxFloorZ = fromNav.getCeilZ() - (STAND_HEIGHT << FRACBITS);
			canRaiseStart = maxFloorZ - fromFloorZ > movementNeeded;
			canRaiseStartHalf = maxFloorZ - fromFloorZ > movementNeeded / 2;
		}

		if (fromNavMovesUp && toNavMovesDown) {
			// Both sectors can move. Try all combinations
			if (trace_jump(start, end - moved, fromMovement, toMovement))
				return true;
			if (canRaiseStart && trace_jump(start + moved, end, fromMovement, toMovement))
				return true;
			if (canRaiseStartHalf && trace_jump(start + halfMoved, end - halfMoved, fromMovement, toMovement))
				return true;

			return false;
		}
		else if (toNavMovesDown) {
			// target sector must be lowered for the jump to be possible
			return trace_jump(start, end - moved, fromMovement, toMovement);
		}
		else if (fromNavMovesUp) {
			// start sector must be raised for the jump to be possible
			return canRaiseStart && trace_jump(start + moved, end, fromMovement, toMovement);
		}
	}

	return trace_jump(start, end, fromMovement, toMovement);
}

bool SectorNavMeshGenerator::create_jump_link(NavSector& fromNav, NavSectorLink& fromLink, NavSector& toNav, NavSectorLink& toLink) {
	if (!is_potential_jump_link(fromNav, fromLink, toNav, toLink))
		return false;

	FVector2 targetPos = toNav.pos();
	FVector2 jumpStart = fromLink.GetJumpStartPos(targetPos);
	FVector2 jumpEnd = fromLink.GetJumpEndPos(targetPos);

	NavSectorLink link;
	link.parent = fromLink.parent;
	link.movePos = jumpStart;
	link.linkWidth = fromLink.linkWidth;
	link.isTeleport = fromLink.isTeleport;
	link.isCliff = fromLink.isCliff;
	link.seg = fromLink.seg;

	link.target = &toNav;
	link.id = g_total_links++;
	link.isJump = true;
	link.jumpDist = (jumpEnd - jumpStart).Length();
	link.jumpNeighbor = fromLink.target;

	fromNav.links.push_back(link);

	return true;
}

void SectorNavMeshGenerator::add_jump_links(NavSector* mesh, int nodeid) {
	NavSector& nav = mesh[nodeid];
	bool srcCanBeCliffsLater = nav.getMoveFlags() & FL_SECTOR_MOVE_FLOOR_UP;

	for (int k = 0; k < nav.links.size(); k++) {
		NavSectorLink& link = nav.links[k];

		if (link.isJump)
			continue;

		if (!link.isCliff && !srcCanBeCliffsLater) {
			// if the neighbor can lower, then its still possible to become a cliff
			if (!(link.target->getMoveFlags() & FL_SECTOR_MOVE_FLOOR_DOWN))
				continue;
		}

		// find other other cliff segments to try linking to
		for (int j = 0; j < numsubsectors; j++) {
			NavSector& otherNav = mesh[j];

			if (j == nodeid)
				continue;

			bool dstCanBeCliffsLater = otherNav.getMoveFlags() & FL_SECTOR_MOVE_FLOOR_UP;

			for (int x = 0; x < otherNav.links.size(); x++) {
				NavSectorLink& otherLink = otherNav.links[x];
				if (!otherLink.isCliff || otherLink.isJump)
					continue;

				if (create_jump_link(nav, nav.links[k], otherNav, otherLink)) {
					break;
				}
			}
		}
	}
}

void SectorNavMeshGenerator::add_jump_links(NavSector* mesh) {
	// add jump links between cliff segments
	for (int i = 0; i < numsubsectors; i++) {
		add_jump_links(mesh, i);
	}
}

void SectorNavMeshGenerator::calc_nav_centers(NavSector* mesh) {
	for (int i = 0; i < numsubsectors; i++) {
		subsector_t& sub = subsectors[i];
		NavSector& nav = mesh[i];

		float area = 0;
		FVector2 centroid(0, 0);

		for (int i = 0; i < sub.numlines; i++)
		{
			seg_t& seg = sub.firstline[i];
			FVector2 a(seg.v1->x, seg.v1->y);
			FVector2 b(seg.v2->x, seg.v2->y);

			float cross = a.X * b.Y - b.X * a.Y;

			area += cross;
			centroid += (a + b) * cross;
		}

		nav.center = centroid / (6.0f * (area * 0.5f));
	}
}

void SectorNavMeshGenerator::add_walkable_links(NavSector* mesh, int nodeid, std::vector<AActor*> propBlockers) {
	subsector_t& sub = subsectors[nodeid];
	NavSector& nav = mesh[nodeid];

	for (int k = 0; k < sub.numlines; k++) {
		seg_t& seg = sub.firstline[k];

		if (!g_wb_mapinfo.is_seg_potentially_crossable(&seg)) {
			//if (i == 304) { // debug possible link not created
			//	g_wb_mapinfo.is_seg_potentially_crossable(&seg);
			//}
			continue;
		}

		LinkSeg linkseg = g_wb_mapinfo.get_neighbor_subsector(&sub, &seg);

		if (linkseg.otherSub < 0)
			continue;

		//if (i == 167 && linkseg.otherSub == 171) {
		//	is_seg_potentially_crossable(&seg); // debug impossible link created
		//}

		fixed_t cx = (linkseg.x1 + linkseg.x2) / 2;
		fixed_t cy = (linkseg.y1 + linkseg.y2) / 2;
		float z = sub.sector->floorplane.ZatPoint(cx, cy);
		

		NavSectorLink link;
		link.target = &mesh[linkseg.otherSub];
		link.movePos = FVector3(cx, cy, z);
		link.seg = &seg;
		link.id = g_total_links++;
		link.parent = &nav;
		link.linkWidth = (int)linkseg.length() >> FRACBITS;
		link.isJump = false;
		link.isTeleport = false;
		link.jumpDist = 0;

		subsector_t& neighbor = subsectors[linkseg.otherSub];

		const fixed_t mesh_radius = PLAYER_RADIUS << FRACBITS;
		if (IsBoxWallClipped(link.movePos, mesh_radius)) {
			int step = 8;
			int steps = (link.linkWidth / step) / 2; // half len for left/right split
			FVector2 a(linkseg.x1, linkseg.y1);
			FVector2 b(linkseg.x2, linkseg.y2);
			FVector2 delta = b - a;
			FVector2 dir = delta.Unit() * FRACUNIT;

			bool foundUnclippedPos = false;
			for (int s = 1; s <= steps; s++) {
				FVector2 testLeft = link.movePos + dir * s * step;
				FVector2 testRight = link.movePos + dir * s * -step;

				if (!IsBoxWallClipped(testLeft, mesh_radius)) {
					foundUnclippedPos = true;
					link.movePos = testLeft;
					break;
				}
				if (!IsBoxWallClipped(testRight, mesh_radius)) {
					foundUnclippedPos = true;
					link.movePos = testRight;
					break;
				}
			}
			if (!foundUnclippedPos) {
				continue; // unable to fit a player anywhere along this link
			}
		}

		link.isCliff = sub.sector->floorplane.ZatPoint(cx, cy)
			- neighbor.sector->floorplane.ZatPoint(cx, cy) > (JUMP_HEIGHT << FRACBITS);

		if (link.isCliff)
			nav.hasCliffs = true;

		// redirect target sector if this is the front side of a teleport
		line_t* line = seg.linedef;
		bool isPlayerTele = line && line->special == Teleport && (line->activation & SPAC_Cross);
		if (isPlayerTele && DistanceToLine(nav.pos(), line) < 0) {
			AActor* dest = SelectTeleDest(line->args[0], line->args[1]);
			if (dest) {
				subsector_t* destSub = R_PointInSubsector(dest->x, dest->y);
				link.target = &mesh[destSub - subsectors];
				link.isTeleport = true;
			}
		}

		// don't add links that are blocked by immovable props
		bool propBlocked = false;
		const fixed_t playerRadius = (PLAYER_WIDTH / 2) << FRACBITS;
		FVector2 linkPos = link.movePos;
		for (int i = 0; i < propBlockers.size(); i++) {
			AActor* actor = propBlockers[i];
			FVector2 propPos(actor->x, actor->y);
			if ((propPos - linkPos).Length() < actor->radius + playerRadius) {
				propBlocked = true;
				break;
			}
		}
		if (propBlocked) {
			continue;
		}

		nav.links.push_back(link);
	}
}

NavSector* SectorNavMeshGenerator::generate(std::vector<AActor*> propBlockers) {
	uint64_t genStart = getEpochMillis();

	NavSector* mesh = new NavSector[numsubsectors];

	g_wb_mapinfo.init();

	g_total_links = 0;

	calc_nav_centers(mesh);

	// link sectors as a nav mesh
	for (int i = 0; i < numsubsectors; i++) {
		subsector_t& sub = subsectors[i];
		NavSector& nav = mesh[i];

		nav.id = i;
		nav.doesDamage = g_wb_mapinfo.subsector_does_damage(&sub);

		add_walkable_links(mesh, i, propBlockers);
	}

	add_jump_links(mesh);

	Printf("Generated %d nodes, %d links in %d ms\n", (int)numsubsectors, g_total_links, (int)(getEpochMillis() - genStart));

	return mesh;
}

int SectorNavMeshGenerator::relink_node(NavSector* mesh, int id, std::vector<AActor*> propBlockers) {
	NavSector& nav = mesh[id];
	int oldLinkCount = nav.links.size();
	nav.links.clear();

	nav.hasCliffs = false;
	add_walkable_links(mesh, id, propBlockers);
	add_jump_links(mesh, id);

	NavSector* toNav = &mesh[id];

	// test that links to this sector are still valid if it's no longer possible to move
	int removed = 0;
	for (int i = 0; i < numsubsectors; i++) {
		NavSector& otherNav = mesh[i];

		// don't unlink yet if the source sectors can be elevated in the future
		bool jumpsMayBePossibleLater = otherNav.getMoveFlags() & FL_SECTOR_MOVE_FLOOR_UP;

		for (int k = 0; k < otherNav.links.size(); k++) {
			NavSectorLink& link = otherNav.links[k];

			if (link.target != toNav)
				continue;

			bool stillValid;
			if (link.isJump)
				stillValid = jumpsMayBePossibleLater || (nav.hasCliffs && link.isJumpValid());
			else
				stillValid = g_wb_mapinfo.is_seg_potentially_crossable(link.seg);

			if (!stillValid) {
				otherNav.links.erase(otherNav.links.begin() + k);
				k--;
				removed++;
			}
		}
	}

	return (nav.links.size() - oldLinkCount) - removed;
}