#include "wb_nav_gen.h"
#include "wb_map.h"
#include "wb_util.h"

#include <string>
#include <cmath>

using namespace std;
using namespace wbot;

bool SectorNavMeshGenerator::trace_jump(vec3 start, vec3 end, int fromMovement, int toMovement) {
	const float rightStep = (PLAYER_WIDTH / 2) / 2;
	vec2 start2d = vec2(start.x, start.y);
	vec2 end2d = vec2(end.x, end.y);
	vec2 jumpDir = (end2d - start2d).normalize();
	vec3 rightDir(jumpDir.y, -jumpDir.x, 0);
	bool fromNavMovesDown = fromMovement & FL_SECTOR_MOVE_FLOOR_DOWN;
	TraceResult tr;

	for (int i = -2; i <= 2; i++) {
		if (g_engine.TraceLine(start + rightDir * i * rightStep, end + rightDir * i * rightStep, true, NULL, &tr)) {
			if (tr.hitType == TRACE_HitWall && tr.line && tr.line->backsector) {
				if (tr.sector != tr.line->backsector && tr.line->backsector->moveFlags) {
					continue; // wall may move out of the way in the future
				}
			}
			else if (tr.hitType == TRACE_HitCeiling && fromNavMovesDown && start.z > end.z) {
				// sector may be fully lifted to the ceiling and so can't connect yet
				continue; // may move out of the way in the future
			}
			else if (tr.hitType == TRACE_HitFloor) {
				if (tr.sector->moveFlags & FL_SECTOR_MOVE_FLOOR_DOWN) {
					continue; // hit a floor that may move out of the way later
				}
			}

			if (tr.hitType == TRACE_HitWall && tr.line) {
				// trace has a bug where you can impact a wall in a different sector
				MapSector* impactSector = g_map.GetSubsector(tr.endPos.x, tr.endPos.y)->sector;
				if (tr.line->frontsector != impactSector && tr.line->backsector != impactSector) {
					if (!PointAlignedSegment(tr.endPos, tr.line->v1, tr.line->v2))
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
	float fromFloorZ = fromNav.sector->getFloorZ();
	float toFloorZ = toNav.sector->getFloorZ();
	float jumpHeight = toFloorZ - fromFloorZ;

	int fromMovement = fromNav.sector->moveFlags;
	int toMovement = toNav.sector->moveFlags;
	bool jumpMayBeLowerLater = (toMovement & FL_SECTOR_MOVE_FLOOR_DOWN) || (fromMovement & FL_SECTOR_MOVE_FLOOR_UP);

	if (jumpHeight >= JUMP_HEIGHT && !jumpMayBeLowerLater) {
		// too high to jump to
		return false;
	}

	float linkDist = (fromLink.pos() - toLink.pos()).length();

	// increase link distance for drops, decrease for jumps
	int maxJumpDist = JUMP_DIST - jumpHeight;
	bool jumpTooFarCurrently = linkDist > maxJumpDist;
	if (jumpTooFarCurrently && !jumpMayBeLowerLater) {
		return false; // too far to link to, and the target won't ever lower
	}

	vec2 e1v1 = fromLink.seg.a;
	vec2 e1v2 = fromLink.seg.b;

	vec2 e2v1 = toLink.seg.a;
	vec2 e2v2 = toLink.seg.b;

	vec2 dir1 = (e1v2 - e1v1).normalize();
	vec2 dir2 = (e2v2 - e2v1).normalize();

	vec2 normalA(dir1.y, -dir1.x);
	vec2 normalB(dir2.y, -dir2.x);

	// flip normals if pointing inward
	if (dotProduct(normalA, (e1v1 - fromNav.center).normalize()) < 0) {
		normalA *= -1;
	}
	if (dotProduct(normalB, (e2v1 - toNav.center).normalize()) < 0) {
		normalB *= -1;
	}

	float normalDot = dotProduct(normalA, normalB);
	if (normalDot > -0.5f) {
		return false; // edges not facing each other enough
	}

	if (fromNav.getLink(toNav.id)) {
		return false; // already have a link to this sector
	}

	vec2 targetPos = toNav.pos();
	vec2 jumpStart = fromLink.GetJumpStartPos(targetPos);
	vec2 jumpEnd = fromLink.GetJumpEndPos(targetPos);
	vec2 jumpDir = (jumpEnd - jumpStart).normalize();

	// move inward a bit to avoid collision with the start/end lines
	jumpStart += jumpDir;
	jumpEnd -= jumpDir;

	float jumpDirDot = dotProduct(normalA, jumpDir);
	if (jumpDirDot < 0.5f) {
		return false; // awkward jump direction
	}

	if (IsBoxWallClipped(jumpStart, PLAYER_RADIUS)) {
		return false; // jump position clipped inside a wall
	}

	for (NavSectorLink* link : fromNav.links) {
		NavSector* neighbor = link->target;
		if (link->isJump || link->isCliff)
			continue;

		for (NavSectorLink* neighborLink : neighbor->links) {
			if (neighborLink->isJump || neighborLink->isCliff)
				continue;

			if (neighborLink->target->sector->moveFlags & FL_SECTOR_MOVE_FLOOR_DOWN)
				continue; // may not be level in the future

			if (neighborLink->target->id == toNav.id && !neighborLink->isJump) {
				// a neighbor has a walkable link to the jump target.
				// This jump would hardly be a shortcut.
				return false;
			}
		}
	}

	// check that the path is clear
	vec3 start = vec3(jumpStart, fromFloorZ + 56);
	vec3 end = vec3(jumpEnd, toFloorZ + 56);

	bool fromNavMovesUp = fromMovement & FL_SECTOR_MOVE_FLOOR_UP;
	bool toNavMovesDown = toMovement & FL_SECTOR_MOVE_FLOOR_DOWN;

	// First test if any impassable walls are intersected.
	if (g_engine.TraceImpassable(jumpStart, jumpEnd))
		return false;

	if (jumpTooFarCurrently && jumpMayBeLowerLater) {
		// One or both sectors must move for the jump to be possible.
		// Test ideal elevations instead of current.
		float movementNeeded = fabs(linkDist - JUMP_DIST);
		vec3 moved(0, 0, movementNeeded);
		vec3 halfMoved(0, 0, movementNeeded / 2);

		bool canRaiseStart = true;
		bool canRaiseStartHalf = true;
		if (!(fromMovement & FL_SECTOR_MOVE_CEIL_UP)) {
			// if the ceiling in the start sector doesn't move, then it can only be lifted so high
			// before jumping is impossible
			float maxFloorZ = fromNav.sector->getCeilZ() - STAND_HEIGHT;
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

bool SectorNavMeshGenerator::create_jump_link(BotMeshData& mesh, NavSector& fromNav, NavSectorLink& fromLink, NavSector& toNav, NavSectorLink& toLink) {
	if (!is_potential_jump_link(fromNav, fromLink, toNav, toLink))
		return false;

	vec2 targetPos = toNav.pos();
	vec2 jumpStart = fromLink.GetJumpStartPos(targetPos);
	vec2 jumpEnd = fromLink.GetJumpEndPos(targetPos);

	NavSectorLink link;
	link.parent = fromLink.parent;
	link.movePos = jumpStart;
	link.linkWidth = fromLink.linkWidth;
	link.isTeleport = fromLink.isTeleport;
	link.isCliff = fromLink.isCliff;
	link.seg = fromLink.seg;
	link.sector = fromLink.sector;
	link.linedef = fromLink.linedef;

	link.target = &toNav;
	link.id = mesh.numLinks;
	link.isJump = true;
	link.jumpDist = (jumpEnd - jumpStart).length();
	link.jumpNeighbor = fromLink.target;

	mesh.links[mesh.numLinks] = link;
	fromNav.links.push_back(&mesh.links[mesh.numLinks]);
	mesh.numLinks++;

	return true;
}

void SectorNavMeshGenerator::add_jump_links(BotMeshData& mesh, int nodeid) {
	NavSector& nav = mesh.nodes[nodeid];
	bool srcCanBeCliffsLater = nav.sector->moveFlags & FL_SECTOR_MOVE_FLOOR_UP;

	for (int k = 0; k < nav.links.size(); k++) {
		NavSectorLink& link = *nav.links[k];

		if (link.isJump)
			continue;

		if (!link.isCliff && !srcCanBeCliffsLater) {
			// if the neighbor can lower, then its still possible to become a cliff
			if (!(link.target->sector->moveFlags & FL_SECTOR_MOVE_FLOOR_DOWN))
				continue;
		}

		// find other other cliff segments to try linking to
		for (int j = 0; j < g_map.numsubsectors; j++) {
			NavSector& otherNav = mesh.nodes[j];

			if (j == nodeid)
				continue;

			bool dstCanBeCliffsLater = otherNav.sector->moveFlags & FL_SECTOR_MOVE_FLOOR_UP;

			for (NavSectorLink* otherLink : otherNav.links) {
				if (!otherLink->isCliff || otherLink->isJump)
					continue;

				if (create_jump_link(mesh, nav, *nav.links[k], otherNav, *otherLink)) {
					break;
				}
			}
		}
	}
}

void SectorNavMeshGenerator::add_jump_links(BotMeshData& mesh) {
	// add jump links between cliff segments
	for (int i = 0; i < g_map.numsubsectors; i++) {
		add_jump_links(mesh, i);
	}
}

void SectorNavMeshGenerator::calc_nav_centers(BotMeshData& mesh) {
	for (int i = 0; i < g_map.numsubsectors; i++) {
		MapSubsector& sub = g_map.subsectors[i];
		NavSector& nav = mesh.nodes[i];

		float area = 0;
		vec2 centroid(0, 0);

		for (int i = 0; i < sub.numsegs; i++)
		{
			MapSeg& seg = g_map.segs[sub.firstseg + i];

			float cross = crossProduct(seg.v1, seg.v2);
			area += cross;
			centroid += (seg.v1 + seg.v2) * cross;
		}

		nav.center = centroid / (6.0f * (area * 0.5f));
	}
}

void SectorNavMeshGenerator::add_walkable_links(BotMeshData& mesh, int nodeid, std::vector<AActor*> propBlockers) {
	MapSubsector& sub = g_map.subsectors[nodeid];
	NavSector& nav = mesh.nodes[nodeid];

	std::unordered_set<MapSector*> neighborSectors = { sub.sector };

	for (int i = 0; i < sub.numsegs; i++) {
		MapSeg& seg = g_map.segs[sub.firstseg + i];

		for (MapLine* line : seg.lines) {
			if (line->frontsector)
				neighborSectors.insert(line->frontsector);
			if (line->backsector)
				neighborSectors.insert(line->backsector);
		}
	}

	for (int k = 0; k < sub.numsegs; k++) {
		MapSeg& seg = g_map.segs[sub.firstseg + k];

		for (LinkSeg& linkseg : g_map.get_neighbor_subsectors(&sub, &seg, neighborSectors)) {
			MapSubsector& neighbor = g_map.subsectors[linkseg.otherSub];

			if (!g_map.is_sector_border_potentially_crossable(sub.sector, neighbor.sector)) {
				continue;
			}

			NavSectorLink link;
			link.target = &mesh.nodes[linkseg.otherSub];
			link.movePos = vec3(linkseg.overlap.center(), sub.sector->getFloorZ());
			link.seg = linkseg.overlap;
			link.linedef = linkseg.line;
			link.id = mesh.numLinks;
			link.sector = link.target->sector;
			link.parent = &nav;
			link.linkWidth = (int)linkseg.overlap.length();
			link.isJump = false;
			link.isTeleport = false;
			link.jumpDist = 0;

			const float mesh_radius = PLAYER_RADIUS;
			if (IsBoxWallClipped(link.movePos, mesh_radius)) {
				int step = 8;
				int steps = (link.linkWidth / step) / 2; // half len for left/right split
				vec2 delta = linkseg.overlap.b - linkseg.overlap.a;
				vec2 dir = delta.normalize();

				bool foundUnclippedPos = false;
				for (int s = 1; s <= steps; s++) {
					vec2 testLeft = link.movePos + dir * s * step;
					vec2 testRight = link.movePos + dir * s * -step;

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

			link.isCliff = sub.sector->getFloorZ()
				- neighbor.sector->getFloorZ() > JUMP_HEIGHT;

			if (link.isCliff)
				nav.hasCliffs = true;

			// redirect target sector if this is the front side of a teleport
			MapLine* line = link.linedef;
			bool isPlayerTele = line && line->isTeleport() && line->canPlayerActivate();
			if (isPlayerTele && DistanceToLine(nav.pos(), line) < 0) {
				vec2 dest = line->getTeleportDest();
				MapSubsector* destSub = g_map.GetSubsector(dest.x, dest.y);
				if (destSub) {
					link.target = &mesh.nodes[destSub - g_map.subsectors];
					link.isTeleport = true;
				}
			}

			// don't add links that are blocked by immovable props
			bool propBlocked = false;
			const float playerRadius = (PLAYER_WIDTH / 2);
			vec2 linkPos = link.movePos;
			for (int i = 0; i < propBlockers.size(); i++) {
				AActor* actor = propBlockers[i];
				ActorState astate = g_engine.get_actor_state(actor);
				vec2 propPos = astate.origin;
				if ((propPos - linkPos).length() < astate.radius + playerRadius) {
					propBlocked = true;
					break;
				}
			}
			if (propBlocked) {
				continue;
			}

			mesh.links[mesh.numLinks] = link;
			nav.links.push_back(&mesh.links[mesh.numLinks]);
			mesh.numLinks++;
		}
	}
}

BotMeshData SectorNavMeshGenerator::generate(std::vector<AActor*> propBlockers) {
	uint64_t genStart = getEpochMillis();

	BotMeshData mesh;
	mesh.nodes = new NavSector[g_map.numsubsectors];
	mesh.links = new NavSectorLink[MAX_MESH_LINKS];
	mesh.numLinks = 0;

	calc_nav_centers(mesh);

	// link sectors as a nav mesh
	for (int i = 0; i < g_map.numsubsectors; i++) {
		MapSubsector& sub = g_map.subsectors[i];
		NavSector& nav = mesh.nodes[i];

		nav.id = i;
		nav.doesDamage = g_map.subsector_does_damage(&sub);
		nav.sector = sub.sector;
	}

	for (int i = 0; i < g_map.numsubsectors; i++) {
		NavSector& nav = mesh.nodes[i];
		add_walkable_links(mesh, i, propBlockers);
	}

	add_jump_links(mesh);

	g_engine.printf("Generated %d nodes, %d links in %d ms\n", (int)g_map.numsubsectors, mesh.numLinks, (int)(getEpochMillis() - genStart));

	return mesh;
}

int SectorNavMeshGenerator::relink_node(BotMeshData& mesh, int id, std::vector<AActor*> propBlockers) {
	NavSector& nav = mesh.nodes[id];
	int oldLinkCount = nav.links.size();
	nav.links.clear();

	nav.hasCliffs = false;
	add_walkable_links(mesh, id, propBlockers);
	add_jump_links(mesh, id);

	NavSector* toNav = &mesh.nodes[id];

	// test that links to this sector are still valid if it's no longer possible to move
	int removed = 0;
	for (int i = 0; i < g_map.numsubsectors; i++) {
		NavSector& otherNav = mesh.nodes[i];

		// don't unlink yet if the source sectors can be elevated in the future
		bool jumpsMayBePossibleLater = otherNav.sector->moveFlags & FL_SECTOR_MOVE_FLOOR_UP;

		for (int k = 0; k < otherNav.links.size(); k++) {
			NavSectorLink& link = *otherNav.links[k];

			if (link.target != toNav)
				continue;

			bool stillValid;
			if (link.isJump)
				stillValid = jumpsMayBePossibleLater || (nav.hasCliffs && link.isJumpValid());
			else
				stillValid = g_map.is_sector_border_potentially_crossable(link.parent->sector, link.target->sector);

			if (!stillValid) {
				otherNav.links.erase(otherNav.links.begin() + k);
				k--;
				removed++;
			}
			else {
				// still valid. Update cliff flag
				link.updateFlags();
			}
		}
	}

	return (nav.links.size() - oldLinkCount) - removed;
}