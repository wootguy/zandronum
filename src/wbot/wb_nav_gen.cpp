#include "wb_nav_gen.h"
#include "wb_map.h"
#include "wb_util.h"
#include "p_trace.h"
#include "r_state.h"
#include "r_utility.h"
#include "p_lnspec.h"
#include "p_spec.h"

using namespace std;

int g_total_links;

bool SectorNavMeshGenerator::create_jump_link(NavSector& fromNav, NavSectorLink& fromLink, NavSector& toNav, NavSectorLink& toLink) {
	fixed_t fromHeight = fromNav.getFloorZ();
	fixed_t toHeight = toNav.getFloorZ();
	fixed_t dropHeight = fromHeight - toHeight;

	if (dropHeight < -(JUMP_HEIGHT << FRACBITS)) {
		// too high to jump to
		if (!(fromNav.getMoveFlags() & FL_SECTOR_MOVE_FLOOR_UP) && !(toNav.getMoveFlags() & FL_SECTOR_MOVE_FLOOR_DOWN))
			return false; // and neither sector moves in a way that would make the jump possible later
	}

	// increase link distance for drops, decrease for jumps
	int maxDist = 200 << FRACBITS;
	maxDist += dropHeight;

	fixed_t linkDist = (fromLink.pos() - toLink.pos()).Length();
	if (linkDist > maxDist) {
		return false; // too far to link to
	}

	FVector2 e1v1(fromLink.seg->v1->x, fromLink.seg->v1->y);
	FVector2 e1v2(fromLink.seg->v2->x, fromLink.seg->v2->y);

	FVector2 e2v1(toLink.seg->v1->x, toLink.seg->v1->y);
	FVector2 e2v2(toLink.seg->v2->x, toLink.seg->v2->y);

	FVector2 edge1 = (e1v2 - e1v1);
	FVector2 edge2 = (e2v2 - e2v1);
	edge1.MakeUnit();
	edge2.MakeUnit();

	FVector2 normalA(edge1.Y, -edge1.X);
	FVector2 normalB(edge2.Y, -edge2.X);

	float dot = DotProduct(normalA, normalB);
	if (dot > -0.5f) {
		return false; // edges not facing each other enough
	}

	FVector2 delta = fromLink.pos() - toLink.pos();
	float planeDist = DotProduct(normalA, delta);
	if (planeDist < 0) {
		return false; // not in front of the segment
	}

	if (fromNav.getLink(toNav.id)) {
		return false; // already have a link to this sector
	}

	// check that the path is clear
	{
		FTraceResults tr;

		FVector3 start = fromLink.pos3D() + FVector3(0, 0, 56 << FRACBITS);
		FVector3 end = toNav.pos3D() + FVector3(0, 0, 56 << FRACBITS);

		FVector2 delta = end - start;
		delta.MakeUnit();
		FVector3 rightDir(delta.Y, -delta.X, 0);
		int radius = PLAYER_WIDTH / 2;
		fixed_t rightStep = (radius / 2) << FRACBITS;
		bool fromNavMoves = fromNav.getMoveFlags() != 0;

		for (int i = -2; i <= 2; i++) {

			if (TraceLine(start + rightDir * i * rightStep, end + rightDir * i * rightStep, true, NULL, &tr)) {
				if (tr.HitType == TRACE_HitWall && tr.Line && tr.Line->backsector) {
					if (g_wb_mapinfo.sector_info[tr.Line->backsector - sectors].moveFlags) {
						continue; // wall may move out of the way in the future
					}
				}
				if (tr.HitType == TRACE_HitCeiling && fromNavMoves) {
					// sector may be fully lifted to the ceiling and so can't connect yet
					continue; // may move out of the way in the future
				}

				return false; // hit an immovable wall
			}
		}
	}

	NavSectorLink link;
	link.parent = fromLink.parent;
	link.overlapCenter = fromLink.overlapCenter;
	link.linkWidth = fromLink.linkWidth;
	link.leftSector = fromLink.leftSector;
	link.rightSector = fromLink.rightSector;
	link.isTeleport = fromLink.isTeleport;
	link.isCliff = fromLink.isCliff;
	link.seg = fromLink.seg;

	link.target = &toNav;
	link.id = g_total_links++;
	link.isJump = true;

	//if (link.id == 2673 && toNav.id == 673) {
		//Printf("");
	//}

	fromNav.links.push_back(link);

	return true;
}

void SectorNavMeshGenerator::add_jump_links(NavSector* mesh) {
	// add jump links between cliff segments
	for (int i = 0; i < numsubsectors; i++) {
		NavSector& nav = mesh[i];
		bool allLinksCanBeCliffsLater = nav.getMoveFlags() & FL_SECTOR_MOVE_FLOOR_UP;

		for (int k = 0; k < nav.links.size(); k++) {
			NavSectorLink& link = nav.links[k];

			if (link.isJump)
				continue;

			if (!link.isCliff && !allLinksCanBeCliffsLater)
				continue;

			// find other other cliff segments to try linking to
			for (int j = 0; j < numsubsectors; j++) {
				NavSector& otherNav = mesh[j];

				if (j == i)
					continue;

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
}

void SectorNavMeshGenerator::calc_nav_centers(NavSector* mesh) {
	for (int i = 0; i < numsubsectors; i++) {
		subsector_t& sub = subsectors[i];
		NavSector& nav = mesh[i];

		int subCenterX = 0;
		int subCenterY = 0;

		for (int k = 0; k < sub.numlines; k++) {
			seg_t& seg = sub.firstline[k];
			subCenterX += seg.v1->x >> FRACBITS;
			subCenterY += seg.v1->y >> FRACBITS;
		}

		nav.center = FVector2(
			(subCenterX / (int)sub.numlines) << FRACBITS,
			(subCenterY / (int)sub.numlines) << FRACBITS
		);
	}
}

NavSector* SectorNavMeshGenerator::generate() {
	uint64_t genStart = getEpochMillis();

	NavSector* mesh = new NavSector[numsubsectors];

	g_wb_mapinfo.init();

	g_total_links = 0;

	calc_nav_centers(mesh);

	vector<AActor*> propBlockers;

	// find all immovable and invulnerable props
	TThinkerIterator<AActor> it;
	AActor* actor;
	while ((actor = it.Next())) {
		if (IsPropBlocker(actor))
			propBlockers.push_back(actor);
		//Printf("Prop '%s' %d\n", actor->GetClass()->TypeName.GetChars(), actor->health);
	}

	// link sectors as a nav mesh
	for (int i = 0; i < numsubsectors; i++) {
		subsector_t& sub = subsectors[i];
		NavSector& nav = mesh[i];

		nav.id = i;
		nav.doesDamage = g_wb_mapinfo.subsector_does_damage(&sub);

		for (int k = 0; k < sub.numlines; k++) {
			seg_t& seg = sub.firstline[k];

			if (!g_wb_mapinfo.is_seg_potentially_crossable(&seg)) {
				//if (i == 167) {
				//	is_seg_potentially_crossable(&seg); // debug possible link not created
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
			link.overlapCenter = FVector3(cx, cy, z);
			link.seg = &seg;
			link.id = g_total_links++;
			link.parent = &nav;
			link.linkWidth = (int)linkseg.length() >> FRACBITS;
			link.isJump = false;
			link.isTeleport = false;
			link.leftSector = -1;
			link.rightSector = -1;

			subsector_t& neighbor = subsectors[linkseg.otherSub];

			if (link.linkWidth <= PLAYER_WIDTH) {
				int dummy;
				if (g_wb_mapinfo.is_link_bordered_by_walls(sub, k, dummy, dummy))
					continue; // link is too narrow to enter and both sides of it are impassable walls
				if (g_wb_mapinfo.is_link_bordered_by_walls(neighbor, linkseg.idx, link.leftSector, link.rightSector))
					continue; // link is too narrow to leave and both sides of it are impassable walls
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
			FVector2 linkPos = link.overlapCenter;
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

	add_jump_links(mesh);

	Printf("Generated %d nodes, %d links in %d ms\n", (int)numsubsectors, g_total_links, (int)(getEpochMillis() - genStart));

	return mesh;
}
