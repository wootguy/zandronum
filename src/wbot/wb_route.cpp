#include "wb_route.h"
#include "wb_bot.h"
#include "wb_nav.h"
#include "wb_map.h"
#include "wb_util.h"
#include "p_local.h"
#include "d_event.h"
#include <algorithm>

using namespace std;
using namespace wbot;

CBotRouteController::CBotRouteController(CWootBot* pBot)
	: pBot(pBot), pActor(pBot->pActor), pPlayer(pBot->GetPlayer()) {}

void CBotRouteController::Think() {
	m_routeSpeed = RUN_SPEED;
	m_nodeRadius = 32;
	pBot->m_forwardMove = 0;
	pBot->m_sideMove = 0;
	pBot->stateFlags &= ~(FL_WBOT_WAIT_ELEV | FL_WBOT_WAIT_DOOR | FL_WBOT_SLOW_DOWN);
	std::vector<int>& route = m_route.route;

	if (route.empty())
		return;

	if (jumpState) {
		// may go thru different sectors while preparing a jump, so don't get confused
		JumpThink();
		if (pBot->StuckThink(500)) {
			JumpFail();
		}
		return;
	}

	UpdateRoute();

	if (route.size() > 1 && !m_navLink) {
		CancelRoute();
		DebugPrint("My route contains invalid links! Sectors must have changed...\n");
		return; // route broke due to invalid links - can happen when sectors are relinked
	}

	// update route speed and node touch distance in dangerous areas
	if (BeCareful()) {
		return; // going too fast
	}

	if (route.size() > 1) {
		// allow slipping off the route into adjacent sectors while heading towards the target sector
		bool onTrack = m_navid == route[0]
			|| m_navCur->getLink(route[1]) || m_navCur->getLink(route[0])
			|| m_navIdeal->touches(pActor) || m_navTarget->touches(pActor);

		if (!onTrack) {
			RouteSlipThink(); // fell off the route
			return;
		}

		if (HandleBlockedPaths())
			return; // wait until the path is unblocked

		MoveThruLink(); // keep on movin
	}
	else if (route.size() == 1) {
		// reached the final node. Move to the center.
		FVector2 centerGoal = m_navIdeal->pos();
		if (pBot->MoveTo(centerGoal, m_nodeRadius, m_routeSpeed)) {
			route.clear(); // don't reset pretendsector in case a goal is inside it
		}
	}

	if (pBot->StuckThink(500)) {
		HandleStuckPath();
	}
}

void CBotRouteController::UpdateRoute() {
	std::vector<int>& route = m_route.route;

	if (route.size() > 1) {
		if (m_navid == route[1]) {
			// inside the target sector. Advance the route.
			route.erase(route.begin());
			pretendRouteSector = -1;
			walkNodeState = WBOT_WALK_NODE_EDGE;

			// complete unblock attempts for the previous path
			BotGoal* curGoal = pBot->CurrentGoal();
			if (m_navLink && curGoal) {
				auto& attempts = curGoal->unblockAttempts;
				for (auto it = attempts.begin(); it != attempts.end(); ) {
					if (it->second == m_navLink->id)
						it = attempts.erase(it);
					else
						++it;
				}
			}
		}
		else {
			FVector2 center = g_wb_nav.mesh.nodes[route[1]].pos();
			fixed_t dist = P_AproxDistance(pActor->x - (fixed_t)center.X, pActor->y - (fixed_t)center.Y);
			if (pBot->stuckCounter >= 200 && dist < (16 << FRACBITS)) {
				// already very close to the center, so this is probably a tiny polygon jammed
				// up against a wall. The bot can't get close enough in this case, so advance
				// the route now and pretend the bot is inside the target sector.
				pretendRouteSector = route[1];
				DebugPrint(VarArgs("Pretending I'm in sector %d. I'm stuck and close enough\n", pretendRouteSector));
				route.erase(route.begin());
				walkNodeState = WBOT_WALK_NODE_EDGE;
			}
		}
	}

	if (pretendRouteSector >= 0) {
		m_navid = pretendRouteSector;
	}

	// complete subgoals when ending up in a sector that another subgoal was trying to unblock
	int completedGoals = 0;
	for (int i = 0; i < pBot->m_goals.size(); i++) {
		BotGoal& goal = pBot->m_goals[i];

		if (goal.purposeLink && goal.purposeLink->target->id == m_navid) {
			completedGoals = (pBot->m_goals.size() - i);
		}
	}
	if (completedGoals) {
		DebugPrint(VarArgs("Accidentally completed %d subgoals by being int sector %d!\n", completedGoals, m_navid));
		for (int i = 0; i < completedGoals; i++) {
			pBot->m_goals.pop_back();
		}
		RouteToGoal();
	}

	if (m_navid != route[0]) {
		// update route if slipped off into a sector adjacent to a target
		NavSector& curNav = g_wb_nav.mesh.nodes[m_navid];

		if (route.size() > 1 && curNav.getLink(route[1])) {
			NavSectorLink* link = curNav.getLink(route[1]);
			if (link && link->walkable()) {
				route[0] = m_navid;
			}
		}

		walkNodeState = WBOT_WALK_NODE_EDGE;
	}

	if (pBot->rushNav == m_navid) {
		pBot->rushNav = -1;
		pBot->stateFlags &= ~FL_WBOT_RUSHING;
	}

	m_navCur = &g_wb_nav.mesh.nodes[m_navid];
	m_navIdeal = &g_wb_nav.mesh.nodes[route[0]];
	m_navTarget = &g_wb_nav.mesh.nodes[route.size() > 1 ? route[1] : route[0]];
	m_navLink = route.size() > 1 ? m_navIdeal->getLink(route[1]) : NULL;
}

bool CBotRouteController::BeCareful() {
	std::vector<int>& route = m_route.route;
	bool headingTowardsCliff = false;
	bool shouldBeCareful = false;

	// be careful near cliffs
	if (route.size() > 1) {
		NavSector& idealNav = g_wb_nav.mesh.nodes[route[0]];
		NavSector& targetNav = g_wb_nav.mesh.nodes[route[1]];
		NavSectorLink* link = idealNav.getLink(route[1]);
		headingTowardsCliff = targetNav.hasCliffs && link->linkWidth < 32
			&& targetNav.getFloorZ() < (pActor->z + (STEP_HEIGHT << FRACBITS));
	}

	if (pBot->m_cliffDist < SAFE_CLIFF_DIST * 0.5f || headingTowardsCliff) {
		m_routeSpeed *= 0.5f;
		m_nodeRadius = 8;
		shouldBeCareful = true;
	}
	else if (pBot->m_cliffDist < SAFE_CLIFF_DIST * 0.75f) {
		m_routeSpeed *= 0.75f;
		m_nodeRadius = 16;
		shouldBeCareful = true;
	}

	if (!(pBot->stateFlags & FL_WBOT_RUSHING)) {
		// slowly drop down cliffs to avoid overshooting the target sector
		if (m_navLink && m_navLink->isCliff && !m_navLink->isJump) {
			m_routeSpeed *= 0.5f;
			m_nodeRadius = 8;
			shouldBeCareful = true;
		}
	}

	if (shouldBeCareful && pBot->GetSpeed2D() > m_routeSpeed) {
		pBot->stateFlags |= FL_WBOT_SLOW_DOWN;
		pBot->StopMoving();
		return true;
	}

	return false;
}

void CBotRouteController::JumpThink() {
	FVector3 pos(pActor->z, pActor->y, pActor->z);

	switch (jumpState) {
	case WBOT_JUMP_PREP: {
		int dist = pBot->GetDistance(jumpBackupPos) >> FRACBITS;
		int startSpeed = RUN_SPEED * 0.2f;
		m_routeSpeed = RUN_SPEED;
		if (dist < 64) {
			m_routeSpeed = RUN_SPEED * 0.2f;
		}
		else if (dist < 256) {
			m_routeSpeed = RUN_SPEED * 0.5f;
		}
		int curSpeed = pBot->GetSpeed2D();

		// move to the running-start position
		if (pBot->MoveTo(jumpBackupPos, 16, m_routeSpeed) && curSpeed < startSpeed) {
			jumpState = WBOT_JUMP_RUN;
		}
		break;
	}
	case WBOT_JUMP_RUN: {
		if (pBot->MoveTo(jumpStartPos, 16, m_routeSpeed)) {
			jumpState = WBOT_JUMP_LAUNCH;
		}
		break;
	}
	case WBOT_JUMP_LAUNCH: {
		pBot->MoveTo(jumpEndPos, 0, m_routeSpeed);

		if (pBot->stateFlags & (FL_WBOT_OVERHANG | FL_WBOT_FLYING)) {
			// bot is over a ledge now, start the jump
			bool bigJump = m_navLink->jumpDist > 100 << FRACBITS;
			if (m_navTarget->getFloorZ() > m_navIdeal->getFloorZ() + (STEP_HEIGHT << FRACBITS))
				bigJump = true;

			if (bigJump)
				pBot->m_lButtons |= BT_JUMP;

			jumpState = WBOT_JUMP_FLY;
		}
		break;
	}
	case WBOT_JUMP_FLY: {
		NavSector& curNav = g_wb_nav.mesh.nodes[g_wb_nav.get_nav_id(pActor)];
		NavSectorLink* linkToTarg = curNav.getLink(m_navTarget->id);
		bool flyingOverWalkableNeighbor = linkToTarg && !linkToTarg->isJump && !linkToTarg->blocked(pActor);

		if (curNav.id == m_navTarget->id || flyingOverWalkableNeighbor) {
			pBot->StopMoving(); // flying over the target sector, try not to fall off now

			if (pPlayer->onground) {
				jumpState = WBOT_JUMP_NONE;
				pBot->MoveTo(jumpEndPos, 0, m_routeSpeed);
				return; // completed the jump
			}
		} 
		else if (pPlayer->onground) {
			// not a gap that required a jump, just keep moving towards the target
			pBot->MoveTo(jumpEndPos, 0, m_routeSpeed);
		}
		else {
			// try to land in the right spot
			FVector3 landPos = FVector3(jumpEndPos, m_navTarget->getFloorZ());
			FVector3 idealDir = (landPos - pos).Unit();
			FVector3 velDir = FVector3(pActor->velx, pActor->vely, pActor->velz).Unit();
			fixed_t distLeft = pBot->GetDistance(landPos);

			if (distLeft > (192 << FRACBITS) || idealDir.Z > velDir.Z) {
				// falling short, keep building speed
				pBot->MoveTo(jumpEndPos, 0, m_routeSpeed);
				pBot->m_lButtons |= BT_CROUCH; // just in case
			}
			else {
				// overshooting the target, try to slow down
				pBot->StopMoving();
			}
		}

		if (m_navTarget->getFloorZ() > pActor->z + (JUMP_HEIGHT << FRACBITS)) {
			JumpFail();
			return;
		}
		break;
	}
	default:
		DebugPrint("Invalid jump state\n");
		jumpState = 0;
		break;
	}
}

void CBotRouteController::JumpFail() {
	jumpState = WBOT_JUMP_NONE; // missed the jump

	NavSector* backupBlocker = m_navLink->GetJumpBackupBlocker(m_navTarget->pos());

	if (backupBlocker) {
		// The running-start position was blocked by something that can be moved.
		// Try moving it before considering this jump to be impossible
		if (pBot->SelectGoal(backupBlocker->getTriggers(), m_navLink)) {
			return;
		}
		else {
			// already tried this goal
			backupBlocker = NULL;
		}
	}

	if (!backupBlocker) {
		BotGoal* curgoal = pBot->CurrentGoal();
		if (curgoal) {
			// don't try the jump again, there are probably other ones to try
			// and many jumps just don't work
			curgoal->blockers.insert(m_navLink->id);
		}

		CancelRoute();
	}
}

bool CBotRouteController::HandleBlockedPaths() {
	std::vector<int>& route = m_route.route;
	int nextSecFlags = m_navLink->target->getMoveFlags();
	bool nextOnElevator = nextSecFlags & (FL_SECTOR_MOVE_FLOOR_UP | FL_SECTOR_MOVE_FLOOR_DOWN);
	int linkBlockReason = m_navLink->blocked(pActor);

	if (ElevatorThink(linkBlockReason != LINK_BLOCK_CLEAR)) {
		return true;
	}

	// handle severe blockages
	if (linkBlockReason != LINK_BLOCK_CLEAR) {
		BlockedPathThink(m_navLink, linkBlockReason);
		return true;
	}
	else if (!nextOnElevator && route.size() > 2 && m_navTarget->touches(pActor)) {
		// if we're touching the next sector and the next path is blocked, also do block
		// handling. Helps in case of doors with tiny sectors in front of them which the
		// bot can't fully get inside.
		NavSectorLink* nextLink = m_navLink->target->getLink(route[2]);

		if (nextLink && !nextLink->isJump) {
			sector_t* nextNextSector = subsectors[route[2]].sector;
			int blockReason = nextLink->blocked(pActor);

			if (blockReason != LINK_BLOCK_CLEAR) {
				BlockedPathThink(nextLink, blockReason);
				return true;
			}
		}
	}

	return false; // not blocked
}

bool CBotRouteController::ElevatorThink(bool linkBlocked) {
	// wait on elevators
	if (pBot->stateFlags & FL_WBOT_ON_ELEV) {
		MapSector* thisSector = m_navCur->sector;
		bool waitedLongEnough = m_navLink->walkable();

		if (m_navLink->isJump) {
			// If the next link is a jump, wait until the very top for better success chance
			fixed_t elevZ = thisSector->getFloorZ();
			fixed_t moveDelta = elevZ - m_lastElevZ;

			// elevator stopped at the top
			waitedLongEnough = !linkBlocked && moveDelta == 0;

			if (linkBlocked && moveDelta < 0 && m_elevRaiseTics > 1) {
				// elevator is lowering after reaching the top or getting blocked,
				// and the jump link still isn't doable.
				BlockedPathThink(m_navLink, LINK_BLOCK_CANT_JUMP);
				return false;
			}

			if (moveDelta > 0) {
				// must be 2 or greater to exclude the first tic the elev is tracked, prevent false positives
				m_elevRaiseTics++;
			}

			m_lastElevZ = elevZ;
		}

		if (!waitedLongEnough) {
			pBot->stateFlags |= FL_WBOT_WAIT_ELEV;

			// stay centered on the elevator to avoid blocking it or falling off
			FVector2 navPos = m_navCur->pos();
			fixed_t dist = P_AproxDistance(pActor->x - (fixed_t)navPos.X, pActor->y - (fixed_t)navPos.Y);
			if (dist > (16 << FRACBITS))
				pBot->MoveTo(navPos, 0, RUN_SPEED / 4);

			if (linkBlocked) {
				fixed_t backFloor = m_navTarget->getFloorZ();
				fixed_t frontCeil = m_navIdeal->getCeilZ();

				if (frontCeil - backFloor < (DUCK_HEIGHT << FRACBITS)) {
					// too low of a ceil in the start sector to duck thru to the target floor
					// see if the ceiling can be raised, to avoid waiting on an elevator
					// forever (doom2 map15, map27)
					BlockedPathThink(m_navLink, LINK_BLOCK_TOO_LOW);
				}
			}

			return true; // wait until the elevator is done moving
		}
	}
	else {
		m_lastElevZ = 0;
		m_elevRaiseTics = 0;
	}

	return false;
}

void CBotRouteController::MoveThruLink() {
	// duck if unable to fit while standing
	int targetHeight = m_navTarget->getHeight() >> FRACBITS;
	int borderHeight = (m_navTarget->getCeilZ() - m_navIdeal->getFloorZ()) >> FRACBITS;
	if (std::min(targetHeight, borderHeight) < STAND_HEIGHT) {
		pBot->m_lButtons |= BT_CROUCH;
	}

	if (m_navLink->isJump) {
		// get a running start for the jump
		if (m_navLink->jumpDist > (PLAYER_WIDTH << FRACBITS)) {
			jumpState = WBOT_JUMP_PREP;
		}
		else {
			jumpState = WBOT_JUMP_RUN; // skip the prep and head straight for the jump point
		}
		
		FVector2 targetPos = m_navLink->target->pos();
		jumpBackupPos = m_navLink->GetJumpBackupPos(targetPos, pActor);
		jumpStartPos = m_navLink->GetJumpStartPos(targetPos);
		jumpEndPos = m_navLink->GetJumpEndPos(targetPos);
	}
	else {
		switch (walkNodeState) {
		default:
		case WBOT_WALK_NODE_EDGE:
			if (pBot->MoveTo(m_navLink->pos(), m_nodeRadius, m_routeSpeed)) {
				walkNodeState = WBOT_WALK_NODE_CENTER;
			}
			break;
		case WBOT_WALK_NODE_CENTER:
			// close enough to the link edge				
			if (m_navLink->isTeleport && m_navLink->linedef) {
				// move behind the teleporter line edge.
				// The target sector may be in a completely different direction.
				FVector2 backDir = m_navLink->linedef->normal() * -1;
				FVector2 teleGoal = m_navLink->pos() + backDir * 200;
				pBot->MoveTo(teleGoal, 0, m_routeSpeed);
			}
			else {
				if (m_navLink->isCliff) {
					// may need to move past the center in case its a tiny polygon under a cliff
					FVector2 dropDir = (m_navTarget->pos() - m_navLink->pos()).Unit();
					pBot->MoveTo(m_navLink->pos() + dropDir * (100 << FRACBITS), 0, m_routeSpeed);
				}
				else {
					// move towards the target sector until we end up inside it
					pBot->MoveTo(m_navTarget->pos(), 0, m_routeSpeed);
				}
			}
			break;
		}
	}
}

void CBotRouteController::RouteSlipThink() {
	std::vector<int>& route = m_route.route;

	if (m_navid != route[0]) {
		// update route if slipped off into a sector adjacent to the previous
		// not done earlier for a reason i forgot on doom2 map24 tightrope area.
		NavSector& curNav = g_wb_nav.mesh.nodes[m_navid];
		NavSectorLink* link = curNav.getLink(route[0]);
		if (link && link->walkable()) {
			route.insert(route.begin(), m_navid);
			walkNodeState = WBOT_WALK_NODE_EDGE;
			return;
		}
	}

	DebugPrint(VarArgs("Fell off the route (expected %d but got %d)\n", route[0], m_navid));
	CancelRoute();
}

void CBotRouteController::BlockedPathThink(NavSectorLink* link, int blockReason) {
	// wait for doors to open
	bool isBlockerMoving = link->target->isMoving();
	if (!isBlockerMoving && blockReason == LINK_BLOCK_CLIPPED) {
		fixed_t linkZ = link->parent->getFloorZ() + (JUMP_HEIGHT << FRACBITS);
		for (MapSector* sec : link->getClippedSectors(pActor)) {
			fixed_t blockerZ = sec->getFloorZ();
			if (sec->isFloorMoving() && linkZ < blockerZ) {
				isBlockerMoving = true;
				break;
			}
			if (sec->isCeilMoving() && linkZ > blockerZ) {
				isBlockerMoving = true;
				break;
			}
		}
	}

	if (isBlockerMoving) {
		// door is raising or elevator is lowering in the next sector
		if (link->isJump && !link->target->isFloorMoving() && !link->isJumpValid()) {
			// a door opening isn't going to make the jump doable if the floor is too high
		}
		else {
			pBot->stateFlags |= FL_WBOT_WAIT_DOOR;
			pBot->StopMoving();
			return; // wait until the door/elevator is done moving
		}
	}

	DebugPrint(VarArgs("Link %d blocked!\n", link->id));
	link->blocked(pActor); // debug here

	BotGoal* curGoal = pBot->CurrentGoal();
	if (!curGoal)
		return;

	curGoal->blockers.insert(link->id);

	int targetMovement = link->target->getMoveFlags();
	int parentMovement = link->parent->getMoveFlags();
	bool tryTargetTrigger = targetMovement != 0;
	bool tryParentTrigger = parentMovement != 0;

	// don't trigger things if it won't help unblock the path
	switch (blockReason) {
	case LINK_BLOCK_TOO_HIGH:
		if (!(targetMovement & FL_SECTOR_MOVE_FLOOR_DOWN)) {
			tryTargetTrigger = false;
		}
		if (!(parentMovement & FL_SECTOR_MOVE_FLOOR_UP)) {
			tryParentTrigger = false;
		}
		break;
	case LINK_BLOCK_TOO_LOW:
		if (!(parentMovement & FL_SECTOR_MOVE_FLOOR_DOWN)) {
			tryParentTrigger = false;
		}
		break;
	case LINK_BLOCK_CLIPPED:
		tryParentTrigger = false;
		tryTargetTrigger = false;

		// try unblocking anything the box is clipping into
		for (MapSector* sec : link->getClippedSectors(pActor)) {
			if (sec->moveFlags && pBot->SelectGoal(sec->triggers, link)) {
				return;
			}
		}
		break;
	case LINK_BLOCK_CANT_JUMP:
		if (!(targetMovement & (FL_SECTOR_MOVE_FLOOR_DOWN | FL_SECTOR_MOVE_CEIL_UP))) {
			tryTargetTrigger = false;
		}
		if (!(parentMovement & FL_SECTOR_MOVE_FLOOR_UP)) {
			tryParentTrigger = false;
		}
		break;
	default:
		tryParentTrigger = false;
		tryTargetTrigger = false;
		break;
	}

	// nothing is moving, try unblocking it ourselves.
	if (tryTargetTrigger && pBot->SelectGoal(link->target->getTriggers(), link)) {
		return;
	}

	// if we're on an elevator, try triggering it.
	if (tryParentTrigger && pBot->SelectGoal(link->parent->getTriggers(), link)) {
		return;
	}

	// nothing can unblock the path that stopped us. Try routing around it.
	curGoal = pBot->CurrentGoal();
	if (curGoal) {
		m_route = RouteToSector(curGoal->getNavId());
		if (m_route.route.size()) {
			DebugPrint("Routing around the blocked path.\n");
			return;
		}
	}

	// try a different route/unblocker for the parent goal with new blockers
	pBot->FailGoal();
}

void CBotRouteController::HandleStuckPath() {
	DebugPrint("I got stuck! Cancelling route.\n");

	BotGoal* goal = pBot->CurrentGoal();

	if (m_navLink && goal) {
		// don't try to take this path again
		goal->blockers.insert(m_navLink->id);
	}

	CancelRoute();
}

void CBotRouteController::CancelRoute() {
	if (m_route.route.size() && m_freezeOnRouteChange) {
		pPlayer->cheats |= CF_FROZEN;
		pActor->velx = 0;
		pActor->vely = 0;
		pActor->velz = 0;
	}

	m_route = BotRoute();
	pretendRouteSector = -1;
	pBot->stuckCounter = 0;
	jumpState = 0;
	walkNodeState = 0;
}

void CBotRouteController::MarkBlockedPaths() {
	NavSectorLink* links = g_wb_nav.mesh.links;

	for (BotGoal& goal : pBot->m_goals) {
		for (int id : goal.blockers) {
			links[id].routeNumIgnore = g_route_ignore_num;
		}
	}
}

BotRoute CBotRouteController::RouteToSector(int subid, int blockSector) {
	RouteOpts opts;
	opts.start = m_navid;
	opts.end = subid;
	opts.timeSensitive = pBot->stateFlags & FL_WBOT_RUSHING;
	opts.blockedPathHandling = WBOT_ROUTE_BLOCK_IGNORE;

	MarkBlockedPaths();
	if (blockSector != -1)
		g_wb_nav.mesh.nodes[blockSector].routeNumIgnore = g_route_ignore_num;

	return g_wb_nav.get_astar_route(opts);
}

bool CBotRouteController::RouteToGoal() {
	CancelRoute();

	if (!pBot->HasGoal()) {
		DebugPrint("No goal to route to\n");
		return false;
	}

	BotGoal* goal = pBot->CurrentGoal();
	int goalNavId = goal->getNavId();
	m_route = RouteToSector(goalNavId);

	if (m_route.route.empty() && goal->actor && goalNavId != -1) {
		// actor origin is in an unreachable sector, but it's collision box may be touching a reachable one
		vector<int> subs = g_map.GetTouchedSubsectors(goal->actor);

		for (const int& subid : subs) {
			if (subid == goalNavId) {
				continue;
			}

			m_route = RouteToSector(subid);
			if (m_route.route.size())
				break;
		}

		if (goal->action == WBOT_GOAL_ACTION_BOSS_BRAIN) {
			unordered_map<int, IndirectShootPos> shootFroms = goal->FindBossBrainShootPositions();
			for (auto item : shootFroms) {
				int subid = item.first;
				m_route = RouteToSector(subid);
				if (m_route.route.size()) {
					 // can route to this position
					goal->shootAlignment = item.second;
				}
			}
		}
	}

	std::vector<int>& route = m_route.route;

	if (route.size()) {
		if (goal->action == WBOT_GOAL_ACTION_BOSS_BRAIN && !pBot->m_combatController.GetWeaponByName("RocketLauncher")) {
			// must have rocket launcher first
			vector<BotGoal> rpgs = g_wb_nav.get_weapon_goals("RocketLauncher");
			if (!pBot->SelectGoal(rpgs, NULL)) {
				// can't route to any rpg
				DebugPrint("Failed to find an RPG to kill the boss brain\n");
				pBot->FailGoal();
				return false;
			}
		}

		if (goal->action == WBOT_GOAL_ACTION_CROSS) {
			// add the back sector of the cross line to the route, in case its part of an elevator
			// this way unblocking logic works (doom2 map06 gold key).
			MapLine& line = g_map.lines[goal->lineid];
			NavSector& goalSector = g_wb_nav.mesh.nodes[route[route.size() - 1]];
			for (int i = 0; i < goalSector.links.size(); i++) {
				NavSectorLink& link = *goalSector.links[i];
				if (link.linedef == &line) {
					route.push_back(link.target->id);
					break;
				}
			}
		}

		//DebugPrint(VarArgs("Routing to goal: %s\n", goal->desc().c_str()));
		return true;
	}

	pBot->FailGoal();
	return false;
}

void CBotRouteController::DebugPrint(const char* msg) {
	pBot->DebugPrint(msg);
}

