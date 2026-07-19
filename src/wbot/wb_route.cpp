#include "wb_route.h"
#include "wb_bot.h"
#include "wb_nav.h"
#include "wb_map.h"
#include "wb_util.h"
#include "p_local.h"
#include "d_event.h"
#include "a_keys.h"
#include "p_lnspec.h"
#include <algorithm>

using namespace std;

CBotRouteController::CBotRouteController(CWootBot* pBot)
	: pBot(pBot), pActor(pBot->pActor), pPlayer(pBot->GetPlayer()) {}

void CBotRouteController::Think() {
	m_routeSpeed = RUN_SPEED;
	m_nodeRadius = 32;
	pBot->m_forwardMove = 0;
	pBot->m_sideMove = 0;
	pBot->stateFlags &= ~(FL_WBOT_WAIT_ELEV | FL_WBOT_WAIT_DOOR);

	UpdateRoute();

	BeCareful(); // updates route speed and node touch distance in dangerous areas

	if ((pBot->stateFlags & FL_WBOT_JUMPING) && m_route.size() > 1) {
		JumpThink(); // perform a jump
	}
	else if (m_route.size() > 1) {
		// allow slipping off the route into adjacent sectors while heading towards the target sector
		bool onTrack = m_navid == m_route[0]
			|| m_navCur->getLink(m_route[1]) || m_navCur->getLink(m_route[0])
			|| m_navIdeal->touches(pActor) || m_navTarget->touches(pActor);

		if (!onTrack) {
			RouteSlipThink(); // fell off the route
			return;
		}

		if (HandleBlockedPaths())
			return; // wait until the path is unblocked

		MoveThruLink(); // keep on movin
	}
	else if (m_route.size() == 1) {
		// reached the final node. Move to the center.
		FVector2 centerGoal = m_navIdeal->pos();
		if (pBot->MoveTo(centerGoal, m_nodeRadius, m_routeSpeed)) {
			m_route.clear(); // don't reset pretendsector in case a goal is inside it
			stuckPath = -1;
			pBot->stateFlags &= ~FL_WBOT_JUMPING;
		}
	}

	if (pBot->StuckThink(500)) {
		DebugPrint("I got stuck! Cancelling route.\n");

		if (m_route.size() > 1) {
			stuckPath = m_navLink ? m_navLink->id : -1;
		}

		CancelRoute();
	}
}

void CBotRouteController::UpdateRoute() {
	if (m_route.size() > 1) {
		if (m_navid == m_route[1]) {
			// inside the target sector. Advance the route.
			m_route.erase(m_route.begin());
			pretendRouteSector = -1;
			pBot->stateFlags &= ~FL_WBOT_JUMPING;
		}
		else {
			FVector2 center = g_wb_nav.mesh[m_route[1]].pos();
			fixed_t dist = P_AproxDistance(pActor->x - (fixed_t)center.X, pActor->y - (fixed_t)center.Y);
			if (pBot->stuckCounter >= 200 && dist < (16 << FRACBITS)) {
				// already very close to the center, so this is probably a tiny polygon jammed
				// up against a wall. The bot can't get close enough in this case, so advance
				// the route now and pretend the bot is inside the target sector.
				pretendRouteSector = m_route[1];
				DebugPrint(VarArgs("Pretending I'm in sector %d. I'm stuck and close enough\n", pretendRouteSector));
				m_route.erase(m_route.begin());
			}
		}
	}

	if (pretendRouteSector >= 0) {
		m_navid = pretendRouteSector;
	}

	if (m_navid != m_route[0]) {
		// update route if slipped off into a sector adjacent to a target
		NavSector& curNav = g_wb_nav.mesh[m_navid];

		if (m_route.size() > 1 && curNav.getLink(m_route[1])) {
			NavSectorLink* link = curNav.getLink(m_route[1]);
			if (link && link->walkable()) {
				m_route[0] = m_navid;
			}
		}
	}

	m_navCur = &g_wb_nav.mesh[m_navid];
	m_navIdeal = &g_wb_nav.mesh[m_route[0]];
	m_navTarget = &g_wb_nav.mesh[m_route.size() > 1 ? m_route[1] : m_route[0]];
	m_navLink = m_route.size() > 1 ? m_navIdeal->getLink(m_route[1]) : NULL;
}

void CBotRouteController::BeCareful() {
	// be careful near cliffs
	if (!(pBot->stateFlags & FL_WBOT_JUMPING)) {
		bool headingTowardsCliff = false;
		if (m_route.size() > 1) {
			NavSector& idealNav = g_wb_nav.mesh[m_route[0]];
			NavSector& targetNav = g_wb_nav.mesh[m_route[1]];
			NavSectorLink* link = idealNav.getLink(m_route[1]);
			headingTowardsCliff = targetNav.hasCliffs && link->linkWidth < 32;
		}

		if (pBot->m_cliffDist < SAFE_CLIFF_DIST * 0.5f || headingTowardsCliff) {
			m_routeSpeed *= 0.5f;
			m_nodeRadius = 8;
		}
		else if (pBot->m_cliffDist < SAFE_CLIFF_DIST * 0.75f) {
			m_routeSpeed *= 0.75f;
			m_nodeRadius = 16;
		}
	}
}

void CBotRouteController::JumpThink() {
	// just try to land in the right spot
	pBot->MoveTo(m_navTarget->pos(), 0, m_routeSpeed);

	if (m_navTarget->getFloorZ() > pActor->z + (JUMP_HEIGHT << FRACBITS)) {
		pBot->stateFlags &= ~FL_WBOT_JUMPING; // missed the jump

		BotGoal* curgoal = pBot->CurrentGoal();
		if (curgoal) {
			// don't try the jump again, there are probably other ones to try
			// and many jumps just don't work
			curgoal->blockers.insert(m_navLink->id);
		}

		return;
	}

	FVector2 target = m_navTarget->pos();
	fixed_t jumpDist = (target - FVector2(pActor->x, pActor->y)).Length();
	bool bigJump = jumpDist > 100 << FRACBITS;
	if (m_navTarget->getFloorZ() > m_navIdeal->getFloorZ() + (STEP_HEIGHT << FRACBITS))
		bigJump = true;

	if (bigJump && (pBot->stateFlags & FL_WBOT_FLYING)) {
		// bot is off a ledge now, start the jump
		pBot->m_lButtons |= BT_JUMP;
	}
}

bool CBotRouteController::HandleBlockedPaths() {
	sector_t* nextSector = subsectors[m_route[1]].sector;
	bool nextOnElevator = nextSector && nextSector->floordata;
	bool linkBlocked = m_navLink->blocked(pActor);

	if (ElevatorThink(linkBlocked)) {
		return true;
	}

	// wait for doors to open
	if (!m_navLink->walkable() && (nextSector->floordata || nextSector->ceilingdata)) {
		// door is raising or elevator is lowering in the next sector
		if (m_navLink->isJump && !nextSector->floordata && !m_navLink->isJumpHeightValid()) {
			// a door opening isn't going to make the jump doable if the floor is too high
		}
		else {
			pBot->stateFlags |= FL_WBOT_WAIT_DOOR;
			return true; // wait until the door/elevator is done moving
		}
	}

	// handle severe blockages
	if (linkBlocked) {
		BlockedPathThink(m_navLink);
		return true;
	}
	else if (!nextOnElevator && m_route.size() > 2 && m_navTarget->touches(pActor)) {
		// if we're touching the next sector and the next path is blocked, also do block
		// handling. Helps in case of doors with tiny sectors in front of them which the
		// bot can't fully get inside.
		NavSectorLink* nextLink = m_navLink->target->getLink(m_route[2]);
		sector_t* nextNextSector = subsectors[m_route[2]].sector;

		if (!nextLink->walkable() && (nextNextSector->floordata || nextNextSector->ceilingdata)) {
			pBot->stateFlags |= FL_WBOT_WAIT_DOOR;
			return true; // wait until the door/elevator is done moving
		}

		if (nextLink && nextLink->blocked(pActor)) {
			BlockedPathThink(nextLink);
			return true;
		}
	}

	return false; // not blocked
}

bool CBotRouteController::ElevatorThink(bool linkBlocked) {
	// wait on elevators
	if (pBot->stateFlags & FL_WBOT_ON_ELEV) {
		sector_t* thisSector = m_navCur->sector();
		bool waitedLongEnough = m_navLink->walkable();

		if (m_navLink->isJump) {
			// If the next link is a jump, wait until the very top for better success chance
			fixed_t elevZ = thisSector->floorplane.ZatPoint(pActor->x, pActor->y);
			fixed_t moveDelta = elevZ - m_lastElevZ;

			// elevator stopped at the top or started going down?
			waitedLongEnough = !linkBlocked && moveDelta <= 0;

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
					// forever (doom2 map15)
					vector<BotGoal>& elevTrigs = m_navIdeal->getTriggers();
					for (BotGoal& goal : elevTrigs) {
						if (goal.lineid < 0)
							continue;
						int moveFlags = g_wb_mapinfo.get_linedef_move_flag(&lines[goal.lineid]);
						if (moveFlags & FL_SECTOR_MOVE_CEIL_UP) {
							// found a trigger that will raise the ceiling on this elevator
							DebugPrint("Raising elevator ceiling to unblock path!\n");
							BotGoal goalCopy = goal;
							for (NavSectorLink& elevLink : m_navIdeal->links) {
								// stop trying to use this elevator to reach high places for now
								if (elevLink.blocked(pActor))
									goalCopy.blockers.insert(elevLink.id);
							}
							pBot->PushGoal(goalCopy, m_navLink);
							break;
						}
					}
				}
			}

			return true; // wait until the elevator is done moving
		}
	}
	else {
		m_lastElevZ = 0;
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
		FVector2 startPos = m_navLink->GetJumpBackupPos();
		int dist = pBot->GetDistance(startPos) >> FRACBITS;
		m_routeSpeed = RUN_SPEED;
		int startSpeed = RUN_SPEED * 0.2f;
		if (dist < 64) {
			m_routeSpeed = RUN_SPEED * 0.2f;
		}
		else if (dist < 256) {
			m_routeSpeed = RUN_SPEED * 0.5f;
		}
		int curSpeed = pBot->GetSpeed2D();

		if (pBot->MoveTo(startPos, 18, m_routeSpeed) && curSpeed < startSpeed) {
			// close enough to the link edge				
			pBot->stateFlags |= FL_WBOT_JUMPING;
		}
	}
	else {
		// move to the next link
		if (pBot->MoveTo(m_navLink->pos(), m_nodeRadius, m_routeSpeed)) {
			// close enough to the link edge				
			if (m_navLink->isTeleport && m_navLink->seg->linedef) {
				// move behind the teleporter line edge.
				// The target sector may be in a completely different direction.
				FVector2 backDir = getLineBackDir(m_navLink->seg->linedef);
				FVector2 teleGoal = m_navLink->pos() + backDir * 200;
				pBot->MoveTo(teleGoal, 0, m_routeSpeed);
			}
			else {
				// move towards the target sector until we end up inside it
				pBot->MoveTo(m_navTarget->pos(), 0, m_routeSpeed);
			}
		}
	}
}

void CBotRouteController::RouteSlipThink() {
	if ((pBot->stateFlags & FL_WBOT_JUMPING) && (pBot->stateFlags & FL_WBOT_FLYING)) {
		// don't abort the route until the jump is complete
		pBot->MoveTo(m_navTarget->pos(), 0, m_routeSpeed);
		return;
	}

	if (m_navid != m_route[0]) {
		// update route if slipped off into a sector adjacent to the previous
		// not done earlier for a reason i forgot on doom2 map24 tightrope area.
		NavSector& curNav = g_wb_nav.mesh[m_navid];
		NavSectorLink* link = curNav.getLink(m_route[0]);
		if (link && link->walkable()) {
			m_route.insert(m_route.begin(), m_navid);
			return;
		}
	}

	DebugPrint(VarArgs("Fell off the route (expected %d but got %d)\n", m_route[0], m_navid));
	CancelRoute();
}

void CBotRouteController::BlockedPathThink(NavSectorLink* link) {
	link->blocked(pActor); // debug here
	string blockMsg = VarArgs("Link %d blocked!", link->id);

	BotGoal* curGoal = pBot->CurrentGoal();
	curGoal->blockers.insert(link->id);

	// don't try to route through previous paths we've been trying to unblock
	unordered_set<int> allBlockedPaths = GetBlockedPaths();

	// nothing is moving, try unblocking it ourselves.
	vector<BotGoal>& targTriggers = link->target->getTriggers();
	for (BotGoal& goal : targTriggers) {
		if (!goal.valid())
			continue;
		int subid = goal.getNavId();

		if (subid == link->parent->id || RouteToSector(subid).size()) {
			DebugPrint(VarArgs("%s Adding unblock subgoal.\n", blockMsg.c_str()));
			if (pBot->PushGoal(goal, link)) {
				return;
			}
		}
	}

	// if we're on an elevator, try triggering it.
	vector<BotGoal>& thisTriggers = link->parent->getTriggers();
	for (BotGoal& goal : thisTriggers) {
		if (!goal.valid())
			continue;
		int subid = goal.getNavId();

		if (subid == link->parent->id || RouteToSector(subid).size()) {
			DebugPrint(VarArgs("%s Adding unblock subgoal.\n", blockMsg.c_str()));
			if (pBot->PushGoal(goal, link)) {
				return;
			}
		}
	}

	// nothing can unblock the path that stopped us. Try routing around it.
	m_route = RouteToSector(curGoal->getNavId());
	if (m_route.size()) {
		DebugPrint(VarArgs("%s Routing around the blocked path.\n", blockMsg.c_str()));
		return;
	}

	// clearing previous blocked links and try again, maybe paths got unblocked
	if (curGoal->blockers.size() > 1 || !curGoal->blockers.count(link->id)) {
		curGoal->blockers.clear();
		m_route = RouteToSector(curGoal->getNavId());
		if (m_route.size()) {
			DebugPrint(VarArgs("%s Forgetting blocked paths and trying again...\n", blockMsg.c_str()));
			return;
		}
	}

	DebugPrint(VarArgs("%s Goals aborted. Failed to reach a subgoal.\n", blockMsg.c_str()));
	CancelRoute();
	pBot->m_goals.clear();

	pBot->m_nextThink = level.time + 10;
}

void CBotRouteController::CancelRoute() {
	if (m_route.size() && m_freezeOnRouteChange) {
		pPlayer->cheats |= CF_FROZEN;
		pActor->velx = 0;
		pActor->vely = 0;
		pActor->velz = 0;
	}

	m_route.clear();
	pretendRouteSector = -1;
	pBot->stuckCounter = 0;
	pBot->stateFlags &= ~FL_WBOT_JUMPING;
}

unordered_set<int> CBotRouteController::GetBlockedPaths() {
	unordered_set<int> allBlockedPaths;
	for (BotGoal& goal : pBot->m_goals) {
		allBlockedPaths.insert(goal.blockers.begin(), goal.blockers.end());
	}
	return allBlockedPaths;
}

vector<int> CBotRouteController::RouteToSector(int subid) {
	unordered_set<int> allBlockedPaths = GetBlockedPaths();

	if (stuckPath >= 0) {
		// avoid the path that got the bot stuck in the last movement
		DebugPrint(VarArgs("Ignoring stucked path %d for this route\n", stuckPath));
		allBlockedPaths.insert(stuckPath);
		stuckPath = -1;
	}

	return g_wb_nav.get_astar_route(m_navid, subid, &allBlockedPaths, pBot->stateFlags & FL_WBOT_RUSHING);
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

	if (m_route.empty() && goal->actor) {
		// actor origin is in an unreachable sector, but it's collision box may be touching a reachable one
		vector<int> subs = g_wb_mapinfo.GetTouchedSubsectors(goal->actor);

		for (const int& subid : subs) {
			if (subid == goalNavId) {
				continue;
			}

			m_route = RouteToSector(subid);
			if (m_route.size())
				break;
		}
	}

	if (m_route.size()) {
		if (goal->action == WBOT_GOAL_ACTION_CROSS) {
			// add the back sector of the cross line to the route, in case its part of an elevator
			// this way unblocking logic works (doom2 map06 gold key).
			line_t& line = lines[goal->lineid];
			NavSector& goalSector = g_wb_nav.mesh[m_route[m_route.size() - 1]];
			for (int i = 0; i < goalSector.links.size(); i++) {
				NavSectorLink& link = goalSector.links[i];
				if (link.seg->linedef == &line) {
					m_route.push_back(link.target->id);
					break;
				}
			}
		}

		DebugPrint(VarArgs("Routing to goal: %s\n", goal->desc().c_str()));
		return true;
	}

	DebugPrint(VarArgs("Failed goal (no route): %s\n", goal->desc().c_str()));
	pBot->m_goals.pop_back();
	pBot->m_nextThink = level.time + 10;
	return false;
}

void CBotRouteController::DebugPrint(const char* msg) {
	pBot->DebugPrint(msg);
}

