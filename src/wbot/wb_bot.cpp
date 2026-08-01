#include "wb_bot.h"
#include "wb_nav.h"
#include "wb_map.h"
#include "wb_util.h"
#include "wb_debug.h"
#include "wb_eiface.h"

#include <string>
#include <float.h>
#include <algorithm>
#include <limits.h>

using namespace std;
using namespace wbot;

CWootBot::CWootBot(const char* pszName, const char* pszTeamName, uint32_t ulPlayerNum)
	: m_pPlayer(g_engine.init_bot(this, ulPlayerNum, "", "", "", pszTeamName)), m_routeController(this), m_combatController(this) {

	m_fov = 180;
	m_debug = true;
	Reset();
}

void CWootBot::Think() {
	if (g_engine.intermission_active()) {
		return;
	}

	if (m_wasDead && m_astate.health > 0) {
		Reset();
		m_wasDead = false;
	}

	UpdatePositionFlags();

	if (m_debug) {
		wbot_debug(this);
	}

	if (g_engine.get_game_tics() < m_nextThink || m_pstate.isFrozen) {
		return;
	}

	if (m_astate.health <= 0) {
		DeadThink();
		return;
	}

	m_lButtons = 0;

	m_combatController.Think();

	if (!target) {
		if (!m_routeController.HasRoute()) {
			if (!HasGoal()) {
				FindGoal();

				if (!HasGoal()) {
					IdleThink();
				}
			}
			else {
				GoalActionThink();
			}
		}
		else {
			m_routeController.Think();
		}
	}

	if (m_speedMult != 1.0f) {
		m_forwardMove *= m_speedMult;
		m_sideMove *= m_speedMult;
	}

	m_lForwardMove = static_cast<int>(0x32 * (m_forwardMove / 100.0f));
	m_lSideMove = static_cast<int>(0x32 * (m_sideMove / 100.0f));
}

void CWootBot::Reset() {
	pActor = g_engine.get_player(m_pPlayer);
	target = NULL;
	m_routeController = CBotRouteController(this);
	m_combatController = CBotCombatController(this);
	m_goals.clear();
	m_lastUse = 0;
	stateFlags = 0;
	m_forwardMove = 0;
	m_sideMove = 0;
	m_nextThink = 0;
	goalFailCounter = 0;
	m_lastAvoidPropDirChange = 0;
	m_lastAvoidPropDir = 0;
	m_lForwardMove = 0;
	m_lSideMove = 0;
	m_lButtons = 0;
	rushNav = -1;
	rushTrigger = BotGoal();
	UpdatePositionFlags();
}

void CWootBot::DebugPrint(const char* msg) {
	if (m_debug) {
		g_engine.PrintNotification(msg);
		g_engine.gprintf(msg);
	}
}

void CWootBot::DeadThink() {
	// tap a button to respawn
	m_lForwardMove = 0;
	m_lSideMove = 0;
	m_lButtons ^= IN_ATTACK;
	m_wasDead = true;
}

void CWootBot::IdleThink() {
	m_forwardMove = 0;
	m_sideMove = 0;
	m_astate.pitch = 0;

	if (StopMoving()) {
		m_lButtons |= IN_DUCK;

		if (rand() % 20 == 0) {
			m_astate.yaw = rand();
		}
	}
}

void CWootBot::GoalActionThink() {
	BotGoal& goal = *CurrentGoal();
	NavSector& nav = m_routeController.m_navIdeal ? *m_routeController.m_navIdeal
		: g_wb_nav.mesh.nodes[m_routeController.m_navid];

	if (!goal.valid()) {
		DebugPrint("Goal became invalid\n");
		CompleteGoal();
		return;
	}

	int goalSector = goal.getNavId();

	if (m_routeController.m_navid != goalSector && m_routeController.pretendRouteSector != goalSector) {
		// actor may be routed here but technically in an adjacent sector
		bool actorReachable = nav.touches(goal.h_actor.get());

		if (!actorReachable) {
			// route was cancelled or the target moved. Route to it again.
			DebugPrint("Goal moved or movement failed. Rerouting...\n");
			m_routeController.RouteToGoal();
			return;
		}
	}

	m_forwardMove = 0;
	m_sideMove = 0;

	switch (goal.action) {
	default:
		DebugPrint(VarArgs("Unknown goal action type %d\n", goal.action));
	case WBOT_GOAL_ACTION_MOVE_TO:
		CompleteGoal(); // nothing to do
		break;
	case WBOT_GOAL_ACTION_USE: {
		int useDist = PLAYER_USE_DIST - 1;
		if (MoveTo(goal.pos(), useDist)) {
			Use();
		}
		break;
	}
	case WBOT_GOAL_ACTION_TOUCH:
		if (MoveTo(goal.pos(), goal.touchDistance((AActor*)pActor))) {
			if (goal.lineid == -1)
				CompleteGoal(); // only lines are hooked and pop goals automatically
		}
		break;
	case WBOT_GOAL_ACTION_CROSS: {
		if (goal.lineid >= 0) {
			// move through the line to the backside of it
			MapLine* line = &g_map.lines[goal.lineid];
			vec2 backDir = line->normal() * -1;
			vec2 backGoal = line->center() + backDir * 32;

			// be careful not to miss skinny lines
			int speed = line->length() > 32 ? RUN_SPEED : RUN_SPEED / 4;

			MoveTo(backGoal, 16, speed);
		}
		else {
			DebugPrint("Can't cross an actor as a goal!\n");
			CompleteGoal();
		}
		break;
	}
	case WBOT_GOAL_ACTION_SHOOT: {
		int shootRange = 200;

		if (m_pstate.weaponName) {
			WeaponInfo& info = g_wbot_weapon_info[m_pstate.weaponName];
			shootRange = info.maxRange;
		}

		MoveTo(goal.pos(), 100);

		TraceResult tr;
		TraceAhead(shootRange, vec3(0, 0, m_pstate.viewHeight), false, &tr);
		if (tr.line && (tr.line - g_map.lines) == goal.lineid) {
			Attack();
		}
		break;
	}
	case WBOT_GOAL_ACTION_BOSS_BRAIN: {
		if (!m_combatController.GetWeaponByName("RocketLauncher")) {
			vector<BotGoal> rpgs = g_wb_nav.get_weapon_goals("RocketLauncher");
			if (!SelectGoal(rpgs, NULL, 0)) {
				DebugPrint("Failed to find an RPG to kill the boss brain\n");
				FailGoal();
			}
			return;
		}

		if (!m_combatController.SelectWeapon("RocketLauncher")) {
			vector<BotGoal> rockets = g_wb_nav.get_ammo_goals("RocketBox", "Rocket");
			if (!SelectGoal(rockets, NULL, 0)) {
				DebugPrint("Failed to find RPG ammo to kill the boss brain\n");
				FailGoal();
			}
			return;
		}

		//draw_debug_line(goal.shootAlignment.shootFrom, goal.shootAlignment.shootAt, pActor);

		int dist = GetDistance(goal.shootAlignment.shootFrom);
		int moveSpeed = std::min(RUN_SPEED, dist);
		if (MoveTo(goal.shootAlignment.shootFrom, 8, moveSpeed) && GetSpeed2D() < 10) {
			AimAtPos(goal.shootAlignment.shootAt);
			Attack();
		}
		break;
	}
	}

	if (StuckThink(1000)) {
		m_routeController.RouteToGoal();
	}
}

bool CWootBot::StuckThink(int maxStuck) {
	float movedDist = (m_astate.origin - lastPos).length();
	lastPos = m_astate.origin;

	if ((m_forwardMove || m_sideMove) && movedDist <= 1.0f * m_speedMult) {
		stuckCounter += 10;
		if (stuckCounter > maxStuck) {
			stuckCounter = 0;
			return true;
		}
	}
	else {
		stuckCounter--;
		if (stuckCounter < 0)
			stuckCounter = 0;
	}

	return false;
}

void CWootBot::AimAtPos(vec3 pos) {
	float viewZ = m_astate.origin.z + m_pstate.viewHeight;
	float dist = (vec2(pos.x, pos.y) - m_astate.origin).length();
	m_astate.pitch = -(int32_t)g_engine.PointToAngle2(0, viewZ, dist, pos.z);
	m_astate.yaw = g_engine.PointToAngle2(m_astate.origin.x, m_astate.origin.y, pos.x, pos.y);
}

bool CWootBot::TraceAhead(int dist, vec3 offset, bool ignoreMonsters, TraceResult* tr) {
	vec3 forward, right;
	g_engine.MakeVectors(m_astate.yaw, forward, right);
	vec3 start = m_astate.origin + offset;
	vec3 end = start + forward * dist;

	return g_engine.TraceLine(start, end, ignoreMonsters, (AActor*)pActor, tr);
}

bool CWootBot::MoveTo(vec2 pos, int radius, int speed) {
	float z = m_astate.origin.z + m_pstate.viewHeight;
	AimAtPos(vec3(pos.x, pos.y, z));

	vec2 wantDir = (pos - m_astate.origin).normalize();

	// jump over walls and activate things in front of us
	TraceResult tr;
	int useDist = PLAYER_USE_DIST - 1;
	if (TraceAhead(useDist, vec3(0, 0, STEP_HEIGHT), true, &tr)) {
		int d = tr.frac * useDist;

		// jump if not too far and this isn't an impassable wall
		if (tr.line->backsector && d < 32 && m_pstate.onGround) {
			float backZ = tr.line->backsector->getFloorZ();
			int jumpHeight = backZ - m_astate.origin.z;

			// ...and it's possible and necessary to jump up
			if (jumpHeight > STEP_HEIGHT && jumpHeight <= JUMP_HEIGHT)
				m_lButtons |= IN_JUMP;
		}

		// only use walls that aren't already moving, so doors aren't closed while opening
		bool lineIsMoving = tr.line && tr.line->backsector && tr.line->backsector->isMoving();

		if (!lineIsMoving) {
			// activate any triggered line to fix face rubbing on walls when the bot is failing
			// to get to a tiny sector in front of a door/button.
			int action = g_engine.get_line_state(tr.line->id).goalAction;

			if (action == WBOT_GOAL_ACTION_USE)
				Use();

			if (action == WBOT_GOAL_ACTION_SHOOT)
				Attack();
		}
	}

	// combine desired vector with avoidance vectors
	vec2 avoidCornersDir = AvoidCornersVector(wantDir);
	vec2 avoidLedgeDir = AvoidLedges((AActor*)pActor, m_cliffDist);
	vec2 moveDir = wantDir + avoidCornersDir;

	if (!m_routeController.jumpState) {
		if (m_cliffDist < 0) {
			// do whatever it takes to get back on solid ground.
			// For best results, speed should be 0.5 or less, so correction force can recover from any velocity
			moveDir = avoidLedgeDir;
			speed = std::min(RUN_SPEED, speed * 2);

			//vec3 dpos(pos, z);
			//draw_debug_line(dpos, dpos + vec3(avoidLedgeDir, 0) * 100, pActor);
		}
		else {
			moveDir += avoidLedgeDir * 0.5f;
		}
	}

	moveDir = moveDir.normalize(speed);

	// convert directinal vectors to forward/strafe movents relative to the look direction
	vec3 forward, right;
	g_engine.MakeVectors(m_astate.yaw, forward, right);
	m_forwardMove = dotProduct(moveDir, forward);
	m_sideMove = dotProduct(moveDir, right);

	float dist = (pos - m_astate.origin).length();
	return dist < radius;
}

bool CWootBot::StopMoving() {
	vec2 vel = m_astate.velocity;
	vec2 opposingPos = m_astate.origin - (vel.normalize() * 100);

	int speed = GetSpeed2D();

	if (speed > 0) {
		MoveTo(opposingPos, 0, std::min(RUN_SPEED, speed*4));
	}
	// else wait for friction to stop us

	return speed == 0;
}

vec2 CWootBot::AvoidCornersVector(vec2 wantDir) {
	// strafe around objects/walls partially blocking the way
	float zTest = STEP_HEIGHT + 1;
	vec3 testDir = vec3(wantDir.x, wantDir.y, 0) * 32;
	vec3 viewPos = m_astate.origin + vec3(0, 0, zTest);
	float rightOfs = 16;
	vec3 rightDir(wantDir.y, -wantDir.x, 0);
	vec3 rightPos = viewPos + rightDir * rightOfs;
	vec3 leftPos = viewPos + rightDir * -rightOfs;
	TraceResult trLeft, trRight;

	g_engine.TraceLine(rightPos, rightPos + testDir, false, (AActor*)pActor, &trRight);
	g_engine.TraceLine(leftPos, leftPos + testDir, false, (AActor*)pActor, &trLeft);

	bool hitProp = trRight.actor || trLeft.actor;
	bool canChangeDir = !hitProp || g_engine.get_game_tics() - m_lastAvoidPropDirChange > 70;
	if (!canChangeDir) {
		if (trLeft.frac < 1.0f || trRight.frac < 1.0f) {
			return rightDir * m_lastAvoidPropDir;
		}
	}
	else {
		if (trLeft.frac != trRight.frac) {
			/*
			if (trRight.frac < trLeft.frac) {
				if (trRight.actor)
					DebugPrint(VarArgs("Avoid %s\n", trRight.actor->GetClass()->TypeName.GetChars()));
				else
					DebugPrint(VarArgs("Avoid line %d\n", trRight.line->id));
				draw_debug_line(rightPos, rightPos + testDir * 32, pActor);
			}
			else {
				if (trLeft.actor)
					DebugPrint(VarArgs("Avoid %s\n", trLeft.actor->GetClass()->TypeName.GetChars()));
				else
					DebugPrint(VarArgs("Avoid line %d\n", trLeft.line->id));
				draw_debug_line(leftPos, leftPos + testDir * 32, pActor);
			}
			*/
			m_lastAvoidPropDirChange = g_engine.get_game_tics();
			m_lastAvoidPropDir = trRight.frac < trLeft.frac ? -1 : 1;
			return rightDir * m_lastAvoidPropDir;
		}
	}

	return vec2(0, 0);
}

vec2 CWootBot::AvoidLedges(AActor* actor, int& cliffDist) {
	int subid = g_map.GetSubsector(m_astate.origin.x, m_astate.origin.y) - g_map.subsectors;
	NavSector* nav = &g_wb_nav.mesh.nodes[subid];
	vec2 plrPos = m_astate.origin;

	int targetNav = -1;
	int idealNav = -1;
	unordered_set<int> ignoreLinks;
	if (m_routeController.m_route.route.size() > 0) {
		idealNav = m_routeController.m_route.route[0];
		nav = &g_wb_nav.mesh.nodes[idealNav];
	}
	if (m_routeController.m_route.route.size() > 1) {
		targetNav = m_routeController.m_route.route[1];
		NavSectorLink* targlink = nav->getLink(targetNav);
		if (targlink) {
			// ignore any links overlapping the current route link
			// (there may be multiple jump links on top of a walkable link)
			for (NavSectorLink* link : nav->links) {
				if (link->seg.a == targlink->seg.a && link->seg.b == targlink->seg.b)
					ignoreLinks.insert(link->id);
			}
		}
	}

	float ignoreZ = m_astate.origin.z + STEP_HEIGHT; // can't fall off a cliff above us

	// get nearby sectors in case nearest ledge is at the corner of the current
	std::vector<NavSector*> sectors;
	if (nav->hasCliffs)
		sectors.push_back(nav);
	for (NavSectorLink* link : nav->links) {
		if (link->isCliff || link->isTeleport)
			continue;

		if (link->target->hasCliffs) {
			sectors.push_back(link->target);
		}
	}

	vec2 worstNormal(0,0);
	int worstDist = INT_MAX;
	int worsetSec = -1;
	int worstLink = -1;

	for (NavSector* testSec : sectors) {
		if (testSec->getFloorZ() > ignoreZ) {
			continue;
		}

		for (NavSectorLink* link : testSec->links) {
			if (!link->isCliff)
				continue;

			if (link->target->id == targetNav || link->target->id == idealNav)
				continue; // don't back off from segments that must be crossed

			if (ignoreLinks.count(link->id))
				continue;

			vec2 v1 = link->seg.a;
			vec2 v2 = link->seg.b;
			vec2 normal = link->seg.normal();

			// distance to axis
			int dist = (int)dotProduct(plrPos - v1, normal);
			
			if (dist < 0 && testSec != nav)
				continue; // can't have fallen off a cliff in a neighbor sector

			ExtendSegment(v1, v2, m_astate.radius);

			if (!PointAlignedSegment(plrPos, v1, v2))
				continue; // off to the side of this segment

			if (dist < worstDist) {
				worstDist = dist;
				worstNormal = normal;
				worsetSec = testSec->id;
				worstLink = link->id;
			}
		}
	}

	vec2 avoidForce(0, 0);

	if (worstDist < SAFE_CLIFF_DIST) {
		cliffDist = worstDist;
		avoidForce = worstNormal;
	}
	else {
		cliffDist = 9999;
	}

	//FVector3 headPos = vec3(plrPos, pActor->z / FRACUNIT + 56);
	//FVector3 dir = vec3(avoidForce, 0);
	//float scale = 200;
	//draw_debug_line(headPos, headPos + dir * scale, pActor);

	return avoidForce;
}

void CWootBot::UpdatePositionFlags() {
	m_routeController.m_navid = g_wb_nav.get_nav_id(m_pPlayer);

	stateFlags &= ~(FL_WBOT_FLYING | FL_WBOT_ON_ELEV | FL_WBOT_OVERHANG);
	if (m_routeController.m_navCur) {
		NavSector& nav = g_wb_nav.mesh.nodes[m_routeController.m_navid];
		if (m_astate.origin.z > nav.getFloorZ())
			stateFlags |= m_pstate.onGround ? FL_WBOT_OVERHANG : FL_WBOT_FLYING;
		if (nav.sector->isFloorMoving())
			stateFlags |= FL_WBOT_ON_ELEV;
	}
}

vec3 CWootBot::GetViewPos() {
	return m_astate.origin + vec3(0, 0, m_pstate.viewHeight);
}

float CWootBot::GetDistance(vec2 p) {
	return (p - m_astate.origin).length();
}

int CWootBot::GetSpeed2D() {
	// TODO: why is this conversion to cmd speeds weird?
	return vec2(m_astate.velocity.x, m_astate.velocity.y).length() * 8.0f;
}

bool CWootBot::FindGoal() {

	if (m_autoWinMap) {
		PushLevelEndGoal();
	}
	else if (m_followPlayer) {
		int thisSubId = g_wb_nav.get_nav_id((AActor*)pActor);
		AActor* player = g_engine.find_followable_player(thisSubId);
	}

	return m_routeController.HasRoute();
}

bool CWootBot::PushLevelEndGoal() {
	m_routeController.CancelRoute();

	for (int i = 0; i < g_map.numlines; i++) {
		MapLine& line = g_map.lines[i];
		if (line.isLevelExit()) {
			if (PushGoal(BotGoal(g_engine.get_line_state(i).goalAction, i), NULL)) {
				return true;
			}
		}
	}

	AActor* boss = g_engine.find_boss_brain();
	if (boss && PushGoal(BotGoal(WBOT_GOAL_ACTION_BOSS_BRAIN, boss), NULL)) {
		return true;
	}

	return false;
}

bool CWootBot::PushGoal(const BotGoal& goal, NavSectorLink* purposeLink) {
	if (!m_goals.empty()) {
		BotGoal& lastGoal = m_goals[m_goals.size() - 1];
		if (lastGoal.matches(goal)) {
			// can happen when hugging the wall of a triggerable sectors
			DebugPrint(VarArgs("Skipping duplicate goal: %s\n", goal.desc().c_str()));
			return false;
		}
		if (goal.purposeLink && lastGoal.purposeLink->target == goal.purposeLink->target) {
			// don't route to multiple goals that activate the same thing, if unblocked
			if (lastGoal.blockers.empty()) {
				DebugPrint(VarArgs("Skipping redundant goal: %s\n", goal.desc().c_str()));
				return false;
			}
		}
	}

	DebugPrint(VarArgs("New goal '%s'\n", goal.desc().c_str()));
	m_goals.push_back(goal);
	m_goals[m_goals.size() - 1].purposeLink = purposeLink;

	// also add key goals needed to use this line, if missing
	if (goal.lineid >= 0 && !PushKeyGoals(&g_map.lines[goal.lineid])) {
		m_goals.pop_back();
		return false;
	}

	if (m_routeController.RouteToGoal()) {
		if (purposeLink && m_goals.size() > 1 && goal.lineid >= 0) {
			BotGoal& parentGoal = m_goals[m_goals.size() - 2];
			parentGoal.unblockAttempts[goal.lineid] = purposeLink->id;
		}

		return true;
	}

	return false;
}

bool CWootBot::PushKeyGoals(MapLine* line) {
	if (line->isLockedDoor() && !g_engine.can_unlock_door((AActor*)pActor, line)) {
		vector<BotGoal> keyGoals;

		m_routeController.MarkBlockedPaths();
		g_wb_nav.get_key_goals_for_line((AActor*)pActor, line, keyGoals);

		for (BotGoal& keyGoal : keyGoals) {
			m_goals.push_back(keyGoal);
			DebugPrint(VarArgs("New key subgoal '%s'\n", keyGoal.desc().c_str()));

			if (!m_routeController.RouteToGoal()) {
				return false;
			}
		}
	}

	return true;
}

bool CWootBot::SelectGoal(vector<BotGoal>& goals, NavSectorLink* purposeLink, int movementNeeded) {
	int unblockSector = purposeLink ? purposeLink->target->id : -1;
	BotGoal* curGoal = CurrentGoal();
	BotGoal* bestGoal = NULL;

	float bestCost = FLT_MAX;

	RouteOpts opts;
	opts.start = m_routeController.m_navid;
	opts.timeSensitive = stateFlags & FL_WBOT_RUSHING;
	opts.blockedPathHandling = WBOT_ROUTE_BLOCK_EXPENSIVE;

	m_routeController.MarkBlockedPaths();
	if (unblockSector != -1)
		g_wb_nav.mesh.nodes[unblockSector].routeNumIgnore = g_route_ignore_num;

	int oldRouteIgnoreNum = g_route_ignore_num;

	for (int i = 0; i < goals.size(); i++) {
		BotGoal& goal = goals[i];
		if (!goal.valid())
			continue;

		MapLine* line = goal.lineid >= 0 ? &g_map.lines[goal.lineid] : NULL;
		if (line && !(g_engine.get_line_state(line->id).moveFlags & movementNeeded)) {
			continue; // line would not move the sector in a way thats needed
		}

		int subid = goal.getNavId();

		if (curGoal && purposeLink) {
			auto unblockAttempt = curGoal->unblockAttempts.find(goal.lineid);
			if (unblockAttempt != curGoal->unblockAttempts.end() && unblockAttempt->second == purposeLink->id) {
				// already tried this goal and the path was still blocked after completing it.
				continue;
			}
		}

		opts.end = subid;
		g_route_ignore_num = oldRouteIgnoreNum;
		BotRoute route = g_wb_nav.get_astar_route(opts);

		if ((purposeLink && subid == purposeLink->parent->id) || route.route.size()) {
			if (!bestGoal || route.cost < bestCost) {
				bestCost = route.cost;
				bestGoal = &goal;
			}
		}
	}

	if (bestGoal && PushGoal(*bestGoal, purposeLink)) {
		return true;
	}

	return false;
}

void CWootBot::CompleteGoal() {
	if (m_goals.empty()) {
		m_routeController.CancelRoute();
		DebugPrint("No goal to pop\n");
		return;
	}

	BotGoal& goal = m_goals[m_goals.size() - 1];

	rushNav = -1;
	goalFailCounter = 0;
	stateFlags &= ~FL_WBOT_RUSHING;

	int purposeLinkId = goal.purposeLink ? goal.purposeLink->id : -1;
	NavSector* purposeNav = goal.purposeLink ? goal.purposeLink->target : NULL;
	if (purposeNav && (purposeNav->getMoveFlags() & FL_SECTOR_MOVE_TIMED)) {
		// the purpose of this goal was to move a timed sector. Better hurry before that sector resets!
		stateFlags |= FL_WBOT_RUSHING;
		rushNav = goal.purposeLink->target->id;
		rushTrigger = goal;
	}

	DebugPrint(VarArgs("Completed %s '%s'\n", m_goals.size() == 1 ? "goal" : "subgoal", goal.desc().c_str()));
	m_goals.pop_back();

	if (m_goals.size()) {
		// unblock the link that the previous goal was for
		for (int i = 0; i < m_goals.size(); i++) {
			m_goals[i].blockers.erase(purposeLinkId);
		}

		m_routeController.RouteToGoal();
	}
	else
		m_routeController.CancelRoute();
}

void CWootBot::FailGoal() {
	m_nextThink = g_engine.get_game_tics() + 7; // failing lots of goals at once could cause lag

	if (m_goals.empty()) {
		return;
	}

	BotGoal& curGoal = m_goals[m_goals.size() - 1];
	m_routeController.CancelRoute();

	DebugPrint(VarArgs("FAILED %s '%s'\n", m_goals.size() == 1 ? "goal" : "subgoal", curGoal.desc().c_str()));

	if (m_routeController.m_freezeOnGoalFail) {
		g_engine.freeze_player(m_pPlayer, true);
	}

	bool bubbleFailure = curGoal.required;

	if ((!bubbleFailure || m_goals.size() == 1)) {
		if (++goalFailCounter > 10) {
			DebugPrint("I can't reach any goals from here! Time to die.\n");
			g_engine.kill_actor((AActor*)pActor);
		}
	}

	if (m_goals.size() == 1) {
		m_goals.clear();
		return;
	}

	// bubble up the blockers from this goal and try another route to the parent goal
	BotGoal& prevGoal = m_goals[m_goals.size() - 2];
	prevGoal.blockers.insert(curGoal.blockers.begin(), curGoal.blockers.end());
	m_goals.pop_back();

	if (bubbleFailure) {
		FailGoal(); // fail until hitting an optional/parent goal
	}
}

void CWootBot::Use(int ticsBetweenUses) {
	if (g_engine.get_game_tics() - m_lastUse < ticsBetweenUses) {
		return;
	}

	m_lButtons |= IN_USE;
	m_lastUse = g_engine.get_game_tics();
}

void CWootBot::Attack() {
	m_lButtons |= IN_ATTACK;
	m_combatController.m_lastAttack = g_engine.get_game_tics();
}

void CWootBot::HandleLineActivation(MapLine* line, AActor* activator) {
	if (line->special() && line->isLockedDoor()) {
		if (!g_engine.can_unlock_door(activator, line)) {
			// door wasn't actually opened
			if (activator == (AActor*)pActor) {
				// we tried to activate this. Add the key goals now. They may have been skipped if this
				// isn't our current goal line, but it blocks access to an unblocked line in the same
				// goal sector ID (doom2 map27 thin blue door in front of switch)
				PushKeyGoals(line);
			}
			return;
		}
	}

	int popIdx = -1;
	for (int i = 0; i < m_goals.size(); i++) {
		if (m_goals[i].lineid == line->id) {
			popIdx = i;
			break;
		}
	}

	if (popIdx != -1) {
		// pop all subgoals that were for activating this line
		int numSubPop = (m_goals.size() - popIdx) - 1;

		if (numSubPop > 0)
			DebugPrint(VarArgs("Something activated line %d! Popping %d subgoals\n", line->id, numSubPop));

		while (numSubPop--) {
			m_goals.pop_back();
		}

		CompleteGoal(); // only final pop does rerouting logic
	}
}
