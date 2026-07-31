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

CWootBot::CWootBot(const char* pszName, const char* pszTeamName, ULONG ulPlayerNum)
	: m_pPlayer(add_bot(this, ulPlayerNum, "", "", "", pszTeamName)), m_routeController(this), m_combatController(this) {

	m_fov = 180;
	m_debug = true;
	Reset();
}

void CWootBot::Think() {
	if (intermission_active()) {
		return;
	}

	if (m_wasDead && m_health > 0) {
		Reset();
		m_wasDead = false;
	}

	UpdatePositionFlags();

	if (m_debug) {
		wbot_debug(this);
	}

	if (get_game_tics() < m_nextThink || m_isFrozen) {
		return;
	}

	if (m_health <= 0) {
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

	m_lForwardMove = static_cast<LONG>(0x32 * (m_forwardMove / 100.0f));
	m_lSideMove = static_cast<LONG>(0x32 * (m_sideMove / 100.0f));
}

void CWootBot::Reset() {
	pActor = get_player(m_pPlayer);
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
		PrintNotification(msg);
		gprintf(msg);
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
	m_pitch = 0;

	if (StopMoving()) {
		m_lButtons |= IN_DUCK;

		if (rand() % 20 == 0) {
			m_yaw = rand();
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
		int useDist = m_useDistance - 1;
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
			FVector2 backDir = line->normal() * -1;
			FVector2 backGoal = line->center() + backDir * 32;

			fixed_t dist = (backGoal - m_origin).Length();

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

		if (m_weaponName) {
			WeaponInfo& info = g_wbot_weapon_info[m_weaponName];
			shootRange = info.maxRange;
		}

		MoveTo(goal.pos(), 100);

		TraceResult tr;
		TraceAhead(shootRange, FVector3(0, 0, m_viewHeight), false, &tr);
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

		int dist = GetDistance(goal.shootAlignment.shootFrom) >> FRACBITS;
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
	fixed_t movedDist = (m_origin - lastPos).Length();
	lastPos = m_origin;

	if ((m_forwardMove || m_sideMove) && movedDist <= (1 << FRACBITS) * m_speedMult) {
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

void CWootBot::AimAtPos(FVector3 pos) {
	fixed_t viewZ = m_origin.Z + m_viewHeight;
	fixed_t dist = (FVector2(pos.X, pos.Y) - m_origin).Length();
	m_pitch = -(SDWORD)PointToAngle2(0, viewZ, dist, pos.Z);
	m_yaw = PointToAngle2(m_origin.X, m_origin.Y, pos.X, pos.Y);
}

bool CWootBot::TraceAhead(int dist, FVector3 offset, bool ignoreMonsters, TraceResult* tr) {
	FVector3 forward, right;
	MakeVectors(m_yaw, forward, right);
	FVector3 start = m_origin + offset;
	FVector3 end = start + forward * dist;

	return g_map.Trace(start, end, ignoreMonsters, (AActor*)pActor, tr);
}

bool CWootBot::MoveTo(FVector2 pos, int radius, int speed) {
	float z = (float)(m_origin.Z + m_viewHeight);
	AimAtPos(FVector3(pos.X, pos.Y, z));

	FVector2 wantDir = pos - m_origin;
	wantDir.MakeUnit();

	// jump over walls and activate things in front of us
	TraceResult tr;
	int useDist = m_useDistance - 1;
	if (TraceAhead(useDist, FVector3(0, 0, STEP_HEIGHT << FRACBITS), true, &tr)) {
		int d = tr.frac * useDist;

		// jump if not too far and this isn't an impassable wall
		if (tr.line->backsector && d < 32 && m_onGround) {
			fixed_t backZ = tr.line->backsector->getFloorZ();
			int jumpHeight = (fixed_t)(backZ - m_origin.Z) >> FRACBITS;

			// ...and it's possible and necessary to jump up
			if (jumpHeight > STEP_HEIGHT && jumpHeight <= JUMP_HEIGHT)
				m_lButtons |= IN_JUMP;
		}

		// only use walls that aren't already moving, so doors aren't closed while opening
		bool lineIsMoving = tr.line && tr.line->backsector && tr.line->backsector->isMoving();

		if (!lineIsMoving) {
			// activate any triggered line to fix face rubbing on walls when the bot is failing
			// to get to a tiny sector in front of a door/button.
			int action = g_map.get_linedef_goal_action(tr.line);

			if (action == WBOT_GOAL_ACTION_USE)
				Use();

			if (action == WBOT_GOAL_ACTION_SHOOT)
				Attack();
		}
	}

	// combine desired vector with avoidance vectors
	FVector2 avoidCornersDir = AvoidCornersVector(wantDir);
	FVector2 avoidLedgeDir = AvoidLedges((AActor*)pActor, m_cliffDist);
	FVector2 moveDir = wantDir + avoidCornersDir;

	if (!m_routeController.jumpState) {
		if (m_cliffDist < 0) {
			// do whatever it takes to get back on solid ground.
			// For best results, speed should be 0.5 or less, so correction force can recover from any velocity
			moveDir = avoidLedgeDir;
			speed = std::min(RUN_SPEED, speed * 2);

			//FVector3 dpos(pos, z);
			//draw_debug_line(dpos, dpos + FVector3(avoidLedgeDir, 0) * (100 << FRACBITS), pActor);
		}
		else {
			moveDir += avoidLedgeDir * 0.5f;
		}
	}

	moveDir.MakeUnit();
	moveDir *= speed;

	// convert directinal vectors to forward/strafe movents relative to the look direction
	FVector3 forward, right;
	MakeVectors(m_yaw, forward, right);
	forward.MakeUnit();
	right.MakeUnit();
	m_forwardMove = DotProduct(moveDir, forward);
	m_sideMove = DotProduct(moveDir, right);

	fixed_t dist = (pos - m_origin).Length();
	return dist < (radius << FRACBITS);
}

bool CWootBot::StopMoving() {
	FVector2 pos = m_origin;
	FVector2 vel = m_velocity;
	FVector2 opposingPos = m_origin - (vel.Unit() * (100 << FRACBITS));

	int speed = GetSpeed2D();

	if (speed > 0) {
		MoveTo(opposingPos, 0, std::min(RUN_SPEED, speed*4));
	}
	// else wait for friction to stop us

	return speed == 0;
}

FVector2 CWootBot::AvoidCornersVector(FVector2 wantDir) {
	// strafe around objects/walls partially blocking the way
	fixed_t zTest = (STEP_HEIGHT + 1) << FRACBITS;
	FVector3 testDir = FVector3(wantDir.X * FRACUNIT, wantDir.Y * FRACUNIT, 0) * 32;
	FVector3 viewPos = m_origin + FVector3(0, 0, zTest);
	fixed_t rightOfs = 16 << FRACBITS;
	FVector3 rightDir(wantDir.Y, -wantDir.X, 0);
	FVector3 rightPos = viewPos + rightDir * rightOfs;
	FVector3 leftPos = viewPos + rightDir * -rightOfs;
	TraceResult trLeft, trRight;

	g_map.Trace(rightPos, rightPos + testDir, false, (AActor*)pActor, &trRight);
	g_map.Trace(leftPos, leftPos + testDir, false, (AActor*)pActor, &trLeft);

	bool hitProp = trRight.actor || trLeft.actor;
	bool canChangeDir = !hitProp || get_game_tics() - m_lastAvoidPropDirChange > 70;
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
			m_lastAvoidPropDirChange = get_game_tics();
			m_lastAvoidPropDir = trRight.frac < trLeft.frac ? -1 : 1;
			return rightDir * m_lastAvoidPropDir;
		}
	}

	return FVector2(0, 0);
}

FVector2 CWootBot::AvoidLedges(AActor* actor, int& cliffDist) {
	//NavSector& nav = g_wb_nav.mesh[m_navid];
	//FVector2 plrPos(pActor->x, pActor->y);
	int subid = g_map.GetSubsector(m_origin.X, m_origin.Y) - g_map.subsectors;
	NavSector* nav = &g_wb_nav.mesh.nodes[subid];
	FVector2 plrPos = m_origin;

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

	fixed_t ignoreZ = m_origin.Z + (STEP_HEIGHT << FRACBITS); // can't fall off a cliff above us

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

	FVector2 worstNormal(0,0);
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

			FVector2 v1 = link->seg.a;
			FVector2 v2 = link->seg.b;
			FVector2 normal = link->seg.normal();

			// distance to axis
			int dist = (int)DotProduct(plrPos - v1, normal) >> FRACBITS;
			
			if (dist < 0 && testSec != nav)
				continue; // can't have fallen off a cliff in a neighbor sector

			ExtendSegment(v1, v2, m_radius);
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

	FVector2 avoidForce(0, 0);

	if (worstDist < SAFE_CLIFF_DIST) {
		cliffDist = worstDist;
		avoidForce = worstNormal;
	}
	else {
		cliffDist = 9999;
	}

	//FVector3 headPos = FVector3(plrPos.X, plrPos.Y, pActor->z + (56 << FRACBITS));
	//FVector3 dir = FVector3(avoidForce.X, avoidForce.Y, 0);
	//fixed_t scale = 200 << FRACBITS;
	//draw_debug_line(headPos, headPos + dir * scale, pActor);

	return avoidForce;
}

void CWootBot::UpdatePositionFlags() {
	m_routeController.m_navid = g_wb_nav.get_nav_id(m_pPlayer);

	stateFlags &= ~(FL_WBOT_FLYING | FL_WBOT_ON_ELEV | FL_WBOT_OVERHANG);
	if (m_routeController.m_navCur) {
		NavSector& nav = g_wb_nav.mesh.nodes[m_routeController.m_navid];
		if (m_origin.Z > nav.getFloorZ())
			stateFlags |= m_onGround ? FL_WBOT_OVERHANG : FL_WBOT_FLYING;
		if (nav.sector->isFloorMoving())
			stateFlags |= FL_WBOT_ON_ELEV;
	}
}

FVector3 CWootBot::GetViewPos() {
	return m_origin + FVector3(0, 0, m_viewHeight);
}

fixed_t CWootBot::GetDistance(FVector2 p) {
	return (p - m_origin).Length();
}

int CWootBot::GetSpeed2D() {
	// TODO: why is this conversion to cmd speeds weird?
	return (FVector2(m_velocity.X, m_velocity.Y).Length() / FRACUNIT) * 8.0f;
}

bool CWootBot::FindGoal() {

	if (m_autoWinMap) {
		PushLevelEndGoal();
	}
	else if (m_followPlayer) {
		int thisSubId = g_wb_nav.get_nav_id((AActor*)pActor);
		AActor* player = find_followable_player(thisSubId);
	}

	return m_routeController.HasRoute();
}

bool CWootBot::PushLevelEndGoal() {
	m_routeController.CancelRoute();

	for (int i = 0; i < g_map.numlines; i++) {
		MapLine& line = g_map.lines[i];
		if (line.isLevelExit()) {
			if (PushGoal(BotGoal(g_map.get_linedef_goal_action(&line), i), NULL)) {
				return true;
			}
		}
	}

	AActor* boss = find_boss_brain();
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
	if (line->isLockedDoor() && !g_map.CheckKeys((AActor*)pActor, line)) {
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
		if (line && !(g_map.get_linedef_move_flag(line) & movementNeeded)) {
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
	m_nextThink = get_game_tics() + 7; // failing lots of goals at once could cause lag

	if (m_goals.empty()) {
		return;
	}

	BotGoal& curGoal = m_goals[m_goals.size() - 1];
	m_routeController.CancelRoute();

	DebugPrint(VarArgs("FAILED %s '%s'\n", m_goals.size() == 1 ? "goal" : "subgoal", curGoal.desc().c_str()));

	if (m_routeController.m_freezeOnGoalFail) {
		freeze_player(m_pPlayer, true);
	}

	bool bubbleFailure = curGoal.required;

	if ((!bubbleFailure || m_goals.size() == 1)) {
		if (++goalFailCounter > 10) {
			DebugPrint("I can't reach any goals from here! Time to die.\n");
			kill_actor((AActor*)pActor);
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
	if (get_game_tics() - m_lastUse < ticsBetweenUses) {
		return;
	}

	m_lButtons |= IN_USE;
	m_lastUse = get_game_tics();
}

void CWootBot::Attack() {
	m_lButtons |= IN_ATTACK;
	m_combatController.m_lastAttack = get_game_tics();
}

void CWootBot::HandleLineActivation(MapLine* line, AActor* activator) {
	if (line->special() && line->isLockedDoor()) {
		if (!g_map.CheckKeys(activator, line)) {
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
