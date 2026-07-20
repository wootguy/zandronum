#include "wb_bot.h"
#include "wb_nav.h"
#include "wb_map.h"
#include "wb_util.h"
#include "wb_debug.h"
#include "botcommands.h"
#include "sv_commands.h"
#include "c_dispatch.h"
#include "network.h"
#include "d_event.h"
#include "p_enemy.h"
#include "p_local.h"
#include "po_man.h"
#include "p_trace.h"
#include "p_lnspec.h"
#include "a_keys.h"
#include "actor.h"
#include "m_cheat.h"
#include <stdlib.h>
#include <time.h>
#include <unordered_set>
#include <unordered_map>
#include <string>

using namespace std;

CWootBot::CWootBot(const char* pszName, const char* pszTeamName, ULONG ulPlayerNum)
	: CSkullBot(pszName, pszTeamName, ulPlayerNum), m_routeController(this), m_combatController(this) {
	
	m_fov = ANGLE_180;
	m_bForwardMovePersist = true;
	m_bSideMovePersist = true;
	m_debug = true;
	Reset();
}

void CWootBot::Think() {
	if (m_wasDead && m_pPlayer->health > 0) {
		Reset();
		m_wasDead = false;
	}

	UpdatePositionFlags();

	if (m_debug) {
		wbot_debug(this);
	}

	if (level.time < m_nextThink || (m_pPlayer->cheats & CF_FROZEN)) {
		return;
	}

	if (m_pPlayer->health <= 0) {
		DeadThink();
		return;
	}

	m_lButtons = 0;

	m_combatController.Think();

	if (!pActor->target) {
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
	pActor = m_pPlayer->mo;
	pActor->target = NULL;
	m_routeController = CBotRouteController(this);
	m_combatController = CBotCombatController(this);
	m_goals.clear();
	m_lastUse = 0;
	stateFlags = 0;
	m_forwardMove = 0;
	m_sideMove = 0;
	m_nextThink = 0;
	rushLinkId = -1;
}

void CWootBot::DebugPrint(const char* msg) {
	if (m_debug) {
		SERVERCOMMANDS_Print(msg, PRINT_CHAT);
		Printf(msg);
	}
}

void CWootBot::DeadThink() {
	// tap a button to respawn
	m_lForwardMove = 0;
	m_lSideMove = 0;
	m_lButtons ^= BT_ATTACK;
	m_wasDead = true;
}

void CWootBot::IdleThink() {
	m_forwardMove = 0;
	m_sideMove = 0;
	pActor->pitch = 0;
	m_lButtons |= BT_CROUCH;

	if (rand() % 20 == 0) {
		pActor->angle = (rand() % 360) * ANGLE_1;
	}
}

void CWootBot::GoalActionThink() {
	BotGoal& goal = *CurrentGoal();
	NavSector& nav = m_routeController.m_navIdeal ? *m_routeController.m_navIdeal
		: g_wb_nav.mesh[m_routeController.m_navid];

	int goalSector = goal.getNavId();

	if (m_routeController.m_navid != goalSector && m_routeController.pretendRouteSector != goalSector) {
		// actor may be routed here but technically in an adjacent sector
		bool actorReachable = nav.touches(goal.actor);

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
		PopGoal(); // nothing to do
		break;
	case WBOT_GOAL_ACTION_USE: {
		int useDist = (pActor->UseRange >> FRACBITS) - 1;
		if (MoveTo(goal.pos(), useDist)) {
			Use();
		}
		break;
	}
	case WBOT_GOAL_ACTION_TOUCH:
		if (MoveTo(goal.pos(), goal.touchDistance(pActor))) {
			if (goal.lineid == -1)
				PopGoal(); // only lines are hooked and pop goals automatically
		}
		break;
	case WBOT_GOAL_ACTION_CROSS: {
		if (goal.lineid >= 0) {
			// move through the line to the backside of it
			line_t* line = &lines[goal.lineid];
			FVector2 backDir = getLineBackDir(line);
			FVector2 backGoal = getLineCenter(line) + backDir * 32;

			fixed_t dist = P_AproxDistance(pActor->x - (fixed_t)backGoal.X, pActor->y - (fixed_t)backGoal.Y);

			// be careful not to miss skinny lines
			int speed = getLineLength(line) > 32 ? RUN_SPEED : RUN_SPEED / 4;

			MoveTo(backGoal, 16, speed);
		}
		else {
			DebugPrint("Can't cross an actor as a goal!\n");
			PopGoal();
		}
		break;
	}
	case WBOT_GOAL_ACTION_SHOOT: {
		int shootRange = 200;

		if (m_pPlayer->ReadyWeapon) {
			WeaponInfo& info = g_wbot_weapon_info[m_pPlayer->ReadyWeapon->GetClass()->TypeName.GetChars()];
			shootRange = info.maxRange;
		}

		MoveTo(goal.pos(), 100);

		FTraceResults tr;
		TraceAhead(shootRange, FVector3(0, 0, m_pPlayer->viewheight), false, &tr);
		if (tr.Line && (tr.Line - lines) == goal.lineid) {
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
	FVector2 curPos = FVector2(pActor->x, pActor->y);
	fixed_t movedDist = (curPos - lastPos).Length();
	lastPos = curPos;

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
	POS_t lookPos = { (fixed_t)pos.X, (fixed_t)pos.Y, (fixed_t)pos.Z };
	fixed_t viewZ = pActor->z + m_pPlayer->viewheight;
	fixed_t dist = P_AproxDistance(pActor->x - lookPos.x, pActor->y - lookPos.y);
	pActor->pitch = -(SDWORD)R_PointToAngle2(0, viewZ, dist, lookPos.z);
	pActor->angle = R_PointToAngle2(pActor->x, pActor->y, lookPos.x, lookPos.y);
}

bool CWootBot::TraceAhead(int dist, FVector3 offset, bool ignoreMonsters, FTraceResults* tr) {
	FVector3 forward, right;
	MakeVectors(pActor->angle, forward, right);
	FVector3 start = FVector3(pActor->x, pActor->y, pActor->z) + offset;
	fixed_t testDist = dist << FRACBITS;

	return Trace((fixed_t)start.X, (fixed_t)start.Y, (fixed_t)start.Z, pActor->Sector,
		(fixed_t)forward.X, (fixed_t)forward.Y, 0, testDist, ignoreMonsters ? 0 : 0xffffffff,
		ML_BLOCKEVERYTHING, pActor, *tr);
}

bool CWootBot::MoveTo(FVector2 pos, int radius, int speed) {
	float z = (float)(pActor->z + m_pPlayer->viewheight);
	AimAtPos(FVector3(pos.X, pos.Y, z));

	FVector2 wantDir = pos - FVector2(pActor->x, pActor->y);
	wantDir.MakeUnit();

	// jump over walls and activate things in front of us
	FTraceResults tr;
	int useDist = (pActor->UseRange >> FRACBITS) - 1;
	if (TraceAhead(useDist, FVector3(0, 0, STEP_HEIGHT << FRACBITS), true, &tr)) {
		int d = (tr.Fraction / (float)FRACUNIT) * useDist;

		// jump if not too far and this isn't an impassable wall
		if (tr.Line->backsector && d < 32) {
			fixed_t curZ = pActor->Sector->floorplane.ZatPoint(pActor->x, pActor->y);
			fixed_t backZ = tr.Line->backsector->floorplane.ZatPoint(tr.X, tr.Y);
			int jumpHeight = (backZ - curZ) >> FRACBITS;

			// ...and it's possible and necessary to jump up
			if (jumpHeight > STEP_HEIGHT && jumpHeight <= JUMP_HEIGHT)
				m_lButtons |= BT_JUMP;
		}

		// only use walls that aren't already moving, so doors aren't closed while opening
		bool lineIsMoving = tr.Line && tr.Line->backsector &&
			(tr.Line->backsector->floordata || tr.Line->backsector->ceilingdata);

		if (!lineIsMoving) {
			// activate any triggered line to fix face rubbing on walls when the bot is failing
			// to get to a tiny sector in front of a door/button.
			int action = g_wb_mapinfo.get_linedef_goal_action(tr.Line);

			if (action == WBOT_GOAL_ACTION_USE)
				Use();

			if (action == WBOT_GOAL_ACTION_SHOOT)
				Attack();
		}
	}

	// combine desired vector with avoidance vectors
	FVector2 avoidCornersDir = AvoidCornersVector(wantDir);
	FVector2 avoidLedgeDir = AvoidLedges(pActor, m_cliffDist);
	FVector2 moveDir = wantDir + avoidCornersDir;

	if (!(stateFlags & FL_WBOT_JUMPING)) {
		if (m_cliffDist < 0) {
			// do whatever it takes to get back on solid ground.
			// For best results, speed should be 0.5 or less, so correction force can recover from any velocity
			moveDir = avoidLedgeDir;
			speed = std::min(RUN_SPEED, speed * 2);
		}
		else {
			moveDir += avoidLedgeDir * 0.5f;
		}
	}

	moveDir.MakeUnit();
	moveDir *= speed;

	// convert directinal vectors to forward/strafe movents relative to the look direction
	FVector3 forward, right;
	MakeVectors(pActor->angle, forward, right);
	forward.MakeUnit();
	right.MakeUnit();
	m_forwardMove = DotProduct(moveDir, forward);
	m_sideMove = DotProduct(moveDir, right);

	fixed_t dist = P_AproxDistance(pActor->x - (fixed_t)pos.X, pActor->y - (fixed_t)pos.Y);
	return dist < (radius << FRACBITS);
}

FVector2 CWootBot::AvoidCornersVector(FVector2 wantDir) {
	// strafe around objects/walls partially blocking the way
	fixed_t zTest = (STEP_HEIGHT + 1) << FRACBITS;
	FVector3 wantDirf(wantDir.X * (1 << FRACBITS), wantDir.Y * (1 << FRACBITS), 0);
	FVector3 viewPos = FVector3(pActor->x, pActor->y, pActor->z + zTest);
	fixed_t testDist = 32 << FRACBITS;
	fixed_t rightOfs = 16 << FRACBITS;
	FVector3 rightDir(wantDir.Y, -wantDir.X, 0);
	FVector3 rightPos = viewPos + rightDir * rightOfs;
	FVector3 leftPos = viewPos + rightDir * -rightOfs;
	FTraceResults trLeft, trRight;

	Trace(rightPos.X, rightPos.Y, rightPos.Z, pActor->Sector,
		wantDirf.X, wantDirf.Y, 0, testDist, 0xffffffff,
		ML_BLOCKEVERYTHING, pActor, trRight);

	Trace(leftPos.X, leftPos.Y, leftPos.Z, pActor->Sector,
		wantDirf.X, wantDirf.Y, 0, testDist, 0xffffffff,
		ML_BLOCKEVERYTHING, pActor, trLeft);

	//draw_debug_line(rightPos, rightPos + wantDirf * 32, pActor);
	//draw_debug_line(leftPos, leftPos + wantDirf * 32, pActor);

	if (trLeft.Fraction != trRight.Fraction) {
		return trRight.Fraction < trLeft.Fraction ? -rightDir : rightDir;
	}

	return FVector2(0, 0);
}

FVector2 CWootBot::AvoidLedges(AActor* actor, int& cliffDist) {
	//NavSector& nav = g_wb_nav.mesh[m_navid];
	//FVector2 plrPos(pActor->x, pActor->y);
	int subid = R_PointInSubsector(actor->x, actor->y) - subsectors;
	NavSector* nav = &g_wb_nav.mesh[subid];
	FVector2 plrPos(actor->x, actor->y);

	int targetNav = -1;
	int idealNav = -1;
	if (m_routeController.m_route.size() > 0) {
		idealNav = m_routeController.m_route[0];
		nav = &g_wb_nav.mesh[idealNav];
	}
	if (m_routeController.m_route.size() > 1) {
		targetNav = m_routeController.m_route[1];
	}

	// get nearby sectors in case nearest ledge is at the corner of the current
	std::vector<NavSector*> sectors;
	if (nav->hasCliffs)
		sectors.push_back(nav);
	for (NavSectorLink& link : nav->links) {
		if (link.isCliff || link.isTeleport)
			continue;

		if (link.target->hasCliffs) {
			sectors.push_back(link.target);
		}
	}

	FVector2 worstNormal(0,0);
	int worstDist = INT_MAX;
	int worsetSec = -1;
	int worstLink = -1;

	for (NavSector* testSec : sectors) {
		for (NavSectorLink& link : testSec->links) {
			if (!link.isCliff)
				continue;

			if (link.target->id == targetNav || link.target->id == idealNav)
				continue; // don't back off from segments that must be crossed

			seg_t* seg = link.seg;
			FVector2 v1(seg->v1->x, seg->v1->y);
			FVector2 v2(seg->v2->x, seg->v2->y);
			FVector2 edge = v2 - v1;
			FVector2 normal(edge.Y, -edge.X);
			normal.MakeUnit();

			// distance to axis
			int dist = (int)DotProduct(plrPos - v1, normal) >> FRACBITS;
			
			if (dist < 0 && testSec != nav)
				continue; // can't have fallen off a cliff in a neighbor sector

			ExtendSegment(v1, v2, actor->radius);
			if (!PointAlignedSegment(plrPos, v1, v2))
				continue; // off to the side of this segment

			if (dist < worstDist) {
				worstDist = dist;
				worstNormal = normal;
				worsetSec = testSec->id;
				worstLink = link.id;
			}
		}
	}

	FVector2 avoidForce(0, 0);

	cliffDist = worstDist;

	if (worstDist < SAFE_CLIFF_DIST) {
		avoidForce = worstNormal;
	}

	//FVector3 headPos = FVector3(plrPos.X, plrPos.Y, pActor->z + (56 << FRACBITS));
	//FVector3 dir = FVector3(avoidForce.X, avoidForce.Y, 0);
	//fixed_t scale = 200 << FRACBITS;
	//draw_debug_line(headPos, headPos + dir * scale, pActor);

	return avoidForce;
}

void CWootBot::UpdatePositionFlags() {
	m_routeController.m_navid = g_wb_nav.get_nav_id(pActor);

	stateFlags &= ~(FL_WBOT_FLYING | FL_WBOT_ON_ELEV);
	if (m_routeController.m_navCur) {
		NavSector& nav = g_wb_nav.mesh[m_routeController.m_navid];
		if (pActor->z > nav.getFloorZ())
			stateFlags |= FL_WBOT_FLYING;
		if (nav.sector()->floordata)
			stateFlags |= FL_WBOT_ON_ELEV;
	}
}

FVector3 CWootBot::GetViewPos() {
	return FVector3(pActor->x, pActor->y, pActor->z + pActor->ViewHeight);
}

fixed_t CWootBot::GetDistance(FVector2 p) {
	return P_AproxDistance(p.X - pActor->x, p.Y - pActor->y);
}

FVector3 CWootBot::GetVelocity() {
	return FVector3(pActor->velx, pActor->vely, pActor->velz);
}

int CWootBot::GetSpeed2D() {
	// TODO: why is this conversion to cmd speeds weird?
	return (FVector2(pActor->velx, pActor->vely).Length() / FRACUNIT) * 8.0f;
}

bool CWootBot::FindGoal() {
	int thisSubId = g_wb_nav.get_nav_id(pActor);

	AActor* player = NULL;
	for (int i = 0; i < MAXPLAYERS; i++)
	{
		if (!playeringame[i])
			continue;

		AActor* actor = players[i].mo;
		if (!actor || actor->player->bIsBot)
			continue;

		if (actor->player->cheats & (CF_NOCLIP | CF_NOCLIP2))
			continue; // for testing

		if (thisSubId == g_wb_nav.get_nav_id(actor))
			continue; // already with this player

		PushGoal(BotGoal(WBOT_GOAL_ACTION_MOVE_TO, actor), NULL);
	}

	return m_routeController.HasRoute();
}

bool CWootBot::PushGoal(BotGoal& goal, NavSectorLink* purposeLink) {
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

	DebugPrint(VarArgs("New goal: %s\n", goal.desc().c_str()));
	m_goals.push_back(goal);
	m_goals[m_goals.size() - 1].purposeLink = purposeLink;

	// also add key goals needed to use this line, if missing
	line_t* line = goal.lineid >= 0 ? &lines[goal.lineid] : NULL;
	if (line && line->special == Door_LockedRaise && !P_CheckKeys(pActor, line->args[3], false)) {
		vector<BotGoal> keyGoals;
		unordered_set<int> allBlockedPaths = m_routeController.GetBlockedPaths();
		g_wb_nav.get_key_goals_for_line(pActor, line, keyGoals, &allBlockedPaths);

		DebugPrint(VarArgs("    Adding locked line subgoals.\n"));

		for (BotGoal& keyGoal : keyGoals) {
			m_goals.push_back(keyGoal);
		}
	}

	return m_routeController.RouteToGoal();
}

bool CWootBot::SelectGoal(vector<BotGoal>& goals, NavSectorLink* purposeLink, bool randomize) {
	BotGoal* bestGoal = NULL;
	vector<BotGoal*> validGoals;

	int unblockSector = purposeLink->target->id;

	for (int i = 0; i < goals.size(); i++) {
		BotGoal& goal = goals[i];
		if (!goal.valid())
			continue;
		int subid = goal.getNavId();

		vector<int> route = m_routeController.RouteToSector(subid, unblockSector);

		if (subid == purposeLink->parent->id || route.size()) {
			if (randomize) {
				validGoals.push_back(&goal);
			} else {
				bestGoal = &goal;
				break;
			}
		}
	}

	if (randomize && validGoals.size()) {
		bestGoal = validGoals[rand() % validGoals.size()];
	}

	if (bestGoal && PushGoal(*bestGoal, purposeLink)) {
		return true;
	}

	return false;
}

void CWootBot::PopGoal() {
	if (m_goals.empty()) {
		m_routeController.CancelRoute();
		DebugPrint("No goal to pop\n");
		return;
	}

	BotGoal& goal = m_goals[m_goals.size() - 1];

	rushLinkId = -1;
	stateFlags &= ~FL_WBOT_RUSHING;

	int purposeLinkId = goal.purposeLink ? goal.purposeLink->id : -1;
	NavSector* purposeNav = goal.purposeLink ? goal.purposeLink->target : NULL;
	if (purposeNav && (purposeNav->getMoveFlags() & FL_SECTOR_MOVE_TIMED)) {
		// the purpose of this goal was to move a timed sector. Better hurry before that sector resets!
		stateFlags |= FL_WBOT_RUSHING;
		rushLinkId = goal.purposeLink->id;
	}

	DebugPrint(VarArgs("Finished goal: %s\n", goal.desc().c_str()));
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

void CWootBot::Use(int ticsBetweenUses) {
	if (level.time - m_lastUse < ticsBetweenUses) {
		return;
	}

	m_lButtons |= BT_USE;
	m_lastUse = level.time;
}

void CWootBot::Attack() {
	m_lButtons |= BT_ATTACK;
	m_combatController.m_lastAttack = level.time;
}

void CWootBot::HandleLineActivation(line_t* line, AActor* activator) {
	int lineid = line - lines;

	if (line->special && line->special == Door_LockedRaise) {
		if (!P_CheckKeys(activator, line->args[3], false)) {
			return; // door wasn't actually opened
		}
	}

	int popIdx = -1;
	for (int i = 0; i < m_goals.size(); i++) {
		if (m_goals[i].lineid == lineid) {
			popIdx = i;
			break;
		}
	}

	if (popIdx != -1) {
		// pop all subgoals that were for activating this line
		int numSubPop = (m_goals.size() - popIdx) - 1;

		if (numSubPop > 0)
			DebugPrint(VarArgs("Something activated line %d! Popping %d subgoals\n", lineid, numSubPop));

		while (numSubPop--) {
			m_goals.pop_back();
		}

		PopGoal(); // only final pop does rerouting logic
	}
}
