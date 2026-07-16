#include "wbot.h"
#include "botcommands.h"
#include "sv_commands.h"
#include "c_dispatch.h"
#include "network.h"
#include "d_event.h"
#include "p_enemy.h"
#include "p_local.h"
#include "po_man.h"
#include "wnav.h"
#include "p_trace.h"
#include "p_lnspec.h"
#include "a_keys.h"
#include "actor.h"

#include <stdlib.h>
#include <time.h>
#include <unordered_set>
#include <unordered_map>
#include <string>

using namespace std;

#define RUN_SPEED 100 // max move speed allowed before the server kicks you
#define MAX_NODE_LINKS 16

extern FRandom g_RandomBotAimSeed;

struct WeaponInfo {
	int priority;
	int minRange;
	int idealRange;
	int maxRange;
};

unordered_map<string, WeaponInfo> g_wbot_weapon_info = {
	{"Fist",			{0,  0,   0,   64}},
	{"Chainsaw",		{1,  0,   0,   64}},
	{"Pistol",			{2,  0,   200, 4000}},
	{"Shotgun",			{3,  0,   200, 2000}},
	{"Chaingun",		{4,  0,   400, 4000}},
	{"Minigun",			{4,  0,   400, 4000}},
	{"GrenadeLauncher",	{5,  200, 400, 2000}},
	{"RocketLauncher",	{5,  200, 500, 4000}},
	{"SuperShotgun",	{6,  0,   200, 2000}},
	{"PlasmaRifle",		{7,  0,   400, 4000}},
	{"Railgun",			{7,	 0,   400, 4000}},
	{"BFG9000",			{8,  0,   200, 2000}},
	{"BFG10K",			{8,  0,   200, 2000}},
};

char* VarArgs(const char* format, ...)
{
	va_list		argptr;
	static char		string[1024];

	va_start(argptr, format);
	vsnprintf(string, 1024, format, argptr);
	va_end(argptr);

	return string;
}

void init_wootbots() {
	static int lastInit;
	static int lastInitTime;
	if (lastInit != level.levelnum || lastInitTime == 0 || lastInitTime > level.time) {
		lastInit = level.levelnum;
		lastInitTime = level.time;

		srand((unsigned int)time(NULL));

		g_wbot_nav.generate_node_graph();
	}
}

AActor* getAnyPlayer() {
	AActor* player = NULL;
	for (int i = 0; i < MAXPLAYERS; i++)
	{
		if (!playeringame[i])
			continue;

		AActor* actor = players[i].mo;
		if (!actor || actor->player->bIsBot)
			continue;
		
		return actor;
	}

	return NULL;
}

// get direction pointing behind the given linedef
FVector3 getLineBackDir(line_t* line) {
	fixed_t dx = line->v2->x - line->v1->x;
	fixed_t dy = line->v2->y - line->v1->y;
	double len = sqrt((double)dx * dx + (double)dy * dy);
	return FVector3(-(double)dy / len, (double)dx / len, 0);
}

FVector3 getLineCenter(line_t* line) {
	return FVector3((line->v1->x + line->v2->x) * 0.5, (line->v1->y + line->v2->y) * 0.5, 0);
}

int getLineLength(line_t* line) {
	return (int)(FVector2(line->v1->x, line->v1->y) - FVector2(line->v2->x, line->v2->y)).Length() >> FRACBITS;
}

void MakeVectors(angle_t angle, FVector3& forward, FVector3& right) {
	fixed_t fsine = finesine[angle >> ANGLETOFINESHIFT];
	fixed_t fcosine = finecosine[angle >> ANGLETOFINESHIFT];
	forward = FVector3(fcosine, fsine, 0);
	right = FVector3(fsine, -fcosine, 0);
}

std::string BotGoal::desc() {
	std::string thingName;
	if (actor) {
		thingName = actor->GetClass()->TypeName.GetChars();
	}
	else if (lineid) {
		thingName = "Linedef " + to_string(lineid);
	}

	thingName += " in sector " + to_string(getNavId());

	string blockerStr;
	for (const int& id : blockers) {
		blockerStr += " " + to_string(id);
	}
	if (blockerStr.size()) {
		thingName += "   (blocked at " + blockerStr + ")";
	}

	switch (action) {
	case WBOT_GOAL_ACTION_MOVE_TO:
		return "Move to " + thingName;
	case WBOT_GOAL_ACTION_USE:
		return "Use " + thingName;
	case WBOT_GOAL_ACTION_TOUCH:
		return "Touch " + thingName;
	case WBOT_GOAL_ACTION_CROSS:
		return "Cross " + thingName;
	case WBOT_GOAL_ACTION_SHOOT:
		return "Shoot " + thingName;
	}

	return "??? " + thingName;
}

int BotGoal::getNavId() {
	if (actor) {
		return g_wbot_nav.get_nav_id(actor);
	}
	else if (lineid >= 0) {
		auto subs = g_wbot_nav.line_subsectors.find(lineid);
		if (subs != g_wbot_nav.line_subsectors.end()) {
			return subs->second;
		}

		Printf("Failed to find subsector for line %d\n", lineid);
	}
	else {
		Printf("Routing not implemented for this type of goal\n");
	}

	return -1;
}

FVector3 BotGoal::pos() {
	if (actor) {
		return FVector3(actor->x, actor->y, actor->Sector->floorplane.ZatPoint(actor->x, actor->y));
	}
	else if (lineid >= 0) {
		line_t& line = lines[lineid];
		fixed_t x = (line.v1->x + line.v2->x) / 2;
		fixed_t y = (line.v1->y + line.v2->y) / 2;
		fixed_t z = line.frontsector->floorplane.ZatPoint(x, y);
		return FVector3(x, y, z);
	}

	Printf("Goal has no actor nor lineid\n");
	return FVector3(0, 0, 0);
}

int BotGoal::touchDistance(AActor* toucher) {
	if (actor) {
		return ((actor->radius + toucher->radius) >> FRACBITS) - 1; // subtracted 1 unit just in case
	}
	else if (lineid >= 0) {
		return (toucher->radius >> FRACBITS) + 1; // added 1 in case wall is solid and you can't go inside it
	}
}

CWootBot::CWootBot(const char* pszName, const char* pszTeamName, ULONG ulPlayerNum)
	: CSkullBot(pszName, pszTeamName, ulPlayerNum) {
	
	m_fov = ANGLE_180;
	m_bForwardMovePersist = true;
	m_bSideMovePersist = true;
	m_debug = true;
}
CWootBot::~CWootBot() {}

void CWootBot::ParseScript() {
	init_wootbots();

	m_navid = g_wbot_nav.get_nav_id(m_pPlayer->mo);

	// level changed
	if (lastInit != level.levelnum) {
		lastInit = level.levelnum;
		m_goals.clear();
		CancelRoute();
		m_lastAttack = 0;
		stateFlags = 0;
		m_targetLastSeenTic = 0;
		stuckPath = -1;
	}

	if (m_debug) {
		ShowDebugInfo();
	}

	if (m_pPlayer->health <= 0) {
		DeadThink();
		return;
	}

	m_lButtons = 0;

	CombatThink();

	if (!m_pPlayer->mo->target) {
		if (m_route.empty()) {
			if (m_goals.empty()) {
				FindGoal();

				if (m_goals.empty()) {
					IdleThink();
				}
			}
			else {
				GoalActionThink();
			}
		}
		else {
			RouteThink();
		}
	}

	m_lForwardMove = static_cast<LONG>(0x32 * (m_forwardMove / 100.0f));
	m_lSideMove = static_cast<LONG>(0x32 * (m_sideMove / 100.0f));
}

void CWootBot::ShowDebugInfo() {
	g_wbot_nav.draw_nodes(m_pPlayer->mo);

	int thisSubId = g_wbot_nav.get_nav_id(m_pPlayer->mo);

	string routeStr = "Route: " + to_string(thisSubId);
	if (pretendRouteSector >= 0) {
		routeStr += " (pretend " + to_string(pretendRouteSector) + " )";
	}
	routeStr += " -> ";
	for (int i = 0; i < m_route.size(); i++) {
		if (i != 0)
			routeStr += " ";
		if (i > 0 && i % 10 == 0) {
			routeStr += "\n                     ";
		}
		routeStr += to_string(m_route[i]);
	}
	
	AActor* player = getAnyPlayer();
	if (!player)
		return;

	string navInfo;
	{
		int plrnavid = g_wbot_nav.get_nav_id(player);
		navInfo = "Sector " + to_string(plrnavid) + ":";
		NavSector& nav = g_wbot_nav.nav_sectors[plrnavid];

		navInfo += "\n   Triggers:";
		for (int i = 0; i < nav.triggers.size(); i++) {
			navInfo += "\n      " + nav.triggers[i].desc();
		}

		navInfo += "\n   Links:";
		for (int i = 0; i < nav.links.size(); i++) {
			NavSectorLink& link = nav.links[i];
			NavSector& targ = g_wbot_nav.nav_sectors[link.target];
			string arrow = link.blocked(player) ? " -X> " : " --> ";
			navInfo += "\n      " + to_string(link.id) + arrow + to_string(link.target);
			if (targ.triggers.size())
				navInfo += " (" + to_string(targ.triggers.size()) + " T)";
		}

		FVector3 forward, right;
		MakeVectors(player->angle, forward, right);
		FVector3 start = FVector3(player->x, player->y, player->z + player->player->viewheight);
		fixed_t testDist = 64 << FRACBITS;
		sector_t* sector = R_PointInSubsector((fixed_t)start.X, (fixed_t)start.Y)->sector;
		FTraceResults tr;
		if (Trace((fixed_t)start.X, (fixed_t)start.Y, (fixed_t)start.Z, sector,
			(fixed_t)forward.X, (fixed_t)forward.Y, 0, testDist, 0,
			ML_BLOCKEVERYTHING | ML_BLOCKHITSCAN, NULL, tr))
		{
			line_t* line = tr.Line;
			navInfo += "\nLine " + to_string(tr.Line - lines) + ":";

			if (line->special) {
				navInfo += "\n   Special: " + to_string(line->special) + "\n   Tags:";
				for (int i = 0; i < 5; i++)
					navInfo += " " + to_string(line->args[i]);
			}
		}
	}

	string stateStr = "State:";
	if (stateFlags & FL_WBOT_WAITING_ELEV) { stateStr += " WAIT_ELEV"; }
	if (stateFlags & FL_WBOT_WAITING_DOOR) { stateStr += " WAIT_DOOR"; }

	string stuckStr = "Stuck: " + to_string(stuckCounter);

	string enemyStr = "Enemy: <none>";
	if (m_pPlayer->mo->target) {
		AActor* targ = m_pPlayer->mo->target;
		fixed_t dist = P_AproxDistance(m_pPlayer->mo->x - targ->x, m_pPlayer->mo->y - targ->y);
		enemyStr = string("Enemy: ") + m_pPlayer->mo->target->GetClass()->TypeName.GetChars()
			+ ", Dist: " + to_string(dist >> FRACBITS);
	}

	string weaponStr = "Weapons:";
	for (AInventory* item = m_pPlayer->mo->Inventory; item != NULL; item = item->Inventory) {
		if (item->IsKindOf(RUNTIME_CLASS(AWeapon))) {
			AWeapon* weapon = static_cast<AWeapon*>(item);

			bool hasAmmo = weapon->Ammo1 && weapon->Ammo1->Amount > 0 && weapon->Ammo1->Amount >= weapon->MinAmmo1;
			if (!weapon->AmmoType1 || hasAmmo) {
				string wepname = weapon->GetClass()->TypeName.GetChars();
				WeaponInfo& info = g_wbot_weapon_info[wepname];
				weaponStr += "\n   " + wepname + " "
					+ to_string(weapon->Ammo1 ? weapon->Ammo1->Amount : 0)
					+ " p" + to_string(info.priority);
					//+ ", [" + to_string(info.minRange) + "," + to_string(info.idealRange) + "," + to_string(info.maxRange) + "] range";
				if (m_pPlayer->ReadyWeapon == weapon) {
					weaponStr += " <--";
				}
			}
		}
	}

	string goalStr = "Goals:";
	for (int i = 0; i < m_goals.size(); i++) {
		goalStr += "\n   " + m_goals[i].desc();
	}

	string botInfo = enemyStr + "\n" + weaponStr + "\n" + routeStr + "\n"
		+ stateStr + "\n" + stuckStr + "\n" + goalStr;

	SERVERCOMMANDS_PrintHUDMessage(navInfo.c_str(), 0.95f, 0.5f, 0, 0, 0, CR_RED, 1.0f, 0, 0, "SmallFont", MAKE_ID('W', 'N', 'A', 'V'));
	SERVERCOMMANDS_PrintHUDMessage(botInfo.c_str(), 0, 0.5f, 0, 0, 0, CR_RED, 1.0f, 0, 0, "SmallFont", MAKE_ID('W', 'B', 'O', 'T'));
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
}

void CWootBot::IdleThink() {
	m_forwardMove = 0;
	m_sideMove = 0;
	m_pPlayer->mo->pitch = 0;
	m_lButtons |= BT_CROUCH;

	if (rand() % 20 == 0) {
		m_pPlayer->mo->angle = (rand() % 360) * ANGLE_1;
	}
}

void CWootBot::CancelRoute() {
	m_route.clear();
	pretendRouteSector = -1;
	stuckCounter = 0;
}

void CWootBot::GoalActionThink() {
	BotGoal& goal = m_goals[m_goals.size() - 1];

	int goalSector = goal.getNavId();

	if (m_navid != goalSector && pretendRouteSector != goalSector) {

		if (goal.action == WBOT_GOAL_ACTION_CROSS && goal.lineid >= 0) {
			if (m_pPlayer->mo->Sector == lines[goal.lineid].backsector) {
				// moving to a different sector was required for this goal
				PopGoal();
				return;
			}
		}

		// route was cancelled or the target moved. Route to it again.
		DebugPrint("Goal moved or movement failed. Rerouting...\n");
		RouteToGoal();
		return;
	}

	m_forwardMove = 0;
	m_sideMove = 0;

	if (goal.lineid >= 0) {
		if (lines[goal.lineid].special == 0) {
			// single-use special is no longer usable
			DebugPrint("Single-use special is no longer usable\n");
			PopGoal();
		}
	}
	
	switch (goal.action) {
	default:
		DebugPrint(VarArgs("Unknown goal action type %d\n", goal.action));
	case WBOT_GOAL_ACTION_MOVE_TO:
		PopGoal(); // nothing to do
		break;
	case WBOT_GOAL_ACTION_USE: {
		int useDist = (m_pPlayer->mo->UseRange >> FRACBITS) - 1;
		if (MoveTo(goal.pos(), useDist)) {
			m_lButtons |= BT_USE;
			PopGoal();
		}
		break;
	}
	case WBOT_GOAL_ACTION_TOUCH:
		if (MoveTo(goal.pos(), goal.touchDistance(m_pPlayer->mo))) {
			PopGoal();
		}
		break;
	case WBOT_GOAL_ACTION_CROSS: {
		if (goal.lineid >= 0) {
			// move through the line to the backside of it
			line_t* line = &lines[goal.lineid];
			FVector3 backDir = getLineBackDir(line);
			FVector3 backGoal = getLineCenter(line) + backDir * 32;

			fixed_t dist = P_AproxDistance(m_pPlayer->mo->x - (fixed_t)backGoal.X, m_pPlayer->mo->y - (fixed_t)backGoal.Y);

			// be careful not to miss skinny lines
			int speed = getLineLength(line) > 32 ? RUN_SPEED : RUN_SPEED / 4;

			if (MoveTo(backGoal, 16, speed)) {
				PopGoal();
			}
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
			// wait a bit in case the gun needs to reload
			if (level.time - m_lastAttack > 35) {
				Attack();
				PopGoal();
			}
		}
		break;
	}
	}

	if (StuckThink(1000)) {
		RouteToGoal();
	}
}

void CWootBot::RouteThink() {
	const int routeSpeed = 100;

	m_forwardMove = 0;
	m_sideMove = 0;
	stateFlags = 0;

	if (m_route.size() > 1) {
		if (m_navid == m_route[1]) {
			// inside the target sector. Advance the route.
			m_route.erase(m_route.begin());
			pretendRouteSector = -1;
		}
		else {
			FVector3 center = g_wbot_nav.nav_sectors[m_route[1]].pos();
			fixed_t dist = P_AproxDistance(m_pPlayer->mo->x - (fixed_t)center.X, m_pPlayer->mo->y - (fixed_t)center.Y);
			if (stuckCounter >= 200 && dist < (16 << FRACBITS)) {
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

	if (m_route.size() > 1) {	
		if (m_navid == m_route[0]) {
			NavSectorLink* link = g_wbot_nav.nav_sectors[m_navid].getLink(m_route[1]);

			if (!link)
				return;

			if (link->blocked(m_pPlayer->mo)) {
				BlockedPathThink(link);
				return;
			}

			// move to the next link
			if (MoveTo(link->pos(), 32, routeSpeed)) {
				// close enough to the link edge
				NavSector& nav = g_wbot_nav.nav_sectors[m_route[1]];
				
				if (link->isTeleport && link->seg->linedef) {
					// move behind the teleporter line edge.
					// The target sector may be in a completely different direction.
					FVector3 backDir = getLineBackDir(link->seg->linedef);
					FVector3 teleGoal = link->pos() + backDir * 200;
					MoveTo(teleGoal, 0, routeSpeed);
				}
				else {
					// move towards the target sector until we end up inside it
					MoveTo(nav.pos(), 16, routeSpeed);
				}

				// duck if unable to fit while standing
				int secHeight = nav.getHeight() >> FRACBITS;
				if (secHeight < STAND_HEIGHT) {
					m_lButtons |= BT_CROUCH;
				}
			}
		}
		else { // fell off the route
			if (g_wbot_nav.nav_sectors[m_navid].getLink(m_route[1])) {
				// slipped into an adjacent node, so just update the route
				m_route[0] = m_navid;
				return;
			}

			DebugPrint(VarArgs("Fell off the route (expected %d but got %d)\n", m_route[0], m_navid));
			CancelRoute();
		}
	}
	else if (m_route.size() == 1) {
		FVector3 centerGoal = g_wbot_nav.nav_sectors[m_route[0]].pos();
		if (MoveTo(centerGoal, 32, routeSpeed)) {
			m_route.clear(); // don't reset pretendsector in case a goal is inside it
			stuckPath = -1;
			DebugPrint("Finished route\n");
		}
	}

	if (StuckThink(500)) {
		DebugPrint("I got stuck! Cancelling route.\n");

		if (m_route.size() > 1) {
			NavSectorLink* failedLink = g_wbot_nav.nav_sectors[m_navid].getLink(m_route[1]);
			stuckPath = failedLink ? failedLink->id : -1;
		}

		CancelRoute();
	}
}

void CWootBot::BlockedPathThink(NavSectorLink* link) {
	link->blocked(m_pPlayer->mo); // debug here

	// if the blocker is moving or we're on an elevator, then be patient
	{
		sector_t* thisSector = subsectors[m_navid].sector;
		sector_t* targetSector = subsectors[link->target].sector;
		if (targetSector && (targetSector->floordata || targetSector->ceilingdata)) {
			stateFlags |= FL_WBOT_WAITING_DOOR;
			return; // wait until the blocker is done moving
		}
		else if (thisSector && (thisSector->floordata || thisSector->ceilingdata)) {
			stateFlags |= FL_WBOT_WAITING_ELEV;

			// stay centered on the elevator to avoid blocking it or falling off
			NavSector& nav = g_wbot_nav.nav_sectors[m_navid];
			FVector3 navPos = nav.pos();
			fixed_t dist = P_AproxDistance(m_pPlayer->mo->x - (fixed_t)navPos.X, m_pPlayer->mo->y - (fixed_t)navPos.Y);
			if (dist > (16 << FRACBITS))
				MoveTo(navPos, 0, RUN_SPEED / 4);
			return; // wait until the elevator is done moving
		}
	}

	string blockMsg = VarArgs("Link %d blocked!", link->id);

	BotGoal& curGoal = m_goals[m_goals.size() - 1];
	curGoal.blockers.insert(link->id);

	// don't try to route through previous paths we've been trying to unblock
	unordered_set<int> allBlockedPaths = GetBlockedPaths();

	// get keys needed to cross this line, if missing
	line_t* line = link->seg->linedef;
	if (line && line->special == Door_LockedRaise && !P_CheckKeys(m_pPlayer->mo, line->args[3], false)) {
		vector<BotGoal> keyGoals;
		g_wbot_nav.get_key_goals_for_line(m_pPlayer->mo, line, keyGoals, &allBlockedPaths);

		bool allGoalsPushed = true;
		for (BotGoal& keyGoal : keyGoals) {
			if (!PushGoal(keyGoal)) {
				allGoalsPushed = false;
			}
		}

		if (allGoalsPushed) {
			DebugPrint(VarArgs("%s Added locked door subgoals.\n", blockMsg.c_str()));
			return;
		}
	}

	// nothing is moving, try unblocking it ourselves.
	NavSector& targetNav = g_wbot_nav.nav_sectors[link->target];
	for (BotGoal& goal : targetNav.triggers) {
		int subid = goal.getNavId();

		if (subid == m_navid || RouteToSector(subid).size()) {
			if (PushGoal(goal)) {
				DebugPrint(VarArgs("%s Added unblock subgoal.\n", blockMsg.c_str()));
				return;
			}
		}
	}

	// nothing can unblock the path that stopped us. Try routing around it.
	m_route = RouteToSector(curGoal.getNavId());
	if (m_route.size()) {
		DebugPrint(VarArgs("%s Routing around the blocked path.\n", blockMsg.c_str()));
		return;
	}

	// clearing previous blocked links and try again, maybe paths got unblocked
	if (curGoal.blockers.size()) {
		curGoal.blockers.clear();
		m_route = RouteToSector(curGoal.getNavId());
		if (m_route.size()) {
			DebugPrint(VarArgs("%s Forgetting blocked paths and trying again...\n", blockMsg.c_str()));
			return;
		}
	}

	DebugPrint(VarArgs("%s Route aborted. No way to reach the goal.\n", blockMsg.c_str()));
	CancelRoute();
}

bool CWootBot::StuckThink(int maxStuck) {
	FVector2 curPos = FVector2(m_pPlayer->mo->x, m_pPlayer->mo->y);
	int movedDist = (int)(curPos - lastPos).Length() >> FRACBITS;
	lastPos = curPos;

	if ((m_forwardMove || m_sideMove) && movedDist <= 1) {
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
	fixed_t viewZ = m_pPlayer->mo->z + m_pPlayer->viewheight;
	fixed_t dist = P_AproxDistance(m_pPlayer->mo->x - lookPos.x, m_pPlayer->mo->y - lookPos.y);
	m_pPlayer->mo->pitch = -(SDWORD)R_PointToAngle2(0, viewZ, dist, lookPos.z);
	m_pPlayer->mo->angle = R_PointToAngle2(m_pPlayer->mo->x, m_pPlayer->mo->y, lookPos.x, lookPos.y);
}

bool CWootBot::TraceAhead(int dist, FVector3 offset, bool ignoreMonsters, FTraceResults* tr) {
	FVector3 forward, right;
	MakeVectors(m_pPlayer->mo->angle, forward, right);
	FVector3 start = FVector3(m_pPlayer->mo->x, m_pPlayer->mo->y, m_pPlayer->mo->z) + offset;
	fixed_t testDist = dist << FRACBITS;
	sector_t* sector = R_PointInSubsector((fixed_t)start.X, (fixed_t)start.Y)->sector;

	return Trace((fixed_t)start.X, (fixed_t)start.Y, (fixed_t)start.Z, sector,
		(fixed_t)forward.X, (fixed_t)forward.Y, 0, testDist, ignoreMonsters ? 0 : 0xffffffff,
		ML_BLOCKEVERYTHING | ML_BLOCKHITSCAN, m_pPlayer->mo, *tr);
}

bool CWootBot::MoveTo(FVector3 pos, int radius, int speed) {
	m_forwardMove = speed;
	m_sideMove = 0;
	pos.Z = (float)(m_pPlayer->mo->z + m_pPlayer->viewheight);
	AimAtPos(pos);

	// jump over short walls and open doors
	FTraceResults tr;
	if (TraceAhead(32, FVector3(0, 0, STEP_HEIGHT << FRACBITS), true, &tr)) {
		m_lButtons |= BT_JUMP | BT_USE;
	}

	// strafe around objects/walls partially blocking the way
	FVector3 forward, right;
	MakeVectors(m_pPlayer->mo->angle, forward, right);
	FVector3 viewOffset = FVector3(0, 0, m_pPlayer->viewheight);
	FTraceResults trLeft, trRight;
	TraceAhead(32, viewOffset + right * 16, false, &trRight);
	TraceAhead(32, viewOffset + right * -16, false, &trLeft);
	if (trLeft.Fraction != trRight.Fraction) {
		m_sideMove = trRight.Fraction < trLeft.Fraction ? -speed : speed;
	}

	fixed_t dist = P_AproxDistance(m_pPlayer->mo->x - (fixed_t)pos.X, m_pPlayer->mo->y - (fixed_t)pos.Y);
	return dist < (radius << FRACBITS);
}

void CWootBot::CombatThink() {
	AActor* bestEnemy = BestEnemy();

	if (bestEnemy) {
		m_pPlayer->mo->target = bestEnemy;
	}

	AActor* targ = m_pPlayer->mo->target;

	if (!targ || targ->health <= 0) {
		m_pPlayer->mo->target = NULL;
		return;
	}

	SelectBestWeapon();

	fixed_t dist = P_AproxDistance(m_pPlayer->mo->x - targ->x, m_pPlayer->mo->y - targ->y);
	fixed_t minChaseDist = 200 << FRACBITS;
	fixed_t maxChaseDist = 500 << FRACBITS;
	fixed_t maxRange = 2000 << FRACBITS;
	fixed_t minRange = 0;
	bool isMeleeWeapon = false;

	if (m_pPlayer->ReadyWeapon) {
		WeaponInfo& info = g_wbot_weapon_info[m_pPlayer->ReadyWeapon->GetClass()->TypeName.GetChars()];
		minChaseDist = (info.minRange << FRACBITS) + 64;
		maxChaseDist = std::max(minChaseDist, (info.idealRange << FRACBITS));
		minRange = info.minRange << FRACBITS;
		maxRange = info.maxRange << FRACBITS;
		isMeleeWeapon = info.maxRange < 200;
	}

	if (isMeleeWeapon && dist > maxRange && targ->Sector != m_pPlayer->mo->Sector) {
		m_pPlayer->mo->target = NULL;
		return; // ignore enemies not close enough to punch
	}

	bool hasLineOfSight = P_CheckSight(m_pPlayer->mo, targ, SF_SEEPASTSHOOTABLELINES);

	if (!hasLineOfSight) {
		// forget about the target if not seen for a while
		if (level.maptime - m_targetLastSeenTic < 35) {
			m_pPlayer->mo->target = NULL;
			return;
		}
	}

	m_targetLastSeenTic = level.maptime;

	// aim at enemy
	AimAtPos(FVector3(targ->x, targ->y, targ->z + targ->height / 2));

	m_forwardMove = 0;
	m_sideMove = 0;

	// don't get too close/far
	if (dist < minChaseDist) {
		m_forwardMove = -RUN_SPEED;
	}
	else if (dist > maxChaseDist) {
		m_forwardMove = RUN_SPEED;
	}

	// randomly strafe around the target
	int r = rand() % 10;
	if (r < 5) {
		m_sideMove = -RUN_SPEED;
	}
	else {
		m_sideMove = RUN_SPEED;
	}

	if (hasLineOfSight && dist > minRange && dist < maxRange) {
		Attack();
	}
}

void CWootBot::SelectBestWeapon() {
	AWeapon* bestWeapon = NULL;
	int bestPriority = -1;
	for (AInventory* item = m_pPlayer->mo->Inventory; item != NULL; item = item->Inventory) {
		if (item->IsKindOf(RUNTIME_CLASS(AWeapon))) {
			AWeapon* weapon = static_cast<AWeapon*>(item);

			int prio = g_wbot_weapon_info[weapon->GetClass()->TypeName.GetChars()].priority;
			bool hasAmmo = weapon->Ammo1 && weapon->Ammo1->Amount > 0 && weapon->Ammo1->Amount >= weapon->MinAmmo1;
			if ((!weapon->AmmoType1 || hasAmmo) && prio > bestPriority) {
				bestPriority = prio;
				bestWeapon = weapon;
			}
		}
	}

	if (bestWeapon && m_pPlayer->ReadyWeapon != bestWeapon && m_pPlayer->PendingWeapon != bestWeapon) {
		DebugPrint(VarArgs("Switching to best weapon '%s'\n", bestWeapon->GetClass()->TypeName.GetChars()));
		m_pPlayer->PendingWeapon = bestWeapon;
		if (m_pPlayer->ReadyWeapon != NULL) {
			P_DropWeapon(m_pPlayer);
		}
		else if (m_pPlayer->PendingWeapon != WP_NOCHANGE) {
			P_BringUpWeapon(m_pPlayer);
		}
	}
}

AActor* wbot_LookForEnemiesInBlock(AActor* lookee, int index, void* extparam)
{
	FBlockNode* block;
	AActor* link;
	CWootBot* pbot = (CWootBot*)extparam;
	angle_t fov = pbot->m_fov;
	AActor* plr = pbot->GetActor();
	AActor* oldTarget = plr->target;
	fixed_t oldDist = oldTarget ? P_AproxDistance(oldTarget->x - plr->x, oldTarget->y - plr->y) : 0;

	for (block = blocklinks[index]; block != NULL; block = block->NextActor)
	{
		link = block->Me;

		if (!(link->flags & MF_SHOOTABLE))
			continue;			// not shootable (observer or dead)

		if (link == lookee)
			continue;

		if (link->health <= 0)
			continue;			// dead

		if (link->flags2 & MF2_DORMANT)
			continue;			// don't target dormant things

		if (!(link->flags3 & MF3_ISMONSTER))
			continue;			// don't target it if it isn't a monster (could be a barrel)

		if (link->flags7 & MF7_NEVERTARGET)
			continue;

		if (lookee->IsFriend(link))
			continue;

		if (fov && fov < ANGLE_MAX)
		{
			angle_t an = R_PointToAngle2(lookee->x, lookee->y, link->x, link->y) - lookee->angle;

			if (an > (fov / 2) && an < (ANGLE_MAX - (fov / 2))) {
				continue;	// outside of fov
			}
		}

		// P_CheckSight is by far the most expensive operation in here so let's do it last.
		if (!P_CheckSight(lookee, link, SF_SEEPASTSHOOTABLELINES)) {
			continue;
		}

		// only retarget to closer enemies
		if (oldTarget && P_AproxDistance(plr->x - link->x, plr->y - link->y) >= oldDist) {
			continue;
		}

		return link;
	}

	return NULL;
}

FVector3 CWootBot::GetViewPos() {
	return FVector3(m_pPlayer->mo->x, m_pPlayer->mo->y, m_pPlayer->mo->z + m_pPlayer->mo->ViewHeight);
}

std::unordered_set<int> CWootBot::GetBlockedPaths() {
	unordered_set<int> allBlockedPaths;
	for (BotGoal& goal : m_goals) {
		allBlockedPaths.insert(goal.blockers.begin(), goal.blockers.end());
	}
	return allBlockedPaths;
}

AActor* CWootBot::BestEnemy() {
	return P_BlockmapSearch(m_pPlayer->mo, 10, wbot_LookForEnemiesInBlock, this);
}

bool CWootBot::FindGoal() {
	int thisSubId = g_wbot_nav.get_nav_id(m_pPlayer->mo);

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

		if (thisSubId == g_wbot_nav.get_nav_id(actor))
			continue; // already with this player

		PushGoal(BotGoal(WBOT_GOAL_ACTION_MOVE_TO, actor));
	}

	return m_route.size();
}

bool CWootBot::PushGoal(BotGoal& goal) {
	if (!m_goals.empty()) {
		BotGoal& lastGoal = m_goals[m_goals.size() - 1];
		if (lastGoal.matches(goal)) {
			// can happen when hugging the wall of a triggerable sectors
			DebugPrint(VarArgs("Skipping duplicate goal push: %s\n", goal.desc().c_str()));
			return false;
		}
	}

	DebugPrint(VarArgs("New goal: %s\n", goal.desc().c_str()));
	m_goals.push_back(goal);
	RouteToGoal();

	return true;
}

void CWootBot::PopGoal() {
	if (m_goals.empty()) {
		CancelRoute();
		DebugPrint("No goal to pop\n");
		return;
	}

	BotGoal& goal = m_goals[m_goals.size() - 1];
	DebugPrint(VarArgs("Finished goal: %s\n", goal.desc().c_str()));
	m_goals.pop_back();

	if (m_goals.size()) {
		BotGoal& lastGoal = m_goals[m_goals.size() - 1];
		lastGoal.blockers.clear(); // if subgoals were completed, then paths to this goal are probably unblocked now
		RouteToGoal();
	}
	else
		CancelRoute();
}

std::vector<int> CWootBot::RouteToSector(int subid) {
	unordered_set<int> allBlockedPaths = GetBlockedPaths();

	if (stuckPath >= 0) {
		// avoid the path that got the bot stuck in the last movement
		allBlockedPaths.insert(stuckPath);
		stuckPath = -1;
		DebugPrint(VarArgs("Ignoring stucked path %d for this route\n", stuckPath));
	}

	return g_wbot_nav.get_astar_route(m_navid, subid, &allBlockedPaths);
}

void CWootBot::RouteToGoal() {
	CancelRoute();

	if (m_goals.empty()) {
		DebugPrint("No goal to route to\n");
		return;
	}

	BotGoal& goal = m_goals[m_goals.size() - 1];
	m_route = RouteToSector(goal.getNavId());

	if (m_route.size()) {
		DebugPrint(VarArgs("Routing to goal: %s\n", goal.desc().c_str()));
	}
	else {
		DebugPrint(VarArgs("Failed goal (no route): %s\n", goal.desc().c_str()));
		m_goals.pop_back();
	}
}

void CWootBot::Attack() {
	m_lButtons |= BT_ATTACK;
	m_lastAttack = level.time;
}

CCMD(addbotw)
{
	if (gamestate != GS_LEVEL)
		return;

	// Don't allow bots in network mode, unless we're the host.
	if (NETWORK_InClientMode())
	{
		Printf("Only the host can add bots!\n");
		return;
	}

	ULONG ulPlayerIdx = BOTS_FindFreePlayerSlot();
	if (ulPlayerIdx == MAXPLAYERS)
	{
		Printf("The maximum number of players/bots has been reached.\n");
		return;
	}

	new CWootBot(NULL, NULL, ulPlayerIdx);
}