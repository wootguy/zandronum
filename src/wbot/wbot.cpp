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
#include "m_cheat.h"
#include "wutil.h"

#include <stdlib.h>
#include <time.h>
#include <unordered_set>
#include <unordered_map>
#include <string>

using namespace std;

#define RUN_SPEED 100 // max move speed allowed before the server kicks you

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
		int ret = g_wbot_nav.line_subsectors[lineid];

		if (ret == -1)
			Printf("Failed to find subsector for line %d\n", lineid);

		return ret;
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

bool BotGoal::valid() {
	if (lineid >= 0) {
		return lines[lineid].special != 0;
	}
	return actor != NULL;
}

CWootBot::CWootBot(const char* pszName, const char* pszTeamName, ULONG ulPlayerNum)
	: CSkullBot(pszName, pszTeamName, ulPlayerNum) {
	
	m_fov = ANGLE_180;
	m_bForwardMovePersist = true;
	m_bSideMovePersist = true;
	m_debug = true;
}

void CWootBot::ParseScript() {
	init_wootbots();

	// level changed
	if (lastInit != level.levelnum) {
		lastInit = level.levelnum;
		Reset();
	}

	m_navid = g_wbot_nav.get_nav_id(m_pPlayer->mo);
	NavSector& nav = g_wbot_nav.nav_sectors[m_navid];
	stateFlags &= ~(FL_WBOT_FLYING | FL_WBOT_ON_ELEV);
	if (m_pPlayer->mo->z > nav.getFloorZ())
		stateFlags |= FL_WBOT_FLYING;
	if (nav.sector()->floordata)
		stateFlags |= FL_WBOT_ON_ELEV;

	if (m_debug) {
		ShowDebugInfo();
	}

	if (m_pPlayer->cheats & CF_FROZEN) {
		return;
	}

	if (level.time < m_nextThink)
		return;

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

	if (m_speedMult != 1.0f) {
		m_forwardMove *= m_speedMult;
		m_sideMove *= m_speedMult;
	}

	m_lForwardMove = static_cast<LONG>(0x32 * (m_forwardMove / 100.0f));
	m_lSideMove = static_cast<LONG>(0x32 * (m_sideMove / 100.0f));
}

void CWootBot::Reset() {
	CancelRoute();
	m_goals.clear();
	m_lastAttack = 0;
	m_lastUse = 0;
	stateFlags = 0;
	m_targetLastSeenTic = 0;
	stuckPath = -1;
	m_pPlayer->mo->target = NULL;
	m_forwardMove = 0;
	m_sideMove = 0;
	m_nextThink = 0;
	m_lastElevZ = 0;
}

void CWootBot::ShowDebugInfo() {
	g_wbot_nav.draw_nodes(m_pPlayer->mo);

	int thisSubId = g_wbot_nav.get_nav_id(m_pPlayer->mo);

	string routeStr = "Route: " + to_string(thisSubId);
	if (pretendRouteSector >= 0) {
		routeStr += " (pretend " + to_string(pretendRouteSector) + " )";
	}
	routeStr += " -> ";
	for (int i = 0; i < m_route.size() && i < 4; i++) {
		if (i != 0)
			routeStr += " ";
		routeStr += to_string(m_route[i]);
	}
	if (m_route.size() > 4) {
		routeStr += " (+" + to_string(m_route.size() - 4) + ")";
	}
	routeStr += "\n                     " + to_string(g_wbot_nav.get_route_distance(m_route)) + " units";
	
	if (m_route.size() > 1) {
		NavSector& nav = g_wbot_nav.nav_sectors[m_route[0]];
		NavSectorLink* link = nav.getLink(m_route[1]);
		if (link) {
			routeStr += "\nLink: " + to_string(link->id) + " -> " + to_string(link->target);
		}
	}
	routeStr += "\nSpeed: " + to_string(m_routeSpeed);
	if (m_speedMult != 1.0f)
		routeStr += " * " + to_string((int)m_speedMult) + "." + to_string((int)(m_speedMult * 10) % 10);
	
	AActor* player = getAnyPlayer();
	if (!player)
		return;

	string navInfo;
	{
		int plrnavid = g_wbot_nav.get_nav_id(player);
		navInfo = "Sector " + to_string(plrnavid) + ":";
		NavSector& nav = g_wbot_nav.nav_sectors[plrnavid];

		//int temp;
		//AvoidLedges(player, temp);

		navInfo += "\n   Triggers:";
		vector<BotGoal>& triggers = nav.getTriggers();
		for (int i = 0; i < triggers.size(); i++) {
			navInfo += "\n      " + triggers[i].desc();
		}

		navInfo += "\n   Links:";
		for (int i = 0; i < nav.links.size(); i++) {
			NavSectorLink& link = nav.links[i];
			NavSector& targ = g_wbot_nav.nav_sectors[link.target];
			string arrow = link.blocked(player) ? " -X> " : " --> ";
			navInfo += "\n      " + to_string(link.id) + arrow + to_string(link.target);
			vector<BotGoal>& targtriggers = targ.getTriggers();
			if (targtriggers.size())
				navInfo += " (" + to_string(targtriggers.size()) + " T)";
			if (link.isCliff) { navInfo += " C"; }
			if (link.isTeleport) { navInfo += " T"; }
			if (link.isJump) { navInfo += " J"; }
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

		navInfo += "\nOrigin: " + to_string(player->x >> FRACBITS) + " " + to_string(player->y >> FRACBITS)
			+ " " + to_string(player->z >> FRACBITS);

		int yaw = (int)((uint64_t)player->angle * 360 / 0x100000000ULL);
		int pitch = (int)((uint64_t)player->pitch * 360 / 0x100000000ULL);
		navInfo += "\nAngles: " + to_string(yaw) + " " + to_string(pitch);
	}

	string stateStr = "State:";
	if (stateFlags & FL_WBOT_WAIT_ELEV) { stateStr += " WAIT_ELEV"; }
	if (stateFlags & FL_WBOT_ON_ELEV) { stateStr += " ON_ELEV"; }
	if (stateFlags & FL_WBOT_WAIT_DOOR) { stateStr += " WAIT_DOOR"; }
	if (stateFlags & FL_WBOT_JUMPING) { stateStr += " JUMP"; }
	if (stateFlags & FL_WBOT_FLYING) { stateStr += " FLY"; }
	if (stateFlags & FL_WBOT_RUSHING) { stateStr += " RUSH"; }
	if (m_pPlayer->cheats & (CF_FROZEN | CF_TOTALLYFROZEN)) { stateStr += " FROZEN"; }

	string btnStr = "Buttons: ";
	if (m_lButtons & BT_ATTACK) btnStr += " ATTACK";
	if (m_lButtons & BT_USE) btnStr += " USE";
	if (m_lButtons & BT_JUMP) btnStr += " JUMP";
	if (m_lButtons & BT_CROUCH) btnStr += " CROUCH";
	if (m_lButtons & BT_TURN180) btnStr += " TURN180";
	if (m_lButtons & BT_ALTATTACK) btnStr += " ALTATTACK";
	if (m_lButtons & BT_RELOAD) btnStr += " RELOAD";
	if (m_lButtons & BT_ZOOM) btnStr += " ZOOM";
	if (m_lButtons & BT_SPEED) btnStr += " SPEED";

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
		+ stateStr + "\n" + btnStr + "\n" + stuckStr + "\n" + goalStr;

	SERVERCOMMANDS_PrintHUDMessage(navInfo.c_str(), 0.94f, 0.5f, 0, 0, 0, CR_RED, 1.0f, 0, 0, "SmallFont", MAKE_ID('W', 'N', 'A', 'V'));
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

	Reset();
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
	if (m_route.size() && m_freezeOnRouteChange) {
		m_pPlayer->cheats |= CF_FROZEN;
		m_pPlayer->mo->velx = 0;
		m_pPlayer->mo->vely = 0;
		m_pPlayer->mo->velz = 0;
	}

	m_route.clear();
	pretendRouteSector = -1;
	stuckCounter = 0;
	stateFlags &= ~FL_WBOT_JUMPING;
}

void CWootBot::GoalActionThink() {
	BotGoal& goal = m_goals[m_goals.size() - 1];
	NavSector& nav = g_wbot_nav.nav_sectors[m_navid];

	int goalSector = goal.getNavId();

	if (m_navid != goalSector && pretendRouteSector != goalSector) {
		// actor may be routed here but technically in an adjacent sector
		bool actorReachable = nav.touches(goal.actor);

		if (!actorReachable) {
			// route was cancelled or the target moved. Route to it again.
			DebugPrint("Goal moved or movement failed. Rerouting...\n");
			RouteToGoal();
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
		int useDist = (m_pPlayer->mo->UseRange >> FRACBITS) - 1;
		if (MoveTo(goal.pos(), useDist)) {
			Use();
		}
		break;
	}
	case WBOT_GOAL_ACTION_TOUCH:
		if (MoveTo(goal.pos(), goal.touchDistance(m_pPlayer->mo))) {
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

			fixed_t dist = P_AproxDistance(m_pPlayer->mo->x - (fixed_t)backGoal.X, m_pPlayer->mo->y - (fixed_t)backGoal.Y);

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
		RouteToGoal();
	}
}

void CWootBot::RouteThink() {
	m_routeSpeed = RUN_SPEED;
	const int cliffSafeDist = SAFE_CLIFF_DIST - 16;

	m_forwardMove = 0;
	m_sideMove = 0;
	stateFlags &= ~(FL_WBOT_WAIT_ELEV | FL_WBOT_WAIT_DOOR);

	if (m_route.size() > 1) {
		if (m_navid == m_route[1]) {
			// inside the target sector. Advance the route.
			m_route.erase(m_route.begin());
			pretendRouteSector = -1;
			stateFlags &= ~FL_WBOT_JUMPING;
		}
		else {
			FVector2 center = g_wbot_nav.nav_sectors[m_route[1]].pos();
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

	NavSector& idealNav = g_wbot_nav.nav_sectors[m_route[0]];
	NavSector& curNav = g_wbot_nav.nav_sectors[m_navid];
	NavSector& targetNav = g_wbot_nav.nav_sectors[m_route.size() > 1 ? m_route[1] : m_route[0]];

	int nodeTouchDist = 32;

	// be careful near cliffs
	if (!(stateFlags & FL_WBOT_JUMPING)) {
		bool headingTowardsCliff = false;
		if (m_route.size() > 1) {
			NavSectorLink* link = idealNav.getLink(m_route[1]);
			headingTowardsCliff = targetNav.hasCliffs && link->linkWidth < 32;
		}

		if (m_cliffDist < SAFE_CLIFF_DIST * 0.5f || headingTowardsCliff) {
			m_routeSpeed *= 0.5f;
			nodeTouchDist = 8;
		}
		else if (m_cliffDist < SAFE_CLIFF_DIST * 0.75f) {
			m_routeSpeed *= 0.75f;
			nodeTouchDist = 16;
		}
	}

	if (m_navid != m_route[0]) {
		// update route if slipped off into a sector adjacent to a target
		NavSector& curNav = g_wbot_nav.nav_sectors[m_navid];

		if (m_route.size() > 1 && curNav.getLink(m_route[1])) {
			NavSectorLink* link = curNav.getLink(m_route[1]);
			if (link && link->walkable()) {
				m_route[0] = m_navid;
			}
		}
	}

	if ((stateFlags & FL_WBOT_JUMPING) && m_route.size() > 1) {
		// just try to land in the right spot
		MoveTo(targetNav.pos(), 0, m_routeSpeed);

		NavSectorLink* link = idealNav.getLink(m_route[1]);

		if (targetNav.getFloorZ() > m_pPlayer->mo->z + (JUMP_HEIGHT << FRACBITS)) {
			stateFlags &= ~FL_WBOT_JUMPING; // missed the jump
			if (m_goals.size()) {
				// don't try the jump again, there are probably other ones to try
				// and many jumps just don't work
				BotGoal& curgoal = m_goals[m_goals.size() - 1];
				curgoal.blockers.insert(link->id);
			}
			
			return;
		}
		
		FVector2 target = targetNav.pos();
		fixed_t jumpDist = (target - FVector2(m_pPlayer->mo->x, m_pPlayer->mo->y)).Length();
		bool bigJump = jumpDist > 100 << FRACBITS;
		if (targetNav.getFloorZ() > idealNav.getFloorZ() + (STEP_HEIGHT << FRACBITS))
			bigJump = true;

		if (bigJump && (stateFlags & FL_WBOT_FLYING)) {
			// bot is off a ledge now, start the jump
			m_lButtons |= BT_JUMP;
		}
	}
	else if (m_route.size() > 1) {
		NavSectorLink* link = idealNav.getLink(m_route[1]);

		// allow slipping off the route into adjacent sectors while heading towards the target sector
		bool onTrack = m_navid == m_route[0]
			|| curNav.getLink(m_route[1]) || curNav.getLink(m_route[0])
			|| idealNav.touches(m_pPlayer->mo) || targetNav.touches(m_pPlayer->mo);		

		if (onTrack) {
			sector_t* nextSector = subsectors[m_route[1]].sector;
			bool nextOnElevator = nextSector && nextSector->floordata;
			bool linkBlocked = link->blocked(m_pPlayer->mo);

			// wait on elevators
			if (stateFlags & FL_WBOT_ON_ELEV) {
				sector_t* thisSector = curNav.sector();
				bool waitedLongEnough = link->walkable();

				if (link->isJump) {
					// If the next link is a jump, wait until the very top for better success chance
					fixed_t elevZ = thisSector->floorplane.ZatPoint(m_pPlayer->mo->x, m_pPlayer->mo->y);;
					fixed_t moveDelta = elevZ - m_lastElevZ;

					// elevator stopped at the top or started going down?
					waitedLongEnough = !linkBlocked && moveDelta <= 0;

					m_lastElevZ = elevZ;
				}

				if (!waitedLongEnough) {
					stateFlags |= FL_WBOT_WAIT_ELEV;

					// stay centered on the elevator to avoid blocking it or falling off
					FVector2 navPos = curNav.pos();
					fixed_t dist = P_AproxDistance(m_pPlayer->mo->x - (fixed_t)navPos.X, m_pPlayer->mo->y - (fixed_t)navPos.Y);
					if (dist > (16 << FRACBITS))
						MoveTo(navPos, 0, RUN_SPEED / 4);
					return; // wait until the elevator is done moving
				}
			}
			else {
				m_lastElevZ = 0;
			}

			// wait for doors to open
			if (!link->walkable() && (nextSector->floordata || nextSector->ceilingdata)) {
				// door is raising or elevator is lowering in the next sector
				if (link->isJump && !nextSector->floordata && !link->isJumpHeightValid()) {
					// a door opening isn't going to make the jump doable if the floor is too high
				}
				else {
					stateFlags |= FL_WBOT_WAIT_DOOR;
					return; // wait until the door/elevator is done moving
				}
			}

			// handle severe blockages
			if (linkBlocked) {
				BlockedPathThink(link);
				return;
			} else if (!nextOnElevator && m_route.size() > 2 && targetNav.touches(m_pPlayer->mo)) {
				// if we're touching the next sector and the next path is blocked, also do block
				// handling. Helps in case of doors with tiny sectors in front of them which the
				// bot can't fully get inside.
				NavSectorLink* nextLink = g_wbot_nav.nav_sectors[link->target].getLink(m_route[2]);
				sector_t* nextNextSector = subsectors[m_route[2]].sector;

				if (!nextLink->walkable() && (nextNextSector->floordata || nextNextSector->ceilingdata)) {
					stateFlags |= FL_WBOT_WAIT_DOOR;
					return; // wait until the door/elevator is done moving
				}
				
				if (nextLink && nextLink->blocked(m_pPlayer->mo)) {
					BlockedPathThink(nextLink);
					return;
				}
			}

			// duck if unable to fit while standing
			int targetHeight = targetNav.getHeight() >> FRACBITS;
			int borderHeight = (targetNav.getCeilZ() - idealNav.getFloorZ()) >> FRACBITS;
			if (std::min(targetHeight, borderHeight) < STAND_HEIGHT) {
				m_lButtons |= BT_CROUCH;
			}
			
			if (link->isJump) {
				FVector2 startPos = link->GetJumpBackupPos();
				int dist = GetDistance(startPos) >> FRACBITS;
				m_routeSpeed = RUN_SPEED;
				int startSpeed = RUN_SPEED * 0.2f;
				if (dist < 64) {
					m_routeSpeed = RUN_SPEED * 0.2f;
				}
				else if (dist < 256) {
					m_routeSpeed = RUN_SPEED * 0.5f;
				}
				int curSpeed = GetSpeed2D();

				if (MoveTo(startPos, 18, m_routeSpeed) && curSpeed < startSpeed) {
					// close enough to the link edge				
					stateFlags |= FL_WBOT_JUMPING;
					return;
				}
			}
			else {
				// move to the next link
				if (MoveTo(link->pos(), nodeTouchDist, m_routeSpeed)) {
					// close enough to the link edge				
					if (link->isTeleport && link->seg->linedef) {
						// move behind the teleporter line edge.
						// The target sector may be in a completely different direction.
						FVector2 backDir = getLineBackDir(link->seg->linedef);
						FVector2 teleGoal = link->pos() + backDir * 200;
						MoveTo(teleGoal, 0, m_routeSpeed);
					}
					else {
						// move towards the target sector until we end up inside it
						MoveTo(targetNav.pos(), 0, m_routeSpeed);
					}
				}
			}
		}
		else { // fell off the route
			if ((stateFlags & FL_WBOT_JUMPING) && (stateFlags & FL_WBOT_FLYING)) {
				// don't abort the route until the jump is complete
				MoveTo(targetNav.pos(), 0, m_routeSpeed);
				return;
			}

			if (m_navid != m_route[0]) {
				// update route if slipped off into a sector adjacent to the previous
				// not done earlier for a reason i forgot on doom2 map24 tightrope area.
				NavSector& curNav = g_wbot_nav.nav_sectors[m_navid];
				NavSectorLink* link = curNav.getLink(m_route[0]);
				if (link && link->walkable()) {
					m_route.insert(m_route.begin(), m_navid);
					return;
				}
			}

			DebugPrint(VarArgs("Fell off the route (expected %d but got %d)\n", m_route[0], m_navid));
			CancelRoute();
		}
	}
	else if (m_route.size() == 1) {
		FVector2 centerGoal = idealNav.pos();
		if (MoveTo(centerGoal, nodeTouchDist, m_routeSpeed)) {
			m_route.clear(); // don't reset pretendsector in case a goal is inside it
			stuckPath = -1;
			stateFlags &= ~FL_WBOT_JUMPING;
			DebugPrint("Finished route\n");
		}
	}

	if (StuckThink(500)) {
		DebugPrint("I got stuck! Cancelling route.\n");

		if (m_route.size() > 1) {
			NavSectorLink* failedLink = curNav.getLink(m_route[1]);
			stuckPath = failedLink ? failedLink->id : -1;
		}

		CancelRoute();
	}
}

void CWootBot::BlockedPathThink(NavSectorLink* link) {
	link->blocked(m_pPlayer->mo); // debug here

	string blockMsg = VarArgs("Link %d blocked!", link->id);

	BotGoal& curGoal = m_goals[m_goals.size() - 1];
	curGoal.blockers.insert(link->id);

	// don't try to route through previous paths we've been trying to unblock
	unordered_set<int> allBlockedPaths = GetBlockedPaths();

	// nothing is moving, try unblocking it ourselves.
	NavSector& targetNav = g_wbot_nav.nav_sectors[link->target];
	vector<BotGoal>& targTriggers = targetNav.getTriggers();
	for (BotGoal& goal : targTriggers) {
		if (!goal.valid())
			continue;
		int subid = goal.getNavId();

		if (subid == link->parent || RouteToSector(subid).size()) {
			DebugPrint(VarArgs("%s Adding unblock subgoal.\n", blockMsg.c_str()));
			if (PushGoal(goal, link)) {
				return;
			}
		}
	}

	// if we're on an elevator, try triggering it.
	NavSector& thisNav = g_wbot_nav.nav_sectors[link->parent];
	vector<BotGoal>& thisTriggers = thisNav.getTriggers();
	for (BotGoal& goal : thisTriggers) {
		if (!goal.valid())
			continue;
		int subid = goal.getNavId();

		if (subid == link->parent || RouteToSector(subid).size()) {
			DebugPrint(VarArgs("%s Adding unblock subgoal.\n", blockMsg.c_str()));
			if (PushGoal(goal, link)) {
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
	if (curGoal.blockers.size() > 1 || !curGoal.blockers.count(link->id)) {
		curGoal.blockers.clear();
		m_route = RouteToSector(curGoal.getNavId());
		if (m_route.size()) {
			DebugPrint(VarArgs("%s Forgetting blocked paths and trying again...\n", blockMsg.c_str()));
			return;
		}
	}

	DebugPrint(VarArgs("%s Goals aborted. Failed to reach a subgoal.\n", blockMsg.c_str()));
	CancelRoute();
	m_goals.clear();

	m_nextThink = level.time + 10;
}

bool CWootBot::StuckThink(int maxStuck) {
	FVector2 curPos = FVector2(m_pPlayer->mo->x, m_pPlayer->mo->y);
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

	return Trace((fixed_t)start.X, (fixed_t)start.Y, (fixed_t)start.Z, m_pPlayer->mo->Sector,
		(fixed_t)forward.X, (fixed_t)forward.Y, 0, testDist, ignoreMonsters ? 0 : 0xffffffff,
		ML_BLOCKEVERYTHING | ML_BLOCKHITSCAN, m_pPlayer->mo, *tr);
}

bool CWootBot::MoveTo(FVector2 pos, int radius, int speed) {
	float z = (float)(m_pPlayer->mo->z + m_pPlayer->viewheight);
	AimAtPos(FVector3(pos.X, pos.Y, z));

	FVector2 wantDir = pos - FVector2(m_pPlayer->mo->x, m_pPlayer->mo->y);
	wantDir.MakeUnit();

	// jump over short walls and open doors
	FTraceResults tr;
	if (TraceAhead(32, FVector3(0, 0, STEP_HEIGHT << FRACBITS), true, &tr)) {
		m_lButtons |= BT_JUMP;

		// only use walls that aren't already moving, so doors aren't closed while moving
		if (tr.Line && tr.Line->backsector) {
			if (!tr.Line->backsector->floordata && !tr.Line->backsector->ceilingdata) {
				Use();
			}
		}
	}

	// combine desired vector with avoidance vectors
	FVector2 avoidCornersDir = AvoidCornersVector(wantDir);
	FVector2 avoidLedgeDir = AvoidLedges(m_pPlayer->mo, m_cliffDist);
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
	MakeVectors(m_pPlayer->mo->angle, forward, right);
	forward.MakeUnit();
	right.MakeUnit();
	m_forwardMove = DotProduct(moveDir, forward);
	m_sideMove = DotProduct(moveDir, right);

	fixed_t dist = P_AproxDistance(m_pPlayer->mo->x - (fixed_t)pos.X, m_pPlayer->mo->y - (fixed_t)pos.Y);
	return dist < (radius << FRACBITS);
}

FVector2 CWootBot::AvoidCornersVector(FVector2 wantDir) {
	// strafe around objects/walls partially blocking the way
	fixed_t zTest = (STEP_HEIGHT + 1) << FRACBITS;
	FVector3 wantDirf(wantDir.X * (1 << FRACBITS), wantDir.Y * (1 << FRACBITS), 0);
	FVector3 viewPos = FVector3(m_pPlayer->mo->x, m_pPlayer->mo->y, m_pPlayer->mo->z + zTest);
	fixed_t testDist = 32 << FRACBITS;
	fixed_t rightOfs = 16 << FRACBITS;
	FVector3 rightDir(wantDir.Y, -wantDir.X, 0);
	FVector3 rightPos = viewPos + rightDir * rightOfs;
	FVector3 leftPos = viewPos + rightDir * -rightOfs;
	FTraceResults trLeft, trRight;

	Trace(rightPos.X, rightPos.Y, rightPos.Z, m_pPlayer->mo->Sector,
		wantDirf.X, wantDirf.Y, 0, testDist, 0xffffffff,
		ML_BLOCKEVERYTHING, m_pPlayer->mo, trRight);

	Trace(leftPos.X, leftPos.Y, leftPos.Z, m_pPlayer->mo->Sector,
		wantDirf.X, wantDirf.Y, 0, testDist, 0xffffffff,
		ML_BLOCKEVERYTHING, m_pPlayer->mo, trLeft);

	//draw_debug_line(rightPos, rightPos + wantDirf * 32, m_pPlayer->mo);
	//draw_debug_line(leftPos, leftPos + wantDirf * 32, m_pPlayer->mo);

	if (trLeft.Fraction != trRight.Fraction) {
		return trRight.Fraction < trLeft.Fraction ? -rightDir : rightDir;
	}

	return FVector2(0, 0);
}

FVector2 CWootBot::AvoidLedges(AActor* actor, int& cliffDist) {
	//NavSector& nav = g_wbot_nav.nav_sectors[m_navid];
	//FVector2 plrPos(m_pPlayer->mo->x, m_pPlayer->mo->y);
	int subid = R_PointInSubsector(actor->x, actor->y) - subsectors;
	NavSector* nav = &g_wbot_nav.nav_sectors[subid];
	FVector2 plrPos(actor->x, actor->y);

	int targetNav = -1;
	int idealNav = -1;
	if (m_route.size() > 0) {
		idealNav = m_route[0];
		nav = &g_wbot_nav.nav_sectors[idealNav];
	}
	if (m_route.size() > 1) {
		targetNav = m_route[1];
	}

	// get nearby sectors in case nearest ledge is at the corner of the current
	std::vector<NavSector*> sectors;
	if (nav->hasCliffs)
		sectors.push_back(nav);
	for (NavSectorLink& link : nav->links) {
		if (link.isCliff || link.isTeleport)
			continue;

		NavSector& target = g_wbot_nav.nav_sectors[link.target];
		if (target.hasCliffs) {
			sectors.push_back(&target);
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

			if (link.target == targetNav || link.target == idealNav)
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

	//FVector3 headPos = FVector3(plrPos.X, plrPos.Y, m_pPlayer->mo->z + (56 << FRACBITS));
	//FVector3 dir = FVector3(avoidForce.X, avoidForce.Y, 0);
	//fixed_t scale = 200 << FRACBITS;
	//draw_debug_line(headPos, headPos + dir * scale, m_pPlayer->mo);

	return avoidForce;
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

FVector3 CWootBot::GetViewPos() {
	return FVector3(m_pPlayer->mo->x, m_pPlayer->mo->y, m_pPlayer->mo->z + m_pPlayer->mo->ViewHeight);
}

fixed_t CWootBot::GetDistance(FVector2 p) {
	return P_AproxDistance(p.X - m_pPlayer->mo->x, p.Y - m_pPlayer->mo->y);
}

FVector3 CWootBot::GetVelocity() {
	return FVector3(m_pPlayer->mo->velx, m_pPlayer->mo->vely, m_pPlayer->mo->velz);
}

int CWootBot::GetSpeed2D() {
	return (int)FVector2(m_pPlayer->mo->velx, m_pPlayer->mo->vely).Length() >> FRACBITS;
}

std::unordered_set<int> CWootBot::GetBlockedPaths() {
	unordered_set<int> allBlockedPaths;
	for (BotGoal& goal : m_goals) {
		allBlockedPaths.insert(goal.blockers.begin(), goal.blockers.end());
	}
	return allBlockedPaths;
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

		PushGoal(BotGoal(WBOT_GOAL_ACTION_MOVE_TO, actor), NULL);
	}

	return m_route.size();
}

bool CWootBot::PushGoal(BotGoal& goal, NavSectorLink* purposeLink) {
	if (!m_goals.empty()) {
		BotGoal& lastGoal = m_goals[m_goals.size() - 1];
		if (lastGoal.matches(goal)) {
			// can happen when hugging the wall of a triggerable sectors
			DebugPrint(VarArgs("Skipping duplicate goal: %s\n", goal.desc().c_str()));
			return false;
		}
		if (goal.purposeLink && lastGoal.purposeLink == goal.purposeLink) {
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
	if (line && line->special == Door_LockedRaise && !P_CheckKeys(m_pPlayer->mo, line->args[3], false)) {
		vector<BotGoal> keyGoals;
		unordered_set<int> allBlockedPaths = GetBlockedPaths();
		g_wbot_nav.get_key_goals_for_line(m_pPlayer->mo, line, keyGoals, &allBlockedPaths);

		DebugPrint(VarArgs("    Adding locked line subgoals.\n"));

		for (BotGoal& keyGoal : keyGoals) {
			m_goals.push_back(keyGoal);
		}
	}

	return RouteToGoal();
}

void CWootBot::PopGoal() {
	if (m_goals.empty()) {
		CancelRoute();
		DebugPrint("No goal to pop\n");
		return;
	}

	BotGoal& goal = m_goals[m_goals.size() - 1];

	stateFlags &= ~FL_WBOT_RUSHING;

	int purposeLinkId = goal.purposeLink ? goal.purposeLink->id : -1;
	NavSector* purposeNav = goal.purposeLink ? goal.purposeLink->getTarget() : NULL;
	if (purposeNav && (purposeNav->getMoveFlags() & FL_SECTOR_MOVE_TIMED)) {
		// the purpose of this goal was to move a timed sector. Better hurry before that sector resets!
		stateFlags |= FL_WBOT_RUSHING;
	}

	DebugPrint(VarArgs("Finished goal: %s\n", goal.desc().c_str()));
	m_goals.pop_back();

	if (m_goals.size()) {
		// unblock the link that the previous goal was for
		for (int i = 0; i < m_goals.size(); i++) {
			m_goals[i].blockers.erase(purposeLinkId); 
		}
		
		RouteToGoal();
	}
	else
		CancelRoute();
}

std::vector<int> CWootBot::RouteToSector(int subid) {
	unordered_set<int> allBlockedPaths = GetBlockedPaths();

	if (stuckPath >= 0) {
		// avoid the path that got the bot stuck in the last movement
		DebugPrint(VarArgs("Ignoring stucked path %d for this route\n", stuckPath));
		allBlockedPaths.insert(stuckPath);
		stuckPath = -1;
	}

	return g_wbot_nav.get_astar_route(m_navid, subid, &allBlockedPaths, stateFlags & FL_WBOT_RUSHING);
}

bool CWootBot::RouteToGoal() {
	CancelRoute();

	if (m_goals.empty()) {
		DebugPrint("No goal to route to\n");
		return false;
	}

	BotGoal& goal = m_goals[m_goals.size() - 1];
	int goalNavId = goal.getNavId();
	m_route = RouteToSector(goalNavId);

	if (m_route.empty() && goal.actor) {
		// actor origin is in an unreachable sector, but it's collision box may be touching a reachable one
		vector<int> subs = g_wbot_nav.GetTouchedSubsectors(goal.actor);

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
		if (goal.action == WBOT_GOAL_ACTION_CROSS) {
			// add the back sector of the cross line to the route, in case its part of an elevator
			// this way unblocking logic works (doom2 map06 gold key).
			line_t& line = lines[goal.lineid];
			NavSector& goalSector = g_wbot_nav.nav_sectors[m_route[m_route.size() - 1]];
			for (int i = 0; i < goalSector.links.size(); i++) {
				NavSectorLink& link = goalSector.links[i];
				if (link.seg->linedef == &line) {
					m_route.push_back(link.target);
					break;
				}
			}
		}

		DebugPrint(VarArgs("Routing to goal: %s\n", goal.desc().c_str()));
		return true;
	}

	DebugPrint(VarArgs("Failed goal (no route): %s\n", goal.desc().c_str()));
	m_goals.pop_back();
	m_nextThink = level.time + 10;
	return false;
}

void CWootBot::Use(int ticsBetweenUses) {
	if (level.time - m_lastUse < ticsBetweenUses) {
		m_lButtons &= ~BT_USE;
	}

	m_lButtons |= BT_USE;
	m_lastUse = level.time;
}

void CWootBot::Attack() {
	m_lButtons |= BT_ATTACK;
	m_lastAttack = level.time;
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
