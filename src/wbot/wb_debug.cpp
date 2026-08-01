#include "wb_debug.h"
#include "wb_bot.h"
#include "wb_nav.h"
#include "wb_util.h"
#include "wb_route.h"
#include "wb_map.h"

#include <limits.h>
#include <cmath>
#include <float.h>

using namespace std;
using namespace wbot;

void wbot_debug_player_nav() {
	player_t* player = getAnyPlayer();
	if (!player)
		return;

	AActor* pActor = (AActor*)get_player(player);
	vec3 playerPos = get_actor_pos((AActor*)get_player(player));

	string navInfo;
	int plrnavid = g_wb_nav.get_nav_id(player);
	NavSector& nav = g_wb_nav.mesh.nodes[plrnavid];
	MapSubsector& sub = g_map.subsectors[plrnavid];

	navInfo += "Hover Node: " + to_string(g_wb_nav.get_nav_id(player));
	navInfo += "\nNode " + to_string(plrnavid) + ":";

	//int temp;
	//AvoidLedges(player, temp);

	navInfo += "\n   Triggers:";
	vector<BotGoal>& triggers = nav.getTriggers();
	for (int i = 0; i < triggers.size(); i++) {
		navInfo += "\n      " + triggers[i].desc();
	}

	int closestLink = -1;
	float bestDist = FLT_MAX;
	for (int i = 0; i < nav.links.size(); i++) {
		NavSectorLink& link = *nav.links[i];
		float dist = (playerPos - link.pos()).length();
		if (dist < bestDist || (dist == bestDist && link.isJump)) {
			bestDist = dist;
			closestLink = link.id;
		}
	}

	navInfo += "\n   Links:";
	for (int i = 0; i < nav.links.size(); i++) {
		NavSectorLink& link = *nav.links[i];
		NavSector& targ = *link.target;
		string arrow = link.blocked(pActor) ? " -X> " : " --> ";
		navInfo += "\n      " + to_string(link.id) + arrow + to_string(link.target->id);
		vector<BotGoal>& targtriggers = targ.getTriggers();
		if (targtriggers.size())
			navInfo += " (" + to_string(targtriggers.size()) + " T)";
		if (link.isCliff) { navInfo += " C"; }
		if (link.isTeleport) { navInfo += " T"; }
		if (link.isJump) { navInfo += " J"; }

		if (closestLink == link.id) {
			RouteOpts opts;
			opts.actor = pActor;
			float dist = g_wb_nav.path_dist(link);
			float cost = g_wb_nav.path_cost(link, dist, opts);

			navInfo += " *";
			navInfo += "\n         dist: " + to_string((int)dist >> 16);
			navInfo += "\n         cost: " + to_string((int)cost);
		}
	}

	int closestSeg = -1;
	bestDist = FLT_MAX;
	for (int i = 0; i < sub.numsegs; i++) {
		MapSeg& seg = g_map.segs[sub.firstseg + i];
		float dist = fabs(DistanceToLine(playerPos, seg.v1, seg.v2));
		if (dist < bestDist) {
			bestDist = dist;
			closestSeg = sub.firstseg + i;
		}
	}
	navInfo += "\n   Segs: " + to_string(sub.numsegs);
	navInfo += "\n   Seg: " + to_string(closestSeg);

	navInfo += "\n   Sector: " + to_string(nav.sector->id);
	if (nav.sector->special())
		navInfo += "\n      Special: " + to_string(nav.sector->special());

	vec3 forward, right;
	MakeVectors(get_actor_angle(pActor), forward, right);
	vec3 start = playerPos + vec3(0, 0, get_player_viewheight(player));
	MapSector* sector = g_map.GetSector(start.x, start.y);
	TraceResult tr;
	if (TraceLine(start, start + forward * 64, false, pActor, &tr))
	{
		MapLine* line = tr.line;

		if (line) {
			navInfo += "\nLine " + to_string(tr.line - g_map.lines) + ":";

			if (line->special()) {
				navInfo += "\n   Special: " + to_string(line->special()) + "\n   Tags:";
				for (int i = 0; i < 5; i++)
					navInfo += " " + to_string(line->getArg(i));
				navInfo += "\n   Use Sector: " + to_string(g_map.line_subsectors[line - g_map.lines]);
			}
		}
		if (tr.actor) {
			navInfo += string("\nActor ") + get_actor_type_name(tr.actor) + ":";
			navInfo += "\n   radius: " + to_string(get_actor_radius(tr.actor));
		}
	}

	navInfo += "\nOrigin: " + to_string((int)playerPos.x)
		+ " "+ to_string((int)playerPos.y)
		+ " " + to_string((int)playerPos.z);

	int yaw = (int)((uint64_t)get_actor_angle(pActor) * 360 / 0x100000000ULL);
	int pitch = (int)((uint64_t)get_actor_pitch(pActor) * 360 / 0x100000000ULL);
	navInfo += "\nAngles: " + to_string(yaw) + " " + to_string(pitch);

	bool isClipped = IsBoxClipped(playerPos, get_actor_radius(pActor), DUCK_HEIGHT);
	//navInfo += string("\nClipped: ") + (isClipped ? "Yes" : "No");

	print_hud_test(navInfo.c_str(), 0.94f, 0.5f, 1234);
}

void wbot_debug(CWootBot* pBot) {
	wbot_debug_player_nav();

	player_t* pPlayer = pBot->m_pPlayer;
	AActor* pActor = (AActor*)pBot->pActor;
	vec3 playerPos = get_actor_pos(pActor);
	g_wb_nav.draw_nodes(pActor);

	int thisSubId = g_wb_nav.get_nav_id(pPlayer);

	string routeStr = "Route: " + to_string(thisSubId);
	if (pBot->m_routeController.pretendRouteSector >= 0) {
		routeStr += " (pretend " + to_string(pBot->m_routeController.pretendRouteSector) + " )";
	}
	routeStr += " -> ";
	for (int i = 0; i < pBot->m_routeController.m_route.route.size() && i < 4; i++) {
		if (i != 0)
			routeStr += " ";
		routeStr += to_string(pBot->m_routeController.m_route.route[i]);
	}
	if (pBot->m_routeController.m_route.route.size() > 4) {
		routeStr += " (+" + to_string(pBot->m_routeController.m_route.route.size() - 4) + ")";
	}
	routeStr += "\n                     " + to_string(pBot->m_routeController.m_route.dist) + " units";

	if (pBot->m_routeController.m_route.route.size() > 1) {
		NavSector& nav = g_wb_nav.mesh.nodes[pBot->m_routeController.m_route.route[0]];
		NavSectorLink* link = nav.getLink(pBot->m_routeController.m_route.route[1]);
		if (link) {
			routeStr += "\nLink: " + to_string(link->id) + " -> " + to_string(link->target->id);
		}
	}

	int curSpeed = pBot->GetSpeed2D();
	routeStr += "\nSpeed: " + to_string(curSpeed) + " -> " + to_string(pBot->m_routeController.m_routeSpeed);
	if (pBot->m_speedMult != 1.0f)
		routeStr += " * " + to_string((int)pBot->m_speedMult) + "." + to_string((int)(pBot->m_speedMult * 10) % 10);

	int jumpState = pBot->m_routeController.jumpState;
	int walkState = pBot->m_routeController.walkNodeState;
	string stateStr = "State:";
	if (walkState == WBOT_WALK_NODE_EDGE) { stateStr += " WALK_EDGE"; }
	if (walkState == WBOT_WALK_NODE_CENTER) { stateStr += " WALK_CENTER"; }
	if (jumpState == WBOT_JUMP_PREP) { stateStr += " JUMP_PREP"; }
	if (jumpState == WBOT_JUMP_RUN) { stateStr += " JUMP_RUN"; }
	if (jumpState == WBOT_JUMP_LAUNCH) { stateStr += " JUMP_LAUNCH"; }
	if (jumpState == WBOT_JUMP_FLY) { stateStr += " JUMP_FLY"; }
	if (pBot->stateFlags & FL_WBOT_WAIT_ELEV) { stateStr += " WAIT_ELEV"; }
	if (pBot->stateFlags & FL_WBOT_ON_ELEV) { stateStr += " ON_ELEV"; }
	if (pBot->stateFlags & FL_WBOT_WAIT_DOOR) { stateStr += " WAIT_DOOR"; }
	if (pBot->stateFlags & FL_WBOT_FLYING) { stateStr += " FLY"; }
	if (pBot->stateFlags & FL_WBOT_OVERHANG) { stateStr += " OVERHANG"; }
	if (pBot->stateFlags & FL_WBOT_RUSHING) { stateStr += " RUSH"; }
	if (pBot->stateFlags & FL_WBOT_SLOW_DOWN) { stateStr += " SLOW_DOWN"; }
	if (is_player_frozen(pPlayer)) { stateStr += " FROZEN"; }

	string btnStr = "Buttons: ";
	if (pBot->m_lButtons & IN_ATTACK) btnStr += " ATTACK";
	if (pBot->m_lButtons & IN_USE) btnStr += " USE";
	if (pBot->m_lButtons & IN_JUMP) btnStr += " JUMP";
	if (pBot->m_lButtons & IN_DUCK) btnStr += " DUCK";

	string stuckStr = "Stuck: " + to_string(pBot->stuckCounter) + ", goals " + to_string(pBot->goalFailCounter);

	string enemyStr = "Enemy: <none>";
	if (pBot->target) {
		AActor* targ = pBot->target;
		int dist = ((vec2)playerPos - get_actor_pos(targ)).length();
		enemyStr = string("Enemy: ") + get_actor_type_name(targ)
			+ ", Dist: " + to_string(dist);
	}

	string weaponStr = "Weapons:";
	for (AActor* item : get_player_weapons((APlayerPawn*)pActor, false)) {
		string wepname = get_actor_type_name(item);
		WeaponInfo& info = g_wbot_weapon_info[wepname];
		weaponStr += "\n   " + wepname + " "
			+ to_string(get_weapon_ammo(item))
			+ " p" + to_string(info.priority);
		//+ ", [" + to_string(info.minRange) + "," + to_string(info.idealRange) + "," + to_string(info.maxRange) + "] range";
		if (pBot->m_weaponName == wepname) {
			weaponStr += " <--";
		}
	}

	string goalStr = "Goals:";
	for (int i = 0; i < pBot->m_goals.size(); i++) {
		goalStr += "\n   " + pBot->m_goals[i].descLong();
	}

	string botInfo = enemyStr + "\n" + weaponStr + "\n" + routeStr + "\n"
		+ stateStr + "\n" + btnStr + "\n" + stuckStr + "\n" + goalStr;

	print_hud_test(botInfo.c_str(), 0, 0.5f, 1235);
}
