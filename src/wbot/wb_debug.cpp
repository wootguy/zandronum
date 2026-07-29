#include "wb_debug.h"
#include "wb_bot.h"
#include "wb_nav.h"
#include "wb_util.h"
#include "wb_route.h"
#include "wb_map.h"
#include "d_event.h"
#include "sv_commands.h"

using namespace std;
using namespace wbot;

void wbot_debug_player_nav() {
	AActor* player = getAnyPlayer();
	if (!player)
		return;

	string navInfo;
	int plrnavid = g_wb_nav.get_nav_id(player);
	NavSector& nav = g_wb_nav.mesh.nodes[plrnavid];
	MapSubsector& sub = g_map.subsectors[plrnavid];

	navInfo += "Node " + to_string(plrnavid) + ":";

	//int temp;
	//AvoidLedges(player, temp);

	navInfo += "\n   Triggers:";
	vector<BotGoal>& triggers = nav.getTriggers();
	for (int i = 0; i < triggers.size(); i++) {
		navInfo += "\n      " + triggers[i].desc();
	}

	navInfo += "\n   Links:";
	for (int i = 0; i < nav.links.size(); i++) {
		NavSectorLink& link = *nav.links[i];
		NavSector& targ = *link.target;
		string arrow = link.blocked(player) ? " -X> " : " --> ";
		navInfo += "\n      " + to_string(link.id) + arrow + to_string(link.target->id);
		vector<BotGoal>& targtriggers = targ.getTriggers();
		if (targtriggers.size())
			navInfo += " (" + to_string(targtriggers.size()) + " T)";
		if (link.isCliff) { navInfo += " C"; }
		if (link.isTeleport) { navInfo += " T"; }
		if (link.isJump) { navInfo += " J"; }
	}

	FVector2 playerPos(player->x, player->y);
	int closestSeg = -1;
	fixed_t bestDist = INT_MAX;
	for (int i = 0; i < sub.numsegs; i++) {
		MapSeg& seg = g_map.segs[sub.firstseg + i];
		fixed_t dist = abs(DistanceToLine(playerPos, seg.start(), seg.end()));
		if (dist < bestDist) {
			bestDist = dist;
			closestSeg = sub.firstseg + i;
		}
	}
	navInfo += "\n   Segs: " + to_string(sub.numsegs);
	navInfo += "\n   Seg: " + to_string(closestSeg);

	navInfo += "\n   Sector: " + to_string(nav.sector->id);

	FVector3 forward, right;
	MakeVectors(player->angle, forward, right);
	FVector3 start = FVector3(player->x, player->y, player->z + player->player->viewheight);
	fixed_t testDist = 64 << FRACBITS;
	MapSector* sector = g_map.GetSector((fixed_t)start.X, (fixed_t)start.Y);
	TraceResult tr;
	if (g_map.Trace(start, start + forward, 0, ML_BLOCKEVERYTHING | ML_BLOCKHITSCAN, NULL, &tr))
	{
		MapLine* line = tr.line;
		navInfo += "\nLine " + to_string(tr.line - g_map.lines) + ":";

		if (line->special()) {
			navInfo += "\n   Special: " + to_string(line->special()) + "\n   Tags:";
			for (int i = 0; i < 5; i++)
				navInfo += " " + to_string(line->getArg(i));
		}
		navInfo += "\n   Use Sector: " + to_string(g_map.line_subsectors[line - g_map.lines]);
	}

	navInfo += "\nOrigin: " + to_string(player->x >> FRACBITS) + " " + to_string(player->y >> FRACBITS)
		+ " " + to_string(player->z >> FRACBITS);

	int yaw = (int)((uint64_t)player->angle * 360 / 0x100000000ULL);
	int pitch = (int)((uint64_t)player->pitch * 360 / 0x100000000ULL);
	navInfo += "\nAngles: " + to_string(yaw) + " " + to_string(pitch);

	bool isClipped = IsBoxClipped(FVector3(player->x, player->y, player->z), player->radius, DUCK_HEIGHT);
	//navInfo += string("\nClipped: ") + (isClipped ? "Yes" : "No");

	SERVERCOMMANDS_PrintHUDMessage(navInfo.c_str(), 0.94f, 0.5f, 0, 0, 0, CR_RED, 1.0f, 0, 0, "SmallFont", MAKE_ID('W', 'N', 'A', 'V'));
}

void wbot_debug(CWootBot* pBot) {
	wbot_debug_player_nav();

	player_t* pPlayer = pBot->GetPlayer();
	AActor* pActor = pBot->pActor;
	g_wb_nav.draw_nodes(pActor);

	int thisSubId = g_wb_nav.get_nav_id(pActor);

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
	if (jumpState == WBOT_JUMP_FLY) { stateStr += " JUMP_FLY"; }
	if (pBot->stateFlags & FL_WBOT_WAIT_ELEV) { stateStr += " WAIT_ELEV"; }
	if (pBot->stateFlags & FL_WBOT_ON_ELEV) { stateStr += " ON_ELEV"; }
	if (pBot->stateFlags & FL_WBOT_WAIT_DOOR) { stateStr += " WAIT_DOOR"; }
	if (pBot->stateFlags & FL_WBOT_FLYING) { stateStr += " FLY"; }
	if (pBot->stateFlags & FL_WBOT_RUSHING) { stateStr += " RUSH"; }
	if (pBot->stateFlags & FL_WBOT_SLOW_DOWN) { stateStr += " SLOW_DOWN"; }
	if (pPlayer->cheats & (CF_FROZEN | CF_TOTALLYFROZEN)) { stateStr += " FROZEN"; }

	string btnStr = "Buttons: ";
	if (pBot->m_lButtons & BT_ATTACK) btnStr += " ATTACK";
	if (pBot->m_lButtons & BT_USE) btnStr += " USE";
	if (pBot->m_lButtons & BT_JUMP) btnStr += " JUMP";
	if (pBot->m_lButtons & BT_CROUCH) btnStr += " CROUCH";
	if (pBot->m_lButtons & BT_TURN180) btnStr += " TURN180";
	if (pBot->m_lButtons & BT_ALTATTACK) btnStr += " ALTATTACK";
	if (pBot->m_lButtons & BT_RELOAD) btnStr += " RELOAD";
	if (pBot->m_lButtons & BT_ZOOM) btnStr += " ZOOM";
	if (pBot->m_lButtons & BT_SPEED) btnStr += " SPEED";

	string stuckStr = "Stuck: " + to_string(pBot->stuckCounter) + ", goals " + to_string(pBot->goalFailCounter);

	string enemyStr = "Enemy: <none>";
	if (pActor->target) {
		AActor* targ = pActor->target;
		fixed_t dist = P_AproxDistance(pActor->x - targ->x, pActor->y - targ->y);
		enemyStr = string("Enemy: ") + pActor->target->GetClass()->TypeName.GetChars()
			+ ", Dist: " + to_string(dist >> FRACBITS);
	}

	string weaponStr = "Weapons:";
	for (AInventory* item = pActor->Inventory; item != NULL; item = item->Inventory) {
		if (item->IsKindOf(RUNTIME_CLASS(AWeapon))) {
			AWeapon* weapon = static_cast<AWeapon*>(item);

			string wepname = weapon->GetClass()->TypeName.GetChars();
			WeaponInfo& info = g_wbot_weapon_info[wepname];
			weaponStr += "\n   " + wepname + " "
				+ to_string(weapon->Ammo1 ? weapon->Ammo1->Amount : 0)
				+ " p" + to_string(info.priority);
			//+ ", [" + to_string(info.minRange) + "," + to_string(info.idealRange) + "," + to_string(info.maxRange) + "] range";
			if (pPlayer->ReadyWeapon == weapon) {
				weaponStr += " <--";
			}
		}
	}

	string goalStr = "Goals:";
	for (int i = 0; i < pBot->m_goals.size(); i++) {
		goalStr += "\n   " + pBot->m_goals[i].descLong();
	}

	string botInfo = enemyStr + "\n" + weaponStr + "\n" + routeStr + "\n"
		+ stateStr + "\n" + btnStr + "\n" + stuckStr + "\n" + goalStr;

	SERVERCOMMANDS_PrintHUDMessage(botInfo.c_str(), 0, 0.5f, 0, 0, 0, CR_RED, 1.0f, 0, 0, "SmallFont", MAKE_ID('W', 'B', 'O', 'T'));
}
