#include "wb_util.h"
#include "wb_bot.h"
#include "wb_map.h"
#include "wb_nav.h"
#include "wb_eiface.h"
#include "c_dispatch.h"
#include "m_cheat.h"
#include "sbar.h"
#include "sv_commands.h"
#include "deathmatch.h"
#include <string>

using namespace std;
using namespace wbot;

extern float g_GameSpeed;
extern bool g_windows_console_mode;

int g_kill_all_shootables_again_tick = 0;
bool g_wbot_test_mode = false;

struct TestResult {
	string mapname;
	uint32_t tics;
	uint32_t solve_millis; // time taken to solve the map
	uint32_t gen_nav_millis; // time taken to generate a nav mesh for the map
	bool success;
};

struct TestState {
	bool currentFailed = false;
	bool debugFailed = false;
	uint64_t levelStartTime = 0;
	uint32_t levelGenMillis = 0;
	uint64_t startTime = 0;
	uint32_t totalTics = 0;
	string startMap;
	vector<TestResult> results;
};

TestState g_wb_test_state;

// max tics a level can last before its considered stuck
#define MAX_TEST_TICS 21000 // 10 minutes

void wbot_handle_line_activation(line_t* line, AActor* activator) {
	MapLine& mapline = g_map.lines[line - lines];

	if (!line->special) {
		// line can no longer be activated. Remove trigger from possibly affected sectors
		int tag = line->args[0];
		for (int i = 0; i < numsectors; i++) {
			if (sectors[i].tag == tag)
				g_map.remove_invalid_goals(i);
		}

		if (line->backsector)
			g_map.remove_invalid_goals(line->backsector - sectors);
		if (line->frontsector)
			g_map.remove_invalid_goals(line->frontsector - sectors);
	}

	for (int i = 0; i < MAXPLAYERS; i++) {
		AActor* player = players[i].mo;
		if (!playeringame[i] || !player || !player->player->bIsBot)
			continue;

		CWootBot* bot = player->player->pWootBot;
		bot->HandleLineActivation(&mapline, activator);
	}
}

void kill_all_shootables() {
	TThinkerIterator<AActor> it;
	AActor* actor;
	while ((actor = it.Next())) {
		if (!strcmp(actor->GetClass()->TypeName.GetChars(), "BossEye")) {
			P_RemoveThing(actor);
			continue;
		}

		if ((actor->flags & MF_SHOOTABLE) && !actor->player) {
			if (!strcmp(actor->GetClass()->TypeName.GetChars(), "BossBrain")) {
				continue;
			}
			P_DamageMobj(actor, actor, actor, actor->health * 2, FName());
		}
	}

	// do it again a second later to kill lost souls that spawn from pain elementals
	g_kill_all_shootables_again_tick = level.time + 35;
}

void wbot_run_tests() {
	kill_all_shootables();

	for (int i = 0; i < MAXPLAYERS; i++) {
		AActor* player = players[i].mo;
		if (!playeringame[i] || !player || !player->player->bIsBot)
			continue;

		CWootBot* bot = (CWootBot*)player->player->pWootBot;
		bot->Reset();
		bot->m_autoWinMap = true;
		bot->m_debug = false;
	}

	g_wb_test_state = TestState();
	g_wbot_test_mode = true;
	g_GameSpeed = 1000.0f;
	g_wb_test_state.startMap = level.mapname;
	g_wb_test_state.startTime = getEpochMillis();
}

void wbot_init() {
	static bool wbot_init_done = false;
	if (!wbot_init_done) {
		srand((unsigned int)time(NULL));

		int anum = Args->CheckParm("-wbtest");
		if (anum) {
			const char* testMap = Args->CheckValue("-wbtest");
			new CWootBot(NULL, NULL, BOTS_FindFreePlayerSlot());

			if (!testMap) {
				wbot_run_tests(); // run all tests
				g_wb_test_state.debugFailed = Args->CheckParm("-wbd");
			}
		}

		wbot_init_done = true;
	}
}

void wbot_next_test() {
	if (g_wbot_test_mode && g_wb_test_state.startMap == string(level.mapname)) {
		g_wbot_test_mode = false;
		g_GameSpeed = 1.0f;
		uint32_t totalTime = getEpochMillis() - g_wb_test_state.startTime;

		uint32_t total_gen_time = 0;
		uint32_t total_solve_time = 0;
		int numPass = 0;
		Printf("\n---------------------\nTESTS FINISHED\n---------------------\n");
		for (TestResult& result : g_wb_test_state.results) {
			Printf("  %-8s = %4dms gen,   %4dms solve,   %4.1f ktics  %s\n", result.mapname.c_str(),
				result.gen_nav_millis, result.solve_millis, result.tics / 1000.0f,
				result.success ? "PASS" : "FAIL <--");
			numPass += result.success;
			total_gen_time += result.gen_nav_millis;
			total_solve_time += result.solve_millis;
		}

		Printf("\n%d / %d tests passed\n\n", numPass, g_wb_test_state.results.size());

		Printf("Total time:  %d ms\n", totalTime);
		Printf("navmesh:     %d ms\n", total_gen_time);
		Printf("solver:      %d ms\n", total_solve_time);
		Printf("tics:        %d ktics\n", g_wb_test_state.totalTics / 1000);
		Printf("---------------------\n");

		if (g_windows_console_mode) {
			printf("\nPress Enter to exit...");
			getchar(); // keep console open to see test results
		}

		// exit program when tests finish
		std::exit(numPass < g_wb_test_state.results.size() ? 1 : 0);
	}
}

void wbot_map_init() {
	init_eiface();
	g_map.init();

	wbot_next_test();
	wbot_init();

	uint64_t nav_gen_start = getEpochMillis();
	g_wb_nav.init();
	g_wb_test_state.levelGenMillis = getEpochMillis() - nav_gen_start;

	for (int i = 0; i < MAXPLAYERS; i++) {
		AActor* player = players[i].mo;
		if (!playeringame[i] || !player || !player->player->bIsBot)
			continue;

		CWootBot* bot = player->player->pWootBot;
		bot->Reset();
	}

	if (g_wbot_test_mode) {
		kill_all_shootables();
	}

	g_wb_test_state.levelStartTime = getEpochMillis();
}

void wbot_map_exit() {
	if (g_wbot_test_mode) {
		int sz = g_wb_test_state.results.size();
		if (sz > 0 && level.mapname == g_wb_test_state.results[sz - 1].mapname)
			return; // level exits are sometimes triggered multiple times

		int testTime = level.time;
		uint32_t millis = getEpochMillis() - g_wb_test_state.levelStartTime;
		g_wb_test_state.totalTics += level.time;
		Printf("Finished level in %d tics (%d ms)\n", testTime, millis);

		TestResult result;
		result.mapname = level.mapname;
		result.success = !g_wb_test_state.currentFailed;
		result.tics = level.time;
		result.solve_millis = millis;
		result.gen_nav_millis = g_wb_test_state.levelGenMillis;
		g_wb_test_state.results.push_back(result);

		g_wb_test_state.currentFailed = false;
	}
}

void wbot_add_bot() {
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

void wbot_abort_test() {
	Printf("---------------------\nTEST FAILED\n---------------------\n");

	g_wb_test_state.currentFailed = true;

	if (g_wb_test_state.debugFailed) {
		g_GameSpeed = 1.0f;
		g_wbot_test_mode = false;

		for (int i = 0; i < MAXPLAYERS; i++) {
			AActor* player = players[i].mo;
			if (!playeringame[i] || !player || !player->player->bIsBot)
				continue;

			CWootBot* bot = player->player->pWootBot;
			bot->m_debug = true;
		}

		Printf("\nFailed after %d tics\n", level.time);
		Printf("\nJoin the server to see what the bot is stuck on.\n");
	}
	else {
		G_ExitLevel(0, false);
	}
}

void wbot_handle_chat_command(unsigned int ulPlayer, const char* msg) {
	if (!sv_cheats)
		return;

	// route test case
	if (!strcmp(msg, "r")) {
		kill_all_shootables();

		for (int i = 0; i < MAXPLAYERS; i++) {
			AActor* player = players[i].mo;
			if (!playeringame[i] || !player)
				continue;

			if (player->player->bIsBot)	set_ori(player, -1771, -1531, ANGLE_1 * 0);
			else						set_ori(player, -1630, -1498, ANGLE_1 * 110);

			if (player->player->bIsBot) {
				CWootBot* bot = player->player->pWootBot;
				bot->Reset();
				bot->m_followPlayer = true;
				player->player->cheats &= ~CF_FROZEN;
				//bot->m_routeController.m_freezeOnGoalFail = true;
				//bot->m_speedMult = 0.6f;
			}
		}
	}

	// change game speed
	if (strstr(msg, "s") == msg) {
		g_GameSpeed = clamp((float)atof(msg + 2), 1.0f, 16.0f);
	}

	// win the level
	if (!strcmp(msg, "w")) {
		kill_all_shootables();

		for (int i = 0; i < MAXPLAYERS; i++) {
			AActor* player = players[i].mo;
			if (!playeringame[i] || !player || !player->player->bIsBot)
				continue;

			CWootBot* bot = player->player->pWootBot;
			//set_ori(player, 3211, -131, ANGLE_1 * 40);
			bot->PushLevelEndGoal();
			bot->m_autoWinMap = true;
		}
	}

	// run tests
	if (!strcmp(msg, "test")) {
		wbot_run_tests();
	}

	// add a line use goal
	if (strstr(msg, "use ") == msg) {
		int id = atoi(msg + 4);

		if (id >= 0 && id < numlines) {
			MapLine* line = &g_map.lines[id];

			BotGoal useGoal(get_linedef_goal_action(line), id);

			for (int i = 0; i < MAXPLAYERS; i++) {
				AActor* player = players[i].mo;
				if (!playeringame[i] || !player || !player->player->bIsBot)
					continue;

				CWootBot* bot = player->player->pWootBot;
				bot->PushGoal(useGoal, NULL);
			}
		}
	}

	// use line test case
	if (strstr(msg, "u") == msg) {
		int id = 685;

		if (id >= 0 && id < numlines) {
			MapLine* line = &g_map.lines[id];

			BotGoal useGoal(get_linedef_goal_action(line), id);

			for (int i = 0; i < MAXPLAYERS; i++) {
				AActor* player = players[i].mo;
				if (!playeringame[i] || !player || !player->player->bIsBot)
					continue;

				CWootBot* bot = player->player->pWootBot;
				bot->Reset();
				set_ori(player, 653, -1008, ANGLE_1 * 0);
				bot->PushGoal(useGoal, NULL);
			}
		}
	}

	// follow player when done with goals
	if (!strcmp(msg, "follow")) {
		for (int i = 0; i < MAXPLAYERS; i++) {
			AActor* player = players[i].mo;
			if (!playeringame[i] || !player || !player->player->bIsBot)
				continue;

			CWootBot* bot = player->player->pWootBot;
			bot->m_followPlayer = true;
		}
	}

	if (!strcmp(msg, "add")) {
		wbot_add_bot();
	}

	// clear all enemies for general pathfinding tests
	if (!strcmp(msg, "y")) {
		kill_all_shootables();
	}

	// restart the bot
	if (!strcmp(msg, "stop")) {
		for (int i = 0; i < MAXPLAYERS; i++) {
			AActor* player = players[i].mo;
			if (!playeringame[i] || !player || !player->player->bIsBot)
				continue;

			CWootBot* bot = player->player->pWootBot;
			bot->Reset();
			bot->m_followPlayer = false;
			bot->m_autoWinMap = false;
		}
	}

	if (!strcmp(msg, "restart")) {
		G_ChangeLevel(level.mapname, 0, CHANGELEVEL_NOINTERMISSION);
	}

	if (!strcmp(msg, "kill")) {
		for (int i = 0; i < MAXPLAYERS; i++) {
			AActor* player = players[i].mo;
			if (!playeringame[i] || !player || !player->player->bIsBot)
				continue;

			P_DamageMobj(player, player, player, player->health * 2, FName());
		}
	}

	// teleport the bot to the player
	if (!strcmp(msg, "tp")) {
		player_t* target = getAnyPlayer();

		if (target) {
			AActor* actor = (AActor*)get_player(target);
			FVector3 pos = get_actor_pos(actor);
			pos.X += ((PLAYER_WIDTH + 1) << FRACBITS);

			for (int i = 0; i < MAXPLAYERS; i++) {
				AActor* player = players[i].mo;
				if (!playeringame[i] || !player || !player->player->bIsBot)
					continue;

				P_Teleport(player, pos.X, pos.Y, pos.Z, 0, true, true, true);
			}
		}
	}

	// go to a subsector
	if (strstr(msg, "gotos ") == msg) {
		int id = atoi(msg + 5);
		if (id >= 0 && id < numsubsectors) {
			NavSector& nav = g_wb_nav.mesh.nodes[id];
			FVector3 pos = nav.pos3D();
			P_Teleport(players[ulPlayer].mo, pos.X, pos.Y, pos.Z, 0, true, true, true);
		}
	}

	// go to a link
	if (strstr(msg, "gotol ") == msg) {
		int id = atoi(msg + 5);

		bool found = false;
		for (int i = 0; i < numsubsectors && !found; i++) {
			NavSector& nav = g_wb_nav.mesh.nodes[i];
			for (int k = 0; k < nav.links.size(); k++) {
				if (nav.links[k]->id == id) {
					FVector3 pos = nav.links[k]->pos3D();
					P_Teleport(players[ulPlayer].mo, pos.X, pos.Y, pos.Z, 0, true, true, true);
					found = true;
					break;
				}
			}
		}
	}

	// goto coordinates
	if (strstr(msg, "goto ") == msg) {
		std::string args = msg + 5;
		int splitter = args.find(" ");
		if (splitter != -1) {
			int x = atoi(args.substr(0, splitter).c_str());
			int y = atoi(args.substr(splitter + 1).c_str());

			FVector3 pos(x << FRACBITS, y << FRACBITS, 0);
			P_Teleport(players[ulPlayer].mo, pos.X, pos.Y, pos.Z, 0, true, true, true);
		}
	}

	// give weapons/ammo for combat testing
	if (!strcmp(msg, "x")) {
		for (int i = 0; i < MAXPLAYERS; i++) {
			AActor* player = players[i].mo;
			if (!playeringame[i] || !player)
				continue;

			cht_Give(player->player, "backpack");
			cht_Give(player->player, "weapons");
			cht_Give(player->player, "ammo");
			cht_Give(player->player, "keys");
			cht_Give(player->player, "armor");
		}
	}

	// slowmotion bots
	if (!strcmp(msg, "f") || !strcmp(msg, "s")) {
		bool slowMotion = !strcmp(msg, "s");
		for (int i = 0; i < MAXPLAYERS; i++) {
			AActor* player = players[i].mo;
			if (!playeringame[i] || !player || !player->player->bIsBot)
				continue;

			CWootBot* bot = player->player->pWootBot;
			bot->m_speedMult = slowMotion ? 0.1f : 1.0f;
		}
	}
}

void wbot_tick() {
	g_wb_nav.relink_pending_sector();

	if (g_kill_all_shootables_again_tick && level.time > g_kill_all_shootables_again_tick) {
		kill_all_shootables();
		g_kill_all_shootables_again_tick = 0;
	}

	if (g_wbot_test_mode && level.time >= MAX_TEST_TICS) {
		wbot_abort_test();
	}
}

void wbot_tick(CWootBot* pBot) {
	ticcmd_t* cmd = &pBot->m_pPlayer->cmd;

	// Don't execute bot logic during demos, or if the console player is a client.
	//if (NETWORK_InClientMode() || (demoplayback)) {
	//	return;
	//}

	// Reset the bots keypresses.
	memset(cmd, 0, sizeof(ticcmd_t));

	// Don't run their script if the game is frozen.
	if (level.flags2 & LEVEL2_FROZEN)
		return;

	// [BB] Don't run their script if they are frozen either.
	if (pBot->m_pPlayer->cheats & CF_TOTALLYFROZEN)
	{
		// [BB] Don't freeze dead bots. Otherwise they can't respawn.
		if (pBot->m_pPlayer->mo && pBot->m_pPlayer->mo->health > 0)
			return;
	}

	player_t* plr = pBot->m_pPlayer;
	APlayerPawn* actor = pBot->m_pPlayer->mo;
	pBot->m_health = actor->health;
	pBot->m_origin = FVector3(actor->x, actor->y, actor->z);
	pBot->m_velocity = FVector3(actor->velx, actor->vely, actor->velz);
	pBot->m_viewHeight = plr->viewheight;
	pBot->m_useDistance = actor->UseRange >> FRACBITS;
	pBot->m_radius = actor->radius;
	pBot->m_isFrozen = plr->cheats & CF_FROZEN;
	pBot->m_onGround = plr->onground;
	pBot->m_pitch = actor->pitch;
	pBot->m_weaponName = NULL;
	pBot->pActor = actor;
	if (plr->ReadyWeapon)
		pBot->m_weaponName = plr->ReadyWeapon->GetClass()->TypeName.GetChars();

	pBot->Think();

	actor->pitch = pBot->m_pitch;
	actor->angle = pBot->m_yaw;

	// [AK] Don't allow the bot to move while frozen.
	if ((pBot->m_pPlayer->cheats & CF_FROZEN) == false)
	{
		pBot->m_pPlayer->cmd.ucmd.forwardmove = static_cast<short>(pBot->m_lForwardMove << 8);
		pBot->m_pPlayer->cmd.ucmd.sidemove = static_cast<short>(pBot->m_lSideMove << 8);
	}
	else
	{
		pBot->m_pPlayer->cmd.ucmd.forwardmove = pBot->m_pPlayer->cmd.ucmd.sidemove = 0;
	}

	pBot->m_pPlayer->cmd.ucmd.buttons |= pBot->m_lButtons;
}

void wbot_pre_delete(CWootBot* pBot) {
	// If this player is the displayplayer, revert the camera back to the console player's eyes.
	if (pBot->m_pPlayer->mo && (pBot->m_pPlayer->mo->CheckLocalView(consoleplayer)) && (NETWORK_GetState() != NETSTATE_SERVER))
	{
		players[consoleplayer].camera = players[consoleplayer].mo;
		S_UpdateSounds(players[consoleplayer].camera);
		StatusBar->AttachToPlayer(&players[consoleplayer]);
	}

	// Remove the bot from the game.
	playeringame[(pBot->m_pPlayer - players)] = false;

	// Delete the actor attached to the player.
	if (pBot->m_pPlayer->mo)
		pBot->m_pPlayer->mo->Destroy();

	// [RK] Remove the corpse's thinkers to prevent a crash later
	if ((NETWORK_GetState() == NETSTATE_SINGLE || NETWORK_GetState() == NETSTATE_SINGLE_MULTIPLAYER) && pBot->m_pPlayer->mo) {
		TThinkerIterator<APlayerPawn> it;
		APlayerPawn* pawn, * next;

		next = it.Next();
		while ((pawn = next) != NULL)
		{
			next = it.Next();

			if ((pawn->player == NULL) && (pBot->m_pPlayer->mo->id == pawn->id))
				pawn->Destroy();
		}
	}

	// Finally, fix some pointers.
	// [BB] We have to delete the CSkullBot pointer before setting it to NULL.
	//m_pPlayer->pWootBot = NULL;
	pBot->m_pPlayer->mo = NULL;
	pBot->m_pPlayer = NULL;
}


CCMD(addbotw) {
	wbot_add_bot();
}
