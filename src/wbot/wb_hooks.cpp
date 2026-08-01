#include "wb_util.h"
#include "wb_bot.h"
#include "wb_map.h"
#include "wb_nav.h"
#include "wb_eiface.h"
#include "wb_hooks.h"

#include <string>
#include <cstring>

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

void wbot_handle_line_activation(line_t* eline, AActor* activator) {
	MapLine* line = get_map_line_from_engine_line(eline);

	if (!line->special()) {
		// line can no longer be activated. Remove trigger from possibly affected sectors
		int tag = line->getArg(0);
		for (int i = 0; i < g_map.numsectors; i++) {
			if (g_map.sectors[i].tag == tag)
				g_map.remove_invalid_goals(i);
		}

		if (line->backsector)
			g_map.remove_invalid_goals(line->backsector->id);
		if (line->frontsector)
			g_map.remove_invalid_goals(line->frontsector->id);
	}

	for (int i = 0; i < MAXPLAYERS; i++) {
		player_t* player = get_player_for_index(i);

		if (!player)
			continue;

		CWootBot* bot = get_player_bot(player);
		if (bot)
			bot->HandleLineActivation(line, activator);
	}
}

void kill_everything() {
	kill_all_shootables();

	// do it again a second later to kill lost souls that spawn from pain elementals
	g_kill_all_shootables_again_tick = get_game_tics() + 35;
}

void wbot_run_tests() {
	kill_all_shootables();

	for (int i = 0; i < MAXPLAYERS; i++) {
		CWootBot* bot = get_bot_for_index(i);

		if (!bot)
			continue;

		bot->Reset();
		bot->m_autoWinMap = true;
		bot->m_debug = false;
	}

	g_wb_test_state = TestState();
	g_wbot_test_mode = true;
	g_GameSpeed = 1000.0f;
	g_wb_test_state.startMap = get_map_name();
	g_wb_test_state.startTime = getEpochMillis();
}

void wbot_init() {
	static bool wbot_init_done = false;
	if (!wbot_init_done) {
		const char* testMap = get_program_arg("-wbtest");
		if (testMap) {
			add_bot();

			if (!testMap[0]) {
				wbot_run_tests(); // run all tests
				g_wb_test_state.debugFailed = get_program_arg("-wbd");
			}
		}

		wbot_init_done = true;
	}
}

void wbot_next_test() {
	if (g_wbot_test_mode && g_wb_test_state.startMap == string(get_map_name())) {
		g_wbot_test_mode = false;
		g_GameSpeed = 1.0f;
		uint32_t totalTime = getEpochMillis() - g_wb_test_state.startTime;

		uint32_t total_gen_time = 0;
		uint32_t total_solve_time = 0;
		int numPass = 0;
		gprintf("\n---------------------\nTESTS FINISHED\n---------------------\n");
		for (TestResult& result : g_wb_test_state.results) {
			gprintf("  %-8s = %4dms gen,   %4dms solve,   %4.1f ktics  %s\n", result.mapname.c_str(),
				result.gen_nav_millis, result.solve_millis, result.tics / 1000.0f,
				result.success ? "PASS" : "FAIL <--");
			numPass += result.success;
			total_gen_time += result.gen_nav_millis;
			total_solve_time += result.solve_millis;
		}

		gprintf("\n%d / %d tests passed\n\n", numPass, g_wb_test_state.results.size());

		gprintf("Total time:  %d ms\n", totalTime);
		gprintf("navmesh:     %d ms\n", total_gen_time);
		gprintf("solver:      %d ms\n", total_solve_time);
		gprintf("tics:        %d ktics\n", g_wb_test_state.totalTics / 1000);
		gprintf("---------------------\n");

		if (g_windows_console_mode) {
			gprintf("\nPress Enter to exit...");
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
		CWootBot* bot = get_bot_for_index(i);

		if (!bot)
			continue;

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
		if (sz > 0 && get_map_name() == g_wb_test_state.results[sz - 1].mapname)
			return; // level exits are sometimes triggered multiple times

		int testTime = get_game_tics();
		uint32_t millis = getEpochMillis() - g_wb_test_state.levelStartTime;
		g_wb_test_state.totalTics += get_game_tics();
		gprintf("Finished level in %d tics (%d ms)\n", testTime, millis);

		TestResult result;
		result.mapname = get_map_name();
		result.success = !g_wb_test_state.currentFailed;
		result.tics = get_game_tics();
		result.solve_millis = millis;
		result.gen_nav_millis = g_wb_test_state.levelGenMillis;
		g_wb_test_state.results.push_back(result);

		g_wb_test_state.currentFailed = false;
	}
}

void wbot_abort_test() {
	gprintf("---------------------\nTEST FAILED\n---------------------\n");

	g_wb_test_state.currentFailed = true;

	if (g_wb_test_state.debugFailed) {
		g_GameSpeed = 1.0f;
		g_wbot_test_mode = false;

		for (int i = 0; i < MAXPLAYERS; i++) {
			CWootBot* bot = get_bot_for_index(i);

			if (!bot)
				continue;

			bot->m_debug = true;
		}

		gprintf("\nFailed after %d tics\n", get_game_tics());
		gprintf("\nJoin the server to see what the bot is stuck on.\n");
	}
	else {
		exit_level();
	}
}

void wbot_handle_chat_command(unsigned int ulPlayer, const char* msg) {
	if (!are_cheats_enabled())
		return;

	// route test case
	if (!strcmp(msg, "r")) {
		kill_everything();

		for (int i = 0; i < MAXPLAYERS; i++) {
			player_t* player = get_player_for_index(i);
			CWootBot* bot = get_bot_for_index(i);

			if (!player)
				continue;

			AActor* actor = (AActor*)get_player(player);

			if (bot)	set_actor_origin(actor, -1771, -1531, 0, false);
			else		set_actor_origin(actor, -1630, -1498, 110, false);

			if (bot) {
				bot->Reset();
				bot->m_followPlayer = true;
				freeze_player(player, false);
				//bot->m_routeController.m_freezeOnGoalFail = true;
				//bot->m_speedMult = 0.6f;
			}
		}
	}

	// change game speed
	if (strstr(msg, "s") == msg) {
		g_GameSpeed = atof(msg + 2);
		if (g_GameSpeed < 1.0f) g_GameSpeed = 1.0f;
		if (g_GameSpeed > 16.0f) g_GameSpeed = 16.0f;
	}

	// win the level
	if (!strcmp(msg, "w")) {
		kill_everything();

		for (int i = 0; i < MAXPLAYERS; i++) {
			CWootBot* bot = get_bot_for_index(i);

			if (!bot)
				continue;
			
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

		if (id >= 0 && id < g_map.numlines) {
			MapLine* line = &g_map.lines[id];

			BotGoal useGoal(get_line_state(id).goalAction, id);

			for (int i = 0; i < MAXPLAYERS; i++) {
				CWootBot* bot = get_bot_for_index(i);

				if (!bot)
					continue;

				bot->PushGoal(useGoal, NULL);
			}
		}
	}

	// use line test case
	if (strstr(msg, "u") == msg) {
		int id = 685;

		if (id >= 0 && id < g_map.numlines) {
			MapLine* line = &g_map.lines[id];

			BotGoal useGoal(get_line_state(id).goalAction, id);

			for (int i = 0; i < MAXPLAYERS; i++) {
				CWootBot* bot = get_bot_for_index(i);

				if (!bot)
					continue;

				bot->Reset();
				set_actor_origin((AActor*)bot->pActor, 653, -1008, 0, false);
				bot->PushGoal(useGoal, NULL);
			}
		}
	}

	// follow player when done with goals
	if (!strcmp(msg, "follow")) {
		for (int i = 0; i < MAXPLAYERS; i++) {
			CWootBot* bot = get_bot_for_index(i);

			if (!bot)
				continue;

			bot->m_followPlayer = true;
		}
	}

	if (!strcmp(msg, "add")) {
		add_bot();
	}

	// clear all enemies for general pathfinding tests
	if (!strcmp(msg, "y")) {
		kill_everything();
	}

	// restart the bot
	if (!strcmp(msg, "stop")) {
		for (int i = 0; i < MAXPLAYERS; i++) {
			CWootBot* bot = get_bot_for_index(i);

			if (!bot)
				continue;

			bot->Reset();
			bot->m_followPlayer = false;
			bot->m_autoWinMap = false;
		}
	}

	if (!strcmp(msg, "restart")) {
		change_level(get_map_name(), true);
	}

	if (!strcmp(msg, "kill")) {
		for (int i = 0; i < MAXPLAYERS; i++) {
			CWootBot* bot = get_bot_for_index(i);

			if (!bot)
				continue;

			kill_actor((AActor*)bot->pActor);
		}
	}

	// teleport the bot to the player
	if (!strcmp(msg, "tp")) {
		player_t* target = getAnyPlayer();

		if (target) {
			AActor* actor = (AActor*)get_player(target);
			vec3 pos = get_actor_state(actor).origin;
			pos.x += (PLAYER_WIDTH + 1);

			for (int i = 0; i < MAXPLAYERS; i++) {
				CWootBot* bot = get_bot_for_index(i);

				if (!bot)
					continue;

				set_actor_origin((AActor*)bot->pActor, pos.x, pos.y, 0, true);
			}
		}
	}

	// go to a subsector
	if (strstr(msg, "gotos ") == msg) {
		int id = atoi(msg + 5);
		if (id >= 0 && id < g_map.numsubsectors) {
			NavSector& nav = g_wb_nav.mesh.nodes[id];
			vec3 pos = nav.pos3D();
			AActor* player = (AActor*)get_player(get_player_for_index(ulPlayer));
			set_actor_origin(player, pos.x, pos.y, 0, true);
		}
	}

	// go to a link
	if (strstr(msg, "gotol ") == msg) {
		int id = atoi(msg + 5);

		bool found = false;
		for (int i = 0; i < g_map.numsubsectors && !found; i++) {
			NavSector& nav = g_wb_nav.mesh.nodes[i];
			for (int k = 0; k < nav.links.size(); k++) {
				if (nav.links[k]->id == id) {
					vec3 pos = nav.links[k]->pos3D();
					AActor* player = (AActor*)get_player(get_player_for_index(ulPlayer));
					set_actor_origin(player, pos.x, pos.y, 0, true);
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

			vec3 pos(x, y, 0);
			AActor* player = (AActor*)get_player(get_player_for_index(ulPlayer));
			set_actor_origin(player, pos.x, pos.y, 0, true);
		}
	}

	// give weapons/ammo for combat testing
	if (!strcmp(msg, "x")) {
		for (int i = 0; i < MAXPLAYERS; i++) {
			player_t* player = get_player_for_index(i);

			if (player)
				give_all_weapons(player);
		}
	}

	// slowmotion bots
	if (!strcmp(msg, "f") || !strcmp(msg, "s")) {
		bool slowMotion = !strcmp(msg, "s");
		for (int i = 0; i < MAXPLAYERS; i++) {
			CWootBot* bot = get_bot_for_index(i);

			if (!bot)
				continue;

			bot->m_speedMult = slowMotion ? 0.1f : 1.0f;
		}
	}
}

void wbot_tick() {
	g_wb_nav.relink_pending_sector();

	if (g_kill_all_shootables_again_tick && get_game_tics() > g_kill_all_shootables_again_tick) {
		kill_everything();
		g_kill_all_shootables_again_tick = 0;
	}

	if (g_wbot_test_mode && get_game_tics() >= MAX_TEST_TICS) {
		wbot_abort_test();
	}
}

void wbot_tick(CWootBot* pBot) {
	simulate_bot(pBot);
}

void wbot_pre_delete(CWootBot* pBot) {
	pre_remove_bot(pBot);
}

