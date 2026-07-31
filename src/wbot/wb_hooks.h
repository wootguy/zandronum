#pragma once
// place these somewhere in the engine according to their comment

struct line_t;
class AActor;
class CWootBot;

// If true, automated tests are running. Level change transitions should be skipped.
extern bool g_wbot_test_mode;

void wbot_handle_chat_command(unsigned int ulPlayer, const char* msg); // call when a player chats
void wbot_handle_line_activation(line_t* line, AActor* activator); // call when a line is activated
void wbot_map_init(); // should be called after a map is loaded, with WAD lumps available to read
void wbot_map_exit(); // should be called when a level exits, and before the next map loads
void wbot_tick(); // should be called every game tick
void wbot_tick(CWootBot* pBot); // per-player game tick. This updates the bot's cmd
void wbot_pre_delete(CWootBot* pBot); // called before a bot is removed