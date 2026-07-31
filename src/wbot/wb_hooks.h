#pragma once
// hooks called elsewhere in the game code

struct line_t;
class AActor;
class CWootBot;

// If true, automated tests are running. Level change transitions should be skipped.
extern bool g_wbot_test_mode;

void wbot_handle_chat_command(unsigned int ulPlayer, const char* msg);
void wbot_handle_line_activation(line_t* line, AActor* activator);
void wbot_map_init();
void wbot_map_exit();
void wbot_tick();
void wbot_tick(CWootBot* pBot);
void wbot_pre_delete(CWootBot* pBot);