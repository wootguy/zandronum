#pragma once

class AActor;
class APlayerPawn;
class player_t;
class CWootBot;

namespace wbot {
	player_t* add_bot(CWootBot* pBot, int ulPlayerNum, const char* color, const char* colorSet,
		const char* skin, const char* pszTeamName);

	float get_player_use_distance(APlayerPawn* plr);
}