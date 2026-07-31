#pragma once
// Engine interface for accessing engine data without including any port-specific headers

#include <vector>
#include <stdint.h>
#include "vectors.h"

class AActor;
class APlayerPawn;
class player_t;
class CWootBot;

#define ZANDRONUM_BUILD

#ifdef ZANDRONUM_BUILD

#define IN_ATTACK (1 << 0)
#define IN_USE (1 << 1)
#define IN_JUMP (1 << 2)
#define IN_DUCK (1 << 3)

#endif

#ifndef M_PI
#define M_PI		3.14159265358979323846	// matches value in gcc v2 math.h
#endif

struct PClass;

namespace wbot {
	struct MapLine;
	struct MapSector;

	// safe pointer to an actor
	struct AHandle {
		uint64_t serial;

		AHandle() : serial(-1) {}
		AHandle(AActor* actor);

		AActor* get() const;
	};

	void init_eiface();

	player_t* add_bot(CWootBot* pBot, int ulPlayerNum, const char* color, const char* colorSet,
		const char* skin, const char* pszTeamName);

	APlayerPawn* get_player(player_t* plr);

	FVector3 get_actor_pos(AActor* actor);

	int get_actor_height(AActor* actor);

	int get_actor_health(AActor* actor);

	MapSector* get_actor_sector(AActor* actor);

	PClass* get_actor_class(AActor* actor);

	int get_actor_radius(AActor* actor);

	uint32_t get_actor_angle(AActor* actor);

	uint32_t get_actor_pitch(AActor* actor);

	int get_player_viewheight(player_t* actor);

	bool player_on_ground(player_t* plr);

	void player_select_weapon(player_t* plr, AActor* weapon);

	bool is_player_frozen(player_t* plr);

	void freeze_player(player_t* plr, bool frozen);

	void kill_actor(AActor* actor);

	bool check_line_of_sight(AActor* looker, AActor* target);

	// todo: convert vector angles to yaw/pitch
	int PointToAngle2(int x1, int y1, int x2, int y2);

	AActor* find_followable_player(int subid);

	AActor* find_boss_brain();

	AActor* find_enemy(CWootBot* pBot);

	std::vector<AActor*> find_prop_blockers(); // find all immovable props with collision

	std::vector<AActor*> find_map_keys();

	std::vector<AActor*> find_map_weapons(const char* name);

	std::vector<AActor*> find_map_ammo(const char* name1, const char* name2);

	std::vector<std::vector<PClass*>> get_required_key_types(MapLine* line);

	std::vector<AActor*> get_player_weapons(APlayerPawn* pActor, bool loadedOnly);

	int get_weapon_ammo(AActor* weapon);

	const char* get_class_type_name(PClass* pclass);

	const char* get_actor_type_name(AActor* pclass);

	bool intermission_active();

	// print a message to the default output stream
	void gprintf(const char* fmt, ...);

	// ID must be unique per hud element
	void print_hud_test(const char* msg, float x, float y, uint32_t id);

	int get_game_tics();
}