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

#define FL_SECTOR_MOVE_FLOOR_DOWN	1
#define FL_SECTOR_MOVE_FLOOR_UP		2
#define FL_SECTOR_MOVE_FLOOR_ANY	4
#define FL_SECTOR_MOVE_CEIL_UP		8
#define FL_SECTOR_MOVE_TIMED		16	// sector resets to its old position shortly after being activated

struct PClass;

namespace wbot {
	struct MapLine;
	struct MapSector;

	// structs without padding for loading from file
#pragma pack(1)
	struct LumpVert {
		int16_t x, y;
	};

	struct LumpNode {
		int16_t x, y;
		int16_t dx, dy;
		int16_t bbox[2][4];
		uint16_t children[2];
	};

	struct LumpSeg {
		uint16_t v1;
		uint16_t v2;
		uint16_t angle;
		uint16_t linedef;
		uint16_t side;
		int16_t offset;
	};

	struct LumpSubSector {
		uint16_t numsegs;
		uint16_t firstseg;
	};

	struct LumpSector {
		int16_t floorheight;
		int16_t ceilingheight;
		char floorpic[8];
		char ceilingpic[8];
		int16_t lightlevel;
		int16_t special;
		int16_t tag;
	};

	struct LumpLine {
		uint16_t v1;
		uint16_t v2;
		uint16_t flags;
		uint16_t special;
		uint16_t tag;
		int16_t sidenum[2];
	};

	struct LumpSide {
		int16_t textureoffset;
		int16_t rowoffset;
		char toptexture[8];
		char bottomtexture[8];
		char midtexture[8];
		int16_t sector;
	};
#pragma pack()

	struct MapLumps {
		LumpVert* verts;
		int numverts;

		LumpNode* nodes;
		int numnodes;

		LumpSeg* segs;
		int numsegs;

		LumpSubSector* subsectors;
		int numsubsectors;

		LumpSector* sectors;
		int numsectors;

		LumpLine* lines;
		int numlines;

		LumpSide* sides;
		int numsides;
	};

	// safe pointer to an actor
	struct AHandle {
		uint64_t serial;

		AHandle() : serial(-1) {}
		AHandle(AActor* actor);

		AActor* get() const;
	};

	enum TraceHitType {
		TRACE_HitNone,
		TRACE_HitFloor,
		TRACE_HitCeiling,
		TRACE_HitWall,
		TRACE_HitActor
	};

	struct TraceResult {
		MapSector* sector;
		FVector3 endPos;
		float frac;
		AActor* actor;		// valid if hit an actor
		MapLine* line;		// valid if hit a line
		TraceHitType hitType;
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

	bool can_unlock_door(AActor* activator, MapLine* line);

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

	bool TraceLine(FVector3 start, FVector3 end, bool ignoreMonsters, AActor* ignore, TraceResult* tr);

	MapLumps load_wad_lump_data();

	bool sector_special_is_damage(int special);

	// type of movement applied to the target sector. returns FL_SECTOR_MOVE_*
	int get_linedef_move_flag(MapLine* line);

	bool special_is_teleport(int special);

	bool special_is_locked_door(int special);

	bool special_is_level_exit(int special);

	void add_stair_sector_info();

	int get_sector_floor_z(int id);

	int get_sector_ceil_z(int id);

	bool is_sector_floor_moving(int id);

	bool is_sector_ceil_moving(int id);

	int get_line_special(int id);

	int get_sector_special(int id);

	int get_line_activation(int id);

	int get_line_arg(int id, int arg);

	bool can_player_activate_line(int id);

	bool is_double_sided_cross_line(int id);

	// how to trigger the given line
	int get_linedef_goal_action(MapLine* line);

	FVector2 get_tele_dest(int lineid);
}