#pragma once
// Engine interface - for accessing engine data without including any port-specific headers

#include <vector>
#include <stdint.h>
#include "wb_vec.h"

class AActor;
class APlayerPawn;
class player_t;
class CWootBot;
struct line_t;

#define ZANDRONUM_BUILD

#ifdef ZANDRONUM_BUILD

#define IN_ATTACK (1 << 0)
#define IN_USE (1 << 1)
#define IN_JUMP (1 << 2)
#define IN_DUCK (1 << 3)

#define MAXPLAYERS 64

#endif

#ifndef M_PI
#define M_PI		3.14159265358979323846	// matches value in gcc v2 math.h
#endif

#define FL_SECTOR_MOVE_FLOOR_DOWN	1
#define FL_SECTOR_MOVE_FLOOR_UP		2
#define FL_SECTOR_MOVE_FLOOR_ANY	4
#define FL_SECTOR_MOVE_CEIL_UP		8
#define FL_SECTOR_MOVE_TIMED		16	// sector resets to its old position shortly after being activated

#define FL_LINE_IS_TELEPORT 1
#define FL_LINE_IS_LOCKED_DOOR 2
#define FL_LINE_IS_LEVEL_EXIT 4
#define FL_LINE_PLAYER_ACTIVATE 8
#define FL_LINE_DOUBLE_SIDE_CROSS 16
#define FL_LINE_IMPASSABLE 32

struct PClass;
struct MapData;

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
		vec3 endPos;
		float frac;
		AActor* actor;		// valid if hit an actor
		MapLine* line;		// valid if hit a line
		TraceHitType hitType;
	};

	struct TraceIsect {
		wbot::MapLine* line;
		wbot::MapSector* sector;
		vec2 pos;
		int fraction;
	};

	// all actor state relevent to bot ai
	struct ActorState {
		vec3 origin;
		vec3 velocity;
		int height;
		int health;
		int radius;
		uint32_t yaw;
		uint32_t pitch;
		MapSector* sector;
		PClass* pClass;
		const char* name;
	};

	// all player state relevent to bot ai
	struct PlayerState {
		int viewHeight;
		bool onGround;
		bool isFrozen;
		const char* weaponName; // NULL if no weapon ready
	};

	struct SectorState {
		float floorZ;
		float ceilZ;
		int special;
		bool floorMoving;
		bool ceilMoving;
		bool floorDamage;
	};

	struct LineState {
		int16_t special;
		uint16_t flags;
		uint16_t moveFlags;
		uint16_t goalAction;
		int args[5];
	};

	class WBotEngineInterface {
	public:
		void init_eiface();

		void add_bot();

		player_t* init_bot(CWootBot* pBot, int ulPlayerNum, const char* color, const char* colorSet,
			const char* skin, const char* pszTeamName);

		void pre_remove_bot(CWootBot* pBot);

		void simulate_bot(CWootBot* pBot);

		APlayerPawn* get_player(player_t* plr);

		ActorState get_actor_state(AActor* actor);

		PlayerState get_player_state(player_t* plr);

		void player_select_weapon(player_t* plr, AActor* weapon);

		CWootBot* get_player_bot(player_t* plr);

		void freeze_player(player_t* plr, bool frozen);

		void give_all_weapons(player_t* plr);

		void kill_actor(AActor* actor);

		bool check_line_of_sight(AActor* looker, AActor* target);

		// todo: convert vector angles to yaw/pitch
		int PointToAngle2(float x1, float y1, float x2, float y2);

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

		bool intermission_active();

		// print a message to the default output stream
		void gprintf(const char* fmt, ...);

		// ID must be unique per hud element
		void print_hud_test(const char* msg, float x, float y, uint32_t id);

		int get_game_tics();

		const char* get_map_name();

		bool TraceLine(vec3 start, vec3 end, bool ignoreMonsters, AActor* ignore, TraceResult* tr);

		// returns all walls/sectors intersected by the given line
		std::vector<TraceIsect> TraceIntersections(vec2 start, vec2 end);

		// returns true if the trace intersects any impassable walls
		bool TraceImpassable(vec2 start, vec2 end);

		// trace until the first intersected sector edge
		bool TraceSectorEdge(vec2 start, vec2 end, vec2& edge, wbot::MapLine** line);

		void* load_wad_lump(MapData* map, int id, int& len, int structSize);

		MapLumps load_wad_lump_data();

		SectorState get_sector_state(int id);

		LineState get_line_state(int id);

		void add_stair_sector_info();

		MapLine* get_map_line_from_engine_line(line_t* line);

		vec2 get_tele_dest(int lineid);

		// get lines that the given box intersects
		std::vector<MapLine*> get_crossed_lines(const vec2& pos, int radius);

		player_t* get_player_for_index(int i); // may be a real player or a bot

		CWootBot* get_bot_for_index(int i);

		// angle is given in degrees
		void set_actor_origin(AActor* actor, int x, int y, uint32_t angle, bool teleportFx);

		void MakeVectors(uint32_t angle, vec3& forward, vec3& right);

		void SpawnBlood(vec3 pos, int damage, AActor* owner);

		void PrintNotification(const char* msg);

		bool is_actor_immovable_solid_prop(AActor* actor);

		bool are_cheats_enabled();

		void kill_all_shootables();

		// returns NUL for a missing arg, empty string for no value, or the argument value
		const char* get_program_arg(const char* name);

		void exit_level();

		void change_level(const char* mapname, bool noIntermission);
	};

	extern WBotEngineInterface g_engine;
}