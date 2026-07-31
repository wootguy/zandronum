#include "wb_eiface.h"
#include "wb_nav.h"
#include "wb_bot.h"
#include "wb_util.h"

#include "actor.h"
#include "bots.h"
#include "network.h"
#include "r_state.h"
#include "team.h"
#include "doomerrors.h"
#include "sv_rcon.h"
#include "botcommands.h"
#include "st_hud.h"
#include "d_event.h"
#include "a_keys.h"
#include "p_trace.h"
#include "p_lnspec.h"

using namespace std;

void	SERVERCONSOLE_ReListPlayers(void);

namespace wbot {
	vector<TObjPtr<AActor>> g_actor_handles;

	AHandle::AHandle(AActor* actor) {
		serial = g_actor_handles.size();
		g_actor_handles.push_back(actor);

		if (serial == 100000) {
			// TODO: clear handles when actors are deleted
			Printf("Warning: 10k actor handles in memory\n");
		}
	}

	AActor* AHandle::get() const {
		return serial >= 0 && serial < g_actor_handles.size() ? g_actor_handles[serial] : NULL;
	}

	void init_eiface() {
		g_actor_handles.clear();
	}

	player_t* add_bot(CWootBot* pBot, int ulPlayerNum, const char* color, const char* colorSet,
		const char* skin, const char* pszTeamName) {
		ULONG	ulIdx;
		char	szColorizedBuffer[256];

		// Link the bot to the player.
		player_t* m_pPlayer = &players[ulPlayerNum];
		m_pPlayer->pWootBot = pBot;
		m_pPlayer->bIsBot = true;

		// [AK] Later on, PLAYER_ShouldSpawnAsSpectator gets called, which in turn
		// calls GAMEMODE_PreventPlayersFromJoining and then DUEL_CountActiveDuelers.
		// Thus, The bot's spectating status should be initialized to true, or else
		// they could be prevented from joining.
		m_pPlayer->bSpectating = true;
		m_pPlayer->bDeadSpectator = false;

		// [AK] Bots have a local connection to the host, so set their country index to LAN.
		m_pPlayer->ulCountryIndex = COUNTRYINDEX_LAN;

		// Update the playeringame slot.
		playeringame[ulPlayerNum] = true;

		// Setup the player's userinfo based on the bot's botinfo.
		// [BB] First clear the userinfo.
		m_pPlayer->userinfo.Reset();
		FString botname = "BOT";
		V_ColorizeString(botname);
		m_pPlayer->userinfo.NameChanged(botname);

		m_pPlayer->userinfo.ColorChanged(color);

		// Store the name of the skin the client gave us, so others can view the skin
		// even if the server doesn't have the skin loaded.
		if (NETWORK_GetState() == NETSTATE_SERVER)
			SERVER_GetClient(ulPlayerNum)->skinName = skin;

		LONG lSkin = R_FindSkin(skin, 0);
		m_pPlayer->userinfo.SkinNumChanged(lSkin);

		// If the skin was hidden, reveal it!
		if (skins[lSkin].bRevealed == false)
		{
			strcpy(szColorizedBuffer, skins[lSkin].name);
			V_ColorizeString(szColorizedBuffer);

			Printf("Hidden skin \"%s" TEXTCOLOR_NORMAL "\" has now been revealed!\n", szColorizedBuffer);
			skins[lSkin].bRevealed = true;
		}

		// [AK] Check if the bot should use a random player class whenever they spawn.
		m_pPlayer->userinfo.PlayerClassNumChanged(-1);

		// [AK] Get all the color set indices this bot's class has.
		if (m_pPlayer->userinfo.GetPlayerClassNum() != -1)
		{
			FName playerclass = m_pPlayer->userinfo.GetPlayerClassType()->TypeName;
			TArray<int> colorsets;
			P_EnumPlayerColorSets(playerclass, &colorsets);

			// [AK] See if the given color set name matches one of the class's color sets.
			for (ulIdx = 0; ulIdx < colorsets.Size(); ulIdx++)
			{
				if (stricmp(colorSet, P_GetPlayerColorSet(playerclass, colorsets[ulIdx])->Name.GetChars()) == 0)
				{
					m_pPlayer->userinfo.ColorSetChanged(ulIdx);
					break;
				}
			}
		}

		//m_pPlayer->userinfo.RailColorChanged(g_BotInfo[m_ulBotInfoIdx].ulRailgunColor);
		//m_pPlayer->userinfo.GenderNumChanged(D_GenderToInt(g_BotInfo[m_ulBotInfoIdx].szGender));
		if (pszTeamName)
		{
			// If we're in teamgame mode, put the bot on a defined team.
			if (GAMEMODE_GetCurrentFlags() & GMF_PLAYERSONTEAMS)
			{
				ULONG ulTeam = TEAM_GetTeamNumberByName(pszTeamName);
				if (TEAM_CheckIfValid(ulTeam))
					PLAYER_SetTeam(m_pPlayer, ulTeam, true);
				else
					PLAYER_SetTeam(m_pPlayer, TEAM_ChooseBestTeamForPlayer(), true);
			}
		}
		else
		{
			// In certain modes, the bot NEEDS to be placed on a team, or else he will constantly
			// respawn.
			if (GAMEMODE_GetCurrentFlags() & GMF_PLAYERSONTEAMS)
				PLAYER_SetTeam(m_pPlayer, TEAM_ChooseBestTeamForPlayer(), true);
		}

		// For now, bots always switch weapons on pickup.
		m_pPlayer->userinfo.SwitchOnPickupChanged(2);
		*static_cast<FIntCVar*>(m_pPlayer->userinfo[NAME_StillBob]) = 0;
		*static_cast<FIntCVar*>(m_pPlayer->userinfo[NAME_MoveBob]) = static_cast<fixed_t>(65536.f * 0.25);

		// If we've added the bot to a single player game, enable "fake multiplayer" mode.
		if (NETWORK_GetState() == NETSTATE_SINGLE)
			NETWORK_SetState(NETSTATE_SINGLE_MULTIPLAYER);

		m_pPlayer->playerstate = PST_ENTER;
		m_pPlayer->fragcount = 0;
		m_pPlayer->killcount = 0;
		m_pPlayer->ulDeathCount = 0;
		m_pPlayer->ulTime = 0;
		PLAYER_ResetSpecialCounters(m_pPlayer);

		// Check and see if this bot should spawn as a spectator.
		m_pPlayer->bSpectating = PLAYER_ShouldSpawnAsSpectator(m_pPlayer);

		// [BB] If the bot is forced to spectate, make sure he is not on a team.
		if (m_pPlayer->bSpectating)
			PLAYER_SetTeam(m_pPlayer, teams.Size(), true);

		// [BB] If any of the spawn functions throw an exception, we
		// have to catch it here. Otherwise the constructor is not properly
		// finished and trying to delete the pointer generated by it leads
		// to a crash.
		try
		{
			// [BB] Spawn the bot at its appropriate start.
			GAMEMODE_SpawnPlayer(ulPlayerNum, true);
		}
		catch (CRecoverableError&/*error*/)
		{
			Printf("Unable to spawn bot %s.\n", players[ulPlayerNum].userinfo.GetName());
			m_pPlayer->bSpectating = true;
			return m_pPlayer;
		}

		if (NETWORK_GetState() != NETSTATE_SERVER)
			Printf("%s entered the game.\n", players[ulPlayerNum].userinfo.GetName());
		else
		{
			// Let the other players know that this bot has entered the game.
			SERVER_Printf("%s entered the game.\n", players[ulPlayerNum].userinfo.GetName());

			// Redo the scoreboard.
			SERVERCONSOLE_ReListPlayers();

			// [RC] Update clients using the RCON utility.
			SERVER_RCON_UpdateInfo(SVRCU_PLAYERDATA);

			// Now send out the player's userinfo out to other players.
			SERVERCOMMANDS_SetAllPlayerUserInfo(ulPlayerNum);

			// If this player is on a team, tell all the other clients that a team has
			// been selected for him.
			if (m_pPlayer->bOnTeam)
				SERVERCOMMANDS_SetPlayerTeam(ulPlayerNum);
		}

		BOTCMD_SetLastJoinedPlayer(m_pPlayer->userinfo.GetName());

		// Tell the bots that a new players has joined the game!
		{
			ULONG	ulIdx;

			for (ulIdx = 0; ulIdx < MAXPLAYERS; ulIdx++)
			{
				if (playeringame[ulIdx] == false)
					continue;

				// Don't tell the bot that joined that it joined the game.
				if (ulIdx == ulPlayerNum)
					continue;

				if (players[ulIdx].pSkullBot)
					players[ulIdx].pSkullBot->PostEvent(BOTEVENT_PLAYER_JOINEDGAME);
			}
		}

		// Refresh the HUD since a new player is now here (this affects the number of players in the game).
		HUD_ShouldRefreshBeforeRendering();

		return m_pPlayer;
	}

	APlayerPawn* get_player(player_t* plr) {
		return plr->mo;
	}

	FVector3 get_actor_pos(AActor* actor) {
		return FVector3(actor->x, actor->y, actor->z);
	}

	int get_actor_height(AActor* actor) {
		return actor->height;
	}

	MapSector* get_actor_sector(AActor* actor) {
		return &g_map.sectors[actor->Sector - ::sectors];
	}

	PClass* get_actor_class(AActor* actor) {
		return actor->GetClass();
	}

	int get_actor_radius(AActor* actor) {
		return actor->radius;
	}

	uint32_t get_actor_angle(AActor* actor) {
		return actor->angle;
	}

	uint32_t get_actor_pitch(AActor* actor) {
		return actor->pitch;
	}

	int get_player_viewheight(player_t* actor) {
		return actor->viewheight;
	}

	bool player_on_ground(player_t* plr) {
		return plr->onground;
	}

	int get_actor_health(AActor* actor) {
		return actor->health;
	}

	void player_select_weapon(player_t* pPlayer, AActor* weapon) {
		if (weapon && pPlayer->ReadyWeapon != weapon && pPlayer->PendingWeapon != weapon) {
			pPlayer->PendingWeapon = (AWeapon*)weapon;
			if (pPlayer->ReadyWeapon != NULL) {
				P_DropWeapon(pPlayer);
			}
			else if (pPlayer->PendingWeapon != WP_NOCHANGE) {
				P_BringUpWeapon(pPlayer);
			}
		}
	}

	bool is_player_frozen(player_t* plr) {
		return plr->cheats & (CF_FROZEN | CF_TOTALLYFROZEN);
	}
	
	void freeze_player(player_t* plr, bool frozen) {
		if (frozen) {
			plr->cheats |= CF_FROZEN;
			plr->mo->velx = 0;
			plr->mo->vely = 0;
			plr->mo->velz = 0;
		}
		else {
			plr->cheats &= ~CF_FROZEN;
		}
	}

	void kill_actor(AActor* actor) {
		P_DamageMobj(actor, actor, actor, TELEFRAG_DAMAGE, NAME_Suicide);
	}

	bool check_line_of_sight(AActor* looker, AActor* target) {
		return P_CheckSight(looker, target, SF_SEEPASTSHOOTABLELINES);
	}

	int PointToAngle2(int x1, int y1, int x2, int y2) {
		return R_PointToAngle2(x1, y1, x2, y2);
	}

	AActor* find_followable_player(int subid) {
		AActor* player = NULL;
		for (int i = 0; i < MAXPLAYERS; i++)
		{
			if (!playeringame[i])
				continue;

			AActor* actor = players[i].mo;
			if (!actor || actor->player->bIsBot)
				continue;

			if (actor->player->cheats & (CF_NOCLIP | CF_NOCLIP2))
				continue; // for testing

			if (subid == g_wb_nav.get_nav_id(actor))
				continue; // already with this player

			return actor;
		}
		
		return NULL;
	}

	AActor* find_boss_brain() {
		TThinkerIterator<AActor> it;
		AActor* actor;
		while ((actor = it.Next())) {
			if ((actor->flags & MF_SHOOTABLE) && !actor->player) {
				if (!strcmp(actor->GetClass()->TypeName.GetChars(), "BossBrain")) {
					return actor;
				}
			}
		}

		return NULL;
	}

	AActor* look_for_enemies_in_block(AActor* lookee, int index, void* extparam)
	{
		FBlockNode* block;
		AActor* link;
		CWootBot* pbot = (CWootBot*)extparam;
		angle_t fov = pbot->m_fov * ANGLE_1;
		AActor* plr = pbot->pActor;
		AActor* oldTarget = plr->target;
		fixed_t oldDist = oldTarget ? P_AproxDistance(oldTarget->x - plr->x, oldTarget->y - plr->y) : 0;

		for (block = blocklinks[index]; block != NULL; block = block->NextActor)
		{
			link = block->Me;

			if (!(link->flags & MF_SHOOTABLE))
				continue;			// not shootable (observer or dead)

			if (link == lookee)
				continue;

			if (link->health <= 0)
				continue;			// dead

			if (link->flags2 & MF2_DORMANT)
				continue;			// don't target dormant things

			if (link->flags7 & MF7_NEVERTARGET)
				continue;

			if (lookee->IsFriend(link))
				continue;

			if (fov && fov < ANGLE_MAX)
			{
				angle_t an = R_PointToAngle2(lookee->x, lookee->y, link->x, link->y) - lookee->angle;

				if (an > (fov / 2) && an < (ANGLE_MAX - (fov / 2))) {
					continue;	// outside of fov
				}
			}

			// P_CheckSight is by far the most expensive operation in here so let's do it last.
			if (!P_CheckSight(lookee, link, SF_SEEPASTSHOOTABLELINES)) {
				continue;
			}

			// only retarget to closer enemies
			if (oldTarget && P_AproxDistance(plr->x - link->x, plr->y - link->y) >= oldDist) {
				continue;
			}

			return link;
		}

		return NULL;
	}

	AActor* find_enemy(CWootBot* pBot) {
		return P_BlockmapSearch(pBot->pActor, 10, look_for_enemies_in_block, pBot);
	}

	bool can_unlock_door(AActor* activator, MapLine* line) {
		return P_CheckKeys(activator, line->getArg(3), false);
	}

	std::vector<AActor*> find_prop_blockers() {
		vector<AActor*> propBlockers;

		TThinkerIterator<AActor> it;
		AActor* actor;
		while ((actor = it.Next())) {
			if (IsPropBlocker(actor))
				propBlockers.push_back(actor);
			//Printf("Prop '%s' %d\n", actor->GetClass()->TypeName.GetChars(), actor->health);
		}

		return propBlockers;
	}

	vector<AActor*> find_map_keys() {
		TThinkerIterator<AKey> it;
		AKey* mapKey;

		int oldRouteIgnoreNum = g_route_ignore_num;

		vector<AActor*> allKeys;

		while ((mapKey = it.Next()) != NULL) {
			if (mapKey->Owner)
				continue; // key in someone's inventory

			allKeys.push_back(mapKey);
		}
		
		return allKeys;
	}

	std::vector<AActor*> find_map_weapons(const char* name) {
		TThinkerIterator<AWeapon> it;
		AWeapon* weapon;

		vector<AActor*> weapons;

		while ((weapon = it.Next()) != NULL) {
			if (weapon->Owner)
				continue; // weapon in someone's inventory

			if (!strcmp(weapon->GetClass()->TypeName.GetChars(), name)) {
				weapons.push_back(weapon);
			}
		}

		return weapons;
	}

	std::vector<AActor*> find_map_ammo(const char* name1, const char* name2) {
		TThinkerIterator<AAmmo> it;
		AAmmo* ammo;

		vector<AActor*> ammos;

		while ((ammo = it.Next()) != NULL) {
			if (ammo->Owner)
				continue; // weapon in someone's inventory

			const char* name = ammo->GetClass()->TypeName.GetChars();
			if (!strcmp(name, name1) || (name2 && !strcmp(name, name2)))
				ammos.push_back(ammo);
		}

		return ammos;
	}

	vector<vector<PClass*>> get_required_key_types(MapLine* line) {
		TArray<TArray<PClass*>> keyGroups = P_GetRequiredKeys(line->getArg(3));

		vector<vector<PClass*>> outGroups;

		for (int i = 0; i < keyGroups.Size(); i++) {
			vector<PClass*> outGroup;
			TArray<PClass*>& group = keyGroups[i];

			for (int k = 0; k < group.Size(); k++) {
				outGroup.push_back(group[k]);
			}

			outGroups.push_back(outGroup);
		}

		return outGroups;
	}

	vector<AActor*> get_player_weapons(APlayerPawn* pActor, bool loadedOnly) {
		vector<AActor*> weapons;

		AWeapon* bestWeapon = NULL;
		int bestPriority = -1;
		for (AInventory* item = pActor->Inventory; item != NULL; item = item->Inventory) {
			if (item->IsKindOf(RUNTIME_CLASS(AWeapon))) {
				AWeapon* weapon = static_cast<AWeapon*>(item);
				WeaponInfo& info = g_wbot_weapon_info[weapon->GetClass()->TypeName.GetChars()];
				
				if (!loadedOnly || !weapon->Ammo1 || weapon->Ammo1->Amount >= info.minAmmo) {
					weapons.push_back(weapon);
				}
			}
		}

		return weapons;
	}

	int get_weapon_ammo(AActor* weapon) {
		AWeapon* wep = (AWeapon*)weapon;
		return wep->Ammo1 ? wep->Ammo1->Amount : 0;
	}

	const char* get_class_type_name(PClass* pclass) {
		return pclass->TypeName.GetChars();
	}

	const char* get_actor_type_name(AActor* actor) {
		return actor->GetClass()->TypeName.GetChars();
	}

	bool intermission_active() {
		return gamestate == GS_INTERMISSION;
	}

	void gprintf(const char* fmt, ...) {
		static char buffer[4096];

		va_list args;
		va_start(args, fmt);
		vsnprintf(buffer, sizeof(buffer), fmt, args);
		va_end(args);

		Printf("%s", buffer);
	}

	void print_hud_test(const char* msg, float x, float y, uint32_t id) {
		SERVERCOMMANDS_PrintHUDMessage(msg, x, y, 0, 0, 0, CR_RED, 1.0f, 0, 0, "SmallFont", id);
	}

	int get_game_tics() {
		return level.time;
	}

	bool TraceLine(FVector3 start, FVector3 end, bool ignoreMonsters, AActor* ignore, TraceResult* tr) {
		FVector3 delta = end - start;
		fixed_t dist = delta.Length();
		delta = delta.Unit() * FRACUNIT;

		sector_t* sector = P_PointInSector(start.X, start.Y);

		FTraceResults trInternal;

		bool hit = ::Trace((fixed_t)start.X, (fixed_t)start.Y, (fixed_t)start.Z, sector,
			(fixed_t)delta.X, (fixed_t)delta.Y, (fixed_t)delta.Z, dist, ignoreMonsters ? 0 : MF_SOLID,
			ML_BLOCKING | ML_BLOCKEVERYTHING | ML_BLOCK_PLAYERS, ignore, trInternal);

		if (tr) {
			tr->endPos = FVector3(trInternal.X, trInternal.Y, trInternal.Z);
			tr->actor = trInternal.Actor;
			tr->frac = trInternal.Fraction / (float)FRACUNIT;
			tr->hitType = (TraceHitType)trInternal.HitType;
			tr->line = trInternal.Line ? &g_map.lines[trInternal.Line - ::lines] : NULL;
			tr->sector = trInternal.Sector ? &g_map.sectors[trInternal.Sector - ::sectors] : NULL;
		}

		return hit;
	}

	void* load_wad_lump(MapData* map, int id, int& len, int structSize) {
		int dataLen = map->Size(id);
		uint8_t* data = new uint8_t[dataLen];
		map->Read(id, data);
		len = dataLen / structSize;
		return data;
	}

	MapLumps load_wad_lump_data() {
		MapLumps lumps;
		MapData* map = P_OpenMapData(level.mapname, true);

		if (!map) {
			Printf("[wbot] Failed to open map data\n");
			memset(&lumps, 0, sizeof(lumps));
			return lumps;
		}

		lumps.verts = (LumpVert*)load_wad_lump(map, ML_VERTEXES, lumps.numverts, sizeof(LumpVert));
		lumps.sides = (LumpSide*)load_wad_lump(map, ML_SIDEDEFS, lumps.numsides, sizeof(LumpSide));
		lumps.segs = (LumpSeg*)load_wad_lump(map, ML_SEGS, lumps.numsegs, sizeof(LumpSeg));
		lumps.subsectors = (LumpSubSector*)load_wad_lump(map, ML_SSECTORS, lumps.numsubsectors, sizeof(LumpSubSector));
		lumps.lines = (LumpLine*)load_wad_lump(map, ML_LINEDEFS, lumps.numlines, sizeof(LumpLine));
		lumps.sectors = (LumpSector*)load_wad_lump(map, ML_SECTORS, lumps.numsectors, sizeof(LumpSector));
		lumps.nodes = (LumpNode*)load_wad_lump(map, ML_NODES, lumps.numnodes, sizeof(LumpNode));

		return lumps;
	}

	bool sector_special_is_damage(int special) {
		switch (special) {
		case dDamage_Hellslime:
		case dDamage_LavaHefty:
		case dDamage_LavaWimpy:
		case dDamage_Nukage:
		case dDamage_SuperHellslime:
			return true;
		}

		return false;
	}

	int get_linedef_move_flag(MapLine* line) {
		int timingFlag = 0;

		switch (line->special()) {
		case Plat_UpWaitDownStay:
		case Plat_UpNearestWaitDownStay:
		case Plat_DownWaitUpStay:
		case Plat_DownWaitUpStayLip:
		case Ceiling_CrushRaiseAndStayA:
		case Ceiling_CrushAndRaiseA:
		case Ceiling_CrushAndRaiseSilentA:
			timingFlag = FL_SECTOR_MOVE_TIMED;
			break;
		}

		// only add specials here that could be potentially helpful for unblocking a path.
		// For instance, raising a door or elevator. A ceiling or door coming down lower 
		// will not help a bot pass the sector
		switch (line->special()) {
		case 0:
			return 0;
		case Door_Open:
		case Door_Raise:
		case Door_LockedRaise:
		case Ceiling_RaiseByValue:
		case Ceiling_RaiseToNearest:
		case Ceiling_RaiseInstant:
		case Ceiling_RaiseByValueTimes8:
		case Generic_Ceiling: // TODO: check if really does move up
		case Generic_Door:
		case Ceiling_CrushAndRaiseDist:
			return timingFlag | FL_SECTOR_MOVE_CEIL_UP;

		case Plat_UpWaitDownStay:
		case Plat_UpByValue:
		case Plat_UpNearestWaitDownStay:
		case Plat_RaiseAndStayTx0:
		case Plat_UpByValueStayTx:
		case Floor_RaiseToHighest:
		case Floor_RaiseToNearest:
		case Floor_RaiseByValueTxTy:
		case Floor_RaiseToLowestCeiling:
		case Elevator_RaiseToNearest:
		case Stairs_BuildUp:
		case Stairs_BuildUpSync:
		case Stairs_BuildUpDoom:
		case Floor_RaiseByTexture:
		case Floor_RaiseByValueTimes8:
		case Floor_RaiseAndCrushDoom:
			return timingFlag | FL_SECTOR_MOVE_FLOOR_UP;

		case Plat_PerpetualRaise:
		case Plat_PerpetualRaiseLip:
			return timingFlag | FL_SECTOR_MOVE_FLOOR_UP | FL_SECTOR_MOVE_FLOOR_DOWN;

		case Plat_DownWaitUpStay:
		case Plat_DownByValue:
		case Plat_DownWaitUpStayLip:
		case Floor_LowerToLowest:
		case Floor_LowerToNearest:
		case Floor_LowerToHighest:
		case Floor_LowerToLowestTxTy:
		case Elevator_LowerToNearest:
		case Stairs_BuildDown:
		case Stairs_BuildDownSync:
		case Floor_LowerByValueTimes8:
			return timingFlag | FL_SECTOR_MOVE_FLOOR_DOWN;

		case Generic_Floor:
		case Elevator_MoveToFloor:
		case Generic_Lift:
		case Generic_Stairs:
			return timingFlag | FL_SECTOR_MOVE_FLOOR_UP | FL_SECTOR_MOVE_FLOOR_DOWN; // TODO: can do both dirs?	

		case Ceiling_LowerToHighestFloor:
		case Ceiling_LowerInstant:
		case Ceiling_CrushRaiseAndStayA:
		case Ceiling_CrushAndRaiseA:
		case Ceiling_CrushAndRaiseSilentA:
		case Ceiling_LowerByValueTimes8:
		case Door_Close:
		case Door_CloseWaitOpen:
			return 0; // a ceiling getting lower is not helpful

		case Scroll_Texture_Left:
		case Scroll_Texture_Right:
		case Scroll_Texture_Up:
		case Scroll_Texture_Down:
		case Light_ForceLightning:
		case Light_RaiseByValue:
		case Light_LowerByValue:
		case Light_ChangeToValue:
		case Light_Fade:
		case Light_Glow:
		case Light_Flicker:
		case Light_Strobe:
		case Light_Stop:
			return 0; // visual-only specials

		case Teleport:
		case Plat_Stop:
		case Ceiling_CrushStop:
		case Exit_Normal:
		case Exit_Secret:
			return 0; // does not cause sectors to move

		default:
			Printf("Unknown special %d for line %d\n", line->special(), line - g_map.lines);
			return 0;
		}
	}

	bool special_is_teleport(int special) {
		return special == Teleport;
	}

	bool special_is_locked_door(int special) {
		return special == Door_LockedRaise;
	}

	bool special_is_level_exit(int special) {
		return (special == Exit_Normal || special == Exit_Secret);
	}

	void add_stair_sector_info() {
		for (int s = 0; s < numlines; s++) {
			MapLine& line = g_map.lines[s];
			line_t& eline = ::lines[s];

			bool isStairBuilder = false;
			int usespecials = 0;
			bool igntxt = false;
			int moveFlags = 0;

			switch (line.special()) {
			case Stairs_BuildDown:
			case Stairs_BuildUp:
				usespecials = 1;
				isStairBuilder = true;
				break;
			case Stairs_BuildDownSync:
			case Stairs_BuildUpSync:
				usespecials = 2;
				isStairBuilder = true;
				break;
			case Stairs_BuildUpDoom:
				isStairBuilder = true;
				break;
			case Generic_Stairs:
				isStairBuilder = true;
				igntxt = eline.args[3] & 2;
				break;
			}

			switch (line.special()) {
			case Stairs_BuildDown:
			case Stairs_BuildDownSync:
				moveFlags |= FL_SECTOR_MOVE_FLOOR_DOWN;
				break;
			case Stairs_BuildUp:
			case Stairs_BuildUpSync:
			case Stairs_BuildUpDoom:
			case Generic_Stairs:
				moveFlags |= FL_SECTOR_MOVE_FLOOR_UP;
				break;
			}

			if (!isStairBuilder)
				continue;

			int tag = line.tag;
			if (tag == 0)
				continue; // only back sector moves

			int i_compatflags = 0;
			int (*FindSector) (int tag, int start) =
				(i_compatflags & COMPATF_STAIRINDEX) ? P_FindSectorFromTagLinear : P_FindSectorFromTag;

			// The compatibility mode doesn't work with a hashing algorithm.
			// It needs the original linear search method. This was broken in Boom.

			BotGoal stairTrigger;
			if (line.canPlayerActivate())
				stairTrigger = BotGoal(get_linedef_goal_action(&line), s);

			int secnum = -1;
			int newsecnum = -1;
			sector_t* prev = NULL;
			while ((secnum = FindSector(tag, secnum)) >= 0) {
				sector_t* sec = &::sectors[secnum];

				// Find next sector to raise
				// 1. Find 2-sided line with same sector side[0] (lowest numbered)
				// 2. Other side is the next sector to raise
				// 3. Unless already moving, or different texture, then stop building
				bool ok;
				do
				{
					ok = false;
					sector_t* tsec = NULL;

					if (usespecials)
					{
						// [RH] Find the next sector by scanning for Stairs_Special?
						tsec = sec->NextSpecialSector(
							(sec->special & 0xff) == Stairs_Special1 ?
							Stairs_Special2 : Stairs_Special1, prev);

						ok = (tsec != NULL);
						newsecnum = (int)(tsec - ::sectors);
					}
					else
					{
						for (int i = 0; i < sec->linecount; i++)
						{
							if (!((sec->lines[i])->flags & ML_TWOSIDED))
								continue;

							tsec = (sec->lines[i])->frontsector;
							newsecnum = (int)(tsec - ::sectors);

							if (secnum != newsecnum)
								continue;

							tsec = (sec->lines[i])->backsector;
							if (!tsec) continue;	//jff 5/7/98 if no backside, continue
							newsecnum = (int)(tsec - ::sectors);

							FTextureID texture = sec->GetTexture(sector_t::floor);

							if (!igntxt && tsec->GetTexture(sector_t::floor) != texture)
								continue;

							ok = true;
							break;
						}
					}

					if (ok) {
						prev = sec;
						sec = tsec;
						secnum = newsecnum;

						MapSector& msec = g_map.sectors[tsec - ::sectors];
						msec.moveFlags |= moveFlags;

						if (line.canPlayerActivate())
							msec.triggers.push_back(stairTrigger);
					}
				} while (ok);
			}
		}
	}

	int get_sector_floor_z(int id) {
		sector_t& sec = ::sectors[id];
		return sec.floorplane.Zat0();
	}

	int get_sector_ceil_z(int id) {
		sector_t& sec = ::sectors[id];
		return sec.ceilingplane.Zat0();
	}

	bool is_sector_floor_moving(int id) {
		return ::sectors[id].floordata;
	}

	bool is_sector_ceil_moving(int id) {
		return ::sectors[id].ceilingdata;
	}

	int get_sector_special(int id) {
		return ::sectors[id].special;
	}

	int get_line_special(int id) {
		return ::lines[id].special;
	}

	int get_line_activation(int id) {
		return ::lines[id].activation;
	}

	int get_line_arg(int id, int arg) {
		return ::lines[id].args[arg];
	}

	bool can_player_activate_line(int id) {
		return ::lines[id].activation & SPAC_PlayerActivate;
	}

	bool is_double_sided_cross_line(int id) {
		MapLine* line = &g_map.lines[id];
		int lineAction = get_linedef_goal_action(line);
		return lineAction == WBOT_GOAL_ACTION_CROSS && (line->flags & ML_TWOSIDED);
	}

	int get_linedef_goal_action(MapLine* line) {
		if (!line)
			return -1;

		line_t* eline = &::lines[line - g_map.lines];

		if (eline->activation & SPAC_Impact) {
			return WBOT_GOAL_ACTION_SHOOT;
		}
		if (eline->activation & (SPAC_Use | SPAC_UseThrough)) {
			return WBOT_GOAL_ACTION_USE;
		}
		if (eline->activation & (SPAC_Cross | SPAC_AnyCross)) {
			return WBOT_GOAL_ACTION_CROSS;
		}
		if (eline->activation & SPAC_Push) {
			return WBOT_GOAL_ACTION_TOUCH;
		}

		if (eline->activation)
			gprintf("Don't know how to activate line %d\n", line - g_map.lines);

		return -1;
	}

	FVector2 get_tele_dest(int lineid) {
		AActor* actor = SelectTeleDest(get_line_arg(lineid, 0), get_line_arg(lineid, 1));
		return actor ? FVector2(actor->x, actor->y) : FVector2(0, 0);
	}
};