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
};