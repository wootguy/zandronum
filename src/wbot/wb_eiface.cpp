#include "wb_eiface.h"
#include "wb_nav.h"
#include "wb_bot.h"
#include "wb_util.h"
#include "wb_hooks.h"

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
#include "m_bbox.h"
#include "deathmatch.h"
#include "sbar.h"
#include "m_cheat.h"
#include "c_dispatch.h"

using namespace std;
using namespace wbot;

void	SERVERCONSOLE_ReListPlayers(void);

namespace wbot {
	WBotEngineInterface g_engine;
	vector<TObjPtr<AActor>> g_actor_handles;
}

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

void WBotEngineInterface::init_eiface() {
	g_actor_handles.clear();
}

void WBotEngineInterface::add_bot() {
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

player_t* WBotEngineInterface::init_bot(CWootBot* pBot, int ulPlayerNum, const char* color, const char* colorSet,
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

void WBotEngineInterface::pre_remove_bot(CWootBot* pBot) {
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

void WBotEngineInterface::simulate_bot(CWootBot* pBot) {
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
	pBot->m_astate = get_actor_state(actor);
	pBot->m_pstate = get_player_state(plr);

	pBot->Think();

	actor->pitch = pBot->m_astate.pitch;
	actor->angle = pBot->m_astate.yaw;

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

APlayerPawn* WBotEngineInterface::get_player(player_t* plr) {
	return plr->mo;
}

ActorState WBotEngineInterface::get_actor_state(AActor* actor) {
	ActorState ret;
	ret.origin = vec3(actor->x, actor->y, actor->z) / (float)FRACUNIT;
	ret.velocity = vec3(actor->velx, actor->vely, actor->velz) / (float)FRACUNIT;
	ret.health = actor->health;
	ret.height = actor->height >> FRACBITS;
	ret.pClass = actor->GetClass();
	ret.pitch = actor->pitch;
	ret.yaw = actor->angle;
	ret.radius = actor->radius >> FRACBITS;
	ret.sector = &g_map.sectors[actor->Sector - ::sectors];
	ret.name = ret.pClass->TypeName.GetChars();
	return ret;
}

PlayerState WBotEngineInterface::get_player_state(player_t* plr) {
	PlayerState ret;
	ret.isFrozen = plr->cheats & (CF_FROZEN | CF_TOTALLYFROZEN);
	ret.onGround = plr->onground;
	ret.viewHeight = plr->viewheight >> FRACBITS;
	ret.weaponName = plr->ReadyWeapon ? plr->ReadyWeapon->GetClass()->TypeName.GetChars() : NULL;
	return ret;
}

void WBotEngineInterface::player_select_weapon(player_t* pPlayer, AActor* weapon) {
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
	
CWootBot* WBotEngineInterface::get_player_bot(player_t* plr) {
	return plr->bIsBot && plr->pWootBot ? plr->pWootBot : NULL;
}

void WBotEngineInterface::freeze_player(player_t* plr, bool frozen) {
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

void WBotEngineInterface::give_all_weapons(player_t* plr) {
	cht_Give(plr, "backpack");
	cht_Give(plr, "weapons");
	cht_Give(plr, "ammo");
	cht_Give(plr, "keys");
	cht_Give(plr, "armor");
}

void WBotEngineInterface::kill_actor(AActor* actor) {
	P_DamageMobj(actor, actor, actor, TELEFRAG_DAMAGE, NAME_Suicide);
}

bool WBotEngineInterface::check_line_of_sight(AActor* looker, AActor* target) {
	return P_CheckSight(looker, target, SF_SEEPASTSHOOTABLELINES);
}

int WBotEngineInterface::PointToAngle2(float x1, float y1, float x2, float y2) {
	return R_PointToAngle2(x1 * FRACUNIT, y1 * FRACUNIT, x2 * FRACUNIT, y2 * FRACUNIT);
}

AActor* WBotEngineInterface::find_followable_player(int subid) {
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

AActor* WBotEngineInterface::find_boss_brain() {
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

AActor* WBotEngineInterface::find_enemy(CWootBot* pBot) {
	return P_BlockmapSearch(pBot->pActor, 10, look_for_enemies_in_block, pBot);
}

bool WBotEngineInterface::can_unlock_door(AActor* activator, MapLine* line) {
	return P_CheckKeys(activator, line->getArg(3), false);
}

std::vector<AActor*> WBotEngineInterface::find_prop_blockers() {
	vector<AActor*> propBlockers;

	TThinkerIterator<AActor> it;
	AActor* actor;
	while ((actor = it.Next())) {
		if (is_actor_immovable_solid_prop(actor))
			propBlockers.push_back(actor);
		//Printf("Prop '%s' %d\n", actor->GetClass()->TypeName.GetChars(), actor->health);
	}

	return propBlockers;
}

vector<AActor*> WBotEngineInterface::find_map_keys() {
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

std::vector<AActor*> WBotEngineInterface::find_map_weapons(const char* name) {
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

std::vector<AActor*> WBotEngineInterface::find_map_ammo(const char* name1, const char* name2) {
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

vector<vector<PClass*>> WBotEngineInterface::get_required_key_types(MapLine* line) {
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

vector<AActor*> WBotEngineInterface::get_player_weapons(APlayerPawn* pActor, bool loadedOnly) {
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

int WBotEngineInterface::get_weapon_ammo(AActor* weapon) {
	AWeapon* wep = (AWeapon*)weapon;
	return wep->Ammo1 ? wep->Ammo1->Amount : 0;
}

const char* WBotEngineInterface::get_class_type_name(PClass* pclass) {
	return pclass->TypeName.GetChars();
}

bool WBotEngineInterface::intermission_active() {
	return gamestate == GS_INTERMISSION;
}

void WBotEngineInterface::gprintf(const char* fmt, ...) {
	static char buffer[4096];

	va_list args;
	va_start(args, fmt);
	vsnprintf(buffer, sizeof(buffer), fmt, args);
	va_end(args);

	Printf("%s", buffer);
}

void WBotEngineInterface::print_hud_test(const char* msg, float x, float y, uint32_t id) {
	SERVERCOMMANDS_PrintHUDMessage(msg, x, y, 0, 0, 0, CR_RED, 1.0f, 0, 0, "SmallFont", id);
}

int WBotEngineInterface::get_game_tics() {
	return level.time;
}

const char* WBotEngineInterface::get_map_name() {
	return level.mapname;
}

bool WBotEngineInterface::TraceLine(vec3 start, vec3 end, bool ignoreMonsters, AActor* ignore, TraceResult* tr) {
	start *= FRACUNIT;
	end *= FRACUNIT;
		
	vec3 delta = end - start;
	fixed_t dist = delta.length();
	delta = delta.normalize(FRACUNIT);		

	sector_t* sector = P_PointInSector(start.x, start.y);

	FTraceResults trInternal;

	bool hit = ::Trace((fixed_t)start.x, (fixed_t)start.y, (fixed_t)start.z, sector,
		(fixed_t)delta.x, (fixed_t)delta.y, (fixed_t)delta.z, dist, ignoreMonsters ? 0 : MF_SOLID,
		ML_BLOCKING | ML_BLOCKEVERYTHING | ML_BLOCK_PLAYERS, ignore, trInternal);

	if (tr) {
		tr->endPos = vec3(trInternal.X, trInternal.Y, trInternal.Z) / (float)(1 << 16);
		tr->actor = trInternal.Actor;
		tr->frac = trInternal.Fraction / (float)FRACUNIT;
		tr->hitType = (TraceHitType)trInternal.HitType;
		tr->line = trInternal.Line ? &g_map.lines[trInternal.Line - ::lines] : NULL;
		tr->sector = trInternal.Sector ? &g_map.sectors[trInternal.Sector - ::sectors] : NULL;
	}

	return hit;
}

vector<TraceIsect> WBotEngineInterface::TraceIntersections(vec2 start, vec2 end) {
	start *= FRACUNIT;
	end *= FRACUNIT;

	vec2 delta = end - start;
	fixed_t maxDist = delta.length();
	delta = delta.normalize(FRACUNIT);

	fixed_t StartX = start.x;
	fixed_t StartY = start.y;
	fixed_t Vx = delta.x;
	fixed_t Vy = delta.y;

	FPathTraverse path(start.x, start.y, end.x, end.y, PT_ADDLINES);
	intercept_t* in;

	vector<TraceIsect> intersections;

	while ((in = path.Next())) {
		line_t* wall = in->d.line;
		fixed_t dist = FixedMul(maxDist, in->frac);

		TraceIsect isect;
		isect.line = &g_map.lines[wall - lines];
		isect.pos = vec2(StartX + FixedMul(Vx, dist), StartY + FixedMul(Vy, dist)) / (float)FRACUNIT;
		isect.fraction = in->frac;

		if (wall->backsector == NULL) {
			isect.sector = &g_map.sectors[in->d.line->frontsector - sectors];
		}
		else {
			int lineside = P_PointOnLineSide(StartX, StartY, in->d.line);
			sector_t* sec = lineside ? wall->backsector : wall->frontsector;
			isect.sector = &g_map.sectors[sec - sectors];
		}

		intersections.push_back(isect);
	}

	return intersections;
}

bool WBotEngineInterface::TraceImpassable(vec2 start, vec2 end) {
	start *= FRACUNIT;
	end *= FRACUNIT;

	FPathTraverse path(start.x, start.y, end.x, end.y, PT_ADDLINES);
	intercept_t* in;

	vec2 delta = end - start;
	fixed_t maxDist = delta.length();
	delta = delta.normalize(FRACUNIT);

	fixed_t StartX = start.x;
	fixed_t StartY = start.y;
	fixed_t Vx = delta.x;
	fixed_t Vy = delta.y;

	while ((in = path.Next())) {
		line_t* line = in->d.line;

		if (!line->backsector || (line->flags & ML_BLOCKING)) {
			fixed_t dist = FixedMul(maxDist, in->frac);
			vec2 pos = vec2(StartX + FixedMul(Vx, dist), StartY + FixedMul(Vy, dist)) / (float)FRACUNIT;
			sector_t* hitSector = R_PointInSubsector(pos.x * FRACUNIT, pos.y * FRACUNIT)->sector;

			if (hitSector != line->frontsector && hitSector != line->backsector) {
				vec2 lineStart = vec2(line->v1->x, line->v1->y) / (float)FRACUNIT;
				vec2 lineEnd = vec2(line->v2->x, line->v2->y) / (float)FRACUNIT;
				if (!PointAlignedSegment(pos, lineStart, lineEnd))
					continue; // bug in trace where lines in other sectors nowhere near the impact point are hit
			}

			return true;
		}
	}

	return false;
}

bool WBotEngineInterface::TraceSectorEdge(vec2 start, vec2 end, vec2& edge, MapLine** line) {
	start *= FRACUNIT;
	end *= FRACUNIT;

	FPathTraverse path(start.x, start.y, end.x, end.y, PT_ADDLINES);
	intercept_t* in = path.Next();

	if (in) {
		vec2 delta = end - start;
		fixed_t dist = FixedMul((fixed_t)delta.length(), in->frac);
		vec2 dir = delta.normalize(FRACUNIT);

		fixed_t StartX = start.x;
		fixed_t StartY = start.y;
		fixed_t Vx = dir.x;
		fixed_t Vy = dir.y;

		if (line) {
			*line = &g_map.lines[in->d.line - ::lines];
		}

		edge = vec2(StartX + FixedMul(Vx, dist), StartY + FixedMul(Vy, dist)) / (float)(1 << 16);
		return true;
	}
	else if (line) {
		*line = NULL;
	}

	edge = end;
	return false;
}

void* WBotEngineInterface::load_wad_lump(MapData* map, int id, int& len, int structSize) {
	int dataLen = map->Size(id);
	uint8_t* data = new uint8_t[dataLen];
	map->Read(id, data);
	len = dataLen / structSize;
	return data;
}

MapLumps WBotEngineInterface::load_wad_lump_data() {
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

SectorState WBotEngineInterface::get_sector_state(int id) {
	sector_t& sec = ::sectors[id];

	SectorState ret;
	ret.floorZ = sec.floorplane.Zat0() / (float)FRACUNIT;
	ret.ceilZ = sec.ceilingplane.Zat0() / (float)FRACUNIT;
	ret.ceilMoving = sec.ceilingdata;
	ret.floorMoving = sec.floordata;
	ret.special = sec.special;
		
	switch (sec.special) {
	case dDamage_Hellslime:
	case dDamage_LavaHefty:
	case dDamage_LavaWimpy:
	case dDamage_Nukage:
	case dDamage_SuperHellslime:
		ret.floorDamage = true;
		break;
	default:
		ret.floorDamage = false;
		break;
	}
		
	return ret;
}

LineState WBotEngineInterface::get_line_state(int id) {
	line_t& line = ::lines[id];

	LineState state;
	memset(&state, 0, sizeof(state));
	memcpy(state.args, line.args, sizeof(int) * 5);
	state.special = line.special;

	if (line.activation & SPAC_PlayerActivate) {
		state.flags |= FL_LINE_PLAYER_ACTIVATE;

		if (line.activation & SPAC_Impact) {
			state.goalAction = WBOT_GOAL_ACTION_SHOOT;
		}
		else if (line.activation & (SPAC_Use | SPAC_UseThrough)) {
			state.goalAction = WBOT_GOAL_ACTION_USE;
		}
		else if (line.activation & (SPAC_Cross | SPAC_AnyCross)) {
			state.goalAction = WBOT_GOAL_ACTION_CROSS;
		}
		else if (line.activation & SPAC_Push) {
			state.goalAction = WBOT_GOAL_ACTION_TOUCH;
		}
		else {
			gprintf("Don't know how to activate line %d\n", id);
		}
	}
	else {
		state.goalAction = -1;
	}

	if (state.goalAction == WBOT_GOAL_ACTION_CROSS && (line.flags & ML_TWOSIDED))
		state.flags |= FL_LINE_DOUBLE_SIDE_CROSS;

	if (!line.backsector || (line.flags & ML_BLOCKING))
		state.flags |= FL_LINE_IMPASSABLE;

	switch (line.special) {
	case Plat_UpWaitDownStay:
	case Plat_UpNearestWaitDownStay:
	case Plat_DownWaitUpStay:
	case Plat_DownWaitUpStayLip:
	case Ceiling_CrushRaiseAndStayA:
	case Ceiling_CrushAndRaiseA:
	case Ceiling_CrushAndRaiseSilentA:
		state.moveFlags |= FL_SECTOR_MOVE_TIMED;
		break;
	}

	// only add specials here that could be potentially helpful for unblocking a path.
	// For instance, raising a door or elevator. A ceiling or door coming down lower 
	// will not help a bot pass the sector
	switch (line.special) {
	case 0:
		break; // not a special line
	case Door_Open:
	case Door_Raise:
	case Ceiling_RaiseByValue:
	case Ceiling_RaiseToNearest:
	case Ceiling_RaiseInstant:
	case Ceiling_RaiseByValueTimes8:
	case Generic_Ceiling: // TODO: check if really does move up
	case Generic_Door:
	case Ceiling_CrushAndRaiseDist:
		state.moveFlags |= FL_SECTOR_MOVE_CEIL_UP;
		break;

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
		state.moveFlags |= FL_SECTOR_MOVE_FLOOR_UP;
		break;

	case Plat_PerpetualRaise:
	case Plat_PerpetualRaiseLip:
		state.moveFlags |= FL_SECTOR_MOVE_FLOOR_UP | FL_SECTOR_MOVE_FLOOR_DOWN;
		break;

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
		state.moveFlags |= FL_SECTOR_MOVE_FLOOR_DOWN;
		break;

	case Generic_Floor:
	case Elevator_MoveToFloor:
	case Generic_Lift:
	case Generic_Stairs:
		state.moveFlags |= FL_SECTOR_MOVE_FLOOR_UP | FL_SECTOR_MOVE_FLOOR_DOWN; // TODO: can do both dirs?	
		break;

	case Ceiling_LowerToHighestFloor:
	case Ceiling_LowerInstant:
	case Ceiling_CrushRaiseAndStayA:
	case Ceiling_CrushAndRaiseA:
	case Ceiling_CrushAndRaiseSilentA:
	case Ceiling_LowerByValueTimes8:
	case Door_Close:
	case Door_CloseWaitOpen:
		break; // a ceiling getting lower is not helpful

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
		break; // visual-only specials

	case Plat_Stop:
	case Ceiling_CrushStop:
		break; // does not cause sectors to move

	case Teleport:
		state.flags |= FL_LINE_IS_TELEPORT;
		break;

	case Door_LockedRaise:
		state.moveFlags |= FL_SECTOR_MOVE_CEIL_UP;
		state.flags |= FL_LINE_IS_LOCKED_DOOR;
		break;

	case Exit_Normal:
	case Exit_Secret:
		state.flags |= FL_LINE_IS_LEVEL_EXIT;
		break;

	default:
		Printf("Unknown special %d for line %d\n", line.special, id);
		break;
	}

	return state;
}

void WBotEngineInterface::add_stair_sector_info() {
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
			stairTrigger = BotGoal(get_line_state(s).goalAction, s);

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

MapLine* WBotEngineInterface::get_map_line_from_engine_line(line_t* line) {
	return &g_map.lines[line - ::lines];
}

vec2 WBotEngineInterface::get_tele_dest(int lineid) {
	line_t& line = ::lines[lineid];
	AActor* actor = SelectTeleDest(line.args[0], line.args[1]);
	return actor ? vec2(actor->x, actor->y) / (float)FRACUNIT : vec2(0, 0);
}

vector<MapLine*> WBotEngineInterface::get_crossed_lines(const vec2& pos, int radius) {
	FBoundingBox box(pos.x * FRACUNIT, pos.y * FRACUNIT, radius * FRACUNIT);
	FBlockLinesIterator it(box);
	line_t* ld;

	vector<MapLine*> lines;

	while ((ld = it.Next())) {
		if (box.Right() <= ld->bbox[BOXLEFT]
			|| box.Left() >= ld->bbox[BOXRIGHT]
			|| box.Top() <= ld->bbox[BOXBOTTOM]
			|| box.Bottom() >= ld->bbox[BOXTOP]) {
			continue;
		}

		if (box.BoxOnLineSide(ld) != -1)
			continue; // doesn't cross the line

		lines.push_back(&g_map.lines[ld - ::lines]);
	}

	return lines;
}

player_t* WBotEngineInterface::get_player_for_index(int i) {
	if (!playeringame[i])
		return NULL;

	AActor* actor = players[i].mo;

	return actor ? actor->player : NULL;
}

CWootBot* WBotEngineInterface::get_bot_for_index(int i) {
	player_t* player = get_player_for_index(i);
	return player && player->pWootBot ? player->pWootBot : NULL;
}

void WBotEngineInterface::set_actor_origin(AActor* actor, int x, int y, uint32_t angle, bool teleportFx) {
	angle *= ANGLE_1;
	fixed_t fx = x << FRACBITS;
	fixed_t fy = y << FRACBITS;
	P_Teleport(actor, fx, fy, ONFLOORZ, angle, teleportFx, teleportFx, false, true, false);
}

void WBotEngineInterface::MakeVectors(uint32_t angle, vec3& forward, vec3& right) {
	fixed_t fsine = finesine[angle >> ANGLETOFINESHIFT];
	fixed_t fcosine = finecosine[angle >> ANGLETOFINESHIFT];
	forward = vec3(fcosine, fsine, 0).normalize();
	right = vec3(fsine, -fcosine, 0).normalize();
}

void WBotEngineInterface::SpawnBlood(vec3 pos, int damage, AActor* owner) {
	pos *= FRACUNIT;
	SERVERCOMMANDS_SpawnBlood(pos.x, pos.y, pos.z, 0, damage, owner);
}

void WBotEngineInterface::PrintNotification(const char* msg) {
	SERVERCOMMANDS_Print(msg, PRINT_CHAT);
}

bool WBotEngineInterface::is_actor_immovable_solid_prop(AActor* actor) {
	if (!(actor->flags & (MF_SOLID)))
		return false;

	if (actor->player || (actor->flags3 & (MF3_ISMONSTER | MF_SHOOTABLE)))
		return false;

	return true;
}

bool WBotEngineInterface::are_cheats_enabled() {
	return sv_cheats;
}

void WBotEngineInterface::kill_all_shootables() {
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
}

const char* WBotEngineInterface::get_program_arg(const char* name) {
	if (Args->CheckParm("-wbtest")) {
		const char* value = Args->CheckValue("-wbtest");
		return value ? value : "";
	}

	return NULL;
}

void WBotEngineInterface::exit_level() {
	G_ExitLevel(0, false);
}

void WBotEngineInterface::change_level(const char* mapname, bool noIntermission) {
	G_ChangeLevel(level.mapname, 0, noIntermission ? CHANGELEVEL_NOINTERMISSION : 0);
}

CCMD(addbotw) {
	wbot::g_engine.add_bot();
}