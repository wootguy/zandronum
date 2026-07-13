#include "wbot.h"
#include "botcommands.h"
#include "c_dispatch.h"
#include "network.h"
#include "d_event.h"

#include <stdlib.h>
#include <time.h>

#define RUN_SPEED 100 // max move speed allowed before the server kicks you

CWootBot::CWootBot(const char* pszName, const char* pszTeamName, ULONG ulPlayerNum)
	: CSkullBot(pszName, pszTeamName, ulPlayerNum) {

	static bool seedRand = false;
	if (!seedRand) {
		seedRand = true;
		srand((unsigned int)time(NULL));
	}

	wantDir = 1;
	m_speed = RUN_SPEED;

	m_bForwardMovePersist = true;
	m_bSideMovePersist = true;
}
CWootBot::~CWootBot() {}

void CWootBot::ParseScript() {
	if (m_pPlayer->health <= 0) {
		// tap a button to respawn
		m_lForwardMove = 0;
		m_lSideMove = 0;
		m_lButtons ^= BT_ATTACK;
		return;
	}

	if (rand() % 10 == 0) {
		wantDir = -wantDir;
	}

	m_lButtons = 0;
	m_lForwardMove = static_cast<LONG>(0x32 * (m_speed * wantDir / 100.0f));
	m_lSideMove = static_cast<LONG>(0x32 * (m_speed * wantDir / 100.0f));

	Printf("Lol bot think %d\n", wantDir);
}

CCMD(addbotw)
{
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