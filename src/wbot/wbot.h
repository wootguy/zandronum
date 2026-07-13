#pragma once
#include "bots.h"

class CWootBot : public CSkullBot {
public:
	int m_forwardMove; // range of +/-100
	int m_sideMove;
	int m_targetLastSeenTic; // last tick the current target was visible
	angle_t m_fov;

	CWootBot(const char* pszName, const char* pszTeamName, ULONG ulPlayerNum);
	~CWootBot();

	// all thinking logic happens here
	void ParseScript(void) override;

	void FindEnemy();
	
	void DeadThink();
	void RoamThink();
	void CombatThink();

	FVector3 GetViewPos();

	void draw_debug_line(FVector3 start, FVector3 end);
};