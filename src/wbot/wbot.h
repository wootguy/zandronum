#pragma once
#include "bots.h"

class CWootBot : public CSkullBot {
public:
	int wantDir;
	int m_speed;

	CWootBot(const char* pszName, const char* pszTeamName, ULONG ulPlayerNum);
	~CWootBot();

	// all thinking logic happens here
	void ParseScript(void) override;
};