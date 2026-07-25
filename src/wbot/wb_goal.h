#pragma once
#include "bots.h"
#include <unordered_set>
#include <unordered_map>

struct NavSectorLink;

enum BotGoalAction {
	WBOT_GOAL_ACTION_MOVE_TO,	// move to the goal sector and do nothing
	WBOT_GOAL_ACTION_USE,		// use the given linedef
	WBOT_GOAL_ACTION_TOUCH,		// touch the given actor
	WBOT_GOAL_ACTION_CROSS,		// cross the given line
	WBOT_GOAL_ACTION_SHOOT,		// shoot the given line
	WBOT_GOAL_ACTION_BOSS_BRAIN,// find some angle where the target can be damaged with rockets, then attack
};

// position to shoot a target from indirectly
struct IndirectShootPos {
	FVector3 shootFrom;
	FVector3 shootAt;
	int subid = -1;
};

struct BotGoal {
	int action = -1; // WBOT_GOAL_ACTION_*
	int lineid = -1; // lindef id to interact with
	int dist = 0; // distance from the goal to the purpose sector
	TObjPtr<AActor> actor = NULL; // actor to interact with

	// per-bot data
	std::unordered_set<int> blockers; // paths that can't be used while reacching this goal
	std::unordered_map<int, int> unblockAttempts; // maps a line to a path it was meant to unblock. Don't try that line again if the path is still blocked.
	IndirectShootPos shootAlignment; // where to aim and where to shoot from

	NavSectorLink* purposeLink = NULL; // link this goal is meant to unblock
	bool required = false; // if true, fail the parent goal and other required subgoals when this fails

	BotGoal() {}
	BotGoal(int action, int lineid) : action(action), lineid(lineid) {}
	BotGoal(int action, AActor* actor) : action(action), actor(actor) {}

	std::string desc();
	std::string descLong();
	int getNavId();
	FVector3 pos();
	int touchDistance(AActor* toucher); // how close the player needs to be to consider this goal as touched

	bool matches(BotGoal& other) {
		return action == other.action && lineid == other.lineid && actor == other.actor;
	}

	// false if the actor or lineid are no longer interactable
	bool valid();

	std::unordered_map<int, IndirectShootPos> FindBossBrainShootPositions();

private:
	void TestBossBrainShootRay(FVector3 brainPos, FVector3 rayStart, FVector3 rayDir, bool isCeilTrace, std::unordered_map<int, IndirectShootPos>& shootNodes);
};