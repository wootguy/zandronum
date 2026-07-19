#pragma once
#include "bots.h"
#include <unordered_set>

struct NavSectorLink;

enum BotGoalAction {
	WBOT_GOAL_ACTION_MOVE_TO,	// move to the goal sector and do nothing
	WBOT_GOAL_ACTION_USE,		// use the given linedef
	WBOT_GOAL_ACTION_TOUCH,		// touch the given actor
	WBOT_GOAL_ACTION_CROSS,		// cross the given line
	WBOT_GOAL_ACTION_SHOOT,		// shoot the given line
};

struct BotGoal {
	int action = -1; // WBOT_GOAL_ACTION_*
	int lineid = -1; // lindef id to interact with
	std::unordered_set<int> blockers; // path IDs that block A* from reaching routing to this goal
	int dist = 0; // distance from the goal to the purpose sector
	NavSectorLink* purposeLink = NULL; // link this goal is meant to unblock
	TObjPtr<AActor> actor = NULL; // actor to interact with

	BotGoal(int action, int lineid) : action(action), lineid(lineid) {}
	BotGoal(int action, AActor* actor) : action(action), actor(actor) {}

	std::string desc();
	int getNavId();
	FVector3 pos();
	int touchDistance(AActor* toucher); // how close the player needs to be to consider this goal as touched

	bool matches(BotGoal& other) {
		return action == other.action && lineid == other.lineid && actor == other.actor;
	}

	// false if the actor or lineid are no longer interactable
	bool valid();
};