#pragma once
#include "wb_nav.h"
#include <vector>

class SectorNavMeshGenerator {
public:
	static BotMeshData generate(std::vector<AActor*> propBlockers); // returns a graph of size numsubsectors

	// regenerates links to/fromt this node. Call when moving sectors are no longer movable
	// returns number of links added/removed
	static int relink_node(BotMeshData& mesh, int id, std::vector<AActor*> propBlockers);

private:
	static void calc_nav_centers(BotMeshData& mesh);
	static void add_jump_links(BotMeshData& mesh);
	static void add_jump_links(BotMeshData& mesh, int nodeid);
	static bool trace_jump(FVector3 start, FVector3 end, int fromMovement, int toMovement);
	static bool is_potential_jump_link(NavSector& fromNav, NavSectorLink& fromLink, NavSector& toNav, NavSectorLink& toLink);
	static bool create_jump_link(BotMeshData& mesh, NavSector& fromNav, NavSectorLink& fromLink, NavSector& toNav, NavSectorLink& toLink);

	static void add_walkable_links(BotMeshData& mesh, int nodeid, std::vector<AActor*> propBlockers);
};