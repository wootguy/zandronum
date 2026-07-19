#pragma once
#include "wb_nav.h"

class SectorNavMeshGenerator {
public:
	static NavSector* generate(); // returns a graph of size numsubsectors

private:
	static void calc_nav_centers(NavSector* mesh);
	static void add_jump_links(NavSector* mesh);
	static bool create_jump_link(NavSector& fromNav, NavSectorLink& fromLink, NavSector& toNav, NavSectorLink& toLink);
};