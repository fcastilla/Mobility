#pragma once

#ifdef DEBUG
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
#define DEBUG_NEW new(_NORMAL_BLOCK, __FILE__, __LINE__)
#define new DEBUG_NEW
#endif

class Location
{

public:
	Location(int _id, int _x, int _y) : id(_id), x(_x), y(_y) {}

	int getId(){ return id; }
	int getX() { return x; }
	int getY() { return y; }

private:
	int id, x, y;
};