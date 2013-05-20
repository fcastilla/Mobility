#pragma once

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