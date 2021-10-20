#pragma once
#include <openvdb/openvdb.h>
class AxDistCell
{
public:
	AxDistCell();
	~AxDistCell();
public:
	size_t ID;
	openvdb::Coord target_cell;
	double Distance;
};

