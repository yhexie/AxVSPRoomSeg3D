#ifndef AXCONFIGURATION_HEADER
#define AXCONFIGURATION_HEADER

struct AxConfiguration
{
public:
	int startId;
	int endId;
	double door_width;
public:
	AxConfiguration():startId(1)
		, endId(25)
		, door_width(1.0)
	{

	}
};
#endif