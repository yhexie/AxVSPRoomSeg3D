// AxInnerSphere3D.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <stdio.h>
#include <cstdlib>
#include <math.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_traits.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <openvdb/openvdb.h>
#include <openvdb/Grid.h>
#include <openvdb/math/DDA.h>
#include <openvdb/math/Ray.h>
#include <openvdb/version.h>
#include <openvdb/Exceptions.h>
#include <openvdb/io/File.h>
#include <openvdb/io/Queue.h>
#include <openvdb/io/Stream.h>
#include <openvdb/Metadata.h>
#include <openvdb/io/GridDescriptor.h>
#include <openvdb/math/Transform.h>
#include "DynamicVDBEDT.h"
#include "utility.h"
#include "AxVSPRoomSeg3DPose.h"
#include "ini.h"

using namespace openvdb;
typedef openvdb::math::dataCell7i dataCell;
using EDTree = tree::Tree4<dataCell, 5, 4, 3>::Type;
//using EDTree = tree::Tree3<dataCell, 6, 4>::Type;
using EDTGrid = Grid<EDTree>;
typedef std::vector<openvdb::math::Coord> CoordList;

static struct ini_file *gamedb = NULL;
int main()
{
	int startId = 1;
	int endId = 25;
	double door_width = 1.0;
	double voxel_size = 0.08;
	double overlap_ratio = 0.8;
	double min_seed_volume = 0.3;
	std::string t_path = "C:\\data\\scan";
	ini_free(gamedb);
	gamedb = ini_read(NULL, NULL, NULL);
	int err, line;
	ini_free(gamedb);
	gamedb = ini_read("configPos.ini", &err, &line);
	if (ini_get(gamedb, "Parameters", "StartID", NULL) != NULL)
	{
		const char* a = ini_get(gamedb, "Parameters", "StartID", NULL);
		startId = atof(a);
		if (startId < 1)
		{
			startId = 1;
		}
	}
	if (ini_get(gamedb, "Parameters", "EndID", NULL) != NULL)
	{
		const char* a = ini_get(gamedb, "Parameters", "EndID", NULL);
		endId = atof(a);
	}
	if (ini_get(gamedb, "Parameters", "DoorWidth", NULL) != NULL)
	{
		const char* a = ini_get(gamedb, "Parameters", "DoorWidth", NULL);
		door_width = atof(a);
	}
	if (ini_get(gamedb, "Parameters", "VoxelSize", NULL) != NULL)
	{
		const char* a = ini_get(gamedb, "Parameters", "VoxelSize", NULL);
		voxel_size = atof(a);
	}
	if (ini_get(gamedb, "Parameters", "OverlapRatio", NULL) != NULL)
	{
		const char* a = ini_get(gamedb, "Parameters", "OverlapRatio", NULL);
		overlap_ratio = atof(a);
	}
	if (ini_get(gamedb, "Parameters", "PCDFilePath", NULL) != NULL)
	{
		const char* a = ini_get(gamedb, "Parameters", "PCDFilePath", NULL);
		t_path = a;
	}
	if (ini_get(gamedb, "Parameters", "MinSeedRegionVolume", NULL) != NULL)
	{
		const char* a = ini_get(gamedb, "Parameters", "MinSeedRegionVolume", NULL);
		min_seed_volume = atof(a);
	}
	//参数设置
	AxVSPRoomSeg3DPose innerSphereGraph;
	innerSphereGraph.setStartAndEndFrame(startId, endId);
	innerSphereGraph.setParameters(door_width, overlap_ratio, min_seed_volume);
	innerSphereGraph.setVoxelSize(voxel_size);
	innerSphereGraph.setFilePath(t_path);
	innerSphereGraph.Run();

	// system("pause");
	return 0;
}

