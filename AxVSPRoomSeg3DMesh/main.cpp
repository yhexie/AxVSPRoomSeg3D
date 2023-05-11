// AxInnerSphere3D.cpp : �������̨Ӧ�ó������ڵ㡣
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
#include "AxVSPRoomSeg3DMesh.h"
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
	double door_width = 1.0;
	double voxel_size = 0.08;
	double overlap_ratio = 0.8;
	double min_seed_volume = 0.3;
	std::string t_path = "C:\\data\\scan";
	ini_free(gamedb);
	gamedb = ini_read(NULL, NULL, NULL);
	int err, line;
	ini_free(gamedb);
	gamedb = ini_read("configMesh.ini", &err, &line);
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
	if (ini_get(gamedb, "Parameters", "MeshFilePath", NULL) != NULL)
	{
		const char* a = ini_get(gamedb, "Parameters", "MeshFilePath", NULL);
		t_path = a;
	}
	if (ini_get(gamedb, "Parameters", "MinSeedRegionVolume", NULL) != NULL)
	{
		const char* a = ini_get(gamedb, "Parameters", "MinSeedRegionVolume", NULL);
		min_seed_volume = atof(a);
	}
	//��������
	AxVSPRoomSeg3DMesh innerSphereGraph;
	innerSphereGraph.setStartAndEndFrame(0, 0);
	innerSphereGraph.setParameters(door_width, overlap_ratio, min_seed_volume);
	innerSphereGraph.setVoxelSize(voxel_size);
	innerSphereGraph.setFilePath(t_path);//ע����Mesh���ļ���off��
	innerSphereGraph.Run();

	//system("pause");
	return 0;
}

