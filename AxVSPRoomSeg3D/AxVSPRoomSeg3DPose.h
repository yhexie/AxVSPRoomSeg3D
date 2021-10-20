#pragma once
#include <math.h>
#include <tuple> 
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_traits.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include "shapefil.h"

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/connected_components.hpp>

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
#include <ppl.h>
#include "DynamicVDBEDT.h"
#include "AxRoom3d.h"
#include "utility.h"
#include "AxDistCell.h"

using namespace openvdb;
typedef openvdb::math::dataCell7i dataCell;
using EDTree = tree::Tree4<dataCell, 5, 4, 3>::Type;
//using EDTree = tree::Tree3<dataCell, 6, 4>::Type;
using EDTGrid = Grid<EDTree>;
typedef std::vector<openvdb::math::Coord> CoordList;
using namespace boost;
typedef adjacency_list<vecS, vecS, undirectedS> Graph;
typedef graph_traits<Graph>::vertex_descriptor Vertex;
typedef graph_traits<Graph>::edge_descriptor  Edge_descriptor;
typedef property_map<Graph, vertex_index_t>::type IndexMap;
typedef graph_traits<Graph>::vertex_iterator vertex_iter;
typedef property_map<Graph, vertex_index_t>::type IndexMapVertex;
typedef std::pair<int, int> AxEdge;

class AxVSPRoomSeg3DPose
{
public:
	AxVSPRoomSeg3DPose();
	~AxVSPRoomSeg3DPose();

	void doInitialize();
	int doEDT();
	void doFillInnerSphere();
	void doFillInnerSphereEx();
	void doFillInnerSphereRandom();
	void doBuildGraph();//构造图
	void doConnectComponent();//连通子图分析
	void doWavefrontGrowing();//水波荡漾
	void Run();

	void doSaveSpherePlyFile();
	void saveInitialEdges();
	int  doSaveFinnalVDB();

	void setStartAndEndFrame(int start_id,int end_id);
	void setParameters(float door_width,float overlap_r, float min_seed_vol);
	void setVoxelSize(float size_vox);
	void setFilePath(std::string path);
	void evalMinMax(EDTGrid::Ptr dist_, dataCell& minVal, dataCell& maxVal);

public:
	openvdb::FloatGrid::Ptr grid_logocc_;//占用概率

	EDTGrid::Ptr dist_map_edt;//EDT距离变换过程用的
	std::shared_ptr<DynamicVDBEDT> grid_distance_transform;//EDT距离变换过程用的

	openvdb::FloatGrid::Ptr grid_dist_map;//存储距离
	openvdb::FloatGrid::Ptr grid_dist_map_threshold;//存储阈值分割之后的距离值

	openvdb::Int32Grid::Ptr grid_Label;//存储标记
	openvdb::Int32Grid::Ptr grid_Label_Spreading;//存储中间标记过程

	
	int max_coor_dist_;
	int max_coor_sqdist_;
	openvdb::Vec3d coor_min;
	openvdb::Vec3d coor_max;
	pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_edt_cloud;//存储距离变换结果
	pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud;//存储中间过程的内部求
	pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_label_cloud;//存储标记结果

	double FILTER_VOX_SIZE;
	double MAX_UPDATE_DIST, HIT_THICKNESS, VERSION;
	double L_FREE, L_OCCU, L_THRESH, L_MAX, L_MIN;
	double SENSOR_RANGE;
	double DIST_EDT_THRESHOLD;

	boost::shared_ptr<Graph> g;//构建图
	boost::shared_ptr<Graph> g_one;
	unsigned numOfVertex = 0;
	float tag_max_Radius = 0;

	int startId = 1;
	int endId = 8;
	float D_2_DOOR = 0.5;//门宽度阈值
	float OVERLAP_RATIO = 0.8;
	float THRESHOLD_VOLUME = 0.3;//种子区域的体积阈值，剔除孤立的球
	float VOX_SIZE;
	std::string tPath;
};

