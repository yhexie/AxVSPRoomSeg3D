#include "AxVSPRoomSeg3DMesh.h"

bool sortDistCellAscending(AxDistCell a, AxDistCell b)
{
	return (a.ActiveDistance > b.ActiveDistance);
}

bool determineRoomIndexFromRoomID(const std::vector<AxRoom3d>& rooms, const int room_id, size_t& room_index)
{
	bool found_id = false;
	for (size_t r = 0; r < rooms.size(); r++)
	{
		if (rooms[r].ID == room_id)
		{
			room_index = r;
			found_id = true;
			break;
		}
	}

	return found_id;
}

void AxVSPRoomSeg3DMesh::getTiangleBoundingBox(Eigen::Vector3d v0, Eigen::Vector3d v1, Eigen::Vector3d v2,
	Eigen::Vector3d& tmpmin_, Eigen::Vector3d& tmpmax_)
{
	double minx = std::min(v0.x(), v1.x()) > v2.x() ? v2.x() : std::min(v0.x(), v1.x());
	double miny = std::min(v0.y(), v1.y()) > v2.y() ? v2.y() : std::min(v0.y(), v1.y());
	double minz = std::min(v0.z(), v1.z()) > v2.z() ? v2.z() : std::min(v0.z(), v1.z());
	double maxx = std::max(v0.x(), v1.x()) > v2.x() ? std::max(v0.x(), v1.x()) : v2.x();
	double maxy = std::max(v0.y(), v1.y()) > v2.y() ? std::max(v0.y(), v1.y()) : v2.y();
	double maxz = std::max(v0.z(), v1.z()) > v2.z() ? std::max(v0.z(), v1.z()) : v2.z();
	tmpmin_[0] = minx;
	tmpmin_[1] = miny;
	tmpmin_[2] = minz;
	tmpmax_[0] = maxx;
	tmpmax_[1] = maxy;
	tmpmax_[2] = maxz;
}

AxVSPRoomSeg3DMesh::AxVSPRoomSeg3DMesh():
	VOX_SIZE(0.1), FILTER_VOX_SIZE(0.4),MAX_UPDATE_DIST(100),	HIT_THICKNESS(1),
	VERSION(1),SENSOR_RANGE(10000)
{

}

void AxVSPRoomSeg3DMesh::doInitialize()
{
	max_coor_dist_ = int(MAX_UPDATE_DIST / VOX_SIZE);
	max_coor_sqdist_ = max_coor_dist_ * max_coor_dist_;

	openvdb::initialize();
	grid_dist2sky = openvdb::Int32Grid::create(0.0);//用于标识室内0和室外>0
	grid_occupied = openvdb::FloatGrid::create(0.0);//存储占用概率255，自由空间0和未知空间（对应grid_dist2sky中的室外）
	grid_dist_map = openvdb::FloatGrid::create(0.0);
	grid_Label = openvdb::Int32Grid::create(0.0);

	grid_distance_transform = std::make_shared<DynamicVDBEDT>(max_coor_dist_);
	grid_distance_transform->initialize(dist_map_edt, VOX_SIZE, VERSION);
	grid_distance_transform->setAccessor(dist_map_edt);

	const openvdb::math::Vec3d offset(VOX_SIZE / 2.0, VOX_SIZE / 2.0, VOX_SIZE / 2.0);
	openvdb::math::Transform::Ptr tf = openvdb::math::Transform::createLinearTransform(VOX_SIZE);
	tf->postTranslate(offset);
	grid_dist2sky->setTransform(tf);
	grid_occupied->setTransform(tf);
	grid_dist_map->setTransform(tf);
	grid_Label->setTransform(tf);

	FILTER_VOX_SIZE = D_2_DOOR - DIST_EDT_THRESHOLD;
}

int AxVSPRoomSeg3DMesh::doEDT()
{
	if (tPath.length()==0)
	{
		PCL_ERROR("Mesh文件路径不存在！\n");
		return 0;
	}
	//体素化，构建稀疏网格------------------------------------------
	Int32Grid::Accessor grid_dist2sky_acc = grid_dist2sky->getAccessor();
	FloatGrid::Accessor grid_acc = grid_occupied->getAccessor();
	FloatGrid::Accessor grid_dist_map_acc = grid_dist_map->getAccessor();

	//读取网格文件-------------------------------------------
	std::string filename = tPath + ".obj";
	std::cout << "读取文件：" << filename << std::endl;
	pcl::PolygonMesh meshData;//读取数据
	pcl::io::loadPolygonFile(filename, meshData);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(meshData.cloud, *cloud);

	Eigen::Vector4f min_pt, max_pt;						// 沿参考坐标系坐标轴的边界值
	pcl::getMinMax3D(*cloud, min_pt, max_pt);

	if (VOX_SIZE <= 0.0) {	PCL_ERROR("voxel_size <= 0.");	}

	Eigen::Vector4f grid_size = max_pt - min_pt;
	if (VOX_SIZE * std::numeric_limits<int>::max() < grid_size.maxCoeff())
	{
		PCL_ERROR("voxel_size is too small.");
	}	
	Vec3d map_min(min_pt.x(), min_pt.y(), min_pt.z());
	Vec3d map_max(max_pt.x(), max_pt.y(), max_pt.z());
	auto coor_min_original = dist_map_edt->worldToIndex(map_min);
	auto coor_max_original = dist_map_edt->worldToIndex(map_max);

	coor_min[0] = (int)std::floor(coor_min_original.x()) - 2;
	coor_min[1] = (int)std::floor(coor_min_original.y()) - 2;
	coor_min[2] = (int)std::floor(coor_min_original.z()) - 8;
	coor_max[0] = (int)std::ceil(coor_max_original.x()) + 2;
	coor_max[1] = (int)std::ceil(coor_max_original.y()) + 2;
	coor_max[2] = (int)std::ceil(coor_max_original.z()) + 2;

	std::vector<Eigen::Vector3i> occupiedVoxels;
	const Eigen::Vector3d box_half_size(VOX_SIZE / 2, VOX_SIZE / 2,	VOX_SIZE / 2);
	for (const pcl::Vertices &tria : meshData.polygons)//太慢
	{
		pcl::PointXYZ pt0 = cloud->points[tria.vertices[0]];
		pcl::PointXYZ pt1 = cloud->points[tria.vertices[1]];
		pcl::PointXYZ pt2 = cloud->points[tria.vertices[2]];
		const Eigen::Vector3d v0(pt0.x, pt0.y, pt0.z);
		const Eigen::Vector3d v1(pt1.x, pt1.y, pt1.z);
		const Eigen::Vector3d v2(pt2.x, pt2.y, pt2.z);
		Eigen::Vector3d tmpMin, tmpMax;
		getTiangleBoundingBox(v0, v1, v2, tmpMin, tmpMax);//获取三角形的包围盒
		//包围盒转体素块
		openvdb::Vec3d p1_xyz(tmpMin.x(), tmpMin.y(), tmpMin.z());
		openvdb::Vec3d p2_xyz(tmpMax.x(), tmpMax.y(), tmpMax.z());
		openvdb::Vec3d bbox_min = grid_occupied->worldToIndex(p1_xyz);
		openvdb::Vec3d bbox_max = grid_occupied->worldToIndex(p2_xyz);

		for (int x = bbox_min.x()-1; x <= bbox_max.x()+1; ++x) {
			for (int y = bbox_min.y()-1; y <= bbox_max.y()+1; ++y) {
				for (int z = bbox_min.z()-1; z <= bbox_max.z()+1; ++z) {
					Coord target_cell(x, y, z);
					auto world_xyz = dist_map_edt->indexToWorld(target_cell);
					const Eigen::Vector3d box_center(world_xyz.x(), world_xyz.y(), world_xyz.z());
					if (TriangleAABB(box_center, box_half_size, v0, v1, v2))
					{
						Eigen::Vector3i grid_index(x, y, z);
						occupiedVoxels.push_back(grid_index);//存入数组
						float ll_new = 255;
						grid_acc.setValueOn(target_cell, ll_new);//更新占用概率栅格
					}
				}
			}
		}
	}
	//标识室内、室外
	for (int x = coor_min.x(); x <= coor_max.x(); ++x) {
		for (int y = coor_min.y(); y <= coor_max.y(); ++y) {
			for (int z = coor_max.z(); z >=coor_min.z() ; --z) {
				Coord target_cell(x, y, z);	
				if (z == coor_max.z())//第一层1，当前层的原始像素=0，当前层为1，如果原始像素==255，则值为0
				{
					float value_t;
					bool isKnown = grid_acc.probeValue(target_cell, value_t);
					if (value_t ==255)
					{
						grid_dist2sky_acc.setValueOn(target_cell, 0);//更新距离天空的距离值
					}
					else
					{
						grid_dist2sky_acc.setValueOn(target_cell, 1);//更新距离天空的距离值1
					}
					
				}
				else
				{
					Coord cell_last(x, y, z+1);//上一层
					int ll_old; float value_t;
					bool isKnown = grid_dist2sky_acc.probeValue(cell_last, ll_old);
					isKnown = grid_acc.probeValue(target_cell, value_t);
					//第i>1层，如果i-1层为i-1，且当前层的原始像素=Unknow，当前层为i；
					//如果i-1层为i-1，且当前层的原始像素=255，，当前层为0；
					if (ll_old== coor_max.z()- z && value_t==0)
					{
						grid_dist2sky_acc.setValueOn(target_cell, coor_max.z() - z + 1);//更新距离天空的距离值z
					}
					else if (ll_old == coor_max.z() - z && value_t == 255)
					{
						grid_dist2sky_acc.setValueOn(target_cell, 0);//更新距离天空的距离值0
					}
					else if (ll_old == 0)//否则，i-1层为0，当前层值为0
					{
						grid_dist2sky_acc.setValueOn(target_cell, 0);//更新距离天空的距离值0,0表示为室内
					}
				}		
			}
		}
	}
	for (int x = coor_min.x() + 1; x <= coor_max.x() - 1; ++x) {
		for (int y = coor_min.y() + 1; y <= coor_max.y() - 1; ++y) {
			for (int z = coor_min.z() + 1; z <= coor_max.z() - 1; ++z) {
				Coord ijk(x, y, z);
				int ll_old0;
				bool isKnown0 = grid_dist2sky_acc.probeValue(ijk, ll_old0);
				if (isKnown0 && ll_old0==0)//标记室内空间
				{
					grid_distance_transform->dist_acc_->setValueOn(ijk);
				} 
			}
		}
	}
	//开始增加占用，开始EDT
	for (std::vector<Eigen::Vector3i>::iterator point = occupiedVoxels.begin(); point != occupiedVoxels.end(); ++point)
	{
		Eigen::Vector3i p_i = *point;
		openvdb::Vec3d p_xyz(p_i.x(), p_i.y(), p_i.z());
		openvdb::Vec3d p_ijk = p_xyz;//整形坐标
		bool truncated = false;
		static const int DIM = 1 << 0;
		openvdb::Coord ijk = Coord::floor(p_ijk);// &(~(DIM - 1));
		float ll_old;
		bool isKnown = grid_acc.probeValue(ijk, ll_old);
		if (!isKnown) {
			grid_distance_transform->dist_acc_->setValueOn(ijk);
		} // unknown -> occupied -> EDT SetObstacle
		else if (ll_old ==255) {
			grid_distance_transform->setObstacle(ijk);
		} // free -> occupied -> EDT SetObstacle
	}
	grid_distance_transform->update();
	
	//保存结果------------------------------------------
	pcl_edt_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
	for (int x = coor_min.x(); x <= coor_max.x(); ++x) {
		for (int y = coor_min.y(); y <= coor_max.y(); ++y) {
			for (int z = coor_min.z(); z <= coor_max.z(); ++z) {
				Coord target_cell(x, y, z);
				Coord nearest_obst;
				auto cell_dist = grid_distance_transform->query_sq_distance(target_cell, nearest_obst);

				if (cell_dist < 0 || cell_dist >= max_coor_sqdist_) { // 注意此处为距离平方
					continue;
				}			
				auto world_xyz = dist_map_edt->indexToWorld(target_cell);
				pcl::PointXYZI pI;
				pI.x = world_xyz.x();
				pI.y = world_xyz.y();
				pI.z = world_xyz.z();
				pI.intensity = sqrt(cell_dist)*VOX_SIZE;
				pcl_edt_cloud->push_back(pI);
				grid_dist_map_acc.setValueOn(target_cell, pI.intensity);
			}// end z loop
		} // end y loop
	} // end x loop

	std::string vdbFilename, edtFilename, gridName;
	vdbFilename = "something.vdb";
	openvdb::io::File file(vdbFilename);
	openvdb::GridPtrVec grids;
	grids.push_back(grid_occupied);//占用概率
	file.write(grids);
	file.close();

	edtFilename = "edt.vdb";
	openvdb::io::File fileedt(edtFilename);
	openvdb::GridPtrVec grids2;
	grids2.push_back(grid_dist_map);//距离变换
	fileedt.write(grids2);
	fileedt.close();

	if (pcl_edt_cloud->points.size() > 0)
	{
		std::string filename3 = "saveEDTIntensity.pcd";
		if (pcl::io::savePCDFile<pcl::PointXYZI>(filename3, *pcl_edt_cloud) == -1) //保存文件
		{
			PCL_ERROR("EDT文件保存失败！\n");
			return -1;
		}
	}
	else
	{
		PCL_ERROR("EDT切片点数为0！\n");
		return -1;
	}
	return 0;
}

void AxVSPRoomSeg3DMesh::doFillInnerSphere()
{
	pcl_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);

	grid_dist_map_threshold = grid_dist_map->deepCopy();//存放阈值分割之后的值
	FloatGrid::Accessor grid_dist_map_acc = grid_dist_map_threshold->getAccessor();
	//进行阈值分割
	for (int x = coor_min.x(); x <= coor_max.x(); ++x) {
		for (int y = coor_min.y(); y <= coor_max.y(); ++y) {
			for (int z = coor_min.z(); z <= coor_max.z(); ++z) {
				Coord target_cell(x, y, z);
				float ll_old;
				bool isKnown = grid_dist_map_acc.probeValue(target_cell, ll_old);
				if (ll_old <= DIST_EDT_THRESHOLD)
				{
					grid_dist_map_acc.setActiveState(target_cell, false);
				}
				else
				{
					float ll_new =  ll_old - DIST_EDT_THRESHOLD;
					grid_dist_map_acc.setValueOn(target_cell, ll_new);
				}
			}
		}
	}
	//进行填充
	double radius = 0;
	double filterVoxelSize = VOX_SIZE * 1.01;
	do
	{
		//找到最大的距离
		double max_dist_edt = -1.0;
		openvdb::Coord max_coord;
		for (int x = coor_min.x(); x <= coor_max.x(); ++x) {
			for (int y = coor_min.y(); y <= coor_max.y(); ++y) {
				for (int z = coor_min.z(); z <= coor_max.z(); ++z) {
					Coord target_cell(x, y, z);
					float ll_old;
					bool isKnown = grid_dist_map_acc.probeValue(target_cell, ll_old);
					if (!isKnown)
					{
						continue;
					}
					auto world_t_cell = grid_dist_map_threshold->indexToWorld(target_cell);
					int tickNum = 0;
					int num = pcl_cloud->size();// ->points;
					if (pcl_cloud->size() == 0)
					{

					}
					else
					{
						for (int i = 0; i < pcl_cloud->size(); i++)
						{
							pcl::PointXYZI pt = pcl_cloud->at(i);
							float centerX_tmp = pt.x;
							float centerY_tmp = pt.y;
							float centerZ_tmp = pt.z;
							float radius_tmp = pt.intensity + 0.5 * radius;
							int flag = inSphere(radius_tmp, centerX_tmp, centerY_tmp, centerZ_tmp, world_t_cell.x(), world_t_cell.y(), world_t_cell.z());
							if (!flag)//如果点不在圆内
							{
								tickNum++;
							}
							else
							{			
								//PCL_INFO("点在圆内\n");
								break;
							}
						}
					}
					//不在任何一个圆内
					num = pcl_cloud->size();// ->points;
					if (tickNum == pcl_cloud->size() )
					{		
						if (ll_old > max_dist_edt) {
							max_dist_edt = ll_old;
							max_coord = target_cell;
						}
					}
					else
					{
						continue;
					}
				}
			}
		}
		if (max_dist_edt==-1)
		{
			break;
		}
		openvdb::Coord max_cell;
		radius = max_dist_edt;
		max_cell.setX(max_coord.x());
		max_cell.setY(max_coord.y());
		max_cell.setZ(max_coord.z());
		numOfVertex++;
		if (numOfVertex==1)
		{
			tag_max_Radius= 2.0 * radius;
		}
		auto world_xyz_tmp = grid_dist_map_threshold->indexToWorld(max_cell);
		double centerX = world_xyz_tmp.x();
		double centerY = world_xyz_tmp.y();
		double centerZ = world_xyz_tmp.z();
		pcl::PointXYZI centerPt;
		centerPt.x = centerX;
		centerPt.y = centerY;
		centerPt.z = centerZ;
		centerPt.intensity = radius;
		pcl_cloud->push_back(centerPt);
		//printf("最小值%f，最大值%f \n", 0, radius);
	}while (radius > filterVoxelSize);
}

void AxVSPRoomSeg3DMesh::doFillInnerSphereEx()
{
	pcl_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
	std::vector<AxDistCell> listDistCells;
	grid_dist_map_threshold = grid_dist_map->deepCopy();//存放阈值分割之后的值
	FloatGrid::Accessor grid_dist_map_acc = grid_dist_map_threshold->getAccessor();
	int gNumX = coor_max.x() - coor_min.x() + 1;
	int gNumY = coor_max.y() - coor_min.y() + 1;
	int gNumZ = coor_max.z() - coor_min.z() + 1;
	//进行阈值分割
	for (int idx = coor_min.x(); idx <= coor_max.x(); ++idx) {
		for (int idy = coor_min.y(); idy <= coor_max.y(); ++idy) {
			for (int idz = coor_min.z(); idz <= coor_max.z(); ++idz) {
				openvdb::Coord target_cell(idx, idy, idz);
				float ll_old;
				bool isKnown = grid_dist_map_acc.probeValue(target_cell, ll_old);
				if (ll_old <= DIST_EDT_THRESHOLD)
				{
					grid_dist_map_acc.setActiveState(target_cell, false);
				}
				else
				{
					float ll_new = ll_old - DIST_EDT_THRESHOLD;
					grid_dist_map_acc.setValueOn(target_cell, ll_new);
					AxDistCell cell;
					cell.ID = idz*gNumX*gNumY + idx*gNumY + idy;
					cell.target_cell = target_cell;
					cell.Distance = ll_new;
					listDistCells.push_back(cell);
				}
			}
		}
	}
	Concurrency::parallel_sort(listDistCells.begin(), listDistCells.end(), sortDistCellAscending);
	AxDistCell maxDist_cell = listDistCells[0];
	openvdb::Coord max_cell = maxDist_cell.target_cell;
	double radius = maxDist_cell.Distance;
	tag_max_Radius = 2.0 * radius;
	auto world_xyz = grid_dist_map_threshold->indexToWorld(max_cell);
	double centerX = world_xyz.x();
	double centerY = world_xyz.y();
	double centerZ = world_xyz.z();
	pcl::PointXYZI centerPt;
	centerPt.x = centerX;
	centerPt.y = centerY;
	centerPt.z = centerZ;
	centerPt.intensity = radius;
	pcl_cloud->push_back(centerPt);

	//将包围盒box插入r-tree，包围盒与i索引关联
	Boost_Point sphere_min(centerX - VOX_SIZE, centerY - VOX_SIZE, centerZ - VOX_SIZE);
	Boost_Point sphere_max(centerX + VOX_SIZE, centerY + VOX_SIZE, centerZ + VOX_SIZE);
	Boost_Box s_box(sphere_min, sphere_max);
	int idx_sphere = pcl_cloud->size() - 1;
	rtree_spheres.insert(std::make_pair(s_box, idx_sphere));

	//printf("最小值%f，最大值%f \n", 0, radius);
	std::vector<AxDistCell>::iterator iter = listDistCells.begin();
	listDistCells.erase(iter);
	//从队列中移除已经被球覆盖的区域
	openvdb::Coord max_coord = max_cell;
	double accelerateRadius = radius;
	openvdb::Coord minSphereCoord;
	minSphereCoord.setX(max_coord.x() - std::floor(accelerateRadius / VOX_SIZE));
	minSphereCoord.setY(max_coord.y() - std::floor(accelerateRadius / VOX_SIZE));
	minSphereCoord.setZ(max_coord.z() - std::floor(accelerateRadius / VOX_SIZE));
	openvdb::Coord maxSphereCoord;
	maxSphereCoord.setX(max_coord.x() + std::ceil(accelerateRadius / VOX_SIZE));
	maxSphereCoord.setY(max_coord.y() + std::ceil(accelerateRadius / VOX_SIZE));
	maxSphereCoord.setZ(max_coord.z() + std::ceil(accelerateRadius / VOX_SIZE));
	int coord_min_x = std::max(minSphereCoord.x(), (int)coor_min.x());
	int coord_min_y = std::max(minSphereCoord.y(), (int)coor_min.y());
	int coord_min_z = std::max(minSphereCoord.z(), (int)coor_min.z());
	int coord_max_x = std::min(maxSphereCoord.x(), (int)coor_max.x());
	int coord_max_y = std::min(maxSphereCoord.y(), (int)coor_max.y());
	int coord_max_z = std::min(maxSphereCoord.z(), (int)coor_max.z());

	for (int idx = coord_min_x; idx <= coord_max_x; ++idx) {
		for (int idy = coord_min_y; idy <= coord_max_y; ++idy) {
			for (int idz = coord_min_z; idz <= coord_max_z; ++idz) {
				Coord target_cell(idx, idy, idz);
				size_t tmp_CELL_ID = idz*gNumX*gNumY + idx*gNumY + idy;
				float ll_old;
				bool isKnown = grid_dist_map_acc.probeValue(target_cell, ll_old);
				if (!isKnown && ll_old <= 0)
				{
					continue;
				}
				auto world_box_cell = grid_dist_map_threshold->indexToWorld(target_cell);
				double centerX_box = world_box_cell.x();
				double centerY_box = world_box_cell.y();
				double centerZ_box = world_box_cell.z();
				int flag = inSphere(radius, centerX, centerY, centerZ, centerX_box, centerY_box, centerZ_box);
				if (flag)//如果点在圆内
				{
					std::vector<AxDistCell>::iterator iter ;
					for (iter= listDistCells.begin(); iter != listDistCells.end(); ++iter)
					{
						AxDistCell dist_cell = *iter;
						size_t id = dist_cell.ID;
						if (tmp_CELL_ID == id)
						{
							listDistCells.erase(iter);
						}
					}
				}
			}
		}
	}
	//进行填充
	double filterVoxelSize = VOX_SIZE * 1.01;
	while (radius > filterVoxelSize)
	{
		//找到最大的距离
		Concurrency::parallel_sort(listDistCells.begin(), listDistCells.end(), sortDistCellAscending);
		AxDistCell maxDist_cell = listDistCells[0];
		max_cell = maxDist_cell.target_cell;
		radius = maxDist_cell.Distance;
		size_t t_id = maxDist_cell.ID;
		auto world_xyz_tmp = grid_dist_map_threshold->indexToWorld(max_cell);
		double centerX_current = world_xyz_tmp.x();
		double centerY_current = world_xyz_tmp.y();
		double centerZ_current = world_xyz_tmp.z();
		//重叠度太大，移除
		int num_sub = pcl_cloud->size();// ->points;
		if (num_sub > 0)
		{
			int idx_count = 0;

			Boost_Point sphere_min(centerX_current - VOX_SIZE, centerY_current - VOX_SIZE, centerZ_current - VOX_SIZE);
			Boost_Point sphere_max(centerX_current + VOX_SIZE, centerY_current + VOX_SIZE, centerZ_current + VOX_SIZE);
			Boost_Box query(sphere_min, sphere_max);
			std::vector<Boost_Value> possible_neighbors;
			rtree_spheres.query(bgi::intersects(query), std::back_inserter(possible_neighbors));

			for (int i = 0; i < possible_neighbors.size(); i++)
			{
				int idx = possible_neighbors[i].second;
				pcl::PointXYZI pt = pcl_cloud->at(idx);
				float centerX_tmp = pt.x;
				float centerY_tmp = pt.y;
				float centerZ_tmp = pt.z;
				float radius_tmp = pt.intensity;
				int flag2 = OverlapOrDepartTwoSphere(OVERLAP_RATIO,radius_tmp, centerX_tmp, centerY_tmp, centerZ_tmp, radius,
					centerX_current, centerY_current, centerZ_current);
				if (flag2)
				{
					idx_count++;
				}
				else
				{
					break;
				}
			}
			if (idx_count == possible_neighbors.size())//和所有的都不重叠
			{
				pcl::PointXYZI centerPt0;
				centerPt0.x = centerX_current;
				centerPt0.y = centerY_current;
				centerPt0.z = centerZ_current;
				centerPt0.intensity = radius;
				pcl_cloud->push_back(centerPt0);

				//将包围盒box插入r-tree，包围盒与i索引关联
				Boost_Point sphere_min(centerX - VOX_SIZE, centerY - VOX_SIZE, centerZ - VOX_SIZE);
				Boost_Point sphere_max(centerX + VOX_SIZE, centerY + VOX_SIZE, centerZ + VOX_SIZE);
				Boost_Box s_box(sphere_min, sphere_max);
				int idx_sphere = pcl_cloud->size() - 1;
				rtree_spheres.insert(std::make_pair(s_box, idx_sphere));

				//printf("最小值%f，最大值%f \n", 0, radius);
				std::vector<AxDistCell>::iterator iter = listDistCells.begin();
				listDistCells.erase(iter);
				//从队列中移除已经被球覆盖的区域
				openvdb::Coord max_coord = max_cell;
				double accelerateRadius = radius;
				openvdb::Coord minSphereCoord;
				minSphereCoord.setX(max_coord.x() - std::floor(accelerateRadius / VOX_SIZE));
				minSphereCoord.setY(max_coord.y() - std::floor(accelerateRadius / VOX_SIZE));
				minSphereCoord.setZ(max_coord.z() - std::floor(accelerateRadius / VOX_SIZE));
				openvdb::Coord maxSphereCoord;
				maxSphereCoord.setX(max_coord.x() + std::ceil(accelerateRadius / VOX_SIZE));
				maxSphereCoord.setY(max_coord.y() + std::ceil(accelerateRadius / VOX_SIZE));
				maxSphereCoord.setZ(max_coord.z() + std::ceil(accelerateRadius / VOX_SIZE));
				int coord_min_x = std::max(minSphereCoord.x(), (int)coor_min.x());
				int coord_min_y = std::max(minSphereCoord.y(), (int)coor_min.y());
				int coord_min_z = std::max(minSphereCoord.z(), (int)coor_min.z());
				int coord_max_x = std::min(maxSphereCoord.x(), (int)coor_max.x());
				int coord_max_y = std::min(maxSphereCoord.y(), (int)coor_max.y());
				int coord_max_z = std::min(maxSphereCoord.z(), (int)coor_max.z());

				for (int idx = coord_min_x; idx <= coord_max_x; ++idx) {
					for (int idy = coord_min_y; idy <= coord_max_y; ++idy) {
						for (int idz = coord_min_z; idz <= coord_max_z; ++idz) {
							Coord target_cell(idx, idy, idz);
							size_t tmp_CELL_Id = idz*gNumX*gNumY + idx*gNumY + idy;
							float ll_old;
							bool isKnown = grid_dist_map_acc.probeValue(target_cell, ll_old);
							if (!isKnown && ll_old <= 0)
							{
								continue;
							}
							auto world_box_cell = grid_dist_map_threshold->indexToWorld(target_cell);
							double centerX_box = world_box_cell.x();
							double centerY_box = world_box_cell.y();
							double centerZ_box = world_box_cell.z();
							int flag = inSphere(radius, centerX_current, centerY_current, centerZ_current, centerX_box, centerY_box, centerZ_box);
							if (flag)//如果点在圆内
							{
								std::vector<AxDistCell>::iterator iter ;
								for (iter= listDistCells.begin(); iter != listDistCells.end(); ++iter)
								{
									AxDistCell dist_cell = *iter;
									size_t id = dist_cell.ID;
									if (tmp_CELL_Id == id)
									{
										listDistCells.erase(iter);
									}
								}
							}
						}
					}
				}
			}
			else//存在重叠
			{
				std::vector<AxDistCell>::iterator iter = listDistCells.begin();
				listDistCells.erase(iter);
			}
		}
	}
}

//随机填充，加快速度
void AxVSPRoomSeg3DMesh::doFillInnerSphereRandom()
{
	pcl_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
	std::vector<AxDistCell> listDistCells;
	std::map<size_t, size_t> indexKeyCell;//存储listDistCells键和下标
	grid_dist_map_threshold = grid_dist_map->deepCopy();//存放阈值分割之后的值
	FloatGrid::Accessor grid_dist_map_acc = grid_dist_map_threshold->getAccessor();
	int gNumX = coor_max.x() - coor_min.x() + 1;
	int gNumY = coor_max.y() - coor_min.y() + 1;
	int gNumZ = coor_max.z() - coor_min.z() + 1;
	//进行阈值分割
	for (int idx = coor_min.x(); idx <= coor_max.x(); ++idx) {
		for (int idy = coor_min.y(); idy <= coor_max.y(); ++idy) {
			for (int idz = coor_min.z(); idz <= coor_max.z(); ++idz) {
				openvdb::Coord target_cell(idx, idy, idz);
				float ll_old;
				bool isKnown = grid_dist_map_acc.probeValue(target_cell, ll_old);
				if (ll_old <= DIST_EDT_THRESHOLD)
				{
					grid_dist_map_acc.setActiveState(target_cell, false);
				}
				else
				{
					float ll_new = ll_old - DIST_EDT_THRESHOLD;
					grid_dist_map_acc.setValueOn(target_cell, ll_new);
					AxDistCell cell;
					cell.ID = idz*gNumX*gNumY + idx*gNumY + idy;
					cell.target_cell = target_cell;
					cell.Distance = ll_new;
					cell.ActiveDistance = cell.Distance;//有效状态
					listDistCells.push_back(cell);
					indexKeyCell[cell.ID] = listDistCells.size() - 1;
				}
			}
		}
	}

	//第一步骤：随机填充
	size_t selectedPointIndex = -1;
	size_t iter_radnum = 0;
	size_t numSlectedPts = floor(0.5 * listDistCells.size());
	size_t numMax = listDistCells.size();
	std::vector<size_t> nCard(numMax, 0);
	srand(time(NULL));
	for (int i = 0; i < numMax; i++)
	{
		nCard[i] = i;
	}
	std::random_shuffle(nCard.begin(), nCard.begin() + numMax);//随机生成n个点
	do
	{
		selectedPointIndex = nCard[iter_radnum];
		if (selectedPointIndex > 0 && selectedPointIndex < numMax)
		{
			AxDistCell randomDistCell = listDistCells[selectedPointIndex];//随机选择一个点
			openvdb::Coord random_cell = randomDistCell.target_cell;
			double radius = randomDistCell.Distance;
			double isActiveradius = randomDistCell.ActiveDistance;
			if (isActiveradius < 0 || radius < VOX_SIZE )//D_2_DOOR)* 1.01
			{
				continue;
			}
			if (radius > 3)//房间的面积，填充半径不超过3m
			{
				continue;
			}
			auto world_xyz = grid_dist_map_threshold->indexToWorld(random_cell);
			int minx_voxel = random_cell.x() - radius / VOX_SIZE;
			int miny_voxel = random_cell.y() - radius / VOX_SIZE;
			int minz_voxel = random_cell.z() - radius / VOX_SIZE;
			int maxx_voxel = random_cell.x() + radius / VOX_SIZE;
			int maxy_voxel = random_cell.y() + radius / VOX_SIZE;
			int maxz_voxel = random_cell.z() + radius / VOX_SIZE;
			if (minx_voxel< coor_min.x() || miny_voxel< coor_min.y()|| minz_voxel< coor_min.z()|| maxx_voxel> coor_max.x() || maxy_voxel> coor_max.y() || maxz_voxel> coor_max.z())
			{
				continue;
			}
			double centerX_current = world_xyz.x();
			double centerY_current = world_xyz.y();
			double centerZ_current = world_xyz.z();
			//重叠度太大，移除
			int num_sub = pcl_cloud->size();// ->points;
			if (num_sub > 0)
			{
				Boost_Point sphere_min(centerX_current - VOX_SIZE, centerY_current - VOX_SIZE, centerZ_current-VOX_SIZE);
				Boost_Point sphere_max(centerX_current + VOX_SIZE, centerY_current + VOX_SIZE, centerZ_current + VOX_SIZE);
				Boost_Box query(sphere_min, sphere_max);
				std::vector<Boost_Value> possible_neighbors;
				rtree_spheres.query(bgi::intersects(query), std::back_inserter(possible_neighbors));
				int idx_count = 0;
				for (int i = 0; i < possible_neighbors.size(); i++)
				{
					int idx = possible_neighbors[i].second;
					pcl::PointXYZI pt = pcl_cloud->at(idx);
					float centerX_tmp = pt.x;
					float centerY_tmp = pt.y;
					float centerZ_tmp = pt.z;
					float radius_tmp = pt.intensity;
					int flag2 = OverlapOrDepartTwoSphere(OVERLAP_RATIO, radius_tmp, centerX_tmp, centerY_tmp, centerZ_tmp, radius,
						centerX_current, centerY_current, centerZ_current);
					if (flag2)
					{
						idx_count++;
					}
					else
					{
						break;
					}
				}
				if (idx_count == possible_neighbors.size())//和所有的近邻球都不重叠
				{
					pcl::PointXYZI centerPt;
					centerPt.x = centerX_current;
					centerPt.y = centerY_current;
					centerPt.z = centerZ_current;
					centerPt.intensity = radius;
					pcl_cloud->push_back(centerPt);
					printf("最小值%f，最大值%f \n", 0, radius);
					Boost_Point sphere_min(centerX_current - VOX_SIZE, centerY_current - VOX_SIZE, centerZ_current - VOX_SIZE);
					Boost_Point sphere_max(centerX_current + VOX_SIZE, centerY_current + VOX_SIZE, centerZ_current + VOX_SIZE);
					Boost_Box s_box(sphere_min, sphere_max);
					int idx_sphere = pcl_cloud->size() - 1;
					rtree_spheres.insert(std::make_pair(s_box, idx_sphere));
				}
			}
			else
			{
				pcl::PointXYZI centerPt;
				centerPt.x = centerX_current;
				centerPt.y = centerY_current;
				centerPt.z = centerZ_current;
				centerPt.intensity = radius;
				pcl_cloud->push_back(centerPt);
				printf("最小值%f，最大值%f \n", 0, radius);
				//将包围盒box插入r-tree，包围盒与i索引关联
				Boost_Point sphere_min(centerX_current - VOX_SIZE, centerY_current - VOX_SIZE, centerZ_current - VOX_SIZE);
				Boost_Point sphere_max(centerX_current + VOX_SIZE, centerY_current + VOX_SIZE, centerZ_current + VOX_SIZE);
				Boost_Box s_box(sphere_min, sphere_max);
				int idx_sphere = pcl_cloud->size() - 1;
				rtree_spheres.insert(std::make_pair(s_box, idx_sphere));
			}
		}
	} while (iter_radnum++ < numSlectedPts);
	//从队列中移除已经被球覆盖的区域
	for (size_t i = 0; i < pcl_cloud->size(); i++)
	{
		pcl::PointXYZI pt = pcl_cloud->at(i);
		float centerX_tmp = pt.x;
		float centerY_tmp = pt.y;
		float centerZ_tmp = pt.z;
		float radius_tmp = pt.intensity;
		Vec3d sphere_together(centerX_tmp, centerY_tmp, centerZ_tmp);
		auto world_sphere_coord = grid_dist_map_threshold->worldToIndex(sphere_together);
		double accelerateRadius = radius_tmp;
		openvdb::Coord minSphereCoord;
		minSphereCoord.setX(world_sphere_coord.x() - std::floor(accelerateRadius / VOX_SIZE));
		minSphereCoord.setY(world_sphere_coord.y() - std::floor(accelerateRadius / VOX_SIZE));
		minSphereCoord.setZ(world_sphere_coord.z() - std::floor(accelerateRadius / VOX_SIZE));
		openvdb::Coord maxSphereCoord;
		maxSphereCoord.setX(world_sphere_coord.x() + std::ceil(accelerateRadius / VOX_SIZE));
		maxSphereCoord.setY(world_sphere_coord.y() + std::ceil(accelerateRadius / VOX_SIZE));
		maxSphereCoord.setZ(world_sphere_coord.z() + std::ceil(accelerateRadius / VOX_SIZE));
		int coord_min_x = std::max(minSphereCoord.x(), (int)coor_min.x());
		int coord_min_y = std::max(minSphereCoord.y(), (int)coor_min.y());
		int coord_min_z = std::max(minSphereCoord.z(), (int)coor_min.z());
		int coord_max_x = std::min(maxSphereCoord.x(), (int)coor_max.x());
		int coord_max_y = std::min(maxSphereCoord.y(), (int)coor_max.y());
		int coord_max_z = std::min(maxSphereCoord.z(), (int)coor_max.z());

		for (int idx = coord_min_x; idx <= coord_max_x; ++idx) {
			for (int idy = coord_min_y; idy <= coord_max_y; ++idy) {
				for (int idz = coord_min_z; idz <= coord_max_z; ++idz) {
					Coord target_cell(idx, idy, idz);
					size_t tmp_CELL_ID = idz* gNumX * gNumY + idx*gNumY + idy;
					float ll_old;
					bool isKnown = grid_dist_map_acc.probeValue(target_cell, ll_old);
					if (!isKnown && ll_old <= 0)
					{
						continue;
					}
					auto world_box_cell = grid_dist_map_threshold->indexToWorld(target_cell);
					double centerX_box = world_box_cell.x();
					double centerY_box = world_box_cell.y();
					double centerZ_box = world_box_cell.z();
					int flag = inSphere(radius_tmp, centerX_tmp, centerY_tmp, centerZ_tmp, centerX_box, centerY_box, centerZ_box);
					if (flag)//如果点在圆内
					{
						std::map<size_t, size_t>::iterator it=indexKeyCell.find(tmp_CELL_ID);
						if (it!= indexKeyCell.end())
						{
							size_t idx = indexKeyCell[tmp_CELL_ID];
							listDistCells[idx].ActiveDistance = -1;//无效状态
						}
					}
				}
			}
		}
	}
	
	//-------------------------------------第二步骤：排序填充--------------------------------------------------
	std::vector<AxDistCell> listDistCells_new;
	for (size_t ix = 0; ix < listDistCells.size(); ix++)
	{
		AxDistCell tmp=	listDistCells[ix];
		if (tmp.ActiveDistance > -1)
		{
			listDistCells_new.push_back(tmp);
		}
	}
	listDistCells = listDistCells_new;
	Concurrency::parallel_sort(listDistCells.begin(), listDistCells.end(), sortDistCellAscending);
	AxDistCell maxDist_cell = listDistCells[0];
	openvdb::Coord max_cell = maxDist_cell.target_cell;
	double radius = maxDist_cell.Distance;
	while (radius > 3.0)
	{
		std::vector<AxDistCell>::iterator iter = listDistCells.begin();
		listDistCells.erase(iter);
		if (listDistCells.size()> 0)
		{
			maxDist_cell = listDistCells[0];
			max_cell = maxDist_cell.target_cell;
			radius = maxDist_cell.Distance;
		}
		else
		{
			break;
		}
	}
	while (true)
	{
		int minx_voxel = max_cell.x() - radius / VOX_SIZE;
		int miny_voxel = max_cell.y() - radius / VOX_SIZE;
		int minz_voxel = max_cell.z() - radius / VOX_SIZE;
		int maxx_voxel = max_cell.x() + radius / VOX_SIZE;
		int maxy_voxel = max_cell.y() + radius / VOX_SIZE;
		int maxz_voxel = max_cell.z() + radius / VOX_SIZE;
		if (minx_voxel< coor_min.x() || miny_voxel< coor_min.y() || minz_voxel< coor_min.z() || maxx_voxel> coor_max.x() || maxy_voxel> coor_max.y() || maxz_voxel> coor_max.z())
		{
			std::vector<AxDistCell>::iterator iter = listDistCells.begin();
			listDistCells.erase(iter);
			if (listDistCells.size()> 0)
			{
				maxDist_cell = listDistCells[0];
				max_cell = maxDist_cell.target_cell;
				radius = maxDist_cell.Distance;
			}
			else
			{
				break;
			}		
		}
		else
		{
			break;
		}
	}
	if (listDistCells.size()== 0)
	{
		return;
	}
	
	tag_max_Radius = 2.0 * radius;
	auto world_xyz = grid_dist_map_threshold->indexToWorld(max_cell);
	double centerX = world_xyz.x();
	double centerY = world_xyz.y();
	double centerZ = world_xyz.z();
	pcl::PointXYZI centerPt;
	centerPt.x = centerX;
	centerPt.y = centerY;
	centerPt.z = centerZ;
	centerPt.intensity = radius;
	pcl_cloud->push_back(centerPt);

	//将包围盒box插入r-tree，包围盒与i索引关联
	Boost_Point sphere_min(centerX - VOX_SIZE, centerY - VOX_SIZE, centerZ - VOX_SIZE);
	Boost_Point sphere_max(centerX + VOX_SIZE, centerY + VOX_SIZE, centerZ + VOX_SIZE);
	Boost_Box s_box(sphere_min, sphere_max);
	int idx_sphere = pcl_cloud->size() - 1;
	rtree_spheres.insert(std::make_pair(s_box, idx_sphere));

	printf("最小值%f，最大值%f \n", 0, radius);
	std::vector<AxDistCell>::iterator iter02 = listDistCells.begin();
	listDistCells.erase(iter02);
	indexKeyCell.clear();
	for (size_t ix = 0; ix < listDistCells.size(); ix++)
	{
		AxDistCell tmp = listDistCells[ix];//无效状态
		size_t tmp_CELL_ID = tmp.ID;
		indexKeyCell[tmp_CELL_ID] = ix;
	}
	//从队列中移除已经被球覆盖的区域
	openvdb::Coord max_coord = max_cell;
	double accelerateRadius = radius;
	openvdb::Coord minSphereCoord;
	minSphereCoord.setX(max_coord.x() - std::floor(accelerateRadius / VOX_SIZE));
	minSphereCoord.setY(max_coord.y() - std::floor(accelerateRadius / VOX_SIZE));
	minSphereCoord.setZ(max_coord.z() - std::floor(accelerateRadius / VOX_SIZE));
	openvdb::Coord maxSphereCoord;
	maxSphereCoord.setX(max_coord.x() + std::ceil(accelerateRadius / VOX_SIZE));
	maxSphereCoord.setY(max_coord.y() + std::ceil(accelerateRadius / VOX_SIZE));
	maxSphereCoord.setZ(max_coord.z() + std::ceil(accelerateRadius / VOX_SIZE));
	int coord_min_x = std::max(minSphereCoord.x(), (int)coor_min.x());
	int coord_min_y = std::max(minSphereCoord.y(), (int)coor_min.y());
	int coord_min_z = std::max(minSphereCoord.z(), (int)coor_min.z());
	int coord_max_x = std::min(maxSphereCoord.x(), (int)coor_max.x());
	int coord_max_y = std::min(maxSphereCoord.y(), (int)coor_max.y());
	int coord_max_z = std::min(maxSphereCoord.z(), (int)coor_max.z());

	for (int idx = coord_min_x; idx <= coord_max_x; ++idx) {
		for (int idy = coord_min_y; idy <= coord_max_y; ++idy) {
			for (int idz = coord_min_z; idz <= coord_max_z; ++idz) {
				Coord target_cell(idx, idy, idz);
				size_t tmp_CELL_ID = idz*gNumX*gNumY + idx*gNumY + idy;
				float ll_old;
				bool isKnown = grid_dist_map_acc.probeValue(target_cell, ll_old);
				if (!isKnown && ll_old <= 0)
				{
					continue;
				}
				auto world_box_cell = grid_dist_map_threshold->indexToWorld(target_cell);
				double centerX_box = world_box_cell.x();
				double centerY_box = world_box_cell.y();
				double centerZ_box = world_box_cell.z();
				int flag = inSphere(radius, centerX, centerY, centerZ, centerX_box, centerY_box, centerZ_box);
				if (flag)//如果点在圆内
				{
					std::map<size_t, size_t>::iterator it = indexKeyCell.find(tmp_CELL_ID);
					if (it != indexKeyCell.end())
					{
						size_t idx = indexKeyCell[tmp_CELL_ID];
						listDistCells[idx].ActiveDistance = -1;//无效状态
					}
				}
			}
		}
	}
	std::vector<AxDistCell> listDistCells_N;
	for (size_t ix = 0; ix < listDistCells.size(); ix++)
	{
		AxDistCell tmp = listDistCells[ix];//无效状态
		if (tmp.ActiveDistance > -1)
		{
			listDistCells_N.push_back(tmp);
		}
	}
	listDistCells = listDistCells_N;
	//进行填充
	double filterVoxelSize = VOX_SIZE * 1.01;
	while (radius > filterVoxelSize)
	{	
		//找到最大的距离
		Concurrency::parallel_sort(listDistCells.begin(), listDistCells.end(), sortDistCellAscending);
		AxDistCell maxDist_cell = listDistCells[0];
		max_cell = maxDist_cell.target_cell;
		radius = maxDist_cell.Distance;
		int minx_voxel = max_cell.x() - radius / VOX_SIZE;
		int miny_voxel = max_cell.y() - radius / VOX_SIZE;
		int minz_voxel = max_cell.z() - radius / VOX_SIZE;
		int maxx_voxel = max_cell.x() + radius / VOX_SIZE;
		int maxy_voxel = max_cell.y() + radius / VOX_SIZE;
		int maxz_voxel = max_cell.z() + radius / VOX_SIZE;
		if (radius > 3.0)
		{
			std::vector<AxDistCell>::iterator iter = listDistCells.begin();
			listDistCells.erase(iter);
			continue;
		}
		if (minx_voxel< coor_min.x() || miny_voxel< coor_min.y() || minz_voxel< coor_min.x() || maxx_voxel> coor_max.x() || maxy_voxel> coor_max.y() || maxz_voxel> coor_max.z())
		{
			std::vector<AxDistCell>::iterator iter = listDistCells.begin();
			listDistCells.erase(iter);
			continue;
		}
		size_t t_id = maxDist_cell.ID;
		auto world_xyz_tmp = grid_dist_map_threshold->indexToWorld(max_cell);
		double centerX_current = world_xyz_tmp.x();
		double centerY_current = world_xyz_tmp.y();
		double centerZ_current = world_xyz_tmp.z();
		//重叠度太大，移除
		int num_sub = pcl_cloud->size();// ->points;
		if (num_sub > 0)
		{
			Boost_Point sphere_min(centerX_current - VOX_SIZE, centerY_current - VOX_SIZE, centerZ_current - VOX_SIZE);
			Boost_Point sphere_max(centerX_current + VOX_SIZE, centerY_current + VOX_SIZE, centerZ_current + VOX_SIZE);
			Boost_Box query(sphere_min, sphere_max);
			std::vector<Boost_Value> possible_neighbors;
			rtree_spheres.query(bgi::intersects(query), std::back_inserter(possible_neighbors));

			int idx_count = 0;
			for (int i = 0; i < possible_neighbors.size(); i++)
			{
				int idx = possible_neighbors[i].second;
				pcl::PointXYZI pt = pcl_cloud->at(idx);
				float centerX_tmp = pt.x;
				float centerY_tmp = pt.y;
				float centerZ_tmp = pt.z;
				float radius_tmp = pt.intensity;
				int flag2 = OverlapOrDepartTwoSphere(OVERLAP_RATIO, radius_tmp, centerX_tmp, centerY_tmp, centerZ_tmp, radius,
					centerX_current, centerY_current, centerZ_current);
				if (flag2)
				{
					idx_count++;
				}
				else
				{
					break;
				}
			}
			if (idx_count == possible_neighbors.size())//和所有的近邻都不重叠
			{
				pcl::PointXYZI centerPt0;
				centerPt0.x = centerX_current;
				centerPt0.y = centerY_current;
				centerPt0.z = centerZ_current;
				centerPt0.intensity = radius;
				pcl_cloud->push_back(centerPt0);
				printf("最小值%f，最大值%f \n", 0, radius);

				//将包围盒box插入r-tree，包围盒与i索引关联
				Boost_Point sphere_min(centerX - VOX_SIZE, centerY - VOX_SIZE, centerZ - VOX_SIZE);
				Boost_Point sphere_max(centerX + VOX_SIZE, centerY + VOX_SIZE, centerZ + VOX_SIZE);
				Boost_Box s_box(sphere_min, sphere_max);
				int idx_sphere = pcl_cloud->size() - 1;
				rtree_spheres.insert(std::make_pair(s_box, idx_sphere));

				std::vector<AxDistCell>::iterator iter = listDistCells.begin();
				listDistCells.erase(iter);

				indexKeyCell.clear();
				for (size_t ix = 0; ix < listDistCells.size(); ix++)
				{
					AxDistCell tmp = listDistCells[ix];
					size_t tmp_CELL_ID = tmp.ID;
					indexKeyCell[tmp_CELL_ID] = ix;
				}

				//从队列中移除已经被球覆盖的区域
				openvdb::Coord max_coord = max_cell;
				double accelerateRadius = radius;
				openvdb::Coord minSphereCoord;
				minSphereCoord.setX(max_coord.x() - std::floor(accelerateRadius / VOX_SIZE));
				minSphereCoord.setY(max_coord.y() - std::floor(accelerateRadius / VOX_SIZE));
				minSphereCoord.setZ(max_coord.z() - std::floor(accelerateRadius / VOX_SIZE));
				openvdb::Coord maxSphereCoord;
				maxSphereCoord.setX(max_coord.x() + std::ceil(accelerateRadius / VOX_SIZE));
				maxSphereCoord.setY(max_coord.y() + std::ceil(accelerateRadius / VOX_SIZE));
				maxSphereCoord.setZ(max_coord.z() + std::ceil(accelerateRadius / VOX_SIZE));
				int coord_min_x = std::max(minSphereCoord.x(), (int)coor_min.x());
				int coord_min_y = std::max(minSphereCoord.y(), (int)coor_min.y());
				int coord_min_z = std::max(minSphereCoord.z(), (int)coor_min.z());
				int coord_max_x = std::min(maxSphereCoord.x(), (int)coor_max.x());
				int coord_max_y = std::min(maxSphereCoord.y(), (int)coor_max.y());
				int coord_max_z = std::min(maxSphereCoord.z(), (int)coor_max.z());

				for (int idx = coord_min_x; idx <= coord_max_x; ++idx) {
					for (int idy = coord_min_y; idy <= coord_max_y; ++idy) {
						for (int idz = coord_min_z; idz <= coord_max_z; ++idz) {
							Coord target_cell(idx, idy, idz);
							size_t tmp_CELL_Id = idz*gNumX*gNumY + idx*gNumY + idy;
							float ll_old;
							bool isKnown = grid_dist_map_acc.probeValue(target_cell, ll_old);
							if (!isKnown && ll_old <= 0)
							{
								continue;
							}
							auto world_box_cell = grid_dist_map_threshold->indexToWorld(target_cell);
							double centerX_box = world_box_cell.x();
							double centerY_box = world_box_cell.y();
							double centerZ_box = world_box_cell.z();
							int flag = inSphere(radius, centerX_current, centerY_current, centerZ_current, centerX_box, centerY_box, centerZ_box);
							if (flag)//如果点在圆内
							{
								std::map<size_t, size_t>::iterator it = indexKeyCell.find(tmp_CELL_Id);
								if (it != indexKeyCell.end())
								{
									size_t idx = indexKeyCell[tmp_CELL_Id];
									listDistCells[idx].ActiveDistance = -1;//无效状态
								}
							}
						}
					}
				}
				std::vector<AxDistCell> listDistCells_N2;
				for (size_t ix = 0; ix < listDistCells.size(); ix++)
				{
					AxDistCell tmp = listDistCells[ix];//无效状态
					if (tmp.ActiveDistance > -1)
					{
						listDistCells_N2.push_back(tmp);
					}
				}
				listDistCells = listDistCells_N2;
			}
			else//存在重叠
			{
				std::vector<AxDistCell>::iterator iter = listDistCells.begin();
				listDistCells.erase(iter);

				//size_t idx = indexKeyCell[t_id];
				//listDistCells[idx].ActiveDistance = -1;//无效状态

				/*indexKeyCell.clear();
				for (size_t ix = 0; ix < listDistCells.size(); ix++)
				{
					AxDistCell tmp = listDistCells[ix];
					size_t tmp_CELL_ID = tmp.ID;
					indexKeyCell[tmp_CELL_ID] = ix;
				}*/
			}
		}
	}
}



void AxVSPRoomSeg3DMesh::doBuildGraph()//构造图
{
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
	tree->setInputCloud(pcl_cloud);
	std::vector<AxEdge> list_edges;
	std::vector<AxEdge> list_edges_addition;
	numOfVertex= pcl_cloud->size();
	for (int i = 0; i < pcl_cloud->size(); i++)
	{
		std::vector<int> nn_indices;
		std::vector<float> nn_dists;
		pcl::PointXYZI pt = pcl_cloud->at(i);

		if (tree->radiusSearch(pt, tag_max_Radius, nn_indices, nn_dists) == 0)//有错误，程序无法进入
		{
			continue;
		}
		if (nn_indices.size() > 1)
		{
			int itick = 1;
			for (int k = 1; k < nn_indices.size(); k++)
			{
				const pcl::PointXYZI source_pt = pcl_cloud->at(nn_indices[k]);
				float dist = nn_dists[k];
				float delta_d = std::sqrt(dist) - pt.intensity - source_pt.intensity;
				if (delta_d < VOX_SIZE)
				{
					AxEdge ee(i, nn_indices[k]);
					list_edges.push_back(ee);
				}
				else
				{
					itick++;
				}
			}
			if (itick == nn_indices.size())//孤立的节点
			{
				AxEdge ee(i, nn_indices[1]);
				list_edges_addition.push_back(ee);
			}
		}
	}
	//构造Graph
	g.reset(new Graph(numOfVertex));
	g_one.reset(new Graph(numOfVertex));	//构造新图Graph
	for (int j = 0; j < list_edges.size(); j++)
	{
		AxEdge e_tmp = list_edges[j];
		Vertex u = vertex(e_tmp.first, *g);
		Vertex v = vertex(e_tmp.second, *g);
		Edge_descriptor e1;
		bool found = false;
		boost::tie(e1, found) = edge(u, v, *g);
		if (found == false)
		{
			boost::add_edge(e_tmp.first, e_tmp.second, *g);
			boost::add_edge(e_tmp.first, e_tmp.second, *g_one);
		}
	}
	for (int j = 0; j < list_edges_addition.size(); j++)//所有孤立的节点都连上
	{
		AxEdge e_tmp = list_edges_addition[j];
		Vertex u = vertex(e_tmp.first, *g_one);
		Vertex v = vertex(e_tmp.second, *g_one);
		Edge_descriptor e1;
		bool found = false;
		boost::tie(e1, found) = edge(u, v, *g_one);
		if (found == false)
		{
			boost::add_edge(e_tmp.first, e_tmp.second, *g_one);
		}
	}
	boost::print_graph(*g, boost::get(vertex_index, *g));
}

void AxVSPRoomSeg3DMesh::doConnectComponent()
{
	std::vector< int > component(boost::num_vertices(*g));
	int num = boost::connected_components(*g, &component[0]);//Total number of components

	std::vector<int>::size_type i_comp;
	FloatGrid::Accessor grid_dist_map_acc = grid_dist_map->getAccessor();//未进行阈值分割的距离变换值
	//FloatGrid::Accessor grid_dist_map_threshold_acc = grid_dist_map_threshold->getAccessor();
	Int32Grid::Accessor label_acc = grid_Label->getAccessor();
	for (int x = coor_min.x(); x <= coor_max.x(); ++x) {
		for (int y = coor_min.y(); y <= coor_max.y(); ++y) {
			for (int z = coor_min.z(); z <= coor_max.z(); ++z) {
				Coord target_cell(x, y, z);
				float ll_value;
				bool isKnown = grid_dist_map_acc.probeValue(target_cell, ll_value);
				if (isKnown && ll_value > 0)
				{
					label_acc.setValueOn(target_cell, 0);//标记为0
				}
			}
		}
	}
	std::map<int, float> m_volumn_seed;
	for (i_comp = 0; i_comp != component.size(); ++i_comp)
	{
		int RoomIdx = component[i_comp] + 1;//Label从1开始，i_comp是结点的索引
		pcl::PointXYZI pt = pcl_cloud->at(i_comp);
		float centerX = pt.x;
		float centerY = pt.y;
		float centerZ = pt.z;
		float radius = pt.intensity;
		m_volumn_seed[RoomIdx] += 4.0 * M_PI*radius*radius*radius / 3;
	}
	for (i_comp = 0; i_comp != component.size(); ++i_comp)
	{
		int RoomIdx = component[i_comp] + 1;//Label从1开始，i_comp是结点的索引
		float curSeedVol = m_volumn_seed[RoomIdx];
		if (curSeedVol < THRESHOLD_VOLUME)//体积小于给定值的剔除！
		{
			continue;
		}
		pcl::PointXYZI pt = pcl_cloud->at(i_comp);
		float centerX = pt.x;
		float centerY = pt.y;
		float centerZ = pt.z;
		float radius = pt.intensity;
		openvdb::Vec3d p_xyz(centerX, centerY, centerZ);
		openvdb::Vec3d p_ijk = grid_dist_map->worldToIndex(p_xyz);
		Coord minSphereCoord;
		minSphereCoord.setX(p_ijk.x() - std::floor(radius / VOX_SIZE));
		minSphereCoord.setY(p_ijk.y() - std::floor(radius / VOX_SIZE));
		minSphereCoord.setZ(p_ijk.z() - std::floor(radius / VOX_SIZE));
		Coord maxSphereCoord;
		maxSphereCoord.setX(p_ijk.x() + std::ceil(radius / VOX_SIZE));
		maxSphereCoord.setY(p_ijk.y() + std::ceil(radius / VOX_SIZE));
		maxSphereCoord.setZ(p_ijk.z() + std::ceil(radius / VOX_SIZE));
		int coord_min_x = std::max(minSphereCoord.x(), (int)coor_min.x());
		int coord_min_y = std::max(minSphereCoord.y(), (int)coor_min.y());
		int coord_min_z = std::max(minSphereCoord.z(), (int)coor_min.z());
		int coord_max_x = std::min(maxSphereCoord.x(), (int)coor_max.x());
		int coord_max_y = std::min(maxSphereCoord.y(), (int)coor_max.y());
		int coord_max_z = std::min(maxSphereCoord.z(), (int)coor_max.z());
		for (int x = coord_min_x; x <= coord_max_x; ++x) {
			for (int y = coord_min_y; y <= coord_max_y; ++y) {
				for (int z = coord_min_z; z <= coord_max_z; ++z) {
					Coord target_cell(x, y, z); 
					float ll_value;
					bool isKnown = grid_dist_map_acc.probeValue(target_cell, ll_value);
					if (isKnown)
					{
						auto world_t_cell = grid_dist_map->indexToWorld(target_cell);
						//如果点在圆内
						int flag = inSphere(radius, centerX, centerY, centerZ, world_t_cell.x(), world_t_cell.y(), world_t_cell.z());
						if (flag)//点在球内
						{
							label_acc.setValueOn(target_cell, RoomIdx);//标签
						}
					}			
				}
			}
		}
	}
	std::string vdbFilename, imgFilename, gridName;
	vdbFilename = "edt_label_initial.vdb";
	openvdb::io::File file(vdbFilename);
	openvdb::GridPtrVec grids;
	grids.push_back(grid_Label);
	file.write(grids);
	file.close();
}

void AxVSPRoomSeg3DMesh::doWavefrontGrowing()
{	
	bool finished = false;
	grid_Label_Spreading = grid_Label->deepCopy();
	Int32Grid::Accessor label_spreading_acc = grid_Label_Spreading->getAccessor();
	Int32Grid::Accessor label_acc = grid_Label->getAccessor();
	while (finished == false)
	{
		finished = true;
		for (int x = coor_min.x() + 1; x <= coor_max.x() - 1; ++x) {
			for (int y = coor_min.y() + 1; y <= coor_max.y() - 1; ++y) {
				for (int z = coor_min.z() + 1; z <= coor_max.z() - 1; ++z) {
					Coord target_cell(x, y, z);
					int ll_old;
					bool isKnown = label_spreading_acc.probeValue(target_cell, ll_old);
					if (isKnown && ll_old==0)		// unassigned pixels
					{
						//check 3x3 area around white pixel for fillcolour, if filled Pixel around fill white pixel with that colour
						bool set_value = false;
						for (int row_counter = -1; row_counter <= 1 && set_value == false; ++row_counter)
						{
							for (int column_counter = -1; column_counter <= 1 && set_value == false; ++column_counter)
							{
								for (int k_counter = -1; k_counter <= 1 && set_value == false; ++k_counter)
								{
									int h1 = x + row_counter;
									int h2 = y + column_counter;
									int h3 = z + k_counter;
									Coord spread_cell(h1, h2, h3);
									int value = -1;
									bool isKnown2 = label_acc.probeValue(spread_cell, value);
									if (value > 0 && isKnown2)
									{
										label_spreading_acc.setValueOn(target_cell, value);
										set_value = true;
										finished = false;	// keep on iterating the wavefront propagation until no more changes occur
									}
								}
							}
						}
					}
				}
			}
		}
		grid_Label = grid_Label_Spreading->deepCopy();
		label_acc = grid_Label->getAccessor();
	}
}

void AxVSPRoomSeg3DMesh::Run()
{
	std::string elefilename = tPath + "timecost.txt";
	FILE *costfile = fopen(elefilename.c_str(), "w");
	clock_t start, finish;
	double  duration;
	/* 测量一个事件持续的时间*/
	start = clock();
	doInitialize();
	finish = clock();
	duration = (double)(finish - start) / CLOCKS_PER_SEC;
	fprintf(costfile,"初始化：%f seconds\n", duration);

	printf("1.距离变换...\n");
	start = clock();
	doEDT();
	finish = clock();
	double duration1 = (double)(finish - start) / CLOCKS_PER_SEC;
	int ux = coor_max.x()- coor_min.x();
	int vy = coor_max.y()- coor_min.y();
	int wz = coor_max.z()- coor_min.z();
	fprintf(costfile, "Grid网格尺寸：%d %d %d;\n", ux,vy,wz);
	fprintf(costfile, "1.距离变换：%f seconds\n", duration1);

	printf("2.内部球填充...\n");
	start = clock();
	//doFillInnerSphere();
	//doFillInnerSphereEx();
	doFillInnerSphereRandom();
	finish = clock();
	doSaveSpherePlyFile();
	double duration2 = (double)(finish - start) / CLOCKS_PER_SEC;
	fprintf(costfile, "2.内部球填充：%f seconds\n", duration2);

	printf("3.阈值分割，构造初始的邻接图...\n");
	start = clock();
	doBuildGraph();
	finish = clock();
	saveInitialEdges();
	double duration3 = (double)(finish - start) / CLOCKS_PER_SEC;
	fprintf(costfile, "3.阈值分割，构造初始的邻接图：%f seconds\n", duration3);

	printf("4.连通分析，种子区域生成...\n");
	start = clock();
	doConnectComponent();
	finish = clock();
	double duration4 = (double)(finish - start) / CLOCKS_PER_SEC;
	fprintf(costfile, "4.连通分析，种子区域生成：%f seconds\n", duration4);

	printf("5.波前算法...\n");
	start = clock();
	doWavefrontGrowing();
	finish = clock();
	double duration5 = (double)(finish - start) / CLOCKS_PER_SEC;
	fprintf(costfile, "5.波前算法：%f seconds\n", duration5);
	double total = duration + duration1 + duration2 + duration3 + duration4+ duration5;
	fprintf(costfile, "总时间：%f seconds\n", total);
	fclose(costfile);

	printf("6.保存最终带标签的VDB...\n");
	doSaveFinnalVDB();

	printf("完成\n");
}

void AxVSPRoomSeg3DMesh::doSaveSpherePlyFile()
{
	int idx = 0; int count = 0;
	std::vector<std::tuple<double, double, double>> vertexs;
	std::vector<std::tuple<int, int, int>> indexs;
	//绘制一个球
	const char* saveFly = "inner_sphere.ply";
	FILE *ply = fopen(saveFly, "w");
	for (int i = 0; i < pcl_cloud->size(); i++)
	{
		std::vector<std::tuple<double, double, double>> vertexs_tmp;
		std::vector<std::tuple<int, int, int>> indexs_tmp;
		pcl::PointXYZI pt = pcl_cloud->at(i);
		double centerX = pt.x;
		double centerY = pt.y;
		double centerZ = pt.z;
		double d = pt.intensity;
		std::tuple<double, double, double> pos = std::make_tuple(centerX, centerY, centerZ);
		//drawSphere(pos, d,i * 2601, vertexs_tmp, indexs_tmp);
		drawSphere(pos, d, i * 121, vertexs_tmp, indexs_tmp);
		//drawSphere(pos, d, i * 441, vertexs_tmp, indexs_tmp);
		std::vector<std::tuple<double, double, double>>::iterator it;
		it = vertexs.begin();
		vertexs.insert(it, vertexs_tmp.begin(), vertexs_tmp.end());
		std::vector<std::tuple<int, int, int>>::iterator it_idx;
		it_idx = indexs.begin();
		indexs.insert(it_idx, indexs_tmp.begin(), indexs_tmp.end());
	}

	idx = vertexs.size();
	count = indexs.size();
	if (ply)
	{
		fprintf(ply, "ply\nformat %s 1.0\n", "ascii");
		fprintf(ply, "element vertex %d\n", idx);
		fprintf(ply, "property float x\n");
		fprintf(ply, "property float y\n");
		fprintf(ply, "property float z\n");
		fprintf(ply, "element face %d\n", count);
		fprintf(ply, "property list uint8 int32 vertex_indices\n");
		fprintf(ply, "end_header\n");
		for (std::vector<std::tuple<double, double, double>>::const_iterator v = vertexs.begin(); v != vertexs.end(); ++v)
		{
			std::tuple <double, double, double> vv = (std::tuple <double, double, double>)(*v);
			double x = std::get<0>(vv);
			double y = std::get<1>(vv);
			double z = std::get<2>(vv);
			fprintf(ply, "%f %f %f\n", x, y, z);
		}
		for (std::vector<std::tuple<int, int, int>>::const_iterator fit = indexs.begin(); fit != indexs.end(); ++fit)
		{
			std::tuple<int, int, int> vertId = (std::tuple<int, int, int>)(*fit);
			int id0 = std::get<0>(vertId);
			int id1 = std::get<1>(vertId);
			int id2 = std::get<2>(vertId);
			fprintf(ply, "%d %d %d %d\n", 3, id0, id1, id2);
		}
	}

	std::cout << "There are " << count << " facets in the domain." << std::endl;
	fclose(ply);
}

void AxVSPRoomSeg3DMesh::saveInitialEdges()
{
	FloatGrid::Accessor grid_dist_map_acc = grid_dist_map->getAccessor();
	char* pSaveRoomFile = "Edges.shp";
	int nShpTpyeSave = SHPT_ARCZ;//注意保存成线
	SHPHandle outShp = SHPCreate(pSaveRoomFile, nShpTpyeSave);
	DBFHandle dbf_h = DBFCreate(pSaveRoomFile);
	int fieldIdx = DBFAddField(dbf_h, "Shape", FTInteger, 2, 0);
	int fieldWeightIdx = DBFAddField(dbf_h, "Weight", FTDouble, 64, 6);
	SHPObject *psShape;
	IndexMap m_index = boost::get(vertex_index, *g);
	graph_traits<Graph>::edge_iterator ej, ej_end;
	for (tie(ej, ej_end) = edges(*g); ej != ej_end; ++ej)
	{
		size_t fromA = m_index[source(*ej, *g)];
		size_t fromB = m_index[target(*ej, *g)];
		//std::cout << "(" << fromA << "," << fromB << ") ";
		pcl::PointXYZI pt0 = pcl_cloud->at(fromA);
		pcl::PointXYZI pt1 = pcl_cloud->at(fromB);
		double *xCoords = new double[2];
		double *yCoords = new double[2];
		double *zCoords = new double[2];
		xCoords[0] = pt0.x;
		yCoords[0] = pt0.y;
		zCoords[0] = pt0.z;
		xCoords[1] = pt1.x;
		yCoords[1] = pt1.y;
		zCoords[1] = pt1.z;
		//求权重
		Eigen::Vector3f fromAA;
		fromAA.x() = pt0.x;
		fromAA.y() = pt0.y;
		fromAA.z() = pt0.z;
		Eigen::Vector3f fromBB;
		fromBB.x() = pt1.x;
		fromBB.y() = pt1.y;
		fromBB.z() = pt1.z;
		Eigen::Vector3f dir = fromBB - fromAA;
		Eigen::Vector3f midpt = fromAA + pt0.intensity / (pt0.intensity + pt1.intensity)*dir;
		openvdb::Vec3d p_xyz(midpt.x(), midpt.y(), midpt.z());
		openvdb::Vec3d p_ijk = grid_dist_map->worldToIndex(p_xyz);
		openvdb::Coord ijk(p_ijk.x(), p_ijk.y(), p_ijk.z());
		float ll_old;
		bool isKnown = grid_dist_map_acc.probeValue(ijk, ll_old);
		psShape = SHPCreateObject(nShpTpyeSave, -1, 0, NULL, NULL, 2, xCoords, yCoords, zCoords, NULL);
		int ishape = SHPWriteObject(outShp, -1, psShape);
		SHPDestroyObject(psShape);
		DBFWriteIntegerAttribute(dbf_h, ishape, 0, ishape);
		DBFWriteDoubleAttribute(dbf_h, ishape, fieldWeightIdx, ll_old);
	}
	SHPClose(outShp);
	DBFClose(dbf_h);
}

int AxVSPRoomSeg3DMesh::doSaveFinnalVDB()
{
	std::string vdbFilename, imgFilename, gridName;
	vdbFilename = "edt_label.vdb";
	openvdb::io::File file(vdbFilename);
	openvdb::GridPtrVec grids;
	grids.push_back(grid_Label);
	file.write(grids);
	file.close();
	//保存结果------------------------------------------
	pcl_label_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
	Int32Grid::Accessor label_acc = grid_Label->getAccessor();
	for (int x = coor_min.x(); x <= coor_max.x(); ++x) {
		for (int y = coor_min.y(); y <= coor_max.y(); ++y) {
			for (int z = coor_min.z(); z <= coor_max.z(); ++z) {
				Coord target_cell(x, y, z);
				int label = 0;
				bool isKnown2 = label_acc.probeValue(target_cell, label);
				auto world_xyz = grid_Label->indexToWorld(target_cell);
				if (isKnown2)
				{
					pcl::PointXYZI pI;
					pI.x = world_xyz.x();
					pI.y = world_xyz.y();
					pI.z = world_xyz.z();
					pI.intensity = label;
					pcl_label_cloud->push_back(pI);
				}			
			}
		} 
	}
	if (pcl_label_cloud->points.size() > 0)
	{
		std::string filename3 = "saveLabelIntensity.pcd";
		if (pcl::io::savePCDFile<pcl::PointXYZI>(filename3, *pcl_label_cloud) == -1) // load the file
		{
			PCL_ERROR("房间分割结果Label文件保存失败！\n");
			return -1;
		}
	}
	else
	{
		PCL_ERROR("房间分割结果Label点数为0！\n");
		return -1;
	}
	return 0;
}

void AxVSPRoomSeg3DMesh::evalMinMax(EDTGrid::Ptr dist_, dataCell& minVal, dataCell& maxVal)
{
	/// @todo optimize
	minVal = maxVal = zeroVal<dataCell>();
	int count_id = 0;
	if (EDTree::ValueOnCIter iter = dist_->cbeginValueOn()) {
		minVal = maxVal = *iter;
		for (++iter; iter; ++iter) {
			const dataCell& val = *iter;
			if (val.dist() < minVal.dist()) minVal = val;
			if (val.dist() > maxVal.dist()) maxVal = val;
			count_id++;
		}
	}
	std::cout << "datacell数目：" << count_id << std::endl;
}

void AxVSPRoomSeg3DMesh::setStartAndEndFrame(int start_id, int end_id)
{
	startId = start_id;
	endId = end_id;
}

void AxVSPRoomSeg3DMesh::setParameters(float door_width, float overlap_r, float min_seed_vol)
{
	D_2_DOOR = door_width / 2;
	DIST_EDT_THRESHOLD = D_2_DOOR;
	OVERLAP_RATIO = overlap_r;
	THRESHOLD_VOLUME = min_seed_vol;
}

void AxVSPRoomSeg3DMesh::setFilePath(std::string path)
{
	tPath =  path;
}

void AxVSPRoomSeg3DMesh::setVoxelSize(float size_vox)
{
	VOX_SIZE = size_vox;
}

AxVSPRoomSeg3DMesh::~AxVSPRoomSeg3DMesh()
{
}
