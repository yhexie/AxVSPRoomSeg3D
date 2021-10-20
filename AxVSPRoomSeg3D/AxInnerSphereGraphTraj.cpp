#include "AxInnerSphereGraphTraj.h"

bool sortDistCellAscending(AxDistCell a, AxDistCell b)
{
	return (a.Distance > b.Distance);
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

AxInnerSphereGraphTraj::AxInnerSphereGraphTraj():
	VOX_SIZE(0.1), FILTER_VOX_SIZE(0.4),MAX_UPDATE_DIST(100),	HIT_THICKNESS(1),
	VERSION(1),SENSOR_RANGE(10000),
	L_FREE(-0.13),L_OCCU(+1.01),
	L_THRESH(0.0), L_MAX(+3.5), L_MIN(-2.0)
{

}

void AxInnerSphereGraphTraj::doInitialize()
{
	max_coor_dist_ = int(MAX_UPDATE_DIST / VOX_SIZE);
	max_coor_sqdist_ = max_coor_dist_ * max_coor_dist_;

	openvdb::initialize();
	grid_logocc_ = openvdb::FloatGrid::create(0.0);
	grid_dist_map = openvdb::FloatGrid::create(0.0);
	grid_Label = openvdb::Int32Grid::create(0.0);

	grid_distance_transform = std::make_shared<DynamicVDBEDT>(max_coor_dist_);
	grid_distance_transform->initialize(dist_map_edt, VOX_SIZE, VERSION);
	grid_distance_transform->setAccessor(dist_map_edt);

	const openvdb::math::Vec3d offset(VOX_SIZE / 2.0, VOX_SIZE / 2.0, VOX_SIZE / 2.0);
	openvdb::math::Transform::Ptr tf = openvdb::math::Transform::createLinearTransform(VOX_SIZE);
	tf->postTranslate(offset);
	grid_logocc_->setTransform(tf);
	grid_dist_map->setTransform(tf);
	grid_Label->setTransform(tf);

	FILTER_VOX_SIZE = D_2_DOOR - DIST_EDT_THRESHOLD;
}

int AxInnerSphereGraphTraj::doEDT()
{
	if (tPath.length() == 0)
	{
		PCL_ERROR("�ļ�·�������ڣ�\n");
		return 0;
	}
	//���ػ�������ϡ������------------------------------------------
	FloatGrid::Accessor grid_acc = grid_logocc_->getAccessor();
	FloatGrid::Accessor grid_dist_map_acc = grid_dist_map->getAccessor();

	//��ȡ�ļ�-------------------------------------------
	char a[10], b[10];
	sprintf(a, "%d", 1);
	std::string filename = tPath + a + ".txt";
	std::cout << "��ȡ�ļ���" << filename << std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	FILE* inTraj = fopen(filename.c_str(), "r");
	if (inTraj == NULL)
	{
		printf("missing file");
		return 0;
	}
	double timep = 0;
	double ptimep = 0;
	float x, y, z;
	float px;	float py;	float pz;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pos_xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	int idj = 0;
	fscanf(inTraj, "%f %f %f %f %f %f %lf %lf", &x, &y, &z, &px, &py, &pz, &timep, &ptimep);
	while (feof(inTraj) == 0) /*�ж��Ƿ��ļ�β��������ѭ��*/
	{
		idj++;
		pcl::PointXYZ xyz;
		xyz.x = x;
		xyz.y = y;
		xyz.z = z;
		xyz_cloud->push_back(xyz);
		pcl::PointXYZ pose;
		pose.x = px;
		pose.y = py;
		pose.z = pz;
		pos_xyz_cloud->push_back(pose);
		fscanf(inTraj, "%f %f %f %f %f %f %lf %lf", &x, &y, &z, &px, &py, &pz, &timep, &ptimep);
	}
	fclose(inTraj);

	std::cout << "points: " << xyz_cloud->points.size() << std::endl;

	Eigen::Vector4f min_pt, max_pt;						// �زο�����ϵ������ı߽�ֵ
	pcl::PointXYZRGB minPt, maxPt;
	pcl::getMinMax3D(*xyz_cloud, min_pt, max_pt);

	//���о���任------------------------------------------
	Vec3d map_min(min_pt.x(), min_pt.y(), min_pt.z());
	Vec3d map_max(max_pt.x(), max_pt.y(), max_pt.z());
	auto coor_min_original = dist_map_edt->worldToIndex(map_min);
	auto coor_max_original = dist_map_edt->worldToIndex(map_max);

	coor_min[0] = (int)std::floor(coor_min_original.x()) - 2;
	coor_min[1] = (int)std::floor(coor_min_original.y()) - 2;
	coor_min[2] = (int)std::floor(coor_min_original.z()) - 2;
	coor_max[0] = (int)std::ceil(coor_max_original.x()) + 2;
	coor_max[1] = (int)std::ceil(coor_max_original.y()) + 2;
	coor_max[2] = (int)std::ceil(coor_max_original.z()) + 2;

	
	//for (auto point = xyz->begin(); point != xyz->end(); ++point)
	for (size_t i = 0; i < xyz_cloud->size(); i++)
		//for (size_t i = 0; i < 10000; i++)
	{
		auto sensor_position = pos_xyz_cloud->points[i];
		openvdb::Vec3d sesor_pos(sensor_position.x, sensor_position.y, sensor_position.z);
		openvdb::Vec3d origin_ijk = grid_logocc_->worldToIndex(sesor_pos);
		auto point = xyz_cloud->points[i];
		openvdb::Vec3d p_xyz(point.x, point.y, point.z);
		openvdb::Vec3d p_ijk = grid_logocc_->worldToIndex(p_xyz);
		openvdb::Vec3d dir(p_ijk - origin_ijk);
		double range = dir.length();
		dir.normalize();
		bool truncated = false;
		openvdb::math::Ray<double> ray(origin_ijk, dir);
		openvdb::math::DDA<openvdb::math::Ray<double>, 0> dda(ray, 0, std::min(SENSOR_RANGE, range));

		do {
			openvdb::Coord ijk(dda.voxel());

			float ll_old;
			bool isKnown = grid_acc.probeValue(ijk, ll_old);
			float ll_new = std::max(L_MIN, ll_old + L_FREE);

			if (!isKnown) {
				grid_distance_transform->dist_acc_->setValueOn(ijk);
			} // unknown -> free -> EDT initialize
			else if (ll_old >= 0 && ll_new < 0) {
				grid_distance_transform->removeObstacle(ijk);
			} // occupied -> free -> EDT RemoveObstacle

			grid_acc.setValueOn(ijk, ll_new);
			dda.step();

		} while (dda.time() < dda.maxTime());

		// increase occupancy
		if ((!truncated) && (range <= SENSOR_RANGE)) {
			for (int i = 0; i < HIT_THICKNESS; ++i) {
				openvdb::Coord ijk(dda.voxel());

				float ll_old;
				bool isKnown = grid_acc.probeValue(ijk, ll_old);
				float ll_new = std::min(L_MAX, ll_old + L_OCCU);

				if (!isKnown) {
					grid_distance_transform->dist_acc_->setValueOn(ijk);
					grid_distance_transform->setObstacle(ijk);
				} // unknown -> occupied -> EDT SetObstacle
				else if (ll_old < 0 && ll_new >= 0) {
					grid_distance_transform->setObstacle(ijk);
				} // free -> occupied -> EDT SetObstacle

				grid_acc.setValueOn(ijk, ll_new);
				dda.step();
			}
		}
	}

	grid_distance_transform->update();
	std::string vdbFilename, imgFilename, gridName;
	vdbFilename = "something.vdb";
	openvdb::io::File file(vdbFilename);
	openvdb::GridPtrVec grids;
	grids.push_back(grid_logocc_);
	file.write(grids);
	file.close();
	//������------------------------------------------
	pcl_edt_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
	for (int x = coor_min.x(); x <= coor_max.x(); ++x) {
		for (int y = coor_min.y(); y <= coor_max.y(); ++y) {
			for (int z = coor_min.z(); z <= coor_max.z(); ++z) {
				Coord target_cell(x, y, z);
				Coord nearest_obst;
				auto cell_dist = grid_distance_transform->query_sq_distance(target_cell, nearest_obst);

				if (cell_dist < 0 || cell_dist >= max_coor_sqdist_) { // ע��˴�Ϊ����ƽ��
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
	if (pcl_edt_cloud->points.size() > 0)
	{
		std::string filename3 = "saveEDTIntensity.pcd";
		if (pcl::io::savePCDFile<pcl::PointXYZI>(filename3, *pcl_edt_cloud) == -1) //�����ļ�
		{
			PCL_ERROR("EDT�ļ�����ʧ�ܣ�\n");
			return -1;
		}
	}
	else
	{
		PCL_ERROR("EDT��Ƭ����Ϊ0��\n");
		return -1;
	}
	return 0;
}

void AxInnerSphereGraphTraj::doFillInnerSphere()
{
	pcl_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);

	grid_dist_map_threshold = grid_dist_map->deepCopy();//�����ֵ�ָ�֮���ֵ
	FloatGrid::Accessor grid_dist_map_acc = grid_dist_map_threshold->getAccessor();
	//������ֵ�ָ�
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
	//�������
	double radius = 0;
	double filterVoxelSize = VOX_SIZE * 1.01;
	do
	{
		//�ҵ����ľ���
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
							if (!flag)//����㲻��Բ��
							{
								tickNum++;
							}
							else
							{			
								//PCL_INFO("����Բ��\n");
								break;
							}
						}
					}
					//�����κ�һ��Բ��
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
		printf("��Сֵ%f�����ֵ%f \n", 0, radius);
	}while (radius > filterVoxelSize);
}

void AxInnerSphereGraphTraj::doFillInnerSphereEx()
{
	pcl_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
	std::vector<AxDistCell> listDistCells;
	grid_dist_map_threshold = grid_dist_map->deepCopy();//�����ֵ�ָ�֮���ֵ
	FloatGrid::Accessor grid_dist_map_acc = grid_dist_map_threshold->getAccessor();
	int gNumX = coor_max.x() - coor_min.x() + 1;
	int gNumY = coor_max.y() - coor_min.y() + 1;
	int gNumZ = coor_max.z() - coor_min.z() + 1;
	//������ֵ�ָ�
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
	//printf("��Сֵ%f�����ֵ%f \n", 0, radius);
	std::vector<AxDistCell>::iterator iter = listDistCells.begin();
	listDistCells.erase(iter);
	//�Ӷ������Ƴ��Ѿ����򸲸ǵ�����
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
				if (flag)//�������Բ��
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
	//�������
	double filterVoxelSize = VOX_SIZE * 1.01;
	while (radius > filterVoxelSize)
	{
		//�ҵ����ľ���
		Concurrency::parallel_sort(listDistCells.begin(), listDistCells.end(), sortDistCellAscending);
		AxDistCell maxDist_cell = listDistCells[0];
		max_cell = maxDist_cell.target_cell;
		radius = maxDist_cell.Distance;
		size_t t_id = maxDist_cell.ID;
		auto world_xyz_tmp = grid_dist_map_threshold->indexToWorld(max_cell);
		double centerX_current = world_xyz_tmp.x();
		double centerY_current = world_xyz_tmp.y();
		double centerZ_current = world_xyz_tmp.z();
		//�ص���̫���Ƴ�
		int num_sub = pcl_cloud->size();// ->points;
		if (num_sub > 0)
		{
			int idx_count = 0;
			for (int i = 0; i < num_sub; i++)
			{
				pcl::PointXYZI pt = pcl_cloud->at(i);
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
			if (idx_count == num_sub)//�����еĶ����ص�
			{
				pcl::PointXYZI centerPt0;
				centerPt0.x = centerX_current;
				centerPt0.y = centerY_current;
				centerPt0.z = centerZ_current;
				centerPt0.intensity = radius;
				pcl_cloud->push_back(centerPt0);
				//printf("��Сֵ%f�����ֵ%f \n", 0, radius);
				std::vector<AxDistCell>::iterator iter = listDistCells.begin();
				listDistCells.erase(iter);
				//�Ӷ������Ƴ��Ѿ����򸲸ǵ�����
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
							if (flag)//�������Բ��
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
			else//�����ص�
			{
				std::vector<AxDistCell>::iterator iter = listDistCells.begin();
				listDistCells.erase(iter);
			}
		}
	}
}

//�����䣬�ӿ��ٶ�
void AxInnerSphereGraphTraj::doFillInnerSphereRandom()
{
	pcl_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
	std::vector<AxDistCell> listDistCells;
	grid_dist_map_threshold = grid_dist_map->deepCopy();//�����ֵ�ָ�֮���ֵ
	FloatGrid::Accessor grid_dist_map_acc = grid_dist_map_threshold->getAccessor();
	int gNumX = coor_max.x() - coor_min.x() + 1;
	int gNumY = coor_max.y() - coor_min.y() + 1;
	int gNumZ = coor_max.z() - coor_min.z() + 1;
	//������ֵ�ָ�
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

	//��һ���裺������
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
	std::random_shuffle(nCard.begin(), nCard.begin() + numMax);//�������n����
	tag_max_Radius = 0;
	do
	{
		selectedPointIndex = nCard[iter_radnum];
		if (selectedPointIndex > 0 && selectedPointIndex < numMax)
		{
			AxDistCell randomDistCell = listDistCells[selectedPointIndex];//���ѡ��һ����
			openvdb::Coord random_cell = randomDistCell.target_cell;
			double radius = randomDistCell.Distance;
			if (radius < VOX_SIZE * 1.01)//D_2_DOOR)
			{
				continue;
			}
			auto world_xyz = grid_dist_map_threshold->indexToWorld(random_cell);
			double centerX_current = world_xyz.x();
			double centerY_current = world_xyz.y();
			double centerZ_current = world_xyz.z();
			//�ص���̫���Ƴ�
			int num_sub = pcl_cloud->size();// ->points;
			if (num_sub > 0)
			{
				int idx_count = 0;
				for (int i = 0; i < num_sub; i++)
				{
					pcl::PointXYZI pt = pcl_cloud->at(i);
					float centerX_tmp = pt.x;
					float centerY_tmp = pt.y;
					float centerZ_tmp = pt.z;
					float radius_tmp = pt.intensity;
					if (tag_max_Radius < 2.0 * radius)
					{
						tag_max_Radius = 2.0 * radius;
					}		
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
				if (idx_count == num_sub)//�����еĶ����ص�
				{
					pcl::PointXYZI centerPt;
					centerPt.x = centerX_current;
					centerPt.y = centerY_current;
					centerPt.z = centerZ_current;
					centerPt.intensity = radius;
					pcl_cloud->push_back(centerPt);
					printf("��Сֵ%f�����ֵ%f \n", 0, radius);
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
				printf("��Сֵ%f�����ֵ%f \n", 0, radius);
			}
		}
	} while (iter_radnum++ < numSlectedPts);

	//�Ӷ������Ƴ��Ѿ����򸲸ǵ�����
	listDistCells.clear();
	for (int idx = coor_min.x(); idx <= coor_max.x(); ++idx) {
		for (int idy = coor_min.y(); idy <= coor_max.y(); ++idy) {
			for (int idz = coor_min.z(); idz <= coor_max.z(); ++idz) {
				openvdb::Coord target_cell(idx, idy, idz);
				size_t tmp_CELL_ID = idz*gNumX*gNumY + idx*gNumY + idy;
				float ll_old;
				bool isKnown = grid_dist_map_acc.probeValue(target_cell, ll_old);
				if (isKnown && ll_old > 0)
				{
					int idx_count = 0;
					for (size_t i = 0; i < pcl_cloud->size(); i++)
					{
						pcl::PointXYZI pt = pcl_cloud->at(i);
						float centerX_tmp = pt.x;
						float centerY_tmp = pt.y;
						float centerZ_tmp = pt.z;
						float radius_tmp = pt.intensity;
						auto world_b_cell = grid_dist_map_threshold->indexToWorld(target_cell);
						double centerX_box = world_b_cell.x();
						double centerY_box = world_b_cell.y();
						double centerZ_box = world_b_cell.z();
						int flag = inSphere(radius_tmp, centerX_tmp, centerY_tmp, centerZ_tmp, centerX_box, centerY_box, centerZ_box);
						if (!flag)//�������Բ��
						{
							idx_count++;
						}
						else
						{
							break;
						}
					}
					if (idx_count == pcl_cloud->size())//�����еĶ����ص�
					{
						AxDistCell cell;
						cell.ID = idz*gNumX*gNumY + idx*gNumY + idy;
						cell.target_cell = target_cell;
						cell.Distance = ll_old;
						listDistCells.push_back(cell);
					}
				}
				//for (size_t i = 0; i < pcl_cloud->size(); i++)
				//{
				//	pcl::PointXYZI pt = pcl_cloud->at(i);
				//	float centerX_tmp = pt.x;
				//	float centerY_tmp = pt.y;
				//	float centerZ_tmp = pt.z;
				//	float radius_tmp = pt.intensity;			
				//	float ll_old;
				//	bool isKnown = grid_dist_map_acc.probeValue(target_cell, ll_old);
				//	if (!isKnown && ll_old <= 0)
				//	{
				//		continue;
				//	}
				//	auto world_b_cell = grid_dist_map_threshold->indexToWorld(target_cell);
				//	double centerX_box = world_b_cell.x();
				//	double centerY_box = world_b_cell.y();
				//	double centerZ_box = world_b_cell.z();
				//	int flag = inSphere(radius_tmp, centerX_tmp, centerY_tmp, centerZ_tmp, centerX_box, centerY_box, centerZ_box);
				//	if (flag)//�������Բ��
				//	{
				//		std::vector<AxDistCell>::iterator iter0;
				//		for (iter0 = listDistCells.begin(); iter0 != listDistCells.end(); ++iter0)
				//		{
				//			AxDistCell dist_cell = *iter0;
				//			size_t id = dist_cell.ID;
				//			if (tmp_CELL_ID == id)
				//			{
				//				listDistCells.erase(iter0);
				//			}
				//		}
				//	}
				//}
			}
		}
	}

	//�ڶ����裺�������
	Concurrency::parallel_sort(listDistCells.begin(), listDistCells.end(), sortDistCellAscending);
	AxDistCell maxDist_cell = listDistCells[0];
	openvdb::Coord max_cell = maxDist_cell.target_cell;
	double radius = maxDist_cell.Distance;
	if (tag_max_Radius < 2.0 * radius)
	{
		tag_max_Radius = 2.0 * radius;
	}
	auto world_xyz = grid_dist_map_threshold->indexToWorld(max_cell);
	double centerX = world_xyz.x();
	double centerY = world_xyz.y();
	double centerZ = world_xyz.z();
	//�ص���̫���Ƴ�
	int num_sub = pcl_cloud->size();// ->points;
	if (num_sub > 0)
	{
		int idx_count = 0;
		for (int i = 0; i < num_sub; i++)
		{
			pcl::PointXYZI pt = pcl_cloud->at(i);
			float centerX_tmp = pt.x;
			float centerY_tmp = pt.y;
			float centerZ_tmp = pt.z;
			float radius_tmp = pt.intensity;
			int flag2 = OverlapOrDepartTwoSphere(OVERLAP_RATIO, radius_tmp, centerX_tmp, centerY_tmp, centerZ_tmp, radius,
				centerX, centerY, centerZ);
			if (flag2)
			{
				idx_count++;
			}
			else
			{
				break;
			}
		}
		if (idx_count == num_sub)//�����еĶ����ص�
		{
			pcl::PointXYZI centerPt;
			centerPt.x = centerX;
			centerPt.y = centerY;
			centerPt.z = centerZ;
			centerPt.intensity = radius;
			pcl_cloud->push_back(centerPt);
			printf("��Сֵ%f�����ֵ%f \n", 0, radius);
			std::vector<AxDistCell>::iterator iter02 = listDistCells.begin();
			listDistCells.erase(iter02);
			//�Ӷ������Ƴ��Ѿ����򸲸ǵ�����
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
						if (flag)//�������Բ��
						{
							std::vector<AxDistCell>::iterator iter;
							for (iter = listDistCells.begin(); iter != listDistCells.end(); ++iter)
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
		}
	}
	//�������
	double filterVoxelSize = VOX_SIZE * 1.01;
	while (radius > filterVoxelSize)
	{
		//�ҵ����ľ���
		Concurrency::parallel_sort(listDistCells.begin(), listDistCells.end(), sortDistCellAscending);

		AxDistCell maxDist_cell = listDistCells[0];
		max_cell = maxDist_cell.target_cell;
		radius = maxDist_cell.Distance;
		size_t t_id = maxDist_cell.ID;
		auto world_xyz_tmp = grid_dist_map_threshold->indexToWorld(max_cell);
		double centerX_current = world_xyz_tmp.x();
		double centerY_current = world_xyz_tmp.y();
		double centerZ_current = world_xyz_tmp.z();
		//�ص���̫���Ƴ�
		int num_sub = pcl_cloud->size();// ->points;
		if (num_sub > 0)
		{
			int idx_count = 0;
			for (int i = 0; i < num_sub; i++)
			{
				pcl::PointXYZI pt = pcl_cloud->at(i);
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
			if (idx_count == num_sub)//�����еĶ����ص�
			{
				pcl::PointXYZI centerPt0;
				centerPt0.x = centerX_current;
				centerPt0.y = centerY_current;
				centerPt0.z = centerZ_current;
				centerPt0.intensity = radius;
				pcl_cloud->push_back(centerPt0);
				printf("��Сֵ%f�����ֵ%f \n", 0, radius);
				std::vector<AxDistCell>::iterator iter = listDistCells.begin();
				listDistCells.erase(iter);
				//�Ӷ������Ƴ��Ѿ����򸲸ǵ�����
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
							int flag = inSphere(radius, centerX_current, centerX_current, centerX_current, centerX_box, centerY_box, centerZ_box);
							if (flag)//�������Բ��
							{
								std::vector<AxDistCell>::iterator iter;
								for (iter = listDistCells.begin(); iter != listDistCells.end(); ++iter)
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
			else//�����ص�
			{
				std::vector<AxDistCell>::iterator iter = listDistCells.begin();
				listDistCells.erase(iter);
			}
		}
	} 
}

void AxInnerSphereGraphTraj::doBuildGraph()//����ͼ
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

		if (tree->radiusSearch(pt, tag_max_Radius, nn_indices, nn_dists) == 0)//�д��󣬳����޷�����
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
			if (itick == nn_indices.size())//�����Ľڵ�
			{
				AxEdge ee(i, nn_indices[1]);
				list_edges_addition.push_back(ee);
			}
		}
	}
	//����Graph
	g.reset(new Graph(numOfVertex));
	g_one.reset(new Graph(numOfVertex));	//������ͼGraph
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
	for (int j = 0; j < list_edges_addition.size(); j++)//���й����Ľڵ㶼����
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

void AxInnerSphereGraphTraj::doConnectComponent()
{
	std::vector< int > component(boost::num_vertices(*g));
	int num = boost::connected_components(*g, &component[0]);//Total number of components

	std::vector<int>::size_type i_comp;
	FloatGrid::Accessor grid_dist_map_acc = grid_dist_map->getAccessor();//δ������ֵ�ָ�ľ���任ֵ
	//FloatGrid::Accessor grid_dist_map_threshold_acc = grid_dist_map_threshold->getAccessor();
	Int32Grid::Accessor label_acc = grid_Label->getAccessor();
	for (int x = coor_min.x(); x <= coor_max.x(); ++x) {
		for (int y = coor_min.y(); y <= coor_max.y(); ++y) {
			for (int z = coor_min.z(); z <= coor_max.z(); ++z) {
				openvdb::Coord target_cell(x, y, z);
				float ll_value;
				bool isKnown = grid_dist_map_acc.probeValue(target_cell, ll_value);
				if (isKnown && ll_value > 0)//����0���Ѿ�������ǽ��
				//if (isKnown && ll_value > VOX_SIZE)
				{
					label_acc.setValueOn(target_cell, 0);//���Ϊ0
				}
			}
		}
	}
	std::map<int, float> m_volumn_seed;
	for (i_comp = 0; i_comp != component.size(); ++i_comp)
	{
		int RoomIdx = component[i_comp] + 1;//Label��1��ʼ��i_comp�ǽ�������
		pcl::PointXYZI pt = pcl_cloud->at(i_comp);
		float centerX = pt.x;
		float centerY = pt.y;
		float centerZ = pt.z;
		float radius = pt.intensity;
		m_volumn_seed[RoomIdx] += 4.0 * M_PI*radius*radius*radius / 3;
	}

	for (i_comp = 0; i_comp != component.size(); ++i_comp)
	{
		int RoomIdx = component[i_comp] + 1;//Label��1��ʼ��i_comp�ǽ�������
		float curSeedVol = m_volumn_seed[RoomIdx];
		if (curSeedVol < THRESHOLD_VOLUME)//���С�ڸ���ֵ���޳���
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
		openvdb::Coord minSphereCoord;
		minSphereCoord.setX(p_ijk.x() - std::floor(radius / VOX_SIZE));
		minSphereCoord.setY(p_ijk.y() - std::floor(radius / VOX_SIZE));
		minSphereCoord.setZ(p_ijk.z() - std::floor(radius / VOX_SIZE));
		openvdb::Coord maxSphereCoord;
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
					openvdb::Coord target_cell(x, y, z);
					float ll_value;
					bool isKnown = grid_dist_map_acc.probeValue(target_cell, ll_value);
					if (isKnown)
					{
						auto world_t_cell = grid_dist_map->indexToWorld(target_cell);
						//�������Բ��
						int flag = inSphere(radius, centerX, centerY, centerZ, world_t_cell.x(), world_t_cell.y(), world_t_cell.z());
						if (flag)//��������
						{
							label_acc.setValueOn(target_cell, RoomIdx);//��ǩ
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

void AxInnerSphereGraphTraj::doWavefrontGrowing()
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
					openvdb::Coord target_cell(x, y, z);
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
									openvdb::Coord spread_cell(h1, h2, h3);
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

void AxInnerSphereGraphTraj::Run()
{
	std::string elefilename = tPath + "timecost.txt";
	FILE *costfile = fopen(elefilename.c_str(), "w");
	clock_t start, finish;
	double  duration;
	/* ����һ���¼�������ʱ��*/
	start = clock();
	doInitialize();
	finish = clock();
	duration = (double)(finish - start) / CLOCKS_PER_SEC;
	fprintf(costfile, "��ʼ����%f seconds\n", duration);

	printf("1.����任...\n");
	start = clock();
	doEDT();
	finish = clock();
	double duration1 = (double)(finish - start) / CLOCKS_PER_SEC;
	int ux = coor_max.x() - coor_min.x();
	int vy = coor_max.y() - coor_min.y();
	int wz = coor_max.z() - coor_min.z();
	fprintf(costfile, "Grid����ߴ磺%d %d %d;\n", ux, vy, wz);
	fprintf(costfile, "1.����任��%f seconds\n", duration1);

	printf("2.�ڲ������...\n");
	start = clock();
	//doFillInnerSphere();
	//doFillInnerSphereEx();
	doFillInnerSphereRandom();
	finish = clock();
	doSaveSpherePlyFile();
	double duration2 = (double)(finish - start) / CLOCKS_PER_SEC;
	fprintf(costfile, "2.�ڲ�����䣺%f seconds\n", duration2);

	printf("3.��ֵ�ָ�����ʼ���ڽ�ͼ...\n");
	start = clock();
	doBuildGraph();
	finish = clock();
	saveInitialEdges();
	double duration3 = (double)(finish - start) / CLOCKS_PER_SEC;
	fprintf(costfile, "3.��ֵ�ָ�����ʼ���ڽ�ͼ��%f seconds\n", duration3);

	printf("4.��ͨ������������������...\n");
	start = clock();
	doConnectComponent();
	finish = clock();
	double duration4 = (double)(finish - start) / CLOCKS_PER_SEC;
	fprintf(costfile, "4.��ͨ�����������������ɣ�%f seconds\n", duration4);

	printf("5.��ǰ�㷨...\n");
	start = clock();
	doWavefrontGrowing();
	finish = clock();
	double duration5 = (double)(finish - start) / CLOCKS_PER_SEC;
	fprintf(costfile, "5.��ǰ�㷨��%f seconds\n", duration5);

	double total = duration + duration1 + duration2 + duration3 + duration4 + duration5;
	fprintf(costfile, "��ʱ�䣺%f seconds\n", total);
	fclose(costfile);

	printf("6.�������մ���ǩ��VDB...\n");
	doSaveFinnalVDB();

	printf("���\n");
}

void AxInnerSphereGraphTraj::doSaveSpherePlyFile()
{
	int idx = 0; int count = 0;
	std::vector<std::tuple<double, double, double>> vertexs;
	std::vector<std::tuple<int, int, int>> indexs;
	//����һ����
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

void AxInnerSphereGraphTraj::saveInitialEdges()
{
	FloatGrid::Accessor grid_dist_map_acc = grid_dist_map->getAccessor();
	char* pSaveRoomFile = "Edges.shp";
	int nShpTpyeSave = SHPT_ARCZ;//ע�Ᵽ�����
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
		//��Ȩ��
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

int AxInnerSphereGraphTraj::doSaveFinnalVDB()
{
	std::string vdbFilename, imgFilename, gridName;
	vdbFilename = "edt_label.vdb";
	openvdb::io::File file(vdbFilename);
	openvdb::GridPtrVec grids;
	grids.push_back(grid_Label);
	file.write(grids);
	file.close();
	//������------------------------------------------
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
			PCL_ERROR("����ָ���Label�ļ�����ʧ�ܣ�\n");
			return -1;
		}
	}
	else
	{
		PCL_ERROR("����ָ���Label����Ϊ0��\n");
		return -1;
	}
	return 0;
}

void AxInnerSphereGraphTraj::evalMinMax(EDTGrid::Ptr dist_, dataCell& minVal, dataCell& maxVal)
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
	std::cout << "datacell��Ŀ��" << count_id << std::endl;
}

void AxInnerSphereGraphTraj::setParameters(float door_width, float overlap_r, float min_seed_vol)
{
	D_2_DOOR = door_width / 2;
	DIST_EDT_THRESHOLD = D_2_DOOR;
	OVERLAP_RATIO = overlap_r;
	THRESHOLD_VOLUME = min_seed_vol;
}

void AxInnerSphereGraphTraj::setFilePath(std::string path)
{
	tPath =  path;
}

void AxInnerSphereGraphTraj::setVoxelSize(float size_vox)
{
	VOX_SIZE = size_vox;
}

AxInnerSphereGraphTraj::~AxInnerSphereGraphTraj()
{
}
