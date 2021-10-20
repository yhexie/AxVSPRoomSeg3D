#pragma once
#ifndef AXROOM3D_HEADER
#define AXROOM3D_HEADER

#include <vector>
#include <map>
#include <algorithm>


class AxRoom3d
{
public:
	AxRoom3d();
	~AxRoom3d();
	//房间包含的填充球
	void insertSphereMembers(size_t cir);
	//填充球体积之和
	void AddSphereVolume(float sphere_vol_);
	//获取填充圆的索引
	std::vector<size_t>& getMembers();
	//获取当前房间的体积
	float getVolume();
	//当前房间的近邻房间ID
	int addNeighborID(size_t new_neighbor_id);

	std::map<size_t, size_t>& getNeighborStatistics();

	std::vector<size_t>& getNeighborIDs();
	//获取与当前房间连通度最大的房间
	bool getMaxConnectedRoom(std::vector<AxRoom3d> rooms_, size_t& roomId_);
	 
	//获取与当前房间连通且体积最大的房间
	bool getMaxVolumeConnectedRoom(std::vector<AxRoom3d> rooms_, size_t& roomIdx);

	void mergeRoom(AxRoom3d& room_to_merge);

	
	bool contains(std::vector<size_t> vector, int element);	
public:
	size_t ID;
	float mVolume;
	std::vector<size_t> mInnerSphereMembers;
	std::vector<size_t> neighbor_room_ids_;//近邻房间索引
	std::map<size_t,size_t> neighbor_room_statistics_;//近邻房间中节点数目

	float T_MAX_VOLUME = 12.0;
	int T_CONNECT_NUM = 10;
};

#endif