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
	//��������������
	void insertSphereMembers(size_t cir);
	//��������֮��
	void AddSphereVolume(float sphere_vol_);
	//��ȡ���Բ������
	std::vector<size_t>& getMembers();
	//��ȡ��ǰ��������
	float getVolume();
	//��ǰ����Ľ��ڷ���ID
	int addNeighborID(size_t new_neighbor_id);

	std::map<size_t, size_t>& getNeighborStatistics();

	std::vector<size_t>& getNeighborIDs();
	//��ȡ�뵱ǰ������ͨ�����ķ���
	bool getMaxConnectedRoom(std::vector<AxRoom3d> rooms_, size_t& roomId_);
	 
	//��ȡ�뵱ǰ������ͨ��������ķ���
	bool getMaxVolumeConnectedRoom(std::vector<AxRoom3d> rooms_, size_t& roomIdx);

	void mergeRoom(AxRoom3d& room_to_merge);

	
	bool contains(std::vector<size_t> vector, int element);	
public:
	size_t ID;
	float mVolume;
	std::vector<size_t> mInnerSphereMembers;
	std::vector<size_t> neighbor_room_ids_;//���ڷ�������
	std::map<size_t,size_t> neighbor_room_statistics_;//���ڷ����нڵ���Ŀ

	float T_MAX_VOLUME = 12.0;
	int T_CONNECT_NUM = 10;
};

#endif