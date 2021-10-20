#include "AxRoom3d.h"


AxRoom3d::AxRoom3d() :mVolume(0)
{
}


AxRoom3d::~AxRoom3d()
{
}

//降序
bool sortRoomsConnectDscending(std::pair<size_t, size_t> a, std::pair<size_t, size_t> b)
{
	return (a.second > b.second);
}

bool sortRoomsAreaDscending(std::pair<size_t, float> a, std::pair<size_t, float> b)
{
	return (a.second > b.second);
}

void AxRoom3d::insertSphereMembers(size_t cir)
{
	mInnerSphereMembers.push_back(cir);
}

std::vector<size_t>& AxRoom3d::getMembers()
{
	return mInnerSphereMembers;
}

int AxRoom3d::addNeighborID(size_t new_neighbor_id)
{
	if (!contains(neighbor_room_ids_, new_neighbor_id))
	{
		neighbor_room_ids_.push_back(new_neighbor_id);
		neighbor_room_statistics_[new_neighbor_id] = 1;
		return 0;
	}
	else
	{
		neighbor_room_statistics_[new_neighbor_id] = neighbor_room_statistics_[new_neighbor_id] + 1;
	}
	return 1;
}

std::vector<size_t>& AxRoom3d::getNeighborIDs()
{
	return neighbor_room_ids_;
}

std::map<size_t, size_t>& AxRoom3d::getNeighborStatistics()
{
	return neighbor_room_statistics_;
}

bool AxRoom3d::contains(std::vector<size_t> vector, int element)
{
	//this functions checks, if the given element is in the given vector (in this case for int elements)
	if (!vector.empty())
	{
		return vector.end() != std::find(vector.begin(), vector.end(), element);
	}
	else
	{
		return false;
	}
}

float AxRoom3d::getVolume()
{
	return mVolume;
}

void AxRoom3d::AddSphereVolume(float sphere_vol_)
{
	mVolume += sphere_vol_;
}

bool AxRoom3d::getMaxConnectedRoom(std::vector<AxRoom3d> rooms_,size_t& roomId_)
{
	if (neighbor_room_statistics_.size()<1)
	{
		return false;
	}
	//遍历map
	std::vector<std::pair<size_t, size_t>> listRoomConnect;
	std::map<size_t, size_t>::iterator it;
	for (it = neighbor_room_statistics_.begin(); it != neighbor_room_statistics_.end(); ++it)
	{
		std::pair<size_t, size_t> tmp(it->first, it->second);
		listRoomConnect.push_back(tmp);
	}
	std::sort(listRoomConnect.begin(), listRoomConnect.end(), sortRoomsConnectDscending);
	std::pair<size_t, size_t> maxConnect = listRoomConnect[0];
	roomId_ = maxConnect.first;
	if (maxConnect.second >= T_CONNECT_NUM)//连通度大于等于3
	{
		bool found_id = false;
		size_t room_index = 0;
		for (size_t r = 0; r < rooms_.size(); r++)
		{
			if (rooms_[r].ID == roomId_)
			{
				room_index = r;
				found_id = true;
				break;
			}
		}
		if (found_id)
		{
			AxRoom3d tmpR = rooms_[room_index];
			float volume_ = tmpR.getVolume();
			if (volume_ < T_MAX_VOLUME)
			{
				return true;
			}
		}
		return false;
	}
	return false;
}

bool AxRoom3d::getMaxVolumeConnectedRoom(std::vector<AxRoom3d> rooms_, size_t& roomId_)
{
	if (neighbor_room_ids_.size() < 1)
	{
		return false;
	}
	//遍历map
	std::vector<std::pair<size_t, float>> listRoomConnect;
	std::vector<size_t>::iterator it;
	for (it = neighbor_room_ids_.begin(); it != neighbor_room_ids_.end(); ++it)
	{
		size_t room_id = *it;
		bool found_id = false;
		size_t room_index = 0;
		for (size_t r = 0; r < rooms_.size(); r++)
		{
			if (rooms_[r].ID == room_id)
			{
				room_index = r;
				found_id = true;
				break;
			}
		}
		if (found_id)
		{
			AxRoom3d tmpR = rooms_[room_index];
			std::pair<size_t, float> tmp(room_id, tmpR.getVolume());
			listRoomConnect.push_back(tmp);
		}	
	}
	std::sort(listRoomConnect.begin(), listRoomConnect.end(), sortRoomsAreaDscending);
	std::pair<size_t, float> maxConnect = listRoomConnect[0];
	roomId_ = maxConnect.first;
	if (maxConnect.second <= T_MAX_VOLUME)//面积小于等于3
	{
		return true;
	}
	return false;
}

void AxRoom3d::mergeRoom(AxRoom3d& room_to_merge)
{
	//把另外的房间中的节点都添加到当前Room
	std::vector<size_t>& memberSphere_ids = room_to_merge.getMembers();
	for (int i = 0; i < memberSphere_ids.size(); i++)
	{
		this->mInnerSphereMembers.push_back(memberSphere_ids[i]);
	}
	//更新体积
	mVolume += room_to_merge.getVolume();
	//另外房间的近邻房间也添加到当前房间
	std::vector<size_t>& neighbor_ids = room_to_merge.getNeighborIDs();
	for (size_t i = 0; i < neighbor_ids.size(); ++i)
	{
		if (!contains(neighbor_room_ids_, neighbor_ids[i]) && neighbor_ids[i] != ID)
			neighbor_room_ids_.push_back(neighbor_ids[i]);
	}
	//删除自身A和另外房间B两个值的近邻，合并之后A的近邻B不存在，B的近邻A也同样不存在了
	for (std::vector<size_t>::iterator it = neighbor_room_ids_.begin(); it != neighbor_room_ids_.end();)
	{
		if (*it == ID || *it == room_to_merge.ID)
			it = neighbor_room_ids_.erase(it);
		else
			++it;
	}
	//更新近邻数目
	for (std::map<size_t, size_t>::const_iterator it = room_to_merge.getNeighborStatistics().begin(); it != room_to_merge.getNeighborStatistics().end(); ++it)
	{
		if (it->first != ID)
		{
			if (neighbor_room_statistics_.find(it->first) != neighbor_room_statistics_.end())
				neighbor_room_statistics_[it->first] += it->second;//共同近邻房间？？
			else
				neighbor_room_statistics_[it->first] = it->second;
		}
	}
	neighbor_room_statistics_.erase(room_to_merge.ID);
}


