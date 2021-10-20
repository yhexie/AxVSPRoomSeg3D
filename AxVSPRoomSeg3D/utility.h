#ifndef _UTILITY_H_
#define _UTILITY_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_traits.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>

inline pcl::RGB rainbow_color_map(double h)
{
	pcl::RGB color;
	color.a = 1;
	// blend over HSV-values (more colors)

	double s = 1.0;
	double v = 1.0;

	h -= floor(h);
	h *= 6;
	int i;
	double m, n, f;

	i = floor(h);
	f = h - i;
	if (!(i & 1))
		f = 1 - f;  // if i is even
	m = v * (1 - s);
	n = v * (1 - s * f);

	switch (i) {
	case 6:
	case 0:color.r = v;
		color.g = n;
		color.b = m;
		break;
	case 1:color.r = n;
		color.g = v;
		color.b = m;
		break;
	case 2:color.r = m;
		color.g = v;
		color.b = n;
		break;
	case 3:color.r = m;
		color.g = n;
		color.b = v;
		break;
	case 4:color.r = n;
		color.g = m;
		color.b = v;
		break;
	case 5:color.r = v;
		color.g = m;
		color.b = n;
		break;
	default:color.r = 1;
		color.g = 0.5;
		color.b = 0.5;
		break;
	}
	return color;
}
inline void drawSphere(std::tuple<double, double, double> pos, double r,std::vector<std::tuple<double, double, double>>& sphereVertices,
	std::vector<std::tuple<int, int, int>>& sphereIndices)
{
	const int Y_SEGMENTS = 50;
	const int X_SEGMENTS = 50;
	double xbase = std::get<0>(pos);
	double ybase = std::get<1>(pos);
	double zbase = std::get<2>(pos);
	/*2-计算球体顶点*/
	//生成球的顶点
	for (int y = 0; y <= Y_SEGMENTS; y++)
	{
		for (int x = 0; x <= X_SEGMENTS; x++)
		{
			float xSegment = (float)x / (float)X_SEGMENTS;
			float ySegment = (float)y / (float)Y_SEGMENTS;
			float xPos = xbase + r* std::cos(xSegment * 2.0f * M_PI) * std::sin(ySegment * M_PI);
			float yPos = ybase + r* std::cos(ySegment * M_PI);
			float zPos = zbase + r* std::sin(xSegment * 2.0f * M_PI) * std::sin(ySegment * M_PI);
			std::tuple<double, double, double> pt(xPos, yPos, zPos);
			sphereVertices.push_back(pt);
		}
	}

	//生成球的Indices
	for (int i = 0; i<Y_SEGMENTS; i++)
	{
		for (int j = 0; j<X_SEGMENTS; j++)
		{
			int idx1 = i * (X_SEGMENTS + 1) + j;
			int idx2 = (i + 1) * (X_SEGMENTS + 1) + j;
			int idx3 = (i + 1) * (X_SEGMENTS + 1) + j + 1;
			std::tuple<int, int, int> idx00(idx1, idx2, idx3);
			sphereIndices.push_back(idx00);
			int idx1_ = i* (X_SEGMENTS + 1) + j;
			int idx2_ = (i + 1) * (X_SEGMENTS + 1) + j + 1;
			int idx3_ = i * (X_SEGMENTS + 1) + j + 1;
			std::tuple<int, int, int> idx01(idx1_, idx2_, idx3_);
			sphereIndices.push_back(idx01);
		}
	}
}

inline void drawSphere(std::tuple<double, double, double> pos, double r,int idx_start, std::vector<std::tuple<double, double, double>>& sphereVertices,
	std::vector<std::tuple<int, int, int>>& sphereIndices)
{
	const int Y_SEGMENTS = 10;
	const int X_SEGMENTS = 10;
	double xbase = std::get<0>(pos);
	double ybase = std::get<1>(pos);
	double zbase = std::get<2>(pos);
	/*2-计算球体顶点*/
	//生成球的顶点
	for (int y = 0; y <= Y_SEGMENTS; y++)
	{
		for (int x = 0; x <= X_SEGMENTS; x++)
		{
			float xSegment = (float)x / (float)X_SEGMENTS;
			float ySegment = (float)y / (float)Y_SEGMENTS;
			float xPos = xbase + r* std::cos(xSegment * 2.0f * M_PI) * std::sin(ySegment * M_PI);
			float yPos = ybase + r* std::cos(ySegment * M_PI);
			float zPos = zbase + r* std::sin(xSegment * 2.0f * M_PI) * std::sin(ySegment * M_PI);
			std::tuple<double, double, double> pt(xPos, yPos, zPos);
			sphereVertices.push_back(pt);
		}
	}

	//生成球的Indices
	for (int i = 0; i<Y_SEGMENTS; i++)
	{
		for (int j = 0; j<X_SEGMENTS; j++)
		{
			int idx1 = idx_start + i * (X_SEGMENTS + 1) + j;
			int idx2 = idx_start + (i + 1) * (X_SEGMENTS + 1) + j;
			int idx3 = idx_start + (i + 1) * (X_SEGMENTS + 1) + j + 1;
			std::tuple<int, int, int> idx00(idx1, idx2, idx3);
			sphereIndices.push_back(idx00);
			int idx1_ = idx_start + i* (X_SEGMENTS + 1) + j;
			int idx2_ = idx_start + (i + 1) * (X_SEGMENTS + 1) + j + 1;
			int idx3_ = idx_start + i * (X_SEGMENTS + 1) + j + 1;
			std::tuple<int, int, int> idx01(idx1_, idx2_, idx3_);
			sphereIndices.push_back(idx01);
		}
	}
}

inline int inCircle(double radius, double centerX, double centerY, double x, double y)
{
	double dx = x - centerX;
	double dy = y - centerY;
	double d = std::sqrt(dx*dx + dy*dy);
	if (d<radius)
	{
		return true;
	}
	return false;
}

inline int inSphere(double radius, double centerX, double centerY, double centerZ, double x, double y, double z)
{
	double dx = x - centerX;
	double dy = y - centerY;
	double dz = z - centerZ;
	double d = std::sqrt(dx*dx + dy*dy+dz*dz);
	if (d <= radius)
	{
		return true;
	}
	return false;
}

inline int DistTwoSphere(double ratio, double r1, double centerX, double centerY, double centerZ, double r2, double x, double y, double z)
{
	double dx = x - centerX;
	double dy = y - centerY;
	double dz = z - centerZ;
	double d = std::sqrt(dx*dx + dy*dy + dz*dz);
	double overlap = (r1 + r2 - d);
	if (overlap <= ratio)
	{
		return true;
	}
	return false;
}


inline int OverlapOrDepartTwoSphere(double ratio, double r1, double centerX, double centerY, double centerZ, double r2, double x, double y, double z)
{
	double dx = x - centerX;
	double dy = y - centerY;
	double dz = z - centerZ;
	double d = std::sqrt(dx*dx + dy*dy + dz*dz);
	double overlap = (r1 + r2 - d) / std::min(r1, r2);
	if (overlap < ratio)
	{
		return true;
	}
	return false;
}

inline int OverlapOrDepartTwoSphere(double r1, double centerX, double centerY, double centerZ, double r2, double x, double y, double z)
{
	double dx = x - centerX;
	double dy = y - centerY;
	double dz = z - centerZ;
	double d = std::sqrt(dx*dx + dy*dy + dz*dz);
	double overlap = (r1 + r2 - d) / std::min(r1, r2);
	if (overlap < 0.5)
	{
		return true;
	}
	return false;
}

inline int OverlapTwoSphere(double r1, double centerX, double centerY, double centerZ, double r2, double x, double y, double z)
{
	double dx = x - centerX;
	double dy = y - centerY;
	double dz = z - centerZ;
	double d = std::sqrt(dx*dx + dy*dy + dz*dz);
	double overlap = (r1 + r2 - d) / std::min(r1, r2);
	if (overlap> -0.01 && overlap < 0.5)
	{
		return true;
	}
	return false;
}
#endif
