#ifndef _UTILITY_H_
#define _UTILITY_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_traits.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>

#define X 0
#define Y 1
#define Z 2

#define CROSS(dest,v1,v2) \
          dest[0]=v1[1]*v2[2]-v1[2]*v2[1]; \
          dest[1]=v1[2]*v2[0]-v1[0]*v2[2]; \
          dest[2]=v1[0]*v2[1]-v1[1]*v2[0];

#define DOT(v1,v2) (v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2])

#define SUB(dest,v1,v2) \
          dest[0]=v1[0]-v2[0]; \
          dest[1]=v1[1]-v2[1]; \
          dest[2]=v1[2]-v2[2];

#define FINDMINMAX(x0,x1,x2,min,max) \
  min = max = x0;   \
  if(x1<min) min=x1;\
  if(x1>max) max=x1;\
  if(x2<min) min=x2;\
  if(x2>max) max=x2;

inline int planeBoxOverlap(double normal[3], double vert[3], double maxbox[3])	// -NJMP-
{
	int q;
	double vmin[3], vmax[3], v;
	for (q = X; q <= Z; q++)
	{
		v = vert[q];					// -NJMP-
		if (normal[q]>0.0f)
		{
			vmin[q] = -maxbox[q] - v;	// -NJMP-
			vmax[q] = maxbox[q] - v;	// -NJMP-
		}
		else
		{
			vmin[q] = maxbox[q] - v;	// -NJMP-
			vmax[q] = -maxbox[q] - v;	// -NJMP-
		}
	}
	if (DOT(normal, vmin)>0.0f) return 0;	// -NJMP-
	if (DOT(normal, vmax) >= 0.0f) return 1;	// -NJMP-

	return 0;
}

/*======================== X-tests ========================*/
#define AXISTEST_X01(a, b, fa, fb)			   \
	p0 = a*v0[Y] - b*v0[Z];			       	   \
	p2 = a*v2[Y] - b*v2[Z];			       	   \
        if(p0<p2) {min=p0; max=p2;} else {min=p2; max=p0;} \
	rad = fa * boxhalfsize[Y] + fb * boxhalfsize[Z];   \
	if(min>rad || max<-rad) return 0;

#define AXISTEST_X2(a, b, fa, fb)			   \
	p0 = a*v0[Y] - b*v0[Z];			           \
	p1 = a*v1[Y] - b*v1[Z];			       	   \
        if(p0<p1) {min=p0; max=p1;} else {min=p1; max=p0;} \
	rad = fa * boxhalfsize[Y] + fb * boxhalfsize[Z];   \
	if(min>rad || max<-rad) return 0;

/*======================== Y-tests ========================*/
#define AXISTEST_Y02(a, b, fa, fb)			   \
	p0 = -a*v0[X] + b*v0[Z];		      	   \
	p2 = -a*v2[X] + b*v2[Z];	       	       	   \
        if(p0<p2) {min=p0; max=p2;} else {min=p2; max=p0;} \
	rad = fa * boxhalfsize[X] + fb * boxhalfsize[Z];   \
	if(min>rad || max<-rad) return 0;

#define AXISTEST_Y1(a, b, fa, fb)			   \
	p0 = -a*v0[X] + b*v0[Z];		      	   \
	p1 = -a*v1[X] + b*v1[Z];	     	       	   \
        if(p0<p1) {min=p0; max=p1;} else {min=p1; max=p0;} \
	rad = fa * boxhalfsize[X] + fb * boxhalfsize[Z];   \
	if(min>rad || max<-rad) return 0;

/*======================== Z-tests ========================*/

#define AXISTEST_Z12(a, b, fa, fb)			   \
	p1 = a*v1[X] - b*v1[Y];			           \
	p2 = a*v2[X] - b*v2[Y];			       	   \
        if(p2<p1) {min=p2; max=p1;} else {min=p1; max=p2;} \
	rad = fa * boxhalfsize[X] + fb * boxhalfsize[Y];   \
	if(min>rad || max<-rad) return 0;

#define AXISTEST_Z0(a, b, fa, fb)			   \
	p0 = a*v0[X] - b*v0[Y];				   \
	p1 = a*v1[X] - b*v1[Y];			           \
        if(p0<p1) {min=p0; max=p1;} else {min=p1; max=p0;} \
	rad = fa * boxhalfsize[X] + fb * boxhalfsize[Y];   \
	if(min>rad || max<-rad) return 0;

inline int triBoxOverlap(double boxcenter[3], double boxhalfsize[3], double** triverts)
{

	/*    use separating axis theorem to test overlap between triangle and box */
	/*    need to test for overlap in these directions: */
	/*    1) the {x,y,z}-directions (actually, since we use the AABB of the triangle */
	/*       we do not even need to test these) */
	/*    2) normal of the triangle */
	/*    3) crossproduct(edge from tri, {x,y,z}-directin) */
	/*       this gives 3x3=9 more tests */
	double v0[3], v1[3], v2[3];
	//   double axis[3];
	double min, max, p0, p1, p2, rad, fex, fey, fez;		// -NJMP- "d" local variable removed
	double normal[3], e0[3], e1[3], e2[3];

	/* This is the fastest branch on Sun */
	/* move everything so that the boxcenter is in (0,0,0) */
	SUB(v0, triverts[0], boxcenter);
	SUB(v1, triverts[1], boxcenter);
	SUB(v2, triverts[2], boxcenter);

	/* compute triangle edges */
	SUB(e0, v1, v0);      /* tri edge 0 */
	SUB(e1, v2, v1);      /* tri edge 1 */
	SUB(e2, v0, v2);      /* tri edge 2 */

						  /* Bullet 3:  */
						  /*  test the 9 tests first (this was faster) */
	fex = std::abs(e0[X]);
	fey = std::abs(e0[Y]);
	fez = std::abs(e0[Z]);
	AXISTEST_X01(e0[Z], e0[Y], fez, fey);
	AXISTEST_Y02(e0[Z], e0[X], fez, fex);
	AXISTEST_Z12(e0[Y], e0[X], fey, fex);

	fex = std::abs(e1[X]);
	fey = std::abs(e1[Y]);
	fez = std::abs(e1[Z]);
	AXISTEST_X01(e1[Z], e1[Y], fez, fey);
	AXISTEST_Y02(e1[Z], e1[X], fez, fex);
	AXISTEST_Z0(e1[Y], e1[X], fey, fex);

	fex = std::abs(e2[X]);
	fey = std::abs(e2[Y]);
	fez = std::abs(e2[Z]);
	AXISTEST_X2(e2[Z], e2[Y], fez, fey);
	AXISTEST_Y1(e2[Z], e2[X], fez, fex);
	AXISTEST_Z12(e2[Y], e2[X], fey, fex);

	/* Bullet 1: */
	/*  first test overlap in the {x,y,z}-directions */
	/*  find min, max of the triangle each direction, and test for overlap in */
	/*  that direction -- this is equivalent to testing a minimal AABB around */
	/*  the triangle against the AABB */

	/* test in X-direction */
	FINDMINMAX(v0[X], v1[X], v2[X], min, max);
	if (min>boxhalfsize[X] || max<-boxhalfsize[X]) return 0;

	/* test in Y-direction */
	FINDMINMAX(v0[Y], v1[Y], v2[Y], min, max);
	if (min>boxhalfsize[Y] || max<-boxhalfsize[Y]) return 0;

	/* test in Z-direction */
	FINDMINMAX(v0[Z], v1[Z], v2[Z], min, max);
	if (min>boxhalfsize[Z] || max<-boxhalfsize[Z]) return 0;

	/* Bullet 2: */
	/*  test if the box intersects the plane of the triangle */
	/*  compute plane equation of triangle: normal*x+d=0 */
	CROSS(normal, e0, e1);
	// -NJMP- (line removed here)
	if (!planeBoxOverlap(normal, v0, boxhalfsize)) return 0;	// -NJMP-

	return 1;   /* box and triangle overlaps */
}


#undef X
#undef Y
#undef Z
#undef CROSS
#undef DOT
#undef SUB
#undef FINDMINMAX
#undef AXISTEST_X01
#undef AXISTEST_X2
#undef AXISTEST_Y02
#undef AXISTEST_Y1
#undef AXISTEST_Z12
#undef AXISTEST_Z0

inline bool TriangleAABB(const Eigen::Vector3d& box_center,
	const Eigen::Vector3d& box_half_size,
	const Eigen::Vector3d& vert0,
	const Eigen::Vector3d& vert1,
	const Eigen::Vector3d& vert2) {
	double* tri_verts[3] = { const_cast<double*>(vert0.data()),
		const_cast<double*>(vert1.data()),
		const_cast<double*>(vert2.data()) };
	return triBoxOverlap(const_cast<double*>(box_center.data()),
		const_cast<double*>(box_half_size.data()),
		tri_verts) != 0;
}

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
