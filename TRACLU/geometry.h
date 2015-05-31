#pragma once
#include <math.h>
#include <vector>
#include <assert.h>

//地球半径，单位为m
#define EARTHRADIUS 6371000
#define w_perpendicular 1.0
#define w_parallel 1.0
#define w_theta 1.0
#define UNCLASSIFIED -1
#define NOISE -2
#define DOUBLE_EPS 1e-4
const double PI = acos(-1.0);

template <class T>
T SQR(T x){return x*x;}

class Point
{
public:
	double x,y,z;//经纬度转化为三维空间坐标

	//默认构造函数，do what？
	Point();

	//此映射在计算角度的时候比较方便，不需要除以R^2，但是在求实际距离的时候需要乘以地球半径
	Point(const double& _lon,const double& _lat);

	//此构造函数用于初始化向量(x,y,z)
	Point(const double& _x,const double& _y,const double& _z);
	
	//求该点到另一点p的欧几里得距离，单位为m
	double EucDisTo(const Point& p);

	//求该点到另一点的地理距离，单位为m
	double GeoDisTo(Point& p);

	//返回两点与地心构成的向量之间的夹角，单位为弧度
	double degBetween(Point& p);

	//重载加号，用于计算向量
	Point operator + (const Point& p);

	//重载减号，用于计算向量
	Point operator - (const Point& p);

	//重载乘号，用于计算向量点乘
	double operator * (const Point& p);

	//重载乘号，用于计算向量数乘
	Point operator * (const double& ratio);

	//重载除号，用于计算数除
	Point operator / (const double& ratio);

	//求向量模长
	double length();
};
typedef Point Vector;

class Trajectory
{
private:
	//轨迹中的点
	std::vector <Point> p;
public:
	//每条轨迹唯一表示id
	int ID;

	Trajectory();
	Trajectory(const int& id,std::vector <Point> np);
	Trajectory(const Trajectory& tra);
	//清空
	void clear();
	
	//设置轨迹点的数量
	void setSize(const int& sz);

	//返回轨迹点的数量
	int size();

	//在轨迹末尾增加一个轨迹点
	void add(const Point& np);

	//删除轨迹末尾的一个点
	void pop_back();
	
	//重载中括号，提供方便访问轨迹点的操作符
	Point& operator [](const int& idx);

	//重载小于号，用于set中轨迹的判重
	bool operator < (const Trajectory& t)const;

	//重载等于号，用于赋值
	Trajectory& operator = (Trajectory& t);
};

class Segment
{	
public:
	Point s,e;
	Segment(const Point& _s,const Point& _e);

	//计算与目标seg_i之间的MDL距离，相当于将此线段投影到目标seg_i上,当前线段即为j
	double DisTo(Segment seg_i);
	
	//单独返回与目标seg_i之间的平行距离
	double ParaDisTo(Segment seg_i);

	//单独返回与目标seg_i之间的垂直距离
	double PerpDisTo(Segment seg_i);

	//单独返回与目标seg_i之间的角度距离
	double ThetaDisTo(Segment seg_i);

	//返回两条seg的角度，单位为弧度
	double DegBetween(Segment seg);
	
	//返回该seg的欧几里得长度，单位为m
	double length();
};

class ClusterSegment:public Segment
{
public:
	//此段属于哪条轨迹
	int TraID;
	//属于哪个聚类
	int clusterID;
	ClusterSegment(int _TraID,int _clusterID,Point s,Point e):TraID(_TraID),clusterID(_clusterID),Segment(s,e){}
};

class SweepPlanePoint:public Point
{
public:
	//此点属于哪个seg
	int SegId;
	//此点与平面的距离
	double dis;
	SweepPlanePoint():SegId(0),dis(0){}
	SweepPlanePoint(const Point& p,int id,double _dis):Point(p),SegId(id),dis(_dis){};
	bool operator < (const SweepPlanePoint& p)const
	{
		return dis < p.dis;
	}
};

//已知角度，返回对应的弧度
double getRad(double deg);

//已知弧度角，返回对应的角度
double getDeg(double rad);

//返回p点的纬度
double getLat(const Point& p);

//返回p点的经度
double getLon(const Point& p);

//求平面参数p1,p2,p3是平面三点，a,b,c,d为平面的参数
void get_panel(Point p1,Point p2,Point p3,double &a,double &b,double &c,double &d);

//已知三点坐标，求法向量
Vector get_Normal(Point p1,Point p2,Point p3);

//点到平面距离，a,b,c,d为平面参数
double dis_pt2panel(Point pt,double a,double b,double c,double d);

/// 求一条直线与平面的交点
Point CalPlaneLineIntersectPoint(Vector planeVector, Point planePoint, Vector lineVector, Point linePoint);