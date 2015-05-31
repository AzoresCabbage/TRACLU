#include "geometry.h"

///////////////////Point Class Begin//////////////////////

//默认构造函数，do what？
Point::Point():x(0),y(0),z(0){}

//lon为经度值，lat为纬度值，单位为弧度rad
//此映射在计算角度的时候比较方便，不需要除以R^2，但是在求实际距离的时候需要乘以地球半径
Point::Point(const double& lon,const double& lat)
{
	x = cos(lat)*cos(lon);
	y = cos(lat)*sin(lon);
	z = sin(lat);
}

//此构造函数用于初始化向量(x,y,z)
Point::Point(const double& _x,const double& _y,const double& _z):x(_x),y(_y),z(_z)
{

}

//求该点到另一点p的欧几里得距离，单位为m
double Point::EucDisTo(const Point& p)
{
	return EARTHRADIUS*sqrt(SQR(x-p.x)+SQR(y-p.y)+SQR(z-p.z));
}

//求该点到另一点的地理距离，单位为m
double Point::GeoDisTo(Point& p)
{
	//return EARTHRADIUS*sqrt(SQR(x-p.x)+SQR(y-p.y)+SQR(z-p.z));
	double cosTheta = degBetween(p);
	return EARTHRADIUS*sqrt(fabs(1.0-SQR(cosTheta)));
}

//返回两点与地心构成的向量之间的夹角，单位为弧度
double Point::degBetween(Point& p)
{
	return (*this * p) / (length()*p.length());
}

//重载加号，用于计算向量
Point Point::operator + (const Point& p)
{
	return Point(x+p.x,y+p.y,z+p.z);
}

//重载减号，用于计算向量
Point Point::operator - (const Point& p)
{
	return Point(x-p.x,y-p.y,z-p.z);
}

//重载乘号，用于计算点乘
double Point::operator * (const Point& p)
{
	return x*p.x + y*p.y + z*p.z;
}

//重载乘号，用于计算数乘
Point Point::operator * (const double& ratio)
{
	return Point(x*ratio,y*ratio,z*ratio);
}

//重载除号，用于计算数除
Point Point::operator / (const double& ratio)
{
	if(ratio == 0)
	{
		fprintf(stderr,"vector devided by zero!\n");
		system("pause");
		exit(-1);
	}
	return Point(this->x/ratio,this->y/ratio,this->z/ratio);
}

//求向量模长
double Point::length()
{
	return sqrt(SQR(x)+SQR(y)+SQR(z));
}

///////////////////Point Class End//////////////////////


///////////////////Trajectory Class Begin//////////////////////

//默认构造函数
Trajectory::Trajectory():ID(-1){}

//构造函数
Trajectory::Trajectory(const int& id,std::vector <Point> np):ID(id)
{
	p.swap(np);
}

//拷贝构造函数
Trajectory::Trajectory(const Trajectory& tra)
{
	ID = tra.ID;
	std::vector<Point> tmp(tra.p);
	p.swap(tmp);
}

//清空
void Trajectory::clear()
{
	p.clear();
	ID = -1;
}

//设置轨迹点的数量
void Trajectory::setSize(const int& sz)
{
	p.resize(sz);
}

//返回轨迹点的数量
int Trajectory::size()
{
	return (int)p.size();
}

//在轨迹末尾增加一个轨迹点
void Trajectory::add(const Point& np)
{
	p.push_back(np);
}

//删除轨迹末尾的一个点
void Trajectory::pop_back()
{
	if(p.empty()) return;
	p.pop_back();
}

//重载中括号，提供方便访问轨迹点的操作符
Point& Trajectory::operator [](const int& idx)
{
	try
	{
		return p[idx];
	}
	catch (std::exception e)
	{
		fprintf(stderr,"access Trajectory out of range idx = %d!\n",idx);
		system("pause");
		exit(-1);
	}
}

//重载小于号，用于set中轨迹的判重
bool Trajectory::operator < (const Trajectory& t)const
{
	return ID < t.ID;
}

//重载等于号，用于赋值
Trajectory& Trajectory::operator = (Trajectory& t)
{
	ID = t.ID;
	p.swap(t.p);
	return *this;
}
///////////////////Trajectory Class End//////////////////////


///////////////////Segment Class Begin//////////////////////

Segment::Segment(const Point& _s,const Point& _e)
	:s(_s),e(_e)
{

}

//计算与目标seg_i之间的MDL距离，相当于将此线段投影到目标seg_i上,当前线段即为j
double Segment::DisTo(Segment seg_i)
{
	bool isSwap = false;
	if(length() > seg_i.length())
		std::swap(*this,seg_i),isSwap = true;

	Vector si_sj = Vector(s-seg_i.s);
	Vector si_ei = Vector(seg_i.e-seg_i.s);
	double u1 = (si_sj*si_ei) / (si_ei*si_ei);
	//Ps 为s点在seg_i上的投影点
	Point Ps = seg_i.s + si_ei*u1;

	Vector si_ej = Vector(e-seg_i.s);
	double u2 = (si_ej*si_ei) / (si_ei*si_ei);
	//Pe 为e点在seg_i上的投影点
	Point Pe = seg_i.s + si_ei*u2;

	double L_perpendicular_1 = s.EucDisTo(Ps);
	double L_perpendicular_2 = e.EucDisTo(Pe);
	double D_perpendicular = 0;
	if(L_perpendicular_1 + L_perpendicular_2 > DOUBLE_EPS)
	{
		D_perpendicular = ( SQR(L_perpendicular_1) + SQR(L_perpendicular_2) )
			/ (L_perpendicular_1 + L_perpendicular_2);
	}

	double L_parallel_1 = seg_i.s.EucDisTo(Ps);
	double L_parallel_2 = seg_i.e.EucDisTo(Pe);
	double D_parallel = std::min(L_parallel_1,L_parallel_2);

	double D_theta = ThetaDisTo(seg_i);

	if(isSwap)
		std::swap(*this,seg_i);
	
	return w_perpendicular*D_perpendicular + w_parallel*D_parallel + w_theta*D_theta;
}

//单独返回与目标seg_i之间的平行距离
double Segment::ParaDisTo(Segment seg_i)
{
	bool isSwap = false;
	if(length() > seg_i.length())
		std::swap(*this,seg_i),isSwap = true;

	Vector si_sj = Vector(s-seg_i.s);
	Vector si_ei = Vector(seg_i.e-seg_i.s);
	double u1 = (si_sj*si_ei) / (si_ei*si_ei);
	//Ps 为s点在seg_i上的投影点
	Point Ps = seg_i.s + si_ei*u1;

	Vector si_ej = Vector(e-seg_i.s);
	double u2 = (si_ej*si_ei) / (si_ei*si_ei);
	//Pe 为e点在seg_i上的投影点
	Point Pe = seg_i.s + si_ei*u2;

	double L_parallel_1 = seg_i.s.EucDisTo(Ps);
	double L_parallel_2 = seg_i.e.EucDisTo(Pe);
	double D_parallel = std::min(L_parallel_1,L_parallel_2);
	if(isSwap)
		std::swap(*this,seg_i);
	return D_parallel;
}

//单独返回与目标seg_i之间的垂直距离
double Segment::PerpDisTo(Segment seg_i)
{
	bool isSwap = false;
	if(length() > seg_i.length())
		std::swap(*this,seg_i),isSwap = true;

	Vector si_sj = Vector(s-seg_i.s);
	Vector si_ei = Vector(seg_i.e-seg_i.s);
	double u1 = (si_sj*si_ei) / (si_ei*si_ei);
	//Ps 为s点在seg_i上的投影点
	Point Ps = seg_i.s + si_ei*u1;

	Vector si_ej = Vector(e-seg_i.s);
	double u2 = (si_ej*si_ei) / (si_ei*si_ei);
	//Pe 为e点在seg_i上的投影点
	Point Pe = seg_i.s + si_ei*u2;

	double L_perpendicular_1 = s.EucDisTo(Ps);
	double L_perpendicular_2 = e.EucDisTo(Pe);

	if(isSwap)
		std::swap(*this,seg_i);

	if(L_perpendicular_1 + L_perpendicular_2 < DOUBLE_EPS)
		return 0;
	double D_perpendicular = ( SQR(L_perpendicular_1) + SQR(L_perpendicular_2) )
		/ (L_perpendicular_1 + L_perpendicular_2);
	
	return D_perpendicular;
}

//单独返回与目标seg_i之间的角度距离
double Segment::ThetaDisTo(Segment seg_i)
{
	bool isSwap = false;
	if(length() > seg_i.length())
		std::swap(*this,seg_i),isSwap = true;
	double cosTheta = DegBetween(seg_i);
	double D_theta = length() ;
	if(isSwap)
		std::swap(*this,seg_i);
	double tmp = D_theta * sqrt(fabs(1.0-SQR(cosTheta)));
	return tmp;
}

//返回两条seg的cos角度
double Segment::DegBetween(Segment seg)
{
	Vector v = e-s;
	Vector v_seg = seg.e-seg.s;
	return (v*v_seg)/(v.length()*v_seg.length());
}

//返回该seg的欧几里得长度，单位为m
double Segment::length()
{
	return s.EucDisTo(e);
}
///////////////////Segment Class End//////////////////////

///////////////////other function////////////////////////

//已知角度，返回对应的弧度
double getRad(double deg)
{
	return PI/180.0*deg;
}

//已知弧度角，返回对应的角度
double getDeg(double rad)
{
	double tmp = 180/PI*rad;
	if(tmp < 0)
		tmp = 180 + tmp;
	return tmp;
}

//返回p点的纬度
double getLat(const Point& p)
{
	assert(p.z >=-1 && p.z <= 1);
	return asin(p.z);
}

//返回p点的经度
double getLon(const Point& p)
{
	assert(fabs(p.x) > DOUBLE_EPS);
	return atan(p.y/p.x);
}

//求平面参数p1,p2,p3是平面三点，a,b,c,d为平面的参数
void get_panel(Point p1,Point p2,Point p3,double &a,double &b,double &c,double &d)  
{
	a = ( (p2.y-p1.y)*(p3.z-p1.z)-(p2.z-p1.z)*(p3.y-p1.y) );  
	b = ( (p2.z-p1.z)*(p3.x-p1.x)-(p2.x-p1.x)*(p3.z-p1.z) );  
	c = ( (p2.x-p1.x)*(p3.y-p1.y)-(p2.y-p1.y)*(p3.x-p1.x) );  
	d = ( 0-(a*p1.x+b*p1.y+c*p1.z) );
}

//已知三点坐标，求法向量  
Vector get_Normal(Point p1,Point p2,Point p3)  
{  
	double a = ( (p2.y-p1.y)*(p3.z-p1.z)-(p2.z-p1.z)*(p3.y-p1.y) );  
	double b = ( (p2.z-p1.z)*(p3.x-p1.x)-(p2.x-p1.x)*(p3.z-p1.z) );  
	double c = ( (p2.x-p1.x)*(p3.y-p1.y)-(p2.y-p1.y)*(p3.x-p1.x) );  
	return Vector(a,b,c);  
}

//点到平面距离，a,b,c,d为平面参数
double dis_pt2panel(Point pt,double a,double b,double c,double d){ 
	return fabs(a*pt.x+b*pt.y+c*pt.z+d)/sqrt(a*a+b*b+c*c);  
}

/// 求一条直线与平面的交点
/// <param name="planeVector">平面的法线向量</param>
/// <param name="planePoint">平面经过的一点坐标</param>
/// <param name="lineVector">直线的方向向量</param>
/// <param name="linePoint">直线经过的一点坐标</param>
/// <returns>返回交点坐标</returns>
Point CalPlaneLineIntersectPoint(Vector planeVector, Point planePoint, Vector lineVector, Point linePoint)
{
	Point returnResult;
	double vp1, vp2, vp3, n1, n2, n3, v1, v2, v3, m1, m2, m3, t,vpt;
	vp1 = planeVector.x;
	vp2 = planeVector.y;
	vp3 = planeVector.z;
	n1 = planePoint.x;
	n2 = planePoint.y;
	n3 = planePoint.z;
	v1 = lineVector.x;
	v2 = lineVector.y;
	v3 = lineVector.z;
	m1 = linePoint.x;
	m2 = linePoint.y;
	m3 = linePoint.z;
	vpt = v1 * vp1 + v2 * vp2 + v3 * vp3;
	//首先判断直线是否与平面平行
	if (vpt == 0)
	{
		
	}
	else
	{
		t = ((n1 - m1) * vp1 + (n2 - m2) * vp2 + (n3 - m3) * vp3) / vpt;
		returnResult.x = m1 + v1 * t;
		returnResult.y = m2 + v2 * t;
		returnResult.z = m3 + v3 * t;
	}
	return returnResult;
}
