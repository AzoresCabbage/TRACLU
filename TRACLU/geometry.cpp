#include "geometry.h"

///////////////////Point Class Begin//////////////////////

//Ĭ�Ϲ��캯����do what��
Point::Point():x(0),y(0),z(0){}

//lonΪ����ֵ��latΪγ��ֵ����λΪ����rad
//��ӳ���ڼ���Ƕȵ�ʱ��ȽϷ��㣬����Ҫ����R^2����������ʵ�ʾ����ʱ����Ҫ���Ե���뾶
Point::Point(const double& lon,const double& lat)
{
	x = cos(lat)*cos(lon);
	y = cos(lat)*sin(lon);
	z = sin(lat);
}

//�˹��캯�����ڳ�ʼ������(x,y,z)
Point::Point(const double& _x,const double& _y,const double& _z):x(_x),y(_y),z(_z)
{

}

//��õ㵽��һ��p��ŷ����þ��룬��λΪm
double Point::EucDisTo(const Point& p)
{
	return EARTHRADIUS*sqrt(SQR(x-p.x)+SQR(y-p.y)+SQR(z-p.z));
}

//��õ㵽��һ��ĵ�����룬��λΪm
double Point::GeoDisTo(Point& p)
{
	//return EARTHRADIUS*sqrt(SQR(x-p.x)+SQR(y-p.y)+SQR(z-p.z));
	double cosTheta = degBetween(p);
	return EARTHRADIUS*sqrt(fabs(1.0-SQR(cosTheta)));
}

//������������Ĺ��ɵ�����֮��ļнǣ���λΪ����
double Point::degBetween(Point& p)
{
	return (*this * p) / (length()*p.length());
}

//���ؼӺţ����ڼ�������
Point Point::operator + (const Point& p)
{
	return Point(x+p.x,y+p.y,z+p.z);
}

//���ؼ��ţ����ڼ�������
Point Point::operator - (const Point& p)
{
	return Point(x-p.x,y-p.y,z-p.z);
}

//���س˺ţ����ڼ�����
double Point::operator * (const Point& p)
{
	return x*p.x + y*p.y + z*p.z;
}

//���س˺ţ����ڼ�������
Point Point::operator * (const double& ratio)
{
	return Point(x*ratio,y*ratio,z*ratio);
}

//���س��ţ����ڼ�������
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

//������ģ��
double Point::length()
{
	return sqrt(SQR(x)+SQR(y)+SQR(z));
}

///////////////////Point Class End//////////////////////


///////////////////Trajectory Class Begin//////////////////////

//Ĭ�Ϲ��캯��
Trajectory::Trajectory():ID(-1){}

//���캯��
Trajectory::Trajectory(const int& id,std::vector <Point> np):ID(id)
{
	p.swap(np);
}

//�������캯��
Trajectory::Trajectory(const Trajectory& tra)
{
	ID = tra.ID;
	std::vector<Point> tmp(tra.p);
	p.swap(tmp);
}

//���
void Trajectory::clear()
{
	p.clear();
	ID = -1;
}

//���ù켣�������
void Trajectory::setSize(const int& sz)
{
	p.resize(sz);
}

//���ع켣�������
int Trajectory::size()
{
	return (int)p.size();
}

//�ڹ켣ĩβ����һ���켣��
void Trajectory::add(const Point& np)
{
	p.push_back(np);
}

//ɾ���켣ĩβ��һ����
void Trajectory::pop_back()
{
	if(p.empty()) return;
	p.pop_back();
}

//���������ţ��ṩ������ʹ켣��Ĳ�����
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

//����С�ںţ�����set�й켣������
bool Trajectory::operator < (const Trajectory& t)const
{
	return ID < t.ID;
}

//���ص��ںţ����ڸ�ֵ
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

//������Ŀ��seg_i֮���MDL���룬�൱�ڽ����߶�ͶӰ��Ŀ��seg_i��,��ǰ�߶μ�Ϊj
double Segment::DisTo(Segment seg_i)
{
	bool isSwap = false;
	if(length() > seg_i.length())
		std::swap(*this,seg_i),isSwap = true;

	Vector si_sj = Vector(s-seg_i.s);
	Vector si_ei = Vector(seg_i.e-seg_i.s);
	double u1 = (si_sj*si_ei) / (si_ei*si_ei);
	//Ps Ϊs����seg_i�ϵ�ͶӰ��
	Point Ps = seg_i.s + si_ei*u1;

	Vector si_ej = Vector(e-seg_i.s);
	double u2 = (si_ej*si_ei) / (si_ei*si_ei);
	//Pe Ϊe����seg_i�ϵ�ͶӰ��
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

//����������Ŀ��seg_i֮���ƽ�о���
double Segment::ParaDisTo(Segment seg_i)
{
	bool isSwap = false;
	if(length() > seg_i.length())
		std::swap(*this,seg_i),isSwap = true;

	Vector si_sj = Vector(s-seg_i.s);
	Vector si_ei = Vector(seg_i.e-seg_i.s);
	double u1 = (si_sj*si_ei) / (si_ei*si_ei);
	//Ps Ϊs����seg_i�ϵ�ͶӰ��
	Point Ps = seg_i.s + si_ei*u1;

	Vector si_ej = Vector(e-seg_i.s);
	double u2 = (si_ej*si_ei) / (si_ei*si_ei);
	//Pe Ϊe����seg_i�ϵ�ͶӰ��
	Point Pe = seg_i.s + si_ei*u2;

	double L_parallel_1 = seg_i.s.EucDisTo(Ps);
	double L_parallel_2 = seg_i.e.EucDisTo(Pe);
	double D_parallel = std::min(L_parallel_1,L_parallel_2);
	if(isSwap)
		std::swap(*this,seg_i);
	return D_parallel;
}

//����������Ŀ��seg_i֮��Ĵ�ֱ����
double Segment::PerpDisTo(Segment seg_i)
{
	bool isSwap = false;
	if(length() > seg_i.length())
		std::swap(*this,seg_i),isSwap = true;

	Vector si_sj = Vector(s-seg_i.s);
	Vector si_ei = Vector(seg_i.e-seg_i.s);
	double u1 = (si_sj*si_ei) / (si_ei*si_ei);
	//Ps Ϊs����seg_i�ϵ�ͶӰ��
	Point Ps = seg_i.s + si_ei*u1;

	Vector si_ej = Vector(e-seg_i.s);
	double u2 = (si_ej*si_ei) / (si_ei*si_ei);
	//Pe Ϊe����seg_i�ϵ�ͶӰ��
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

//����������Ŀ��seg_i֮��ĽǶȾ���
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

//��������seg��cos�Ƕ�
double Segment::DegBetween(Segment seg)
{
	Vector v = e-s;
	Vector v_seg = seg.e-seg.s;
	return (v*v_seg)/(v.length()*v_seg.length());
}

//���ظ�seg��ŷ����ó��ȣ���λΪm
double Segment::length()
{
	return s.EucDisTo(e);
}
///////////////////Segment Class End//////////////////////

///////////////////other function////////////////////////

//��֪�Ƕȣ����ض�Ӧ�Ļ���
double getRad(double deg)
{
	return PI/180.0*deg;
}

//��֪���Ƚǣ����ض�Ӧ�ĽǶ�
double getDeg(double rad)
{
	double tmp = 180/PI*rad;
	if(tmp < 0)
		tmp = 180 + tmp;
	return tmp;
}

//����p���γ��
double getLat(const Point& p)
{
	assert(p.z >=-1 && p.z <= 1);
	return asin(p.z);
}

//����p��ľ���
double getLon(const Point& p)
{
	assert(fabs(p.x) > DOUBLE_EPS);
	return atan(p.y/p.x);
}

//��ƽ�����p1,p2,p3��ƽ�����㣬a,b,c,dΪƽ��Ĳ���
void get_panel(Point p1,Point p2,Point p3,double &a,double &b,double &c,double &d)  
{
	a = ( (p2.y-p1.y)*(p3.z-p1.z)-(p2.z-p1.z)*(p3.y-p1.y) );  
	b = ( (p2.z-p1.z)*(p3.x-p1.x)-(p2.x-p1.x)*(p3.z-p1.z) );  
	c = ( (p2.x-p1.x)*(p3.y-p1.y)-(p2.y-p1.y)*(p3.x-p1.x) );  
	d = ( 0-(a*p1.x+b*p1.y+c*p1.z) );
}

//��֪�������꣬������  
Vector get_Normal(Point p1,Point p2,Point p3)  
{  
	double a = ( (p2.y-p1.y)*(p3.z-p1.z)-(p2.z-p1.z)*(p3.y-p1.y) );  
	double b = ( (p2.z-p1.z)*(p3.x-p1.x)-(p2.x-p1.x)*(p3.z-p1.z) );  
	double c = ( (p2.x-p1.x)*(p3.y-p1.y)-(p2.y-p1.y)*(p3.x-p1.x) );  
	return Vector(a,b,c);  
}

//�㵽ƽ����룬a,b,c,dΪƽ�����
double dis_pt2panel(Point pt,double a,double b,double c,double d){ 
	return fabs(a*pt.x+b*pt.y+c*pt.z+d)/sqrt(a*a+b*b+c*c);  
}

/// ��һ��ֱ����ƽ��Ľ���
/// <param name="planeVector">ƽ��ķ�������</param>
/// <param name="planePoint">ƽ�澭����һ������</param>
/// <param name="lineVector">ֱ�ߵķ�������</param>
/// <param name="linePoint">ֱ�߾�����һ������</param>
/// <returns>���ؽ�������</returns>
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
	//�����ж�ֱ���Ƿ���ƽ��ƽ��
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
