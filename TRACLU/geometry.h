#pragma once
#include <math.h>
#include <vector>
#include <assert.h>

//����뾶����λΪm
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
	double x,y,z;//��γ��ת��Ϊ��ά�ռ�����

	//Ĭ�Ϲ��캯����do what��
	Point();

	//��ӳ���ڼ���Ƕȵ�ʱ��ȽϷ��㣬����Ҫ����R^2����������ʵ�ʾ����ʱ����Ҫ���Ե���뾶
	Point(const double& _lon,const double& _lat);

	//�˹��캯�����ڳ�ʼ������(x,y,z)
	Point(const double& _x,const double& _y,const double& _z);
	
	//��õ㵽��һ��p��ŷ����þ��룬��λΪm
	double EucDisTo(const Point& p);

	//��õ㵽��һ��ĵ�����룬��λΪm
	double GeoDisTo(Point& p);

	//������������Ĺ��ɵ�����֮��ļнǣ���λΪ����
	double degBetween(Point& p);

	//���ؼӺţ����ڼ�������
	Point operator + (const Point& p);

	//���ؼ��ţ����ڼ�������
	Point operator - (const Point& p);

	//���س˺ţ����ڼ����������
	double operator * (const Point& p);

	//���س˺ţ����ڼ�����������
	Point operator * (const double& ratio);

	//���س��ţ����ڼ�������
	Point operator / (const double& ratio);

	//������ģ��
	double length();
};
typedef Point Vector;

class Trajectory
{
private:
	//�켣�еĵ�
	std::vector <Point> p;
public:
	//ÿ���켣Ψһ��ʾid
	int ID;

	Trajectory();
	Trajectory(const int& id,std::vector <Point> np);
	Trajectory(const Trajectory& tra);
	//���
	void clear();
	
	//���ù켣�������
	void setSize(const int& sz);

	//���ع켣�������
	int size();

	//�ڹ켣ĩβ����һ���켣��
	void add(const Point& np);

	//ɾ���켣ĩβ��һ����
	void pop_back();
	
	//���������ţ��ṩ������ʹ켣��Ĳ�����
	Point& operator [](const int& idx);

	//����С�ںţ�����set�й켣������
	bool operator < (const Trajectory& t)const;

	//���ص��ںţ����ڸ�ֵ
	Trajectory& operator = (Trajectory& t);
};

class Segment
{	
public:
	Point s,e;
	Segment(const Point& _s,const Point& _e);

	//������Ŀ��seg_i֮���MDL���룬�൱�ڽ����߶�ͶӰ��Ŀ��seg_i��,��ǰ�߶μ�Ϊj
	double DisTo(Segment seg_i);
	
	//����������Ŀ��seg_i֮���ƽ�о���
	double ParaDisTo(Segment seg_i);

	//����������Ŀ��seg_i֮��Ĵ�ֱ����
	double PerpDisTo(Segment seg_i);

	//����������Ŀ��seg_i֮��ĽǶȾ���
	double ThetaDisTo(Segment seg_i);

	//��������seg�ĽǶȣ���λΪ����
	double DegBetween(Segment seg);
	
	//���ظ�seg��ŷ����ó��ȣ���λΪm
	double length();
};

class ClusterSegment:public Segment
{
public:
	//�˶����������켣
	int TraID;
	//�����ĸ�����
	int clusterID;
	ClusterSegment(int _TraID,int _clusterID,Point s,Point e):TraID(_TraID),clusterID(_clusterID),Segment(s,e){}
};

class SweepPlanePoint:public Point
{
public:
	//�˵������ĸ�seg
	int SegId;
	//�˵���ƽ��ľ���
	double dis;
	SweepPlanePoint():SegId(0),dis(0){}
	SweepPlanePoint(const Point& p,int id,double _dis):Point(p),SegId(id),dis(_dis){};
	bool operator < (const SweepPlanePoint& p)const
	{
		return dis < p.dis;
	}
};

//��֪�Ƕȣ����ض�Ӧ�Ļ���
double getRad(double deg);

//��֪���Ƚǣ����ض�Ӧ�ĽǶ�
double getDeg(double rad);

//����p���γ��
double getLat(const Point& p);

//����p��ľ���
double getLon(const Point& p);

//��ƽ�����p1,p2,p3��ƽ�����㣬a,b,c,dΪƽ��Ĳ���
void get_panel(Point p1,Point p2,Point p3,double &a,double &b,double &c,double &d);

//��֪�������꣬������
Vector get_Normal(Point p1,Point p2,Point p3);

//�㵽ƽ����룬a,b,c,dΪƽ�����
double dis_pt2panel(Point pt,double a,double b,double c,double d);

/// ��һ��ֱ����ƽ��Ľ���
Point CalPlaneLineIntersectPoint(Vector planeVector, Point planePoint, Vector lineVector, Point linePoint);