#pragma once
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <string>
#include <fstream>
#include <queue>
#include <set>
#include <map>
#include <vector>
#include "database.h"
#include "geometry.h"
#include "R_Tree.h"
#include "matrix.h"

typedef std::vector <ClusterSegment> Cluster;
typedef std::vector < Cluster > setOfCluster;
typedef RTree<int,double,3,double> myTree;

#define EPS 30
#define MinLns 2
#define gamma 100

struct Rect
{
  Rect()  {}

  Rect(double a_minX, double a_minY, double a_minZ, double a_maxX, double a_maxY, double a_maxZ)
  {
    min[0] = a_minX;
    min[1] = a_minY;
	min[2] = a_minZ;

    max[0] = a_maxX;
    max[1] = a_maxY;
	max[2] = a_maxZ;
  }


  double min[3];
  double max[3];
};

double toRad(double deg);

//读入轨迹数据
void readDataFromSplitFile(std::vector<Trajectory>& Tra);
void readDataFromOneFile(std::vector<Trajectory>& Tra);

//把聚类后的轨迹写入数据库
void writeToDB(Trajectory& Tra,Database*,int clusterID);

//计算seg段以头尾为关键点的代价
double costPar(std::vector<Segment>& seg);

//计算seg段的距离和
double costNoPar(const double* sum,int l,int r);

//计算聚类的基，即聚类中有多少seg是属于不同tra的
int PTR(Cluster& C);

//扩展core segment，用于Micor_clustering
void expandCluster(Cluster& D,std::queue<int>& Q,int clusterId,myTree* tree);

//返回D集合中编号为id的seg的eps邻居集合
std::vector <int> N_eps(Cluster& D,int id,myTree* tree);

//程序的初始化代码
void Init(std::vector<Trajectory>& Tra);

//分解阶段，如下两个函数将轨迹近似化
void PartitionPhase(std::vector <Trajectory>& st);
Trajectory ApproximateTrajectoryPartitioning(Trajectory Tra);

//聚类阶段，以下三个函数，作用同论文
setOfCluster GroupPhase(std::vector <Trajectory>& Tra);
void Micro_Clustering(std::vector <Trajectory>& Tra);
setOfCluster Macro_Clustering(Cluster& D,myTree* tree);


//展示阶段，将聚类结果转换为轨迹并写入数据库
void DisplayPhase(setOfCluster& clusters,Database*);
//此函数用于旋转坐标系，修改点集p为新坐标系下的点
//XZtheta为绕y轴,从y轴正半轴看过去逆时针旋转的角度
//YZtheta为绕x轴，从x轴正半轴看过去逆时针旋转的旋转的角度
void RotateXZ(Trajectory& p,double XZtheta);
void RotateYZ(Trajectory& p,double YZtheta);

//以下三个函数用于求矩阵的LUP分解，然后求出矩阵的逆矩阵
int* LUP_Decomposition(Matrix A,Matrix& L,Matrix& U,Matrix& P);
Matrix LUP_SolveEquationGroup(const Matrix& L,const Matrix& U,const int* const pi,const Matrix& b);
Matrix getInvMatrix(Matrix A);

//将坐标系旋转到使Z轴与平均向量平行，isRetrive默认为false，即旋转至平行，如果为true即旋转回原来的情况。
void RotateToPara(Trajectory& p,double XZtheta,double YZtheta,bool isRetrive);

//以下两个函数将聚类表示为轨迹，用来展示
Trajectory RepresentativeTrajectoryGeneration(Cluster cluster);