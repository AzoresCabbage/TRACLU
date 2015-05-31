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

//����켣����
void readDataFromSplitFile(std::vector<Trajectory>& Tra);
void readDataFromOneFile(std::vector<Trajectory>& Tra);

//�Ѿ����Ĺ켣д�����ݿ�
void writeToDB(Trajectory& Tra,Database*,int clusterID);

//����seg����ͷβΪ�ؼ���Ĵ���
double costPar(std::vector<Segment>& seg);

//����seg�εľ����
double costNoPar(const double* sum,int l,int r);

//�������Ļ������������ж���seg�����ڲ�ͬtra��
int PTR(Cluster& C);

//��չcore segment������Micor_clustering
void expandCluster(Cluster& D,std::queue<int>& Q,int clusterId,myTree* tree);

//����D�����б��Ϊid��seg��eps�ھӼ���
std::vector <int> N_eps(Cluster& D,int id,myTree* tree);

//����ĳ�ʼ������
void Init(std::vector<Trajectory>& Tra);

//�ֽ�׶Σ����������������켣���ƻ�
void PartitionPhase(std::vector <Trajectory>& st);
Trajectory ApproximateTrajectoryPartitioning(Trajectory Tra);

//����׶Σ�������������������ͬ����
setOfCluster GroupPhase(std::vector <Trajectory>& Tra);
void Micro_Clustering(std::vector <Trajectory>& Tra);
setOfCluster Macro_Clustering(Cluster& D,myTree* tree);


//չʾ�׶Σ���������ת��Ϊ�켣��д�����ݿ�
void DisplayPhase(setOfCluster& clusters,Database*);
//�˺���������ת����ϵ���޸ĵ㼯pΪ������ϵ�µĵ�
//XZthetaΪ��y��,��y�������ῴ��ȥ��ʱ����ת�ĽǶ�
//YZthetaΪ��x�ᣬ��x�������ῴ��ȥ��ʱ����ת����ת�ĽǶ�
void RotateXZ(Trajectory& p,double XZtheta);
void RotateYZ(Trajectory& p,double YZtheta);

//����������������������LUP�ֽ⣬Ȼ���������������
int* LUP_Decomposition(Matrix A,Matrix& L,Matrix& U,Matrix& P);
Matrix LUP_SolveEquationGroup(const Matrix& L,const Matrix& U,const int* const pi,const Matrix& b);
Matrix getInvMatrix(Matrix A);

//������ϵ��ת��ʹZ����ƽ������ƽ�У�isRetriveĬ��Ϊfalse������ת��ƽ�У����Ϊtrue����ת��ԭ���������
void RotateToPara(Trajectory& p,double XZtheta,double YZtheta,bool isRetrive);

//�������������������ʾΪ�켣������չʾ
Trajectory RepresentativeTrajectoryGeneration(Cluster cluster);