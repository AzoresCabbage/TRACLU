#include "TRACLU.h"
using namespace std;
const string carGPSdataPath = "carGPSpath.txt";
const string oneFilePath = "E:/research/data/morning.csv";//"E:/research/data/T_drive_080202_080208/2008-02-02/allDay.csv";
const double lg2 = log(2.0);
char buf[2000];
int clusterPointID;
int clusterSegID;

int main()
{
	/*Segment s1(Point(0,0,0),Point(1,1,1));
	Segment s2(Point(2,2,3),Point(4,5,6));
	cout<<s1.DisTo(s2)<<" "<<s1.PerpDisTo(s2)<<" "<<s1.ThetaDisTo(s2)<<endl;
	cout<<s2.DisTo(s1)<<" "<<s2.PerpDisTo(s1)<<" "<<s2.ThetaDisTo(s1)<<endl;
	return 0;
	double paraA,paraB,paraC,paraD;
	Vector v1(1,0,0);
	Vector v2(0,1,0);
	get_panel(Point(0,0,0),v1,v2,paraA,paraB,paraC,paraD);
	Vector n = get_Normal(v1,v2,Point(0,0,0));
	cout<<n.x<<" "<<n.y<<" "<<n.z<<endl;
	cout<<dis_pt2panel(Point(0,0,-1),paraA,paraB,paraC,paraD)<<endl;
	Point intersect = CalPlaneLineIntersectPoint(n,v1,Vector(0,0,1),Point(1,1,2));
	cout<<intersect.x<<" "<<intersect.y<<" "<<intersect.z<<" "<<endl;
	return 0;*/
	//Trajectory p;
	//p.add(Point(1,1,1));
	//double XZtheta = atan(p[0].x / p[0].z);
	//double YZtheta = atan(p[0].y / p[0].z);
	
	/*RotateXZ(p,XZtheta);
	cout<<p[0].x<<" "<<p[0].y<<" "<<p[0].z<<endl;
	RotateYZ(p,-YZtheta);
	cout<<p[0].x<<" "<<p[0].y<<" "<<p[0].z<<endl;*/

	/*RotateToPara(p,XZtheta,-YZtheta,false);
	cout<<p[0].x<<" "<<p[0].y<<" "<<p[0].z<<endl;
	RotateToPara(p,XZtheta,-YZtheta,true);
	cout<<p[0].x<<" "<<p[0].y<<" "<<p[0].z<<endl;
	return 0;*/

	/*Trajectory tra;
	tra.add(Point(0,0,1));
	tra.add(Point(0,0.1,2));
	tra.add(Point(0.1,0,3));
	tra.add(Point(0.1,0.1,4));
	tra.add(Point(0,0,5));
	for(int i=0;i<5;++i)
	{
		cout<<tra[i].x<<" "<<tra[i].y<<" "<<tra[i].z<<endl;
	}
	
	tra = ApproximateTrajectoryPartitioning(tra);


	cout<<tra.size()<<endl;

	return 0;*/


	Database* DB = new Database();
	vector<Trajectory> Tra;
	Init(Tra);
	PartitionPhase(Tra);
	setOfCluster cur = GroupPhase(Tra);
	DisplayPhase(cur,DB);
	return 0;
}

double toRad(double deg)
{//deg / 180 = x / pi
	return PI / 180 * deg;
}

//程序的初始化代码
void Init(std::vector<Trajectory>& Tra)
{
	//readDataFromSplitFile(Tra);
	readDataFromOneFile(Tra);
}

//读入轨迹数据
void readDataFromSplitFile(std::vector<Trajectory>& Tra)
{
	long long ID;
	int y,m,d;
	int h,Min,s;
	double lat,lon;
	string path;
	ifstream fin(carGPSdataPath.c_str());
	int id = 0;
	while(fin>>path)
	{
		Trajectory tmp;
		ifstream file(path.c_str());
		while(file.getline(buf,2000))
		{
			sscanf_s(buf,"%lld,%d-%d-%d %d:%d:%d,%lf,%lf",&ID,&y,&m,&d,&h,&Min,&s,&lon,&lat);
			tmp.add(Point(getRad(lon),getRad(lat)));
		}
		if((int)tmp.size() < 2)
			continue;
		tmp.ID = id++;
		file.close();
		Tra.push_back(tmp);
	}
	fin.close();
}

void readDataFromOneFile(std::vector<Trajectory>& Tra)
{
	ifstream fin(oneFilePath.c_str());
	
	long long ID,pre;
	int id = 0;
	int y,m,d;
	int h,Min,s;
	double lat,lon;

	fin.getline(buf,2000);
	sscanf_s(buf,"%lld,%d-%d-%d %d:%d:%d,%lf,%lf",&ID,&y,&m,&d,&h,&Min,&s,&lon,&lat);
	
	Trajectory tmp;
	tmp.add(Point(getRad(lon),getRad(lat)));
	pre = ID;

	while(fin.getline(buf,2000))
	{
		sscanf_s(buf,"%lld,%d-%d-%d %d:%d:%d,%lf,%lf",&ID,&y,&m,&d,&h,&Min,&s,&lon,&lat);
		if(pre == ID)
		{
			tmp.add(Point(getRad(lon),getRad(lat)));
		}
		else
		{
			if((int)tmp.size() >= 2)
			{
				tmp.ID = id++;
				Tra.push_back(tmp);
			}
			tmp.clear();
			tmp.add(Point(getRad(lon),getRad(lat)));
			pre = ID;
		}
	}
	if((int)tmp.size() >= 2)
	{
		tmp.ID = id++;
		Tra.push_back(tmp);
	}
	fin.close();
}

//把聚类后的轨迹写入数据库
void writeToDB(Trajectory& Tra,Database* DB,int clusterID)
{
	int sz = Tra.size();
	for(int i=0;i<sz;++i)
	{
		DB->insertPoint(Tra[i],clusterPointID++,clusterID);
	}
	//puts("insert seg");
	DB->insertTra(Tra,clusterSegID++,clusterID);
	/*for(int i=1;i<sz;++i)
	{
		DB->insertSeg(Tra[i-1],Tra[i],clusterSegID++,clusterID);
	}*/
}

//计算seg段以头尾为关键点的代价
double costPar(std::vector<Segment>& seg)
{
	int sz = (int)seg.size();
	Segment LR(seg[0].s,seg[sz-1].e);
	double sumPerp = 0, sumTheta = 0;
	if(sz > 1)
	{
		for(int i=0;i<sz;++i)
		{
			sumPerp += LR.PerpDisTo(seg[i]);
			sumTheta += LR.ThetaDisTo(seg[i]);
		}
		return ( log(0.05*sumPerp)+log(0.05*sumTheta)+log(LR.length()) ) / lg2;
	}
	else
		return log(LR.length()) / lg2;
}

//计算seg段的距离和
double costNoPar(const double* sum,int l,int r)
{
	//此处需要给最后的结果加一个很小的值，具体从论文中找
	static double lg2 = log(2.0);
	return log(sum[r] - sum[l]) / lg2;
}

//返回D集合中编号为id的seg的eps邻居集合
vector <int> N_eps(Cluster& D,int id,myTree* tree)
{
	vector <int> tmp,res;
	const double varepsilon = 0.0005;
	Rect rect(
		min(D[id].s.x,D[id].e.x)-varepsilon,
		min(D[id].s.y,D[id].e.y)-varepsilon,
		min(D[id].s.z,D[id].e.z)-varepsilon,
		max(D[id].s.x,D[id].e.x)+varepsilon,
		max(D[id].s.y,D[id].e.y)+varepsilon,
		max(D[id].s.z,D[id].e.z)+varepsilon
		);
	tree->Search(rect.min,rect.max,tmp);
	int sz=(int)tmp.size();
//#pragma omp parallel for
	for(int i=0;i<sz;++i)
	{
		double dis = D[id].DisTo(D[tmp[i]]);
		if(dis <= EPS)
		{
//#pragma omp critical
			{
				res.push_back(tmp[i]);
			}
		}
	}
	return res;
}

//扩展core segment，用于Micor_clustering
void expandCluster(Cluster& D,std::queue<int>& Q,int clusterId,myTree* tree)
{
	while(!Q.empty())
	{
		int m = Q.front();
		Q.pop();
		vector <int> neiborOfm = N_eps(D,m,tree);
		if((int)neiborOfm.size() >= MinLns)
		{
			for(int i=0,sz=(int)neiborOfm.size();i<sz;++i)
			{
				if(D[neiborOfm[i]].clusterID == UNCLASSIFIED || D[neiborOfm[i]].clusterID == NOISE)
				{
					if(D[neiborOfm[i]].clusterID == UNCLASSIFIED && neiborOfm[i] != m)
						Q.push(neiborOfm[i]);
					D[neiborOfm[i]].clusterID = clusterId;
				}
			}
		}
	}
}

//计算聚类的基，即聚类中有多少seg是属于不同tra的
int PTR(Cluster& C)
{
	map <int,int> mp;
	for(int i=0,sz=(int)C.size();i<sz;++i)
		mp[C[i].TraID] = 1;
	return (int)mp.size();
}

//此函数用于旋转坐标系，修改点集p为新坐标系下的点，XZtheta为绕y轴旋转的角度，YZtheta为绕x轴旋转的角度
void RotateXZ(Trajectory& p,double XZtheta)
{
	int sz = (int)p.size();
	double XZmat[3][3] = {
		{cos(XZtheta) , 0 , -sin(XZtheta)},
		{     0       , 1 ,        0     },
		{sin(XZtheta) , 0 , cos(XZtheta) }
	};
	for(int iter=0;iter<sz;++iter)
	{
		double x[3] = {p[iter].x,p[iter].y,p[iter].z};
		double y[3];
		for(int i=0;i<3;++i)//for row
		{
			double tmp = 0;
			for(int j=0;j<3;++j)//for col
			{
				tmp += x[j]*XZmat[i][j];
			}
			y[i] = tmp;
		}
		p[iter].x = y[0];
		p[iter].y = y[1];
		p[iter].z = y[2];
	}
}
void RotateYZ(Trajectory& p,double YZtheta)
{
	int sz = (int)p.size();
	double YZmat[3][3] = {
		{1 ,       0       ,       0     },
		{0 , cos(YZtheta)  , sin(YZtheta)},
		{0 , -sin(YZtheta) , cos(YZtheta)}
	};
	for(int iter=0;iter<sz;++iter)
	{
		double x[3] = {p[iter].x,p[iter].y,p[iter].z};
		double y[3];
		for(int i=0;i<3;++i)//for row
		{
			double tmp = 0;
			for(int j=0;j<3;++j)//for col
			{
				tmp += x[j]*YZmat[i][j];
			}
			y[i] = tmp;
		}
		p[iter].x = y[0];
		p[iter].y = y[1];
		p[iter].z = y[2];
	}
}
//reference:<Introduction to Algorithms> ch28
//input:initial matrix A, a instance of matrix L,U,P
//output:the pi array
int* LUP_Decomposition(Matrix A,Matrix& L,Matrix& U,Matrix& P)
{
	int n = A.row;
	int *pi = new int[n];
	for(int i=0;i<n;++i)
		pi[i] = i;
	for(int k=0;k<n;++k)
	{
		double p = 0;
		int kk = 0;
		for(int i=k;i<n;++i)
		{
			if(fabs(A[i][k]) > p)
				p = fabs(A[i][k]) , kk=i;
		}
		if(p == 0)
		{
			printf("sigular matrix!\n");
			delete[] pi;
			exit(-1);
		}
		swap(pi[k],pi[kk]);
		for(int i=0;i<n;++i)
			swap(A[k][i],A[kk][i]);
		for(int i=k+1;i<n;++i)
		{
			A[i][k] /= A[k][k];
			for(int j=k+1;j<n;++j)
				A[i][j] -= A[i][k]*A[k][j];
		}
	}
	for(int i=0;i<n;++i)
		P[i][pi[i]] = 1;
	for(int i=0;i<n;++i)
	{
		for(int j=0;j<n;++j)
		{
			if(i > j)
				L[i][j] = A[i][j];
			else
				U[i][j] = A[i][j];
		}
		L[i][i] = 1;
	}
	return pi;
	//delete[] pi;
}

//reference:<Introduction to Algorithms> ch28
//input:a instance of matrix L,U. pi array, equation's answer vector b
//output:the solution vector
Matrix LUP_SolveEquationGroup(const Matrix& L,const Matrix& U,const int* const pi,const Matrix& b)
{
	int n = L.row;
	Matrix x(n,1);
	Matrix y(n,1);
	for(int i=0;i<n;++i)
	{
		y[i][0] = b[pi[i]][0];
		for(int j=0;j<i;++j)
		{
			y[i][0] -= L[i][j]*y[j][0];
		}
	}
	for(int i=n-1;i>=0;--i)
	{
		x[i][0] = y[i][0];
		for(int j=n-1;j>i;--j)
			x[i][0] -= U[i][j]*x[j][0];
		x[i][0] /= U[i][i];
	}
	return x;
}

//reference:<Introduction to Algorithms> ch28
//input:a matrix A
//output:the inverse matrix of A
Matrix getInvMatrix(Matrix A)
{
	int n = A.row;
	Matrix InvA(n,n);
	Matrix L(n,n);
	Matrix U(n,n);
	Matrix P(n,n);
	int *pi = LUP_Decomposition(A,L,U,P);
	for(int i=0;i<n;++i)
	{
		Matrix b(n,1);
		for(int j=0;j<n;++j)
			b[j][0] = i == j?1:0;
		Matrix x = LUP_SolveEquationGroup(L,U,pi,b);
		for(int j=0;j<n;++j)
			InvA[j][i] = x[j][0];
	}
	return InvA;
}

void RotateToPara(Trajectory& p,double XZtheta,double YZtheta,bool isRetrive=false)
{
	XZtheta = toRad(XZtheta);
	YZtheta = toRad(YZtheta);
	double tmp[3][3] = {
		{cos(XZtheta) , sin(XZtheta)*sin(YZtheta)  , -sin(XZtheta)*cos(YZtheta)},
		{     0       ,       cos(YZtheta)		   ,          sin(YZtheta)     },
		{sin(XZtheta) , -sin(YZtheta)*cos(XZtheta) , cos(XZtheta)*cos(YZtheta) }
	};
	Matrix mat(tmp,3,3);
	if(isRetrive)
		mat = getInvMatrix(mat);
	int sz = (int)p.size();
	for(int iter=0;iter<sz;++iter)
	{
		double x[3] = {p[iter].x,p[iter].y,p[iter].z};
		Matrix X(x,3);
		Matrix Y(3,1);
		Y = mat*X;
		p[iter].x = Y[0][0];
		p[iter].y = Y[1][0];
		p[iter].z = Y[2][0];
	}
}

//分解阶段，将轨迹近似化
Trajectory ApproximateTrajectoryPartitioning(Trajectory Tra)
{
	int sz = Tra.size();
	if(sz <= 0) 
	{
		fprintf(stderr,"In Approximate, Trajectory size is 0!\n");
		system("pause");
		exit(-1);
	}
	else if(sz == 1 || sz == 2)
		return Tra;

	/*double *sumLen = new double[sz];
	memset(sumLen,0,sizeof(double)*sz);
	for(int i=1;i<sz;++i)
	{
		sumLen[i] = sumLen[i-1] + Tra[i].EucDisTo(Tra[i-1]);
	}*/

	Trajectory res;
	res.ID = Tra.ID;
	res.add(Tra[0]);
	vector <Segment> seg;
	int startIndex = 0, length = 1;
	while(startIndex + length < sz)
	{
		int curIndex = startIndex + length;
		seg.push_back(Segment(Tra[curIndex-1],Tra[curIndex]));
		double costP = costPar(seg);
		double costN = log(seg[0].s.EucDisTo(seg[seg.size()-1].e))/lg2;//costNoPar(sumLen,startIndex,curIndex);
		if(costP-costN > DOUBLE_EPS)
		{
			if(curIndex != 1)//第一个点不要重复加入
				res.add(Tra[curIndex-1]);
			startIndex = curIndex-1;
			length = 1;
			seg.clear();
		}
		else
			++ length;
	}
	res.add(Tra[sz-1]);
	//delete[] sumLen;
	return res;
}

setOfCluster Macro_Clustering(Cluster& D,myTree* tree)
{
	cerr<<"start Macro_Clustering..."<<endl;
	queue <int> Q;
	int clusterId = 0;
	int sz = (int)D.size();
	//for(int i=0;i<sz;++i)
	//{
	//	D[i].clusterID = UNCLASSIFIED;
	//}
	for(int i=0;i<sz;++i)
	{
		if(i%1000 == 0)
			cerr<<"judge segment "<<i<<endl;
		if(D[i].clusterID == UNCLASSIFIED)
		{
			vector <int> neiborOfi = N_eps(D,i,tree);
			if((int)neiborOfi.size() >= MinLns)
			{
				for(int j=0,jsz=(int)neiborOfi.size();j<jsz;++j)
				{
					D[neiborOfi[j]].clusterID = clusterId;
					if(neiborOfi[j] != i)
						Q.push(neiborOfi[j]);
				}
				expandCluster(D,Q,clusterId,tree);
				++ clusterId;
			}
			else
				D[i].clusterID = NOISE;
		}
	}
	setOfCluster res,tmp;
	tmp.resize(clusterId);
	for(int i=0;i<sz;++i)
	{
		if(D[i].clusterID == UNCLASSIFIED || D[i].clusterID == NOISE) continue;
		tmp[D[i].clusterID].push_back(D[i]);
	}
	for(int i=0;i<clusterId;++i)
	{
		if(PTR(tmp[i]) >= MinLns)
			res.push_back(tmp[i]);
	}
	cerr<<"Macro_Clustering done!"<<endl;
	return res;
}

Trajectory RepresentativeTrajectoryGeneration(Cluster cluster)
{
	Trajectory res;
	Vector v;
	int sz = (int)cluster.size();

	Trajectory p;
	//p.setSize(sz*2+2);
	
	for(int i=0;i<sz;++i)
	{
		v = v + (cluster[i].e - cluster[i].s);
		p.add(cluster[i].s);
		p.add(cluster[i].e);
	}
	v = v / sz;
	double XZtheta = atan(v.x/v.z);
	double YZtheta = atan(v.y/v.z);
	//RotateXZ(p,-XZtheta);
	//RotateYZ(p,-YZtheta);
	//int IDDD = cluster[0].clusterID;
	RotateToPara(p,XZtheta,-YZtheta);
	sz = p.size();
	Point v1 = Point( 0 , 0 ,-10);
	Point v2 = Point(-10, 0 ,-10);
	Point v3 = Point( 0 ,-10,-10);
	double paraA,paraB,paraC,paraD;
	get_panel(v1,v2,v3,paraA,paraB,paraC,paraD);
	Vector n = get_Normal(v1,v2,v3);
	SweepPlanePoint* allPoint = new SweepPlanePoint[sz];
	int pid = 0;
	for(int i=0;i<sz;++i)
	{
		allPoint[i] = SweepPlanePoint(p[i],pid,dis_pt2panel(p[i],paraA,paraB,paraC,paraD));
		if(i&1)
			++ pid;
	}
	
	sort(allPoint,allPoint+sz);
	map<int,int> mp;
	map<int,int>::iterator it;
	for(int i=0;i<sz;++i)
	{
		bool isStart = false;
		if(( it = mp.find(allPoint[i].SegId) ) == mp.end())//开始点
		{
			isStart = true;
			mp[allPoint[i].SegId] = 1;
		}
		if(mp.size() > MinLns)
		{
			//平移v1,v2,v3，得到新平面三点
			Vector newV1 = v1 + (allPoint[i] - v1);
			Vector newV2 = v2 + (allPoint[i] - v1);
			Vector newV3 = v3 + (allPoint[i] - v1);

			Vector n = get_Normal(newV1,newV2,newV3);

			Point vmean;
			for(map<int,int>::iterator k=mp.begin();k!=mp.end();++k)
			{
				vmean = vmean + CalPlaneLineIntersectPoint(
					n,
					newV1,
					Vector(p[k->first]-p[(k->first)^1]),
					p[k->first]);
			}
			vmean = vmean / (int)mp.size();
			if(res.size() == 0) res.add(vmean);
			else if(vmean.GeoDisTo(res[res.size()-1]) > gamma 
				&& vmean.GeoDisTo(res[res.size()-1]) < 10*gamma)//过滤不合理的
				res.add(vmean);
		}
		if(!isStart)
			mp.erase(it);
	}
	delete[] allPoint;
	RotateToPara(res,XZtheta,-YZtheta,true);
	/*for(int i=0,sz=(int)res.size();i<sz;++i)
	{
		if(getDeg(getLat(res[i])) > 40.1 || getDeg(getLon(res[i])) > 116.7)
		{
			system("pause");
		}
	}*/
	return res;
}


//轨迹分割阶段
void PartitionPhase(std::vector <Trajectory>& Tra)
{
	int sz = (int)Tra.size();
//#pragma omp parallel for
	for(int i=0;i<sz;++i)
	{
		Tra[i] = ApproximateTrajectoryPartitioning(Tra[i]);
		fprintf(stderr,"Trajectory %d done!\n",i);
	}
}

//聚类阶段
setOfCluster GroupPhase(std::vector <Trajectory>& Tra)
{
	myTree* tree = new myTree();
	int cnt = 0;
	//Micro_Clustering(Tra);
	Cluster cur;
	for(int i=0,isz=(int)Tra.size();i<isz;++i)
	{
		for(int j=1,jsz=(int)Tra[i].size();j<jsz;++j)
		{
			cur.push_back(ClusterSegment(i,UNCLASSIFIED,Tra[i][j-1],Tra[i][j]));
			Rect rect(min(Tra[i][j-1].x,Tra[i][j].x),
				min(Tra[i][j-1].y,Tra[i][j].y),
				min(Tra[i][j-1].z,Tra[i][j].z),
				max(Tra[i][j-1].x,Tra[i][j].x),
				max(Tra[i][j-1].y,Tra[i][j].y),
				max(Tra[i][j-1].z,Tra[i][j].z));
			tree->Insert(rect.min,rect.max,cnt++);
		}
	}
	return Macro_Clustering(cur,tree);
}

//展示阶段，将聚类结果转换为轨迹并写入数据库
void DisplayPhase(setOfCluster& clusters,Database* DB)
{
	int sz = (int)clusters.size();
	//cout<<sz<<endl;
	for(int i=0;i<sz;++i)
	{
		cerr<<"start display trajectory "<<i<<"..."<<endl;
		Trajectory Tra = RepresentativeTrajectoryGeneration(clusters[i]);
		writeToDB(Tra,DB,i);
	}
}