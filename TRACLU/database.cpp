#include "database.h"
using namespace std;

Database::Database(/*string dbname,string port,string dbaddr*/):conn(NULL){
	std::string dbname = "osm";//数据库名称
	std::string port = "5432";//数据库端口号
	std::string dbaddr = "127.0.0.1";//数据库地址
	char buff[500];
	sprintf_s(buff,"port = '%s' dbname = '%s' hostaddr = '%s' ",port.c_str(),dbname.c_str(),dbaddr.c_str());
	connInfo = buff;
	if(!connDB()){
		exit(0);
	}
	clearOrBuildTable(POINT_TABLE_NAME,"Point");
	clearOrBuildTable(SEG_TABLE_NAME,"LineString");
}

bool Database::connDB(){
	std::string tmp,username,password;
	
	username = "postgres";
	password = "wyjcool";
	
	char buff[500];
	sprintf_s(buff,"user = '%s' password = '%s' ",username.c_str(),password.c_str());
	connInfo = connInfo + buff;
	
	conn = PQconnectdb(connInfo.c_str());

	if(PQstatus(conn) == CONNECTION_BAD){
		std::cerr<<"Database connection failed!"<<std::endl;
		return false;
	}
	else{
		std::cerr<<"Database connection success!"<<std::endl;
		return true;
	}
}

void Database::clearOrBuildTable(string tbname,string geom_type)
{
	char SQL[200];
	memset(SQL,0,sizeof(SQL));
	sprintf_s(SQL,"select * from pg_class where relname = '%s'",tbname.c_str());
	PGresult* res = execQuery(SQL);
	int num = PQntuples(res);
	PQclear(res);

	if(num != 0){
		memset(SQL,0,sizeof(SQL));
		sprintf_s(SQL,"Delete from %s",tbname.c_str());
		execUpdate(SQL);
	}
	else{
		memset(SQL,0,sizeof(SQL));
		sprintf_s(SQL,"create table %s (id integer primary key,clusterID integer,way geometry(%s,4326))",tbname.c_str(),geom_type.c_str());
		execUpdate(SQL);
	}
}

void Database::closeConn(){
	PQfinish(conn);
}

PGresult* Database::execQuery(std::string SQL){
	PGresult* res = PQexec(conn,SQL.c_str());
	if(PQresultStatus(res) != PGRES_TUPLES_OK){
		std::cerr<<"SQL query error! statement:"<<SQL<<std::endl;
		PQclear(res);
		//closeConn(conn);
		return NULL;
	}
	return res;
}

bool Database::execUpdate(char* SQL){
	PGresult* res = PQexec(conn,SQL);
	if(PQresultStatus(res) != PGRES_COMMAND_OK){
		//std::ofstream fout("output.txt");
		std::cerr<<"SQL update error! statement:"<<SQL<<std::endl;
		//fout<<SQL<<std::endl;
		//fout.close();
		PQclear(res);
		//closeConn(conn);
		return false;
	}
	return true;
}

void Database::insertPoint(Point p,int id,int clusterID)
{
	char buff[300];
	memset(buff,0,sizeof(buff));
	sprintf_s(buff,"insert into %s values(%d,%d,ST_GeomFromText('Point(%lf %lf)',4326))",POINT_TABLE_NAME.c_str(),id,clusterID,getDeg(getLon(p)),getDeg(getLat(p)));
	execUpdate(buff);
}

void Database::insertSeg(Point a,Point b,int id,int clusterID)
{
	char buff[300];
	memset(buff,0,sizeof(buff));
	sprintf_s(buff,"insert into %s values(%d,%d,ST_GeomFromText('LineString(%lf %lf,%lf %lf)',4326))",
		SEG_TABLE_NAME.c_str(),
		id,
		clusterID,
		getDeg(getLon(a)),
		getDeg(getLat(a)),
		getDeg(getLon(b)),
		getDeg(getLat(b))
		);

	execUpdate(buff);
}

void Database::insertTra(Trajectory Tra,int id,int clusterID)
{
	const int buffSize = 5000;
	char buff[buffSize];
	memset(buff,0,sizeof(buff));
	int sz = Tra.size();
	if(sz < 2)
		return;

	sprintf_s(buff,"insert into %s values(%d,%d,ST_GeomFromText('LineString(%lf %lf",
		SEG_TABLE_NAME.c_str(),
		id,
		clusterID,
		getDeg(getLon( Tra[0] )),
		getDeg(getLat( Tra[0] ))
		);
	for(int i=1;i<sz;++i)
	{
		sprintf_s(buff+strlen(buff),buffSize-strlen(buff),",%lf %lf",getDeg(getLon( Tra[i] )),getDeg(getLat( Tra[i] )));
	}
	sprintf_s(buff+strlen(buff),buffSize-strlen(buff),")',4326))");
	//cout<<buff<<endl;
	//system("pause");
	execUpdate(buff);
}