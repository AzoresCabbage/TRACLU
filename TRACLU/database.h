/*
PQntuples Returns the number of tuples (instances) in the query result.
PQnfields Returns the number of fields (attributes) in each tuple of the query result.
PQfname Returns the field (attribute) name associated with the given field index. Field indices start at 0.
PQgetvalue Returns a single field (attribute) value of one tuple of a PGresult. Tuple and field indices start at 0.
PQprint Prints out all the tuples and, optionally, the attribute names to the specified output stream.
PQclear Frees the storage associated with the PGresult. Every query result should be freed via PQclear when it is no longer needed.
*/
#pragma once
#include "libpq-fe.h"
#include <iostream>
#include <string>
#include "geometry.h"
using namespace std;

const string POINT_TABLE_NAME = "clusterpoint";
const string SEG_TABLE_NAME = "clusterseg";

class Database{
private:
	string connInfo;//Á¬½Ó×Ö
	PGconn* conn;
	void closeConn();
	bool connDB();
	PGresult* execQuery(string SQL);
	bool execUpdate(char* SQL);
	void clearOrBuildTable(string tbname,string geom_type);
public:
	Database();
	~Database(){closeConn();}
	void insertPoint(Point p,int id,int clusterID);
	void insertSeg(Point a,Point b,int id,int clusterID);
	void insertTra(Trajectory Tra,int id,int clusterID);
};