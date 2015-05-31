#pragma once
#include <stdio.h>
#include <iostream>

class Matrix{
public:
	int row,col;
	double** mat;
	Matrix(const int n,const int m);
	Matrix(const Matrix& a);
	Matrix(double a[3][3],int n,int m);
	Matrix(double a[3],int n);
	Matrix& operator = (const Matrix& a);
	
	~Matrix();

	Matrix transpose();
	void print();

	double* const operator [] (const int& idx)const;

	Matrix operator + (const Matrix& a)const;
	Matrix operator - (const Matrix& a)const;
	Matrix operator - (const double a)const;
	Matrix operator * (const Matrix& a)const;
	Matrix operator * (const double a)const;
	bool operator == (const Matrix& a)const;
	bool operator != (const Matrix& a)const;
};
