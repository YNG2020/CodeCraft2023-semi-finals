#pragma once
#include <vector>
#include "Path.h"
#include "workbench.h"
using namespace std;
// ·���滮����
class basePathPlanning {
public:
	// ��i�Ź���̨��j�Ź���̨��·��������Ʒ��
	vector<vector<Path>> Path_Wbi2Wbj_HaveProd;
	// ��i�Ź���̨��j�Ź���̨��·����������Ʒ��
	vector<vector<Path>> Path_Wbi2Wbj_NoProd;
	// ��i�Ż����˵�j�Ź���̨��·��������ʼ��ʹ��
	vector<vector<Path>> Path_Boti2Wbj;

	bool fesible_lattice_haveProd[100][100] = { false }; // ���ػ���ʱ�Ŀ��и��ӱ�ǣ��ݲ�����·����գ�����ñ��;�������
	bool fesible_lattice_noProd[100][100] = { false };   // δ���ػ���ʱ�Ŀ��и��ӱ�ǣ��ݲ�����·����գ�����ñ��;�������
	bool block_lattice[100][100] = { false };

	// ���и������꣨(0,0)��ʾ�����У�
	vector<vector<pair<double, double>>> fesible_point
	{ 100, vector<pair<double, double>>(100, pair<double, double>(0, 0)) };

	// ���и������꣨(0,0)��ʾ�����У�
	vector<vector<Point>> fesible_point_haveProd{ 100, vector<Point>(100) };
	// ���и������꣨(0,0)��ʾ�����У�
	vector<vector<Point>> fesible_point_noProd{ 100, vector<Point>(100) };


	// ��main�е���, �����ͼ���������, ��������·��; 
	virtual void initPaths(vector<vector<char>>& h_map) = 0;

	// ��ô� fromWb �� toWb��·��
	virtual Path getPath(const hwWorkbench& fromWb, const hwWorkbench& toWb, bool haveProd) = 0;

	// ��ô� fromPoint �� toWb��·��
	virtual Path getPath(const Point& fromPoint, const hwWorkbench& toWb, bool haveProd) = 0;

	// ��ô� fromPoint �� toPoint��·��
	virtual Path getPath(const Point& fromPoint, const Point& toPoint, bool haveProd) = 0;

	// ��ô� fromWb �� toPoint��·��
	virtual Path getPath(const hwWorkbench& fromWb, const Point& toPoint) = 0;

	// ��ô� fromBot �� toWb��·��
	virtual Path getPath(const hwRobot_2& fromBot, const hwWorkbench& toWb) = 0;
};