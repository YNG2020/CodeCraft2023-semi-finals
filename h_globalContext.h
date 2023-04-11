#pragma once
#include<vector>
using namespace std;

class hwRobot_2;
class hwWorkbench;
class basePathPlanning;

extern const double calDist(const double x1, const double y1, const double x2, const double y2);

extern const double calDistSq(const double x1, const double y1, const double x2, const double y2);

extern void getLatticeIndex(double x, double y, int& idx, int& idy);

// 满足全局使用的要求, 可以自行添加成员
class h_globalContext {
public:
	h_globalContext();

	int wbNum = 0;
	int frameId = 0;
	int money = 200000;
	hwRobot_2* botArr;
	hwWorkbench* wbArr;
	vector<vector<char>> gameMap{ 100,vector<char>(100, '.')};

	basePathPlanning* pathPlan;
	int OriSellPrice[8] = { 0,6000,7600,9200,22500,25000,27500,105000 };
	int jamBufferSize;

};

extern h_globalContext H_GlobalContext;
