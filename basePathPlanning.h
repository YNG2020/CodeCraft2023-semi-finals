#pragma once
#include <vector>
#include "Path.h"
#include "workbench.h"
using namespace std;
// 路径规划基类
class basePathPlanning {
public:
	// 从i号工作台到j号工作台的路径（带货品）
	vector<vector<Path>> Path_Wbi2Wbj_HaveProd;
	// 从i号工作台到j号工作台的路径（不带货品）
	vector<vector<Path>> Path_Wbi2Wbj_NoProd;
	// 从i号机器人到j号工作台的路径，仅初始化使用
	vector<vector<Path>> Path_Boti2Wbj;

	bool fesible_lattice_haveProd[100][100] = { false }; // 搭载货物时的可行格子标记（暂不考虑路径封闭，这个用别的途径解决）
	bool fesible_lattice_noProd[100][100] = { false };   // 未搭载货物时的可行格子标记（暂不考虑路径封闭，这个用别的途径解决）
	bool block_lattice[100][100] = { false };

	// 可行格子坐标（(0,0)表示不可行）
	vector<vector<pair<double, double>>> fesible_point
	{ 100, vector<pair<double, double>>(100, pair<double, double>(0, 0)) };

	// 可行格子坐标（(0,0)表示不可行）
	vector<vector<Point>> fesible_point_haveProd{ 100, vector<Point>(100) };
	// 可行格子坐标（(0,0)表示不可行）
	vector<vector<Point>> fesible_point_noProd{ 100, vector<Point>(100) };


	// 在main中调用, 传入地图数组的引用, 用来计算路径; 
	virtual void initPaths(vector<vector<char>>& h_map) = 0;

	// 获得从 fromWb 到 toWb的路径
	virtual Path getPath(const hwWorkbench& fromWb, const hwWorkbench& toWb, bool haveProd) = 0;

	// 获得从 fromPoint 到 toWb的路径
	virtual Path getPath(const Point& fromPoint, const hwWorkbench& toWb, bool haveProd) = 0;

	// 获得从 fromPoint 到 toPoint的路径
	virtual Path getPath(const Point& fromPoint, const Point& toPoint, bool haveProd) = 0;

	// 获得从 fromWb 到 toPoint的路径
	virtual Path getPath(const hwWorkbench& fromWb, const Point& toPoint) = 0;

	// 获得从 fromBot 到 toWb的路径
	virtual Path getPath(const hwRobot_2& fromBot, const hwWorkbench& toWb) = 0;
};