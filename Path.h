#pragma once
#include<vector>
#include<iostream>
#include <algorithm>
#include <cmath>
using namespace std;
class HWCout;

struct Point
{
	Point(double _x,double _y):x(_x),y(_y){}
	Point():x(0),y(0){}
	Point(const Point& point) {
		x = point.x;
		y = point.y;
	}
	double x, y;

	friend std::ostream& operator<<(std::ostream& os, const Point& p);
	friend HWCout& operator<<(HWCout& os, const Point& p);
};

class lattice {
public:
	int x, y;
	lattice(double p_x, double p_y) {
		x = round(2 * p_x - 0.5);
		y = round(2 * p_y - 0.5);
	}
	lattice(int idx, int idy) {
		x = idx;
		y = idy;
	}
	lattice() {}
};

// 定义一条路径为 多个点组成的序列
// 待添加成员函数
class Path {
public:
	vector<Point> pointList;

	Path(){}

	Path(const Path& cp) {
		this->pointList = cp.pointList;
	}

	void push_back(Point& p) {
		this->pointList.emplace_back(p);
	}

	void push_back(double x, double y) {
		this->pointList.emplace_back(x, y);
	}

	int size() const {
		return pointList.size();
	}
	Point& operator [](int i) {
		if (i < size()) {
			return pointList[i];
		}
		else {
			throw "error: Path 访问越界";
		}
		Point p;
		return p;
	}
	/// <summary>
	/// 反转当前路径, 并返回当前路径
	/// </summary>
	/// <returns></returns>
	Path& reverse() {
		std::reverse(pointList.begin(), pointList.end());
		return *this;
	}

	friend std::ostream& operator<<(std::ostream& os, const Path& p);
	friend HWCout& operator<<(HWCout& os, Path& p);
};

