#pragma once
#include"Path.h"
#include"parameter.h"
#include"h_globalContext.h"

class hwRobot_2;

class moveControl {
public:
	moveControl(hwRobot_2& _bot):bot(_bot) {}

	// 所属bot
	hwRobot_2& bot;

	// 当前状态
	MOVE_CONTROL_STATE state = MC_NO_PATH;

	// 当前路径, 当state为MC_NO_PATH时无效
	Path nowPath;

	Point nowTarPoint;

	Point finalTarPoint;

	// 当前目标点为当前原始路径的第几个点
	int nowPointId = 0;

	// 当前机器人位置处于当前原始路径的第几个点
	int nowTruePointId = 0;

	// 设置移动路径, 设置成虚函数是为了方便不同算法对路径进行处理(平滑/减少关键点等)
	virtual void setPath(const Path& p) = 0;

	MOVE_CONTROL_STATE getNowState() const {
		return state;
	}

	// 即原moveUtillTarget
	virtual MOVE_CONTROL_STATE move() = 0;

};