#pragma once

#include <iostream>
#include "workbench.h"
#include "parameter.h"
#include "moveControl.h"
#include "h_globalContext.h"
// 引入M_PI等常数
#define _USE_MATH_DEFINES
#include <math.h>
using namespace std;

/// <summary>
/// botId: bot的id [0,3]
/// workbenchType: 所处工作台id
/// </summary>
class hwRobot_2 {
public:

	int botId = 0;

	/// <summary>
	/// 所处工作台id
	/// -1表示不在任何工作台附近
	/// [0,工作台总数-1] 表示在对应工作台附近将, 当前机器人所有购买、出售行为均针对该工作台进行
	/// </summary>
	int workbenchId = -1;

	/// <summary>
	/// 0表示未携带
	/// 1-7表示携带物品id
	/// </summary>
	int gootsId = 0;
	/// <summary>
	/// 时间价值系数
	/// </summary>
	double tvr = 1;
	/// <summary>
	/// 碰撞价值系数
	/// </summary>
	double cvr = 1;

	// 货物当前的价值
	double curValue = 0;
	
	double angleSpeed = 0;
	double lineSpeed[2] = { 0,0 };
	/// <summary>
	/// 当前朝向
	/// </summary>
	double faceTo = 0;

	double x;
	double y;

	// 通过设置期望速度和期望角速度来控制bot, 
	// 设置完后并不会马上cout控制语句, 
	// 而是在updateAfterHandle中执行
	// 期望角速度
	double expectAngleSpeed = 0;
	// 期望速度
	double expectSpeed = 0;

	// 是否修改过期望角速度
	bool ifChangeEAS = false;
	// 是否修改过期望速度
	bool ifChangeES = false;

	moveControl* mc = nullptr;

	BOT_MOVE_STATE state;

	// 机器人的堵塞检测缓冲区
	lattice jamDetectBuffer[6];
	// 机器人的堵塞检测缓冲区的PathId
	int jamDetectBufferPathId[6];
	// 是否位于狭窄区域的标识
	bool inNarrowZone = false;

	// 避让状态
	BOT_AVOID_STATE avoidState;
	// 记录下需要避让的对方机器人的编号（只记录优先级最高的机器人）
	int avoidBotId;
	// 记录下机器人当前的目标工作台Id
	int curTarWbId;
	// 记录卡死时间
	int stickTime;

	// 历史平均速度
	double vHistory;

	/// <summary>
	/// [-2, 6] 之间的浮点数, 表示前进速度
	/// </summary>
	/// <param name="speed"></param>
	void forward(double speed) {
		expectSpeed = speed;
		ifChangeES = true;
	}

	// 打印前进命令
	void printForwardCMD() {
		cout << "forward " << botId << ' ' << expectSpeed << endl;
	}

	/// <summary>
	/// [-π,π]  设置旋转速度, 单位弧度每秒
	/// </summary>
	/// <param name="angle"></param>
	void rotate(double angle) {
		expectAngleSpeed = angle;
		ifChangeEAS = true;
	}

	// 打印旋转命令
	void printRotateCMD() {
		cout << "rotate " << botId << ' ' << expectAngleSpeed << endl;
	}

	/// <summary>
	/// 注意, 调用该函数后, 并不会自动更新goodsId, 需要等到下一帧更新, 或用户自行修改 bot.goodsId
	/// </summary>
	void buy()const {
		cout << "buy " << botId << endl;
	}
	/// <summary>
	/// 注意, 调用该函数后, 并不会自动更新goodsId, 需要等到下一帧更新, 或用户自行修改 bot.goodsId
	/// </summary>
	void sell()const {
		cout << "sell " << botId << endl;
	}
	/// <summary>
	/// 注意, 调用该函数后, 并不会自动更新goodsId, 需要等到下一帧更新, 或用户自行修改 bot.goodsId
	/// </summary>
	void destroy()const {
		cout << "destroy " << botId << endl;
	}

	double getAngleBetween2Point(double p1_x, double p1_y, double p2_x, double p2_y)const {
		double dx = p2_x - p1_x;
		double dy = p2_y - p1_y;
		double angleBetweenWbAndZero = atan2(dy, dx);
		double ans = angleBetweenWbAndZero - this->faceTo;
		return ans > M_PI ? ans - 2 * M_PI : (ans <= -M_PI ? ans + 2 * M_PI : ans);
	}

	double getAngleBetween2PointAndWorkbench(double p1_x, double p1_y, double p2_x, double p2_y)const {
		double dx = p2_x - p1_x;
		double dy = p2_y - p1_y;
		double angleBetweenWbAndZero = atan2(dy, dx);
		double ans = angleBetweenWbAndZero - this->faceTo;
		return ans > M_PI ? ans - 2 * M_PI : (ans <= -M_PI ? ans + 2 * M_PI : ans);
	}

	/// <summary>
	/// 获取与对应工作台之间夹角, (-pi,pi], 正数逆时针,负数顺时针(与赛题保持一致
	/// </summary>
	/// <returns>返回值属于[-pi,pi]</returns>
	const double getAngleBetweenThisBotAndWorkbench(const hwWorkbench& wb)const {
		double dx = wb.x - this->x;
		double dy = wb.y - this->y;
		double angleBetweenWbAndZero = atan2(dy, dx);
		double ans = angleBetweenWbAndZero - this->faceTo;
		return ans > M_PI ? ans - 2 * M_PI : (ans <= -M_PI ? ans + 2 * M_PI : ans);
	}

	/// <summary>
	/// 获取与对应点之间夹角, (-pi,pi], 正数逆时针,负数顺时针(与赛题保持一致
	/// </summary>
	/// <returns>返回值属于[-pi,pi]</returns>
	const double getAngleBetweenThisBotAndWorkbench(const double _x, const double _y)const {
		double dx = _x - this->x;
		double dy = _y - this->y;
		double angleBetweenWbAndZero = atan2(dy, dx);
		double ans = angleBetweenWbAndZero - this->faceTo;
		return ans > M_PI ? ans - 2 * M_PI : (ans <= -M_PI ? ans + 2 * M_PI : ans);
	}

	void init(int _botId, int _x, int _y) {
		botId = _botId;
		x = _x * 0.5 + 0.25;
		y = _y * 0.5 + 0.25;
		inNarrowZone = false;
		avoidState = NOTAVOID;
		curTarWbId = -1;
		avoidBotId = -1;
		stickTime = 0;
		vHistory = 0;
	}

	void updateBeforeHandle() {
		cin >> workbenchId >> gootsId >>
			tvr >> cvr >> angleSpeed >>
			lineSpeed[0] >> lineSpeed[1] >>
			faceTo >> x >> y;

		this->expectAngleSpeed = 0;
		this->expectSpeed = 0;
		this->curValue = H_GlobalContext.OriSellPrice[gootsId] * tvr * cvr;
		
	}

	void updateAfterHandle() {
		if (ifChangeEAS) {
			ifChangeEAS = false;
			printRotateCMD();
		}
		if (ifChangeES) {
			ifChangeES = false;
			printForwardCMD();
		}
	}

};
