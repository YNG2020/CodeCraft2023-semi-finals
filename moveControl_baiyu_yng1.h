#pragma once
#include "moveControl.h"
#include "parameter.h"
#include "basePathPlanning.h"
#include <queue>
using namespace std;

class moveControl_baiyu_yng1 :public moveControl {
public:

	// 存储当前路径的平滑版本的id的数组
	vector<int> nowPathSmoothId;
	// 行驶到当前平滑路径的第几个点;
	int nowPointSmoothId = 0;
	// 抵达平滑路径转折点附近的标识
	bool arriveCorner = false;

	moveControl_baiyu_yng1(hwRobot_2& _bot) : moveControl(_bot) {}
	
	/// <summary>
	/// 设置目标点
	/// </summary>
	/// <param name="_x"></param>
	/// <param name="_y"></param>
	/// <returns></returns>
	bool setMoveTarget(Point& p) {
		state = MC_ON_THE_WAY;
		nowTarPoint = p;
		return true;
	}

	double getMaxSpeed1() {

		double dis = sqrt(pow(this->bot.x - nowTarPoint.x, 2) + pow(this->bot.y - nowTarPoint.y, 2));

		static int outNum = -1;

		// 与朝向相切, 且目标点在其上的圆弧的弧度;
		double rad = bot.getAngleBetweenThisBotAndWorkbench(nowTarPoint.x, nowTarPoint.y);

		const double offset = f1_offset;

		// 旋转死区外
		// todo: 当帧率跑不满时,优化重复计算
		if (dis > M_2_PI * 6 + offset) {
			return 6;
		}

		// 前方矩形区域, 无需减速
		if (dis * abs(sin(rad)) < offset) {
			return 6;
		}

		// 目标在朝向两侧分别讨论
		double offset_x, offset_y;
		if (rad > 0) {
			offset_x = offset * cos(this->bot.faceTo + M_PI_2);
			offset_y = offset * sin(this->bot.faceTo + M_PI_2);
		}
		else {
			offset_x = offset * cos(this->bot.faceTo - M_PI_2);
			offset_y = offset * sin(this->bot.faceTo - M_PI_2);
		}

		dis = sqrt(pow(this->bot.x + offset_x - nowTarPoint.x, 2) + pow(this->bot.y + offset_y - nowTarPoint.y, 2));

		double rad2 = bot.getAngleBetween2PointAndWorkbench(this->bot.x + offset_x, this->bot.y + offset_y, nowTarPoint.x, nowTarPoint.y);

		double minSpeed = dis / 2 / abs(sin(rad2)) * f1_getMaxSpeed;
		if (outNum != 1) {
			outNum = 2;
		}
		return minSpeed;
	}

	void move_yng1() {
		double dAngle = bot.getAngleBetweenThisBotAndWorkbench(nowTarPoint.x, nowTarPoint.y);

		double ratio = 1.0;
		double omega;
		double omegaRatio;
		if (arriveCorner) {
			omegaRatio = 0.02;
		}
		else {
			omegaRatio = 0.02;
		}

		if (dAngle < 0) {
			dAngle = dAngle * ratio < -M_PI ? -M_PI : dAngle * ratio;
			omega = dAngle / (omegaRatio);
			omega = omega < -M_PI ? -M_PI : omega;
		}
		else {
			dAngle = dAngle * ratio > M_PI ? M_PI : dAngle * ratio;
			omega = dAngle / (omegaRatio);
			omega = omega > M_PI ? M_PI : omega;
		}

		this->bot.rotate(omega);

		double minSpeed;
		double maxSpeed;
		if (this->bot.inNarrowZone)
			maxSpeed = 5;
		else
			maxSpeed = 6;

		if (abs(dAngle) > M_PI_2) {
			minSpeed = -2;
		}
		else {
			minSpeed = getMaxSpeed1();
			if (arriveCorner) {
				minSpeed = minSpeed / 1.1;
				maxSpeed = maxSpeed / 1.1;
			}			
		}
		if (minSpeed < maxSpeed) {
			this->bot.forward(minSpeed);
		}
		else {
			this->bot.forward(maxSpeed);
		}

	}

	/// <summary>
	/// 在handleframe中调用, 当你设置target后, 每帧自动控制bot前往target
	/// </summary>
	/// <returns></returns>
	MOVE_CONTROL_STATE moveUntillArriveTarget_YNG1() {

		double botX = this->bot.x, botY = this->bot.y;

		for (int i = 0; i < 4; ++i) {	// 遍历除自身外的三个机器人
			if (i == this->bot.botId)	// 自己跟自己无须进行预期堵塞检测
				continue;
			if (H_GlobalContext.botArr[i].inNarrowZone == false && 
				this->bot.inNarrowZone == false) {	// 如果双方都在空旷区域，无须进行防堵塞处理
				continue;
			}
			if (this->bot.gootsId > 0 && H_GlobalContext.botArr[i].gootsId > 0) {
				if (this->bot.curValue > H_GlobalContext.botArr[i].curValue) {	// 自己的货物价值比较高，自己不进行让路处理
					continue;
				}
				if (this->bot.curValue == H_GlobalContext.botArr[i].curValue && this->bot.botId > i)
					continue;
				if (this->bot.avoidState != NOTAVOID) {		// 自己本身已经有要避让的对象
					if (H_GlobalContext.botArr[this->bot.avoidBotId].curValue > H_GlobalContext.botArr[i].curValue)
						continue;	// 如果自己要避让的优先级比这新的要避让的机器人的优先级高，则不对新的机器人做避让处理
					if (H_GlobalContext.botArr[this->bot.avoidBotId].curValue == H_GlobalContext.botArr[i].curValue
						&& this->bot.avoidBotId > i)
						continue;	// 如果自己要避让的优先级比这新的要避让的机器人的优先级高，则不对新的机器人做避让处理
				}
				jamDetect(i, 1);
			}
			else if (this->bot.gootsId > 0 && H_GlobalContext.botArr[i].gootsId == 0) {	// 自己有货物，别人没有货物，自己不进行让路处理
				continue;
			}
			else if (this->bot.gootsId == 0 && H_GlobalContext.botArr[i].gootsId > 0) {
				if (this->bot.avoidState != NOTAVOID) {		// 自己本身已经有要避让的对象
					if (H_GlobalContext.botArr[this->bot.avoidBotId].curValue > H_GlobalContext.botArr[i].curValue)
						continue;	// 如果自己要避让的优先级比这新的要避让的机器人的优先级高，则不对新的机器人做避让处理
					if (H_GlobalContext.botArr[this->bot.avoidBotId].curValue == H_GlobalContext.botArr[i].curValue
						&& this->bot.avoidBotId > i)
						continue;	// 如果自己要避让的优先级比这新的要避让的机器人的优先级高，则不对新的机器人做避让处理
				}
				jamDetect(i, 1);
			}
			else {	// 两个机器人都没有货物
				if (this->bot.botId > i) {	// 自己的id比较大，自己不进行让路处理
					continue;
				}
				if (this->bot.avoidState != NOTAVOID) {		// 自己本身已经有要避让的对象
					if (this->bot.avoidBotId > i || H_GlobalContext.botArr[this->bot.avoidBotId].gootsId > 0)
						continue;	// 如果自己要避让的优先级比这新的要避让的机器人的优先级高，则不对新的机器人做避让处理
				}
				jamDetect(i, 1);
			}
		}

		MOVE_CONTROL_STATE res = MC_ON_THE_WAY;

		unJamDetect();

		if (abs(botX - nowTarPoint.x) <= 0.4 && abs(botY - nowTarPoint.y) <= 0.4) {
			// 0.4的平方
			if (calDistSq(botX, botY, nowTarPoint.x, nowTarPoint.y) < 0.16) {

				this->state = MC_ARRIVE_POINT;

				if (nowPointId == nowPath.size() - 1) {
					unJamDetect1();
					this->state = MC_COMPLETION_PATH;
					if (this->bot.workbenchId != -1 && (this->bot.workbenchId == this->bot.curTarWbId)) {
						this->bot.state = BOT_MOVE_STATE::HW_ARRIVE_TAR1;
					}

					if (this->bot.avoidState == AVOIDING) {
						this->bot.forward(0);
						this->bot.avoidState = AVOIDED;
						bool ifHaveProd = this->bot.gootsId > 0;
						this->bot.mc->finalTarPoint = Point(H_GlobalContext.wbArr[this->bot.curTarWbId].x, 
							H_GlobalContext.wbArr[this->bot.curTarWbId].y);
						this->bot.mc->setPath(H_GlobalContext.pathPlan->
							getPath(Point(botX, botY), H_GlobalContext.wbArr[this->bot.curTarWbId], ifHaveProd));
					}
					if (this->bot.avoidState == AVOIDED)
						this->bot.forward(0);
						
					return MC_COMPLETION_PATH;
				}
				else {
					// 目标切换为下一个路径点
					++nowPointSmoothId;
					nowPointId = nowPathSmoothId[nowPointSmoothId];
					nowTarPoint = nowPath[nowPointId];
				}
				res = MC_ARRIVE_POINT;
			}
		}

		setMoveTarget(nowPath[nowPointId]);

		if (nowPointId != 0 && nowPointId != nowPath.size() - 1 &&	// 一前一后不减速
			nowTruePointId > nowPointId - 2) {	// 剩下两点的时候减速
			arriveCorner = true;
		}
		if (bot.avoidState != AVOIDED)
			move_yng1();
			
		return res;
	}

	void setPath(const Path& p) {
		nowPath = p;
		this->nowPointId = 0;
		this->nowPointSmoothId = 0;
		this->nowTruePointId = 0;
		this->arriveCorner = false;
		this->finalTarPoint = p.pointList[p.pointList.size() - 1];

		vector<int> tmp1;

		double rHaveProd = 0.53, rNoProd = 0.45;

		bool haveProd = this->bot.gootsId > 0;
		if (haveProd) {
			tmp1 = pathSmooth(p, rHaveProd);		// 一阶平滑
			tmp1 = pathSmooth(p, rHaveProd, tmp1);	// 二阶平滑
		}
		else {
			tmp1 = pathSmooth(p, rNoProd);			// 一阶平滑
			tmp1 = pathSmooth(p, rNoProd, tmp1);	// 二阶平滑
		}

		this->nowPathSmoothId = tmp1;

		setMoveTarget(nowPath[nowPointId]);
		this->bot.state = HW_ON_THE_WAY_TO_TAR2;

		int n = bot.mc->nowPath.size();
		if (n == 0) {
			for (int j = 0; j <= H_GlobalContext.jamBufferSize - 1; ++j) {
				bot.jamDetectBuffer[j] = lattice(-1, -1);
				bot.jamDetectBufferPathId[j] = -1;
			}
			return;
		}

		double minDist = 100000, curDist;
		int minPointId = bot.mc->nowPointId;
		for (int j = 0; j < n; ++j) {
			curDist = calDistSq(bot.x, bot.y, bot.mc->nowPath[j].x, bot.mc->nowPath[j].y);
			if (curDist < minDist) {
				minDist = curDist;
				minPointId = j;
			}
		}

		int nowTruePointId = minPointId;
		bot.mc->nowTruePointId = nowTruePointId;
		int oriPointId = nowTruePointId;
		int lastPointId = min(nowTruePointId + H_GlobalContext.jamBufferSize - 1, n - 1);
		for (; nowTruePointId <= lastPointId; ++nowTruePointId) {
			bot.jamDetectBuffer[nowTruePointId - oriPointId] =
				lattice(bot.mc->nowPath[nowTruePointId].x, bot.mc->nowPath[nowTruePointId].y);
			bot.jamDetectBufferPathId[nowTruePointId - oriPointId] = nowTruePointId;
		}
		for (; nowTruePointId <= oriPointId + H_GlobalContext.jamBufferSize - 1; ++nowTruePointId) {
			bot.jamDetectBuffer[nowTruePointId - oriPointId] = lattice(-1, -1);
			bot.jamDetectBufferPathId[nowTruePointId - oriPointId] = -1;
		}

		vector<lattice> tmpBuffer(H_GlobalContext.jamBufferSize);
		for (int j = 0; j < H_GlobalContext.jamBufferSize; ++j) {
			tmpBuffer[j] = lattice(bot.jamDetectBuffer[j].x, bot.jamDetectBuffer[j].y);
		}

		for (int j = 1, k = 1; k < H_GlobalContext.jamBufferSize;) {
			if (tmpBuffer[j].x == -1)
				break;
			int dirX, dirY;
			dirX = tmpBuffer[j].x - bot.jamDetectBuffer[k - 1].x;
			dirY = tmpBuffer[j].y - bot.jamDetectBuffer[k - 1].y;
			if (abs(dirX) > 1 && abs(dirY) == 1) {
				if (dirX > 0)
					bot.jamDetectBuffer[k].x = bot.jamDetectBuffer[k - 1].x + 1;
				else
					bot.jamDetectBuffer[k].x = bot.jamDetectBuffer[k - 1].x - 1;
				dirX = tmpBuffer[j].x - bot.jamDetectBuffer[k].x;
				if (abs(dirX) <= 1)
					++j;
				++k;
			}
			else if (abs(dirX) == 1 && abs(dirY) > 1) {
				if (dirY > 0)
					bot.jamDetectBuffer[k].y = bot.jamDetectBuffer[k - 1].y + 1;
				else
					bot.jamDetectBuffer[k].y = bot.jamDetectBuffer[k - 1].y - 1;
				dirY = tmpBuffer[j].y - bot.jamDetectBuffer[k].y;
				if (abs(dirY) <= 1)
					++j;
				++k;
			}
			else {
				bot.jamDetectBuffer[k].x = bot.jamDetectBuffer[j].x;
				bot.jamDetectBuffer[k].y = bot.jamDetectBuffer[j].y;
				++k;
				++j;
			}

		}

	}

	vector<int> pathSmooth(const Path& path, double r) {

		int n = path.size();	// 在调用处保证n>0

		vector<int> smoothPath;
		smoothPath.push_back(0);

		for (int i = 0; i < n - 1;) {
			int j = i + 2;	// 正常来说，点i+1与点i的连线不会跨过障碍物，这点应该由路径搜索算法保证
			for (; j < n; ++j) {	// j是前向探针
				if (blockDectect(path.pointList[i], path.pointList[j], r)) {
					break;
				}
			}
			--j;	// 回到上一个与点i的连线不会跨过障碍物的点
			i = j;
			smoothPath.push_back(j);
		}
		return smoothPath;
	}

	vector<int> pathSmooth(const Path& path, double r, const vector<int>& pathId) {

		int n = pathId.size();	// 在调用处保证n>0

		vector<int> smoothPath;
		smoothPath.push_back(0);

		for (int i = 0; i < n - 1;) {
			int j = i + 2;	// 正常来说，点i+1与点i的连线不会跨过障碍物，这点应该由路径搜索算法保证
			for (; j < n; ++j) {	// j是前向探针
				if (blockDectect(path.pointList[pathId[i]], path.pointList[pathId[j]], r)) {
					break;
				}
			}
			--j;	// 回到上一个与点i的连线不会跨过障碍物的点
			i = j;
			smoothPath.push_back(pathId[j]);
		}
		return smoothPath;
	}

	bool blockDectect(const Point& p1, const Point& p2, double r) {	// 检测到障碍物，返回true

		double delta = 0.2;
		double Ax = p1.x, Ay = p1.y, Bx = p2.x, By = p2.y;
		double Cx, Cy, Dx, Dy, Ex, Ey, Fx, Fy;
		double deltaX = Ax - Bx, deltaY = Ay - By;
		double dist = sqrt(deltaX * deltaX + deltaY * deltaY);

		if (Ax == Bx) {		// 特殊情况特殊处理
			Cx = Ax - r, Cy = Ay;
			Dx = Ax + r, Dy = Ay;
			Ex = Bx - r, Ey = By;
			Fx = Bx + r, Fy = By;
		}
		else if (Ay == By) { // 特殊情况特殊处理
			Cx = Ax, Cy = Ay - r;
			Dx = Ax, Dy = Ay + r;
			Ex = Bx, Ey = By - r;
			Fx = Bx, Fy = By + r;
		}
		else {
			double theta1 = fabs(atan2(deltaY, deltaX));
			if (theta1 > M_PI_2)	// 强行扳回锐角
				theta1 = M_PI - theta1;
			double theta2 = M_PI_2 - theta1;	// 保证theta2是正的锐角
			if (deltaY * deltaX > 0) { // 同号，斜率为正
				Cx = Ax - r * cos(theta2), Cy = Ay + r * sin(theta2);
				Dx = Ax + r * cos(theta2), Dy = Ay - r * sin(theta2);
				Ex = Bx - r * cos(theta2), Ey = By + r * sin(theta2);
				Fx = Bx + r * cos(theta2), Fy = By - r * sin(theta2);
			}
			else {	// 异号，斜率为负
				Cx = Ax - r * cos(theta2), Cy = Ay - r * sin(theta2);
				Dx = Ax + r * cos(theta2), Dy = Ay + r * sin(theta2);
				Ex = Bx - r * cos(theta2), Ey = By - r * sin(theta2);
				Fx = Bx + r * cos(theta2), Fy = By + r * sin(theta2);
			}
		}
		double np = ceil(dist / delta);	// 往上取整，使得点更密集
		delta = dist / np;	// 得到一个能均分dist的delta
		double tmpX1, tmpY1, tmpX2, tmpY2;
		int idx1, idy1, idx2, idy2;
		for (int i = 1; i < np; ++i) {
			tmpX1 = Cx - deltaX * (i / np), tmpY1 = Cy - deltaY * (i / np);
			tmpX2 = Dx - deltaX * (i / np), tmpY2 = Dy - deltaY * (i / np);
			getLatticeIndex(tmpX1, tmpY1, idx1, idy1);
			getLatticeIndex(tmpX2, tmpY2, idx2, idy2);
			if ((H_GlobalContext.gameMap[idy1][idx1] == '#') || 
				(H_GlobalContext.gameMap[idy2][idx2] == '#'))
				return true;
		}
		return false;
	}

	bool blockDectect1(const Point& p1, const Point& p2, double r, vector<int>& tmpBlockX, vector<int>& tmpBlockY) {	// 检测到障碍物，返回true

		double delta = 0.2;
		double Ax = p1.x, Ay = p1.y, Bx = p2.x, By = p2.y;
		double Cx, Cy, Dx, Dy, Ex, Ey, Fx, Fy;
		double deltaX = Ax - Bx, deltaY = Ay - By;
		double dist = sqrt(deltaX * deltaX + deltaY * deltaY);

		if (Ax == Bx) {		// 特殊情况特殊处理
			Cx = Ax - r, Cy = Ay;
			Dx = Ax + r, Dy = Ay;
			Ex = Bx - r, Ey = By;
			Fx = Bx + r, Fy = By;
		}
		else if (Ay == By) { // 特殊情况特殊处理
			Cx = Ax, Cy = Ay - r;
			Dx = Ax, Dy = Ay + r;
			Ex = Bx, Ey = By - r;
			Fx = Bx, Fy = By + r;
		}
		else {
			double theta1 = fabs(atan2(deltaY, deltaX));
			if (theta1 > M_PI_2)	// 强行扳回锐角
				theta1 = M_PI - theta1;
			double theta2 = M_PI_2 - theta1;	// 保证theta2是正的锐角
			if (deltaY * deltaX > 0) { // 同号，斜率为正
				Cx = Ax - r * cos(theta2), Cy = Ay + r * sin(theta2);
				Dx = Ax + r * cos(theta2), Dy = Ay - r * sin(theta2);
				Ex = Bx - r * cos(theta2), Ey = By + r * sin(theta2);
				Fx = Bx + r * cos(theta2), Fy = By - r * sin(theta2);
			}
			else {	// 异号，斜率为负
				Cx = Ax - r * cos(theta2), Cy = Ay - r * sin(theta2);
				Dx = Ax + r * cos(theta2), Dy = Ay + r * sin(theta2);
				Ex = Bx - r * cos(theta2), Ey = By - r * sin(theta2);
				Fx = Bx + r * cos(theta2), Fy = By + r * sin(theta2);
			}
		}
		double np = ceil(dist / delta);	// 往上取整，使得点更密集
		delta = dist / np;	// 得到一个能均分dist的delta
		double tmpX1, tmpY1, tmpX2, tmpY2;
		int idx1, idy1, idx2, idy2;
		for (int i = 1; i < np; ++i) {
			tmpX1 = Cx - deltaX * (i / np), tmpY1 = Cy - deltaY * (i / np);
			tmpX2 = Dx - deltaX * (i / np), tmpY2 = Dy - deltaY * (i / np);
			getLatticeIndex(tmpX1, tmpY1, idx1, idy1);
			getLatticeIndex(tmpX2, tmpY2, idx2, idy2);

			for (int i = 0; i < tmpBlockX.size(); ++i) {
				if (tmpBlockX[i] == idx1 && tmpBlockY[i] == idy1)
					return true;
				if (tmpBlockX[i] == idx2 && tmpBlockY[i] == idy2)
					return true;
			}

			if ((H_GlobalContext.gameMap[idy1][idx1] == '#') ||
				(H_GlobalContext.gameMap[idy2][idx2] == '#')) {
				return true;
			}
				
		}
		return false;
	}

	MOVE_CONTROL_STATE move() {
		return moveUntillArriveTarget_YNG1();
	}

	void BFSFindAvoidPath(int gap) {

		bool haveProduct = bot.gootsId > 0;

		Path avoidPath;
		double startX = bot.x, startY = bot.y;
		int nSize = bot.mc->nowPath.size();

		BFS1(startX, startY, gap, haveProduct, avoidPath);
		if (this->bot.avoidBotId == -1)
			return;

		avoidPath.reverse();
		
		if (avoidPath.size() == 0) {		// 找不到可行的避让路径，调换避让优先级

			hwRobot_2& reverseBot = H_GlobalContext.botArr[bot.avoidBotId];
			double startX = reverseBot.x;
			double startY = reverseBot.y;
			haveProduct = reverseBot.gootsId > 0;

			BFS2(startX, startY, gap, haveProduct, avoidPath, reverseBot);
			avoidPath.reverse();
			
			if (avoidPath.size() == 0) {	// 找不到可行的避让路径，则退避到受避让的机器人的路径终点
				reverseBot.mc->finalTarPoint = Point(bot.mc->nowPath[nSize - 1].x, bot.mc->nowPath[nSize - 1].y);
				H_GlobalContext.botArr[bot.avoidBotId].mc->setPath(H_GlobalContext.pathPlan->getPath
				(Point(reverseBot.x, reverseBot.y), 
					Point(bot.mc->nowPath[nSize - 1].x, bot.mc->nowPath[nSize - 1].y), haveProduct));
			}
			else {
				reverseBot.avoidState = AVOIDING;
				reverseBot.avoidBotId = bot.botId;
				reverseBot.mc->setPath(avoidPath);
				reverseBot.mc->finalTarPoint = 
					Point(avoidPath[avoidPath.size() - 1].x, avoidPath[avoidPath.size() - 1].y);
				this->bot.avoidState = NOTAVOID;
				this->bot.avoidBotId = -1;
			}
		}
		else {
			bot.mc->finalTarPoint = Point(avoidPath[avoidPath.size() - 1].x, avoidPath[avoidPath.size() - 1].y);
			this->bot.mc->setPath(avoidPath);
		}
			
	}

	void BFS1(double startX, double startY, int gap, bool haveProduct, Path& avoidPath) {

		int startIdx = -1, startIdy = -1;
		getLatticeIndex(startX, startY, startIdx, startIdy);
		int avoidBotX = -1, avoidBotY = -1;
		getLatticeIndex(H_GlobalContext.botArr[this->bot.avoidBotId].x,
			H_GlobalContext.botArr[this->bot.avoidBotId].y, avoidBotX, avoidBotY);

		lattice path1[100][100];	// 用于存储在BFS过程中所遍历到的点的父节点
		lattice start;
		start.x = startIdx;
		start.y = startIdy;
		queue<lattice> q;
		q.push(start);
		int dx[8] = { 0,1,0,-1,-1,1,1,-1 };
		int dy[8] = { 1,0,-1,0,1,1,-1,-1 };
		int n = H_GlobalContext.botArr[this->bot.avoidBotId].mc->nowPath.size();

		bool visit[100][100] = { false };	// 标记是否被访问过
		visit[startIdy][startIdx] = true;

		int dirX = 0, dirY = 0;	// 利用dx，dy数组和dirX，dirY来标识路径的前进方向
		int nSelfPath = this->bot.mc->nowPath.size();
		if (nSelfPath <= 1) {	// 极端情况，此时无法得知机器人的路径前进方向，让它直接避让到要避让的机器人的路径终点
			return;
		}
		if (bot.curTarWbId == -1) {	// 机器人暂时没有目标，让它直接避让到要避让的机器人的路径终点
			return;
		}

		int pointId1 = this->bot.mc->nowTruePointId;
		int pointId2 = pointId1 + 1;
		if (pointId2 == nSelfPath) {
			pointId2 -= 1;
			pointId1 -= 1;
		}
		int latticeIdx1 = -1, latticeIdy1 = -1, latticeIdx2 = -1, latticeIdy2 = -1, latticeTrueIdx, latticeTrueIdy;
		getLatticeIndex(this->bot.mc->nowPath[pointId1].x, this->bot.mc->nowPath[pointId1].y, latticeIdx1, latticeIdy1);
		getLatticeIndex(this->bot.mc->nowPath[pointId2].x, this->bot.mc->nowPath[pointId2].y, latticeIdx2, latticeIdy2);
		getLatticeIndex(this->bot.x, this->bot.y, latticeTrueIdx, latticeTrueIdy);
		dirX = latticeIdx2 - latticeIdx1;
		dirY = latticeIdy2 - latticeIdy1;

		if (abs(dirX) > 1) {	// 防止因四舍五入误差导致latticeIdx多算了一格
			if (dirX > 0) {
				latticeIdx2 = latticeIdx1 + 1;
			}
			else {
				latticeIdx2 = latticeIdx1 - 1;
			}
		}
		if (abs(dirY) > 1) {	// 防止因四舍五入误差导致latticeIdy多算了一格
			if (dirY > 0) {
				latticeIdy2 = latticeIdy1 + 1;
			}
			else {
				latticeIdy2 = latticeIdy1 - 1;
			}
		}
		dirX = latticeIdx2 - latticeIdx1;
		dirY = latticeIdy2 - latticeIdy1;

		vector<int> tmpBlockX(5, -1);
		vector<int> tmpBlockY(5, -1);	// BFS寻路屏障
		tmpBlockX[0] = latticeIdx2;
		tmpBlockY[0] = latticeIdy2;		

		swap(dirX, dirY);	// 垂直
		dirX = -dirX;

		for (int i = 1; i <= (tmpBlockX.size() / 2); ++i) {
			tmpBlockX[i] = tmpBlockX[0] + i * dirX;
			tmpBlockX[tmpBlockX.size() - i] = tmpBlockX[0] - i * dirX;

			tmpBlockY[i] = tmpBlockY[0] + i * dirY;
			tmpBlockY[tmpBlockX.size() - i] = tmpBlockY[0] - i * dirY;
		}

		dirX = -dirX;
		swap(dirX, dirY);	// 垂直

		for (int i = 0; i < tmpBlockX.size(); ++i) {
			if (tmpBlockX[i] == startIdx && tmpBlockY[i] == startIdy) {
				tmpBlockX[2] = latticeTrueIdx + dirX;
				tmpBlockY[2] = latticeTrueIdy + dirY;
				swap(dirX, dirY);	// 垂直
				dirX = -dirX;
				tmpBlockX[0] = tmpBlockX[2] + 2 * dirX;
				tmpBlockX[1] = tmpBlockX[2] + dirX;
				tmpBlockX[3] = tmpBlockX[2] - dirX;
				tmpBlockX[4] = tmpBlockX[2] - 2 * dirX;

				tmpBlockY[0] = tmpBlockY[2] + 2 * dirY;
				tmpBlockY[1] = tmpBlockY[2] + dirY;
				tmpBlockY[3] = tmpBlockY[2] - dirY;
				tmpBlockY[4] = tmpBlockY[2] - 2 * dirY;
				break;
			}
		}

		while (!q.empty()) {

			lattice curNode = q.front();
			q.pop();

			for (int i = 0; i < 8; ++i) {

				lattice newNode;
				int x = curNode.x + dx[i], y = curNode.y + dy[i];

				
				if (haveProduct) {
					if (x < 0 || x > 99 || y < 0 || y > 99 || visit[y][x] || !H_GlobalContext.pathPlan->fesible_lattice_haveProd[y][x]) {
						continue;
					}
				}
				else {
					if (x < 0 || x > 99 || y < 0 || y > 99 || visit[y][x] || !H_GlobalContext.pathPlan->fesible_lattice_noProd[y][x]) {
						continue;
					}
				}

				if (haveProduct) {
					if (blockDectect1(H_GlobalContext.pathPlan->fesible_point_haveProd[curNode.y][curNode.x],
						H_GlobalContext.pathPlan->fesible_point_haveProd[y][x], 0.53, tmpBlockX, tmpBlockY))
						continue;
				}
				else {
					if (blockDectect1(H_GlobalContext.pathPlan->fesible_point_noProd[curNode.y][curNode.x],
						H_GlobalContext.pathPlan->fesible_point_noProd[y][x], 0.45, tmpBlockX, tmpBlockY))
						continue;
				}

				bool getOutFlag = false;
				for (int i = 0; i < tmpBlockX.size(); ++i) {	// 尽可能不往路径前进方向去搜索
					if (x == tmpBlockX[i] && y == tmpBlockY[i]) {
						getOutFlag = true;
						break;
					}
				}
				if (getOutFlag)
					continue;

				newNode.x = x;
				newNode.y = y;
				q.push(newNode);
				path1[y][x].x = curNode.x;
				path1[y][x].y = curNode.y;
				visit[y][x] = true;

				bool findFlag = true;

				double avoidBotPathX, avoidBotPathY, doubleX, doubleY;

				if (haveProduct) {
					doubleX = H_GlobalContext.pathPlan->fesible_point_haveProd[y][x].x;
					doubleY = H_GlobalContext.pathPlan->fesible_point_haveProd[y][x].y;
				}
				else {
					doubleX = H_GlobalContext.pathPlan->fesible_point_noProd[y][x].x;
					doubleY = H_GlobalContext.pathPlan->fesible_point_noProd[y][x].y;
				}

				double curGap;
				for (int j = 0; j < n; ++j) {	// 遍历avoidBot在当前路径所有的路径点
					avoidBotPathX = H_GlobalContext.botArr[this->bot.avoidBotId].mc->nowPath[j].x;
					avoidBotPathY = H_GlobalContext.botArr[this->bot.avoidBotId].mc->nowPath[j].y;
	
					curGap = (avoidBotPathX - doubleX) * (avoidBotPathX - doubleX) + 
						(avoidBotPathY - doubleY) * (avoidBotPathY - doubleY);

					if (curGap <= 1.2 * 1.2) {	// 如果点（x，y）与avoidBot余下路径的某一个点在gap范围之内
						findFlag = false;	// 说明（x，y）这一点未满足要求，舍去
						break;
					}
				}

				if (findFlag) {

					if (haveProduct) 
						avoidPath.push_back(H_GlobalContext.pathPlan->fesible_point_haveProd[y][x]);
					else
						avoidPath.push_back(H_GlobalContext.pathPlan->fesible_point_noProd[y][x]);

					int father_x = path1[y][x].x;
					int father_y = path1[y][x].y;
					int temp_x = father_x, temp_y = father_y;
					while (father_x != startIdx || father_y != startIdy) {

						if (haveProduct)
							avoidPath.push_back(H_GlobalContext.pathPlan->fesible_point_haveProd[father_y][father_x]);
						else
							avoidPath.push_back(H_GlobalContext.pathPlan->fesible_point_noProd[father_y][father_x]);

						father_x = path1[temp_y][temp_x].x;
						father_y = path1[temp_y][temp_x].y;
						temp_x = father_x, temp_y = father_y;
					}

					if (haveProduct)
						avoidPath.push_back(H_GlobalContext.pathPlan->fesible_point_haveProd[startIdy][startIdx]);
					else
						avoidPath.push_back(H_GlobalContext.pathPlan->fesible_point_noProd[startIdy][startIdx]);

					return;
				}
			}
		}
	}

	void BFS2(double startX, double startY, int gap, bool haveProduct, Path& avoidPath, hwRobot_2& mybot) {

		int startIdx = -1, startIdy = -1;
		getLatticeIndex(startX, startY, startIdx, startIdy);

		lattice path1[100][100];	// 用于存储在BFS过程中所遍历到的点的父节点
		lattice start;
		start.x = startIdx;
		start.y = startIdy;
		queue<lattice> q;
		q.push(start);
		int dx[8] = { 0,1,0,-1,-1,1,1,-1 };
		int dy[8] = { 1,0,-1,0,1,1,-1,-1 };
		int n = bot.mc->nowPath.size();

		bool visit[100][100] = { false };	// 标记是否被访问过
		visit[startIdy][startIdx] = true;

		int dirX = 0, dirY = 0;	// 利用dx，dy数组和dirX，dirY来标识路径的前进方向
		int nSelfPath = mybot.mc->nowPath.size();
		if (nSelfPath <= 1) {	// 极端情况，此时无法得知机器人的路径前进方向，让它直接避让到要避让的机器人的路径终点
			return;
		}
		if (bot.curTarWbId == -1) {	// 机器人暂时没有目标，让它直接避让到要避让的机器人的路径终点
			return;
		}

		int pointId1 = mybot.mc->nowTruePointId;
		int pointId2 = pointId1 + 1;
		if (pointId2 == nSelfPath) {
			pointId2 -= 1;
			pointId1 -= 1;
		}
		int latticeIdx1 = -1, latticeIdy1 = -1, latticeIdx2 = -1, latticeIdy2 = -1;
		getLatticeIndex(mybot.mc->nowPath[pointId1].x, mybot.mc->nowPath[pointId1].y, latticeIdx1, latticeIdy1);
		getLatticeIndex(mybot.mc->nowPath[pointId2].x, mybot.mc->nowPath[pointId2].y, latticeIdx2, latticeIdy2);
		dirX = latticeIdx2 - latticeIdx1;
		dirY = latticeIdy2 - latticeIdy1;

		if (abs(dirX) > 1) {	// 防止因四舍五入误差导致latticeIdx多算了一格
			if (dirX > 0) {
				latticeIdx2 = latticeIdx1 + 1;
			}
			else {
				latticeIdx2 = latticeIdx1 - 1;
			}
		}
		if (abs(dirY) > 1) {	// 防止因四舍五入误差导致latticeIdy多算了一格
			if (dirY > 0) {
				latticeIdy2 = latticeIdy1 + 1;
			}
			else {
				latticeIdy2 = latticeIdy1 - 1;
			}
		}
		dirX = latticeIdx2 - latticeIdx1;
		dirY = latticeIdy2 - latticeIdy1;

		vector<int> tmpBlockX(5, -1);
		vector<int> tmpBlockY(5, -1);	// BFS寻路屏障
		tmpBlockX[2] = latticeIdx2 + dirX;
		tmpBlockY[2] = latticeIdy2 + dirY;
		swap(dirX, dirY);	// 垂直
		dirX = -dirX;

		tmpBlockX[0] = tmpBlockX[2] + 2 * dirX;
		tmpBlockX[1] = tmpBlockX[2] + dirX;
		tmpBlockX[3] = tmpBlockX[2] - dirX;
		tmpBlockX[4] = tmpBlockX[2] - 2 * dirX;

		tmpBlockY[0] = tmpBlockY[2] + 2 * dirY;
		tmpBlockY[1] = tmpBlockY[2] + dirY;
		tmpBlockY[3] = tmpBlockY[2] - dirY;
		tmpBlockY[4] = tmpBlockY[2] - 2 * dirY;

		while (!q.empty()) {

			lattice curNode = q.front();
			q.pop();

			for (int i = 0; i < 8; ++i) {

				lattice newNode;
				int x = curNode.x + dx[i], y = curNode.y + dy[i];

				if (haveProduct) {
					if (x < 0 || x > 99 || y < 0 || y > 99 || visit[y][x] || !H_GlobalContext.pathPlan->fesible_lattice_haveProd[y][x]) {
						continue;
					}
				}
				else {
					if (x < 0 || x > 99 || y < 0 || y > 99 || visit[y][x] || !H_GlobalContext.pathPlan->fesible_lattice_noProd[y][x]) {
						continue;
					}
				}

				if (haveProduct) {
					if (blockDectect1(H_GlobalContext.pathPlan->fesible_point_haveProd[curNode.y][curNode.x],
						H_GlobalContext.pathPlan->fesible_point_haveProd[y][x], 0.53, tmpBlockX, tmpBlockY))
						continue;
				}
				else {
					if (blockDectect1(H_GlobalContext.pathPlan->fesible_point_noProd[curNode.y][curNode.x],
						H_GlobalContext.pathPlan->fesible_point_noProd[y][x], 0.45, tmpBlockX, tmpBlockY))
						continue;
				}

				bool getOutFlag = false;
				for (int i = 0; i < 5; ++i) {	// 尽可能不往路径前进方向去搜索
					if (x == tmpBlockX[i] && y == tmpBlockY[i]) {
						getOutFlag = true;
						break;
					}
				}
				if (getOutFlag)
					continue;

				newNode.x = x;
				newNode.y = y;
				q.push(newNode);
				path1[y][x].x = curNode.x;
				path1[y][x].y = curNode.y;
				visit[y][x] = true;

				bool findFlag = true;

				double avoidBotPathX, avoidBotPathY, doubleX, doubleY;

				if (haveProduct) {
					doubleX = H_GlobalContext.pathPlan->fesible_point_haveProd[y][x].x;
					doubleY = H_GlobalContext.pathPlan->fesible_point_haveProd[y][x].y;
				}
				else {
					doubleX = H_GlobalContext.pathPlan->fesible_point_noProd[y][x].x;
					doubleY = H_GlobalContext.pathPlan->fesible_point_noProd[y][x].y;
				}

				double curGap;
				for (int j = 0; j < n; ++j) {	// 遍历avoidBot在当前路径所有的路径点
					avoidBotPathX = bot.mc->nowPath[j].x;
					avoidBotPathY = bot.mc->nowPath[j].y;

					curGap = (avoidBotPathX - doubleX) * (avoidBotPathX - doubleX) +
						(avoidBotPathY - doubleY) * (avoidBotPathY - doubleY);

					if (curGap <= 1.2 * 1.2) {	// 如果点（x，y）与avoidBot余下路径的某一个点在gap范围之内
						findFlag = false;	// 说明（x，y）这一点未满足要求，舍去
						break;
					}
				}

				if (findFlag) {

					if (haveProduct)
						avoidPath.push_back(H_GlobalContext.pathPlan->fesible_point_haveProd[y][x]);
					else
						avoidPath.push_back(H_GlobalContext.pathPlan->fesible_point_noProd[y][x]);

					int father_x = path1[y][x].x;
					int father_y = path1[y][x].y;
					int temp_x = father_x, temp_y = father_y;
					while (father_x != startIdx || father_y != startIdy) {

						if (haveProduct)
							avoidPath.push_back(H_GlobalContext.pathPlan->fesible_point_haveProd[father_y][father_x]);
						else
							avoidPath.push_back(H_GlobalContext.pathPlan->fesible_point_noProd[father_y][father_x]);

						father_x = path1[temp_y][temp_x].x;
						father_y = path1[temp_y][temp_x].y;
						temp_x = father_x, temp_y = father_y;
					}

					if (haveProduct)
						avoidPath.push_back(H_GlobalContext.pathPlan->fesible_point_haveProd[startIdy][startIdx]);
					else
						avoidPath.push_back(H_GlobalContext.pathPlan->fesible_point_noProd[startIdy][startIdx]);

					return;
				}
			}
		}
	}
	
	void jamDetect(int botId, int gap) {
		int idmX, idmY, idnX, idnY;

		if (this->bot.avoidState != NOTAVOID && this->bot.avoidBotId == botId)	// 自己的避让目标无须再次处理
			return;
		if (H_GlobalContext.botArr[botId].avoidBotId == this->bot.botId)	// botId正在给自己避让，无须处理
			return;

		for (int m = 0; m < H_GlobalContext.jamBufferSize; ++m) {
			idmX = this->bot.jamDetectBuffer[m].x;
			idmY = this->bot.jamDetectBuffer[m].y;
			if (idmX == -1)
				continue;
			for (int n = 0; n < H_GlobalContext.jamBufferSize; ++n) {
				idnX = H_GlobalContext.botArr[botId].jamDetectBuffer[n].x;
				idnY = H_GlobalContext.botArr[botId].jamDetectBuffer[n].y;
				if (idnX == -1)
					continue;

				if (n != 0 && m != 0) {
					int deltaMX = idmX - this->bot.jamDetectBuffer[m - 1].x;
					int deltaMY = idmY - this->bot.jamDetectBuffer[m - 1].y;
					int deltaNX = idnX - H_GlobalContext.botArr[botId].jamDetectBuffer[n - 1].x;
					int deltaNY = idnY - H_GlobalContext.botArr[botId].jamDetectBuffer[n - 1].y;

					if (deltaMX * deltaNX + deltaMY * deltaNY >= 0)		// 同向运动的无须进行防堵塞处理
						continue;
				}
				else if (n == 0 && m != 0) {
					int deltaMX = idmX - this->bot.jamDetectBuffer[m - 1].x;
					int deltaMY = idmY - this->bot.jamDetectBuffer[m - 1].y;

					if (H_GlobalContext.botArr[botId].jamDetectBuffer[1].x != -1) {
						int deltaNX = H_GlobalContext.botArr[botId].jamDetectBuffer[1].x - idnX;
						int deltaNY = H_GlobalContext.botArr[botId].jamDetectBuffer[1].y - idnY;
						if (deltaMX * deltaNX + deltaMY * deltaNY >= 0)		// 同向运动的无须进行防堵塞处理
							continue;
					}
					else {
						double vNX = H_GlobalContext.botArr[botId].lineSpeed[0];
						double vNY = H_GlobalContext.botArr[botId].lineSpeed[1];
						if (deltaMX * vNX + deltaMY * vNY >= 0)		// 同向运动的无须进行防堵塞处理
							continue;
					}

				}
				else if (n != 0 && m == 0) {
					int deltaNX = idnX - H_GlobalContext.botArr[botId].jamDetectBuffer[n - 1].x;
					int deltaNY = idnY - H_GlobalContext.botArr[botId].jamDetectBuffer[n - 1].y;

					if (this->bot.jamDetectBuffer[1].x != -1) {
						int deltaMX = this->bot.jamDetectBuffer[1].x - idmX;
						int deltaMY = this->bot.jamDetectBuffer[1].y - idmY;
						if (deltaMX * deltaNX + deltaMY * deltaNY >= 0)		// 同向运动的无须进行防堵塞处理
							continue;
					}
					else {
						double vMX = this->bot.lineSpeed[0];
						double vMY = this->bot.lineSpeed[1];
						if (deltaNX * vMX + deltaNY * vMY >= 0)		// 同向运动的无须进行防堵塞处理
							continue;
					}

				}
				else {
					int deltaMX = this->bot.jamDetectBuffer[1].x - idmX;
					int deltaMY = this->bot.jamDetectBuffer[1].y - idmY;
					int deltaNX = H_GlobalContext.botArr[botId].jamDetectBuffer[1].x - idnX;
					int deltaNY = H_GlobalContext.botArr[botId].jamDetectBuffer[1].y - idnY;

					if (H_GlobalContext.botArr[botId].jamDetectBuffer[1].x != -1
						&& this->bot.jamDetectBuffer[1].x != -1) {
						if (deltaMX * deltaNX + deltaMY * deltaNY >= 0)		// 同向运动的无须进行防堵塞处理
							continue;
					}
					else if (H_GlobalContext.botArr[botId].jamDetectBuffer[1].x != -1
						&& this->bot.jamDetectBuffer[1].x == -1) {
						double vMX = this->bot.lineSpeed[0];
						double vMY = this->bot.lineSpeed[1];
						if (deltaNX * vMX + deltaNY * vMY >= 0)		// 同向运动的无须进行防堵塞处理
							continue;
					}
					else if (H_GlobalContext.botArr[botId].jamDetectBuffer[1].x == -1
						&& this->bot.jamDetectBuffer[1].x != -1) {
						double vNX = H_GlobalContext.botArr[botId].lineSpeed[0];
						double vNY = H_GlobalContext.botArr[botId].lineSpeed[1];
						if (deltaMX * vNX + deltaMY * vNY >= 0)		// 同向运动的无须进行防堵塞处理
							continue;
					}
					else {
						double vMX = this->bot.lineSpeed[0];
						double vMY = this->bot.lineSpeed[1];
						double vNX = H_GlobalContext.botArr[botId].lineSpeed[0];
						double vNY = H_GlobalContext.botArr[botId].lineSpeed[1];
						if (vMX * vNX + vMY * vNY >= 0)		// 同向运动的无须进行防堵塞处理
							continue;
					}

				}

				//// 以下进行预期堵塞检测
				if (idmX == idnX && idmY == idnY) {

					this->bot.avoidState = AVOIDING;
					this->bot.avoidBotId = botId;
					BFSFindAvoidPath(gap);
					return;
				}
				else if (idmX != idnX && idmY == idnY && abs(idmX - idnX) <= gap) {

					this->bot.avoidState = AVOIDING;
					this->bot.avoidBotId = botId;
					BFSFindAvoidPath(gap);
					return;
				}
				else if (idmX == idnX && idmY != idnY && abs(idmY - idnY) <= gap) {

					this->bot.avoidState = AVOIDING;
					this->bot.avoidBotId = botId;
					BFSFindAvoidPath(gap);
					return;
				}

			}
		}
	}

	bool jamDetect1(int botId, int gap) {
		int idmX, idmY, idnX, idnY;
		for (int m = 0; m < H_GlobalContext.jamBufferSize; ++m) {
			idmX = this->bot.jamDetectBuffer[m].x;
			idmY = this->bot.jamDetectBuffer[m].y;
			if (idmX == -1)
				continue;
			for (int n = 0; n < H_GlobalContext.jamBufferSize; ++n) {
				idnX = H_GlobalContext.botArr[botId].jamDetectBuffer[n].x;
				idnY = H_GlobalContext.botArr[botId].jamDetectBuffer[n].y;
				if (idnX == -1)
					continue;

				if (n != 0 && m != 0) {
					int deltaMX = idmX - this->bot.jamDetectBuffer[m - 1].x;
					int deltaMY = idmY - this->bot.jamDetectBuffer[m - 1].y;
					int deltaNX = idnX - H_GlobalContext.botArr[botId].jamDetectBuffer[n - 1].x;
					int deltaNY = idnY - H_GlobalContext.botArr[botId].jamDetectBuffer[n - 1].y;

					if (deltaMX * deltaNX + deltaMY * deltaNY >= 0)		// 同向运动的无须进行防堵塞处理
						continue;
				}
				else if (n == 0 && m != 0) {
					int deltaMX = idmX - this->bot.jamDetectBuffer[m - 1].x;
					int deltaMY = idmY - this->bot.jamDetectBuffer[m - 1].y;
					int deltaNX = H_GlobalContext.botArr[botId].jamDetectBuffer[1].x - idnX;
					int deltaNY = H_GlobalContext.botArr[botId].jamDetectBuffer[1].y - idnY;

					if (deltaMX * deltaNX + deltaMY * deltaNY >= 0)		// 同向运动的无须进行防堵塞处理
						continue;
				}
				else if (n != 0 && m == 0) {
					int deltaMX = this->bot.jamDetectBuffer[1].x - idmX;
					int deltaMY = this->bot.jamDetectBuffer[1].y - idmY;
					int deltaNX = idnX - H_GlobalContext.botArr[botId].jamDetectBuffer[n - 1].x;
					int deltaNY = idnY - H_GlobalContext.botArr[botId].jamDetectBuffer[n - 1].y;

					if (deltaMX * deltaNX + deltaMY * deltaNY >= 0)		// 同向运动的无须进行防堵塞处理
						continue;
				}
				else {
					int deltaMX = this->bot.jamDetectBuffer[1].x - idmX;
					int deltaMY = this->bot.jamDetectBuffer[1].y - idmY;
					int deltaNX = H_GlobalContext.botArr[botId].jamDetectBuffer[1].x - idnX;
					int deltaNY = H_GlobalContext.botArr[botId].jamDetectBuffer[1].y - idnY;

					if (deltaMX * deltaNX + deltaMY * deltaNY >= 0)		// 同向运动的无须进行防堵塞处理
						continue;
				}

				// 以下进行预期堵塞检测
				if (idmX == idnX && idmY == idnY) {
					return true;
				}
				else if (idmX != idnX && idmY == idnY && abs(idmX - idnX) <= gap) {
					return true;
				}
				else if (idmX == idnX && idmY != idnY && abs(idmY - idnY) <= gap) {
					return true;
				}

			}
		}

		return false;
	}

	void unJamDetect() {

		if (this->bot.avoidState == AVOIDED) {

			bool isJam = jamDetect1(this->bot.avoidBotId, 2);

			if (!isJam) {
				this->bot.avoidState = NOTAVOID;
				this->bot.avoidBotId = -1;

				bool ifHaveProd = this->bot.gootsId > 0;
				this->bot.mc->setPath(H_GlobalContext.pathPlan->
					getPath(Point(this->bot.x, this->bot.y),
						H_GlobalContext.wbArr[this->bot.curTarWbId], ifHaveProd));
			}
		}

	}

	void unJamDetect1() {

		for (int i = 0; i < 4; ++i) {
			if (i == this->bot.botId)
				continue;

			hwRobot_2& bot = H_GlobalContext.botArr[i];
			if (bot.avoidBotId == this->bot.botId) {
				bot.avoidState = NOTAVOID;
				bot.avoidBotId = -1;

				bool ifHaveProd = bot.gootsId > 0;
				Point tmpPoint = Point(bot.x, bot.y);
				bot.mc->setPath(H_GlobalContext.pathPlan->
					getPath(tmpPoint,
						H_GlobalContext.wbArr[bot.curTarWbId], ifHaveProd));
			}
		}
	}

};
