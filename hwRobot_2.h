#pragma once

#include <iostream>
#include "workbench.h"
#include "parameter.h"
#include "moveControl.h"
#include "h_globalContext.h"
// ����M_PI�ȳ���
#define _USE_MATH_DEFINES
#include <math.h>
using namespace std;

/// <summary>
/// botId: bot��id [0,3]
/// workbenchType: ��������̨id
/// </summary>
class hwRobot_2 {
public:

	int botId = 0;

	/// <summary>
	/// ��������̨id
	/// -1��ʾ�����κι���̨����
	/// [0,����̨����-1] ��ʾ�ڶ�Ӧ����̨������, ��ǰ���������й��򡢳�����Ϊ����Ըù���̨����
	/// </summary>
	int workbenchId = -1;

	/// <summary>
	/// 0��ʾδЯ��
	/// 1-7��ʾЯ����Ʒid
	/// </summary>
	int gootsId = 0;
	/// <summary>
	/// ʱ���ֵϵ��
	/// </summary>
	double tvr = 1;
	/// <summary>
	/// ��ײ��ֵϵ��
	/// </summary>
	double cvr = 1;

	// ���ﵱǰ�ļ�ֵ
	double curValue = 0;
	
	double angleSpeed = 0;
	double lineSpeed[2] = { 0,0 };
	/// <summary>
	/// ��ǰ����
	/// </summary>
	double faceTo = 0;

	double x;
	double y;

	// ͨ�����������ٶȺ��������ٶ�������bot, 
	// ������󲢲�������cout�������, 
	// ������updateAfterHandle��ִ��
	// �������ٶ�
	double expectAngleSpeed = 0;
	// �����ٶ�
	double expectSpeed = 0;

	// �Ƿ��޸Ĺ��������ٶ�
	bool ifChangeEAS = false;
	// �Ƿ��޸Ĺ������ٶ�
	bool ifChangeES = false;

	moveControl* mc = nullptr;

	BOT_MOVE_STATE state;

	// �����˵Ķ�����⻺����
	lattice jamDetectBuffer[6];
	// �����˵Ķ�����⻺������PathId
	int jamDetectBufferPathId[6];
	// �Ƿ�λ����խ����ı�ʶ
	bool inNarrowZone = false;

	// ����״̬
	BOT_AVOID_STATE avoidState;
	// ��¼����Ҫ���õĶԷ������˵ı�ţ�ֻ��¼���ȼ���ߵĻ����ˣ�
	int avoidBotId;
	// ��¼�»����˵�ǰ��Ŀ�깤��̨Id
	int curTarWbId;
	// ��¼����ʱ��
	int stickTime;

	// ��ʷƽ���ٶ�
	double vHistory;

	/// <summary>
	/// [-2, 6] ֮��ĸ�����, ��ʾǰ���ٶ�
	/// </summary>
	/// <param name="speed"></param>
	void forward(double speed) {
		expectSpeed = speed;
		ifChangeES = true;
	}

	// ��ӡǰ������
	void printForwardCMD() {
		cout << "forward " << botId << ' ' << expectSpeed << endl;
	}

	/// <summary>
	/// [-��,��]  ������ת�ٶ�, ��λ����ÿ��
	/// </summary>
	/// <param name="angle"></param>
	void rotate(double angle) {
		expectAngleSpeed = angle;
		ifChangeEAS = true;
	}

	// ��ӡ��ת����
	void printRotateCMD() {
		cout << "rotate " << botId << ' ' << expectAngleSpeed << endl;
	}

	/// <summary>
	/// ע��, ���øú�����, �������Զ�����goodsId, ��Ҫ�ȵ���һ֡����, ���û������޸� bot.goodsId
	/// </summary>
	void buy()const {
		cout << "buy " << botId << endl;
	}
	/// <summary>
	/// ע��, ���øú�����, �������Զ�����goodsId, ��Ҫ�ȵ���һ֡����, ���û������޸� bot.goodsId
	/// </summary>
	void sell()const {
		cout << "sell " << botId << endl;
	}
	/// <summary>
	/// ע��, ���øú�����, �������Զ�����goodsId, ��Ҫ�ȵ���һ֡����, ���û������޸� bot.goodsId
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
	/// ��ȡ���Ӧ����̨֮��н�, (-pi,pi], ������ʱ��,����˳ʱ��(�����Ᵽ��һ��
	/// </summary>
	/// <returns>����ֵ����[-pi,pi]</returns>
	const double getAngleBetweenThisBotAndWorkbench(const hwWorkbench& wb)const {
		double dx = wb.x - this->x;
		double dy = wb.y - this->y;
		double angleBetweenWbAndZero = atan2(dy, dx);
		double ans = angleBetweenWbAndZero - this->faceTo;
		return ans > M_PI ? ans - 2 * M_PI : (ans <= -M_PI ? ans + 2 * M_PI : ans);
	}

	/// <summary>
	/// ��ȡ���Ӧ��֮��н�, (-pi,pi], ������ʱ��,����˳ʱ��(�����Ᵽ��һ��
	/// </summary>
	/// <returns>����ֵ����[-pi,pi]</returns>
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
