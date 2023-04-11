#pragma once
#include"Path.h"
#include"parameter.h"
#include"h_globalContext.h"

class hwRobot_2;

class moveControl {
public:
	moveControl(hwRobot_2& _bot):bot(_bot) {}

	// ����bot
	hwRobot_2& bot;

	// ��ǰ״̬
	MOVE_CONTROL_STATE state = MC_NO_PATH;

	// ��ǰ·��, ��stateΪMC_NO_PATHʱ��Ч
	Path nowPath;

	Point nowTarPoint;

	Point finalTarPoint;

	// ��ǰĿ���Ϊ��ǰԭʼ·���ĵڼ�����
	int nowPointId = 0;

	// ��ǰ������λ�ô��ڵ�ǰԭʼ·���ĵڼ�����
	int nowTruePointId = 0;

	// �����ƶ�·��, ���ó��麯����Ϊ�˷��㲻ͬ�㷨��·�����д���(ƽ��/���ٹؼ����)
	virtual void setPath(const Path& p) = 0;

	MOVE_CONTROL_STATE getNowState() const {
		return state;
	}

	// ��ԭmoveUtillTarget
	virtual MOVE_CONTROL_STATE move() = 0;

};