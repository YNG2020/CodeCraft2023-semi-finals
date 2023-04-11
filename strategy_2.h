#pragma once
#include <iostream>
#include "hwRobot_2.h"
#include "workbench.h"
#include "h_globalContext.h"

/// <summary>
/// ������������Ϊ
/// 
/// readMap  
/// ��ȡ��ͼ��ʼ������ ִֻ��һ��
///	(�������޸ĸú���)
/// 
/// afterReadMap 
/// ��ȡ��ͼ��ʼ��������, ��ȡ��һ֡����ǰ ִ��, ִֻ��һ��
/// 
/// cout << "OK" << endl; 
/// ��afterReadMap������OK, 
/// ��readMap��ʼ�����OK,�ܼ���Լ5���ʱ��
/// 
/// for(int frameId = 1; frameId < 9001; frameId++){
/// 
///		readFrame 
///		��ȡÿһ֡����, ������ workbench �� bot ������; 
///		ÿִ֡��һ��
///		(�������޸ĸú���)
///		
///		afterReadFrame 
///		��ȡÿһ֡���ݺ�, ��ȡ��һ֡ǰִ��; 
///		ÿִ֡��һ��
///		 
///     cout << "OK" << endl;
///		��readFrame��ʼ�����OK, �ܼ���15msʱ��
/// }
/// 
/// </summary>
/// 
/// 


class Strategy_2 {
public:
	// Ϊ��ά�������ĵ��÷�ʽ, ��afterReadMap()
	// Ҳ����ֱ��ͨ�� H_GlobalContext.wbNum ��ʽ����
	// H_GlobalContext.wbArr��
	int& wbNum;
	hwRobot_2* botArr;
	hwWorkbench* wbArr;
	int& frameId;
	int& money;

	// �ѱ�����ȫ���������еı�����, ʵ���޸��滻
	Strategy_2():wbNum(H_GlobalContext.wbNum),
		frameId(H_GlobalContext.frameId),
		money(H_GlobalContext.money),
		botArr(H_GlobalContext.botArr),
		wbArr(H_GlobalContext.wbArr){}

	virtual void afterReadMap() = 0;

	virtual void afterReadFrame(int) = 0;

};
