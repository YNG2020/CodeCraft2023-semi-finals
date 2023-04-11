#pragma once
#include <iostream>
#include "hwRobot_2.h"
#include "workbench.h"
#include "h_globalContext.h"

/// <summary>
/// 整体运行流程为
/// 
/// readMap  
/// 读取地图初始化参数 只执行一次
///	(不建议修改该函数)
/// 
/// afterReadMap 
/// 读取地图初始化参数后, 读取第一帧数据前 执行, 只执行一次
/// 
/// cout << "OK" << endl; 
/// 在afterReadMap后才输出OK, 
/// 从readMap开始到输出OK,总计有约5秒的时间
/// 
/// for(int frameId = 1; frameId < 9001; frameId++){
/// 
///		readFrame 
///		读取每一帧数据, 并更新 workbench 与 bot 的数据; 
///		每帧执行一次
///		(不建议修改该函数)
///		
///		afterReadFrame 
///		读取每一帧数据后, 读取下一帧前执行; 
///		每帧执行一次
///		 
///     cout << "OK" << endl;
///		从readFrame开始到输出OK, 总计有15ms时间
/// }
/// 
/// </summary>
/// 
/// 


class Strategy_2 {
public:
	// 为了维持往常的调用方式, 在afterReadMap()
	// 也可以直接通过 H_GlobalContext.wbNum 方式调用
	// H_GlobalContext.wbArr等
	int& wbNum;
	hwRobot_2* botArr;
	hwWorkbench* wbArr;
	int& frameId;
	int& money;

	// 把变量和全局上下文中的变量绑定, 实现无感替换
	Strategy_2():wbNum(H_GlobalContext.wbNum),
		frameId(H_GlobalContext.frameId),
		money(H_GlobalContext.money),
		botArr(H_GlobalContext.botArr),
		wbArr(H_GlobalContext.wbArr){}

	virtual void afterReadMap() = 0;

	virtual void afterReadFrame(int) = 0;

};
