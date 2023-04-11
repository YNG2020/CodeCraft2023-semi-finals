#pragma once

#include <iostream>

using namespace std;

class hwWorkbench {
public:
	hwWorkbench() {};
	hwWorkbench(int _wbId, int _x, int _y, int _wbt):
		wbId(_wbId),
		x( (_x) * 0.5 + 0.25 ), 
		y( (_y) * 0.5 + 0.25 ),
		wbType(_wbt){
	
		switch (wbType)
		{
		case 1:
			neededMaterials = 0b0;
			break;
		case 2:
			neededMaterials = 0b0;
			break;
		case 3:
			neededMaterials = 0b0;
			break;
		case 4:
			neededMaterials = 0b0110;
			break;
		case 5:
			neededMaterials = 0b1010;
			break;
		case 6:
			neededMaterials = 0b1100;
			break;
		case 7:
			neededMaterials = 0b1110000;
			break;
		case 8:
			neededMaterials = 0b10000000;
			break;
		case 9:
			neededMaterials = 0b11111110;
			break;
		
		default:
			neededMaterials = 0;
			break;
		}
	};

	void init(int _wbId, int _x, int _y, int _wbt) {
		wbId = _wbId;
		x = (_x) * 0.5 + 0.25;
		y = ((_y) * 0.5 + 0.25);
		wbType = (_wbt);
		switch (wbType)
		{
		case 1:
			neededMaterials = 0b0;
			break;
		case 2:
			neededMaterials = 0b0;
			break;
		case 3:
			neededMaterials = 0b0;
			break;
		case 4:
			neededMaterials = 0b0110;
			break;
		case 5:
			neededMaterials = 0b1010;
			break;
		case 6:
			neededMaterials = 0b1100;
			break;
		case 7:
			neededMaterials = 0b1110000;
			break;
		case 8:
			neededMaterials = 0b10000000;
			break;
		case 9:
			neededMaterials = 0b11111110;
			break;

		default:
			neededMaterials = 0;
			break;
		}
	};

	/// <summary>
	/// 是地图中第几个工作台, (按字符串顺序排序
	/// </summary>
	int wbId;

	/// <summary>
	/// workbenchType
	/// </summary>
	int wbType;
	double x, y;

	/// <summary>
	/// 剩余生产时间
	/// </summary>
	int rpt = 0;

	/// <summary>
	/// 当前拥有的材料
	/// </summary>
	int materialsState = 0;

	/// <summary>
	/// 生产产品所需材料
	/// </summary>
	int neededMaterials;

	/// <summary>
	/// 是否有产品
	/// </summary>
	int ifHaveProduct = 0;

	bool isTarget = false;

	/// <summary>
	/// 获取 当前工作台缺少的材料的列表
	/// 判断是否需要n号材料, 可使用下述代码
	/// (getNeed() & (1 << n)) == 0;
	/// </summary>
	/// <returns>返回一个int, 第n位为 1, 则表示缺少该材料</returns>
	int getNeed()const {
		return neededMaterials ^ materialsState;
	}
	/// <summary>
	/// 判断是否需要该材料
	/// </summary>
	/// <param name="goodId">材料id [1,7]</param>
	/// <returns>如需要该材料则返回true, 输入为0或不需要该材料返回false</returns>
	bool ifNeed(int goodId)const{
		if (goodId == 0)
			return false;
		return (neededMaterials ^ materialsState) & (1 << (goodId));
	}

	void update() {
		int tempI;
		double tempD;
		/// <summary>
		/// 前三项为常数项
		/// </summary>
		cin >> tempI >> tempD >> tempD >> 
			rpt >> materialsState >> ifHaveProduct;
	}

};
