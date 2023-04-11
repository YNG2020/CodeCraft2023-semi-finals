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
	/// �ǵ�ͼ�еڼ�������̨, (���ַ���˳������
	/// </summary>
	int wbId;

	/// <summary>
	/// workbenchType
	/// </summary>
	int wbType;
	double x, y;

	/// <summary>
	/// ʣ������ʱ��
	/// </summary>
	int rpt = 0;

	/// <summary>
	/// ��ǰӵ�еĲ���
	/// </summary>
	int materialsState = 0;

	/// <summary>
	/// ������Ʒ�������
	/// </summary>
	int neededMaterials;

	/// <summary>
	/// �Ƿ��в�Ʒ
	/// </summary>
	int ifHaveProduct = 0;

	bool isTarget = false;

	/// <summary>
	/// ��ȡ ��ǰ����̨ȱ�ٵĲ��ϵ��б�
	/// �ж��Ƿ���Ҫn�Ų���, ��ʹ����������
	/// (getNeed() & (1 << n)) == 0;
	/// </summary>
	/// <returns>����һ��int, ��nλΪ 1, ���ʾȱ�ٸò���</returns>
	int getNeed()const {
		return neededMaterials ^ materialsState;
	}
	/// <summary>
	/// �ж��Ƿ���Ҫ�ò���
	/// </summary>
	/// <param name="goodId">����id [1,7]</param>
	/// <returns>����Ҫ�ò����򷵻�true, ����Ϊ0����Ҫ�ò��Ϸ���false</returns>
	bool ifNeed(int goodId)const{
		if (goodId == 0)
			return false;
		return (neededMaterials ^ materialsState) & (1 << (goodId));
	}

	void update() {
		int tempI;
		double tempD;
		/// <summary>
		/// ǰ����Ϊ������
		/// </summary>
		cin >> tempI >> tempD >> tempD >> 
			rpt >> materialsState >> ifHaveProduct;
	}

};
