#pragma once
#include "strategy_2.h"
#include <vector>
#include "parameter.h"
#include "moveControl.h"
#include "moveControl_baiyu_yng1.h"
#include "basePathPlanning.h"

//#include <windows.h>

class strategy_2_baiyu_yng1 :public Strategy_2 {
	
public:

	strategy_2_baiyu_yng1() {};

	vector<vector<double>> wbDisHaveProd;
	vector<vector<double>> wbDisSqHaveProd;
	vector<vector<double>> wbTrueDisHaveProd;
	vector<vector<double>> wbDisNoProd;
	vector<vector<double>> wbDisSqNoProd;
	vector<vector<double>> wbTrueDisNoProd;
	vector<vector<int>> wbTypeArr = vector<vector<int>>(10, vector<int>());
	hwWorkbench curWb[4];	// ��¼�����˽���buy��sell�����󣬵�ǰ�����������Ĺ���̨λ��

	// �ϰ�����ӱ��
	bool block_lattice[100][100] = { false };

	// ��0���ʾ������Ƿ���Ϊ������Ŀ��, ��ʶ����̨������˱������������ͻ���Ļ�����˭ѡ�У�������λ��ʾ�����磬0b110����ζ����������˱�����1�Ż����2�Ż���Ļ�����������
	// ��1���ʾ�Ƿ���Ϊ�����Ŀ��
	vector<vector<int>> wbIfTar{ 50,vector<int>(2, 0) };

	// ��0���ʾĿǰ�ƶ����ڼ���Ŀ��, ֵΪ0��ʾ��Ŀ��, ֵΪ1��ʾ��һ��Ŀ��(����, ֵΪ2��ʾ�ڶ���Ŀ��(����
	// ��1��, ��2��Ϊ ����Ŀ���wbid
	vector<vector<int>> botTarId{ 4, vector<int>(3, 0) };
	// ��ǰĳ������Ĳ�Ʒ����Ҫ�ĳ̶ȣ���ϵ�������ѡ��
	char productNeed[8];
	// �����͹���̨�ĸ���
	int n1, n2, n3, n4, n5, n6, n7, n8, n9;
	// ��ʶ��һ��״̬�Ƿ�ɹ�����
	bool successUpdate[4] = { false };
	// ��¼ÿ�Ի����������ı���״̬��0��û�ڱ��ϣ�1����ʼ���ϣ�
	int avoidCrashState[4][4] = { {0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0} };
	double avoidCrashOmega[4] = { 0 };
	// ���ϱ���ʱ��
	int n_crash_t[4] = { 0 };
	int n_crash = 1;

	bool NOT_IN_WB[4] = {true,true,true,true};	// ��ʶ��������Ѱ·��ʼǰ���Ƿ�λ�ڹ���̨�������߼��ϵģ����������ϵģ�

	const int valueArr[8] = { 0,3000,3200,3400,7100,15000,8300,29000 };

	void initProductNeed(char* productNeed) {

		for (int i = 0; i < 8; ++i)
			productNeed[i] = 0;
		if (n9 >= 1)     // ���if-else��䣬Ӱ����ǻ����˵Ĺ�������������������Ʒ�Ƿ���Ҫ���粻����Ҫ���򲻹���
			for (int i = 1; i < 8; ++i)
				productNeed[i] = 10;
		else {
			if (n8 >= 1)
				productNeed[7] = 10;
			for (int i = 0; i < n7; ++i)
				for (int j = 4; j <= 6; ++j)
					if (wbArr[wbTypeArr[7][i]].ifNeed(j))
						++productNeed[j];
			for (int i = 0; i < n6; ++i) {
				if (wbArr[wbTypeArr[6][i]].ifNeed(3))
					++productNeed[3];
				if (wbArr[wbTypeArr[6][i]].ifNeed(2))
					++productNeed[2];
			}
			for (int i = 0; i < n5; ++i) {
				if (wbArr[wbTypeArr[5][i]].ifNeed(3))
					++productNeed[3];
				if (wbArr[wbTypeArr[5][i]].ifNeed(1))
					++productNeed[1];
			}
			for (int i = 0; i < n4; ++i) {
				if (wbArr[wbTypeArr[4][i]].ifNeed(2))
					++productNeed[2];
				if (wbArr[wbTypeArr[4][i]].ifNeed(1))
					++productNeed[1];
			}
		}

	}

	const double calDist(const double x1, const double y1, const double x2, const double y2)const {
		return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
	}

	const double calDistSq(const double x1, const double y1, const double x2, const double y2)const {
		return (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
	}

	void initDisV2AndWbTypeArr() {
		wbDisHaveProd.resize(wbNum);
		wbDisSqHaveProd.resize(wbNum);
		for (auto& one : wbDisHaveProd) {
			one.resize(wbNum, -1);
		}
		for (auto& one : wbDisSqHaveProd) {
			one.resize(wbNum, -1);
		}
		wbDisNoProd.resize(wbNum);
		wbDisSqNoProd.resize(wbNum);
		for (auto& one : wbDisNoProd) {
			one.resize(wbNum, -1);
		}
		for (auto& one : wbDisSqNoProd) {
			one.resize(wbNum, -1);
		}
		wbTrueDisHaveProd.resize(wbNum);
		wbTrueDisNoProd.resize(wbNum);
		for (auto& one : wbTrueDisHaveProd) {
			one.resize(wbNum, -1);
		}
		for (auto& one : wbTrueDisNoProd) {
			one.resize(wbNum, -1);
		}

		for (int i = 0; i < wbNum; ++i) {
			wbTypeArr[wbArr[i].wbType].emplace_back(i);

			for (int j = 0; j < i; ++j) {
				int jType = wbArr[j].wbType;
				bool dis_flag = false;
				switch (wbArr[i].wbType)
				{
				case 1:
					if (jType == 4 || jType == 5 || jType == 9) dis_flag = true;
					break;
				case 2:
					if (jType == 4 || jType == 6 || jType == 9) dis_flag = true;
					break;
				case 3:
					if (jType == 5 || jType == 6 || jType == 9) dis_flag = true;
					break;
				case 4:
					if (jType == 1 || jType == 2 || jType == 7 || jType == 9) dis_flag = true;
					break;
				case 5:
					if (jType == 1 || jType == 3 || jType == 7 || jType == 9) dis_flag = true;
					break;
				case 6:
					if (jType == 2 || jType == 3 || jType == 7 || jType == 9) dis_flag = true;
					break;
				case 7:
					if (jType == 4 || jType == 5 || jType == 6 || jType == 8 || jType == 9) dis_flag = true;
					break;
				case 8:
					if (jType == 7) dis_flag = true;
					break;
				case 9:
					if (jType != 8 && jType != 9) dis_flag = true;
					break;
				default:
					break;
				}
				if (dis_flag) {
					Path pathHaveProd = H_GlobalContext.pathPlan->Path_Wbi2Wbj_HaveProd[i][j];
					double _dis = pathHaveProd.size();
					if (_dis != 0) {
						wbDisHaveProd[i][j] = _dis;
						wbDisHaveProd[j][i] = _dis;
						wbDisSqHaveProd[i][j] = _dis * _dis;
						wbDisSqHaveProd[j][i] = _dis * _dis;

						double dist = 0;
						for (int i = 0; i < _dis - 1; ++i) {
							dist += calDist(pathHaveProd[i].x, pathHaveProd[i].y,
								pathHaveProd[i + 1].x, pathHaveProd[i + 1].y);
						}
						wbTrueDisHaveProd[i][j] = dist;
						wbTrueDisHaveProd[j][i] = dist;

					}
					Path pathNoProd = H_GlobalContext.pathPlan->Path_Wbi2Wbj_NoProd[i][j];
					_dis = pathNoProd.size();
					if (_dis != 0) {
						wbDisNoProd[i][j] = _dis;
						wbDisNoProd[j][i] = _dis;
						wbDisSqNoProd[i][j] = _dis * _dis;
						wbDisSqNoProd[j][i] = _dis * _dis;

						double dist = 0;
						for (int i = 0; i < _dis - 1; ++i) {
							dist += calDist(pathNoProd[i].x, pathNoProd[i].y,
								pathNoProd[i + 1].x, pathNoProd[i + 1].y);
						}
						wbTrueDisNoProd[i][j] = dist;
						wbTrueDisNoProd[j][i] = dist;
					}
				}
			}
		}
	}

	// botId ȥwbid1 ȡ��Ʒgoodid �͵�wbId2 �� Ȩ��ֵ
	double getWeight(int botId, int wbId1, int wbId2, int goodId) {

		double value = valueArr[goodId];
		int mstate = wbArr[wbId2].materialsState;
		for (int i = 0; i < 8; ++i) {
			if (mstate & 1) {		// �ж�wbArr[wbid2]�費��Ҫ�������
				value += valueArr[i] / 2;	// Ϊ�˲�ȫ����ӵ�Ȩ�أ����wbArr[wbid2]��ǰӵ�еĲ���Խ�࣬����Ȩ�ؾ�Խ���������ȱ���ϣ�������Ӧ�Ļ��ƴ���
			}
			mstate = mstate >> 1;
		}

		double _disBot2Wb1 = 0;

		if (botArr[botId].workbenchId != -1) {
			_disBot2Wb1 = wbTrueDisNoProd[botArr[botId].workbenchId][wbId1];
		}
		else {
			_disBot2Wb1 = calDist(botArr[botId].x, botArr[botId].y, wbArr[wbId1].x, wbArr[wbId1].y);
		}
		return pow(value, 1.0) / (pow(_disBot2Wb1 + wbTrueDisHaveProd[wbId1][wbId2], 2.0));
	}

	// botId����Ʒgoodid�͵�wbId2��Ȩ��ֵ
	double getWeight2(int botId, int id2, int goodId) {

		double value = valueArr[goodId];
		int mstate = wbArr[id2].materialsState;	// ���ڹ���̨8��9����ӵ�е�ԭ���Ͽ���˵һֱ��0�������Կ���
		for (int i = 0; i < 8; ++i) {
			if (mstate & 1) {		// �ж�wbArr[id2]�費��Ҫ�������
				value += valueArr[i] / 2;	// Ϊ�˲�ȫ����ӵ�Ȩ�أ����wbArr[id2]��ǰӵ�еĲ���Խ�࣬����Ȩ�ؾ�Խ���������ȱ���ϣ�������Ӧ�Ļ��ƴ���
			}
			mstate = mstate >> 1;
		}

		double _disBot2Wb2 = calDist(botArr[botId].x, botArr[botId].y, wbArr[id2].x, wbArr[id2].y);
		return value / _disBot2Wb2;
	}

	// �ڵ�ǰ��ͼ�У�����getWeight()�����Ȩ�أ�����botID���ʺ����������Ĺ���̨wbArr[id1]�����ʺ����������Ĺ���̨wbArr[id2]
	int loopId1ToId2(int tp1, int tp2, int botId, int goodId, int& wbId1, int& wbId2, double& maxWeight) {

		if (productNeed[tp1] <= 0)		// �鿴��ǰtp1���͵Ļ����Ƿ���Ҫ���漰�������
			return 0;

		// ����8�Ź���̨�Ƿ�����Ҫ��
		for (int id2 : wbTypeArr[tp2]) {
			if (id2 == 21 || id2 == 17)
				continue;
			// wbArr[id2]�Ƿ���Ҫ��tp1��Ʒ, �Ƿ���Ϊ������Ŀ�꣨��Ϊ��������Ŀ��Ļ����Ͳ�ѡ����
			if (wbArr[id2].ifNeed(tp1) && !(wbIfTar[id2][0] & (1 << tp1))) {
				for (int id1 : wbTypeArr[tp1]) {

					if (H_GlobalContext.pathPlan->Path_Boti2Wbj[botId][id1].pointList.size() == 0)
						continue;
					if (wbDisHaveProd[id1][id2] == -1)
						continue;

					//double x = botArr[botId].x, y = botArr[botId].y;
					//double dist = calDist(x, y, wbArr[id1].x, wbArr[id1].y);
					//bool checkProduct = wbArr[id1].ifHaveProduct;
					//if (frameId < 50 && wbArr[id1].rpt > 0 && dist > (wbArr[id1].rpt * 0.02 * 6))
					//	checkProduct = true;

					double x = botArr[botId].x, y = botArr[botId].y;
					
					double dist;
					if (H_GlobalContext.botArr[botId].curTarWbId != -1)
						dist = wbTrueDisNoProd[H_GlobalContext.botArr[botId].curTarWbId][wbId1];
					else
						dist = calDist(x, y, wbArr[id1].x, wbArr[id1].y);

					bool checkProduct = wbArr[id1].ifHaveProduct;
					if (wbArr[id1].rpt > 0 && dist > ((wbArr[id1].rpt) * 0.02 * H_GlobalContext.botArr[botId].vHistory) + 0.4) {
						checkProduct = true;
					}

					//if (wbArr[id1].ifHaveProduct && !wbIfTar[id1][1]) {		// �Ƿ񼴽�ӵ��tp2��Ʒ, �Ƿ���Ϊ�����Ŀ��
					if (checkProduct && !wbIfTar[id1][1]) {		// �Ƿ񼴽�ӵ��tp2��Ʒ, �Ƿ���Ϊ�����Ŀ��
						double nowWeight = getWeight(botId, id1, id2, goodId);

						if (nowWeight > maxWeight) {
							maxWeight = nowWeight;
							wbId1 = id1;
							wbId2 = id2;
						}
					}
				}
			}
		}
		return 0;
	};

	// �ڵ�ǰ��ͼ�У�����getWeight()�����Ȩ�أ�����botID���ʺ�����sell�Ĺ���̨wbArr[wb2Id]
	void loopWbId2(int botId, int goodId, int wb2Type, int& wb2Id, double& maxWeight) {

		// ������productNeeed����Ϊ��֤goodId����Ҫ
		for (int id2 : wbTypeArr[wb2Type]) {
			if (wbArr[id2].ifNeed(goodId) && !(wbIfTar[id2][0] & (1 << goodId))) {
				double nowWeight = getWeight2(botId, id2, goodId);
				if (nowWeight > maxWeight) {
					maxWeight = nowWeight;
					wb2Id = id2;
				}
			}
		}
	}

	// ��ͼ�ҵ���ѵ�buy����̨ ���������ﲻ���ǹ���̨���ޱ���������������������
	int findBestWbBuy(int botId, int goodId, double& maxWeight) {

		int tarId = -1;
		int id2 = botTarId[botId][2];
		for (int id1 : wbTypeArr[goodId]) {
			if (wbArr[id1].ifHaveProduct) {		// �Ƿ�ӵ��goodId��Ʒ
				double nowWeight = getWeight(botId, id1, id2, goodId);
				if (nowWeight > maxWeight) {
					maxWeight = nowWeight;
					tarId = id1;
				}
			}
		}
		return tarId;

	};

	int findNext(int botId) {
		int wbId1 = -1;
		int wbId2 = -1;

		double maxWeight = 0;

		loopId1ToId2(7, 8, botId, 7, wbId1, wbId2, maxWeight);

		loopId1ToId2(6, 7, botId, 6, wbId1, wbId2, maxWeight);
		loopId1ToId2(5, 7, botId, 5, wbId1, wbId2, maxWeight);
		loopId1ToId2(4, 7, botId, 4, wbId1, wbId2, maxWeight);

		loopId1ToId2(3, 6, botId, 3, wbId1, wbId2, maxWeight);
		loopId1ToId2(2, 6, botId, 2, wbId1, wbId2, maxWeight);

		loopId1ToId2(3, 5, botId, 3, wbId1, wbId2, maxWeight);
		loopId1ToId2(1, 5, botId, 1, wbId1, wbId2, maxWeight);

		loopId1ToId2(2, 4, botId, 2, wbId1, wbId2, maxWeight);
		loopId1ToId2(1, 4, botId, 1, wbId1, wbId2, maxWeight);

		loopId1ToId2(7, 9, botId, 7, wbId1, wbId2, maxWeight);
		loopId1ToId2(6, 9, botId, 6, wbId1, wbId2, maxWeight);
		loopId1ToId2(5, 9, botId, 5, wbId1, wbId2, maxWeight);
		loopId1ToId2(4, 9, botId, 4, wbId1, wbId2, maxWeight);
		loopId1ToId2(3, 9, botId, 3, wbId1, wbId2, maxWeight);
		loopId1ToId2(2, 9, botId, 2, wbId1, wbId2, maxWeight);
		loopId1ToId2(1, 9, botId, 1, wbId1, wbId2, maxWeight);

		if (wbId1 != -1 && wbId2 != -1) {
			botTarId[botId] = { 1,wbId1,wbId2 };
			wbIfTar[wbId1][1] = 1;

			if (wbArr[wbId2].wbType != 8 && wbArr[wbId2].wbType != 9)
				wbIfTar[wbId2][0] = (wbIfTar[wbId2][0] | (1 << wbArr[wbId1].wbType));
			return 0;
		}
		else {
			botTarId[botId] = { 0, 0, 0 };
			return 1;
		}
	}

	void findNext2(int botId, int& wbId1, int& wbId2, double& maxWeight) {

		if (botArr[botId].gootsId)		// ���������ĿǰЯ���Ż���Ͳ�����
			return;

		loopId1ToId2(7, 8, botId, 7, wbId1, wbId2, maxWeight);

		loopId1ToId2(6, 7, botId, 6, wbId1, wbId2, maxWeight);
		loopId1ToId2(5, 7, botId, 5, wbId1, wbId2, maxWeight);
		loopId1ToId2(4, 7, botId, 4, wbId1, wbId2, maxWeight);

		loopId1ToId2(3, 6, botId, 3, wbId1, wbId2, maxWeight);
		loopId1ToId2(2, 6, botId, 2, wbId1, wbId2, maxWeight);

		loopId1ToId2(3, 5, botId, 3, wbId1, wbId2, maxWeight);
		loopId1ToId2(1, 5, botId, 1, wbId1, wbId2, maxWeight);

		loopId1ToId2(2, 4, botId, 2, wbId1, wbId2, maxWeight);
		loopId1ToId2(1, 4, botId, 1, wbId1, wbId2, maxWeight);


		loopId1ToId2(7, 9, botId, 7, wbId1, wbId2, maxWeight);
		loopId1ToId2(6, 9, botId, 6, wbId1, wbId2, maxWeight);
		loopId1ToId2(5, 9, botId, 5, wbId1, wbId2, maxWeight);
		loopId1ToId2(4, 9, botId, 4, wbId1, wbId2, maxWeight);
		loopId1ToId2(3, 9, botId, 3, wbId1, wbId2, maxWeight);
		loopId1ToId2(2, 9, botId, 2, wbId1, wbId2, maxWeight);
		loopId1ToId2(1, 9, botId, 1, wbId1, wbId2, maxWeight);
	}

	void findNext3(int botId, int goodId, int& wb2Id, double& maxWeight) {

		if (goodId == 1) {
			loopWbId2(botId, 1, 4, wb2Id, maxWeight);
			loopWbId2(botId, 1, 5, wb2Id, maxWeight);
			loopWbId2(botId, 1, 9, wb2Id, maxWeight);
		}
		else if (goodId == 2) {
			loopWbId2(botId, 2, 4, wb2Id, maxWeight);
			loopWbId2(botId, 2, 6, wb2Id, maxWeight);
			loopWbId2(botId, 2, 9, wb2Id, maxWeight);
		}
		else if (goodId == 3) {
			loopWbId2(botId, 3, 5, wb2Id, maxWeight);
			loopWbId2(botId, 3, 6, wb2Id, maxWeight);
			loopWbId2(botId, 3, 9, wb2Id, maxWeight);
		}
		else if (goodId == 4) {
			loopWbId2(botId, 4, 7, wb2Id, maxWeight);
			loopWbId2(botId, 4, 9, wb2Id, maxWeight);
		}
		else if (goodId == 5) {
			loopWbId2(botId, 5, 7, wb2Id, maxWeight);
			loopWbId2(botId, 5, 9, wb2Id, maxWeight);
		}
		else if (goodId == 6) {
			loopWbId2(botId, 6, 7, wb2Id, maxWeight);
			loopWbId2(botId, 6, 9, wb2Id, maxWeight);
		}
		else if (goodId == 7) {
			loopWbId2(botId, 7, 8, wb2Id, maxWeight);
			loopWbId2(botId, 7, 9, wb2Id, maxWeight);
		}
	}

	void avoidCrash() {

		int crashNum[4] = { 0 };
		int crashId[4][4] = { {-1,-1,-1,-1},{-1,-1,-1,-1},{-1,-1,-1,-1},{-1,-1,-1,-1} };
		double vi_par, vi_ver, vj_par, vj_ver, xi, yi, xj, yj, boti_vx, boti_vy, botj_vx, botj_vy, angleTowPoint;
		double vij_par, vij_ver, botk_vx, botk_vy, botm_vx, botm_vy, cosAngle, sinAngle, dist, t_catch;
		double ri, rj, face_to_i, face_to_j;
		bool rotate_dir[4][4];		// ֵΪtrue����ʾ���߶�Ӧ����ʱ����ת��ֵΪfalse����ʾ���߶�Ӧ��˳ʱ����ת
		bool rotate_flag[4];		// ֵΪtrue����ʾӦ����ת��ֵΪfalse����ʾ��Ӧ����ת����ֹ���˵�ʱ����ת

		// ��ײ���ģ�飨Ŀǰֻ���۽���ײ��
		for (int i = 0; i < 4; ++i) {
			boti_vx = botArr[i].lineSpeed[0];
			boti_vy = botArr[i].lineSpeed[1];
			face_to_i = botArr[i].faceTo;

			double deltaAngle_i = fabs(atan2(boti_vy, boti_vx) - face_to_i);
			if (deltaAngle_i > M_PI) {
				deltaAngle_i = 2 * M_PI - deltaAngle_i;
			}

			if (deltaAngle_i > M_PI_2) {	// ˵����ʱ�������������˶�
				rotate_flag[i] = false;
			}
			else {
				rotate_flag[i] = true;
			}
		}

		for (int i = 0; i < 4; ++i) {

			xi = botArr[i].x;
			yi = botArr[i].y;
			boti_vx = botArr[i].lineSpeed[0];
			boti_vy = botArr[i].lineSpeed[1];
			face_to_i = botArr[i].faceTo;
			if (botArr[i].gootsId)
				ri = 0.53;
			else
				ri = 0.45;

			for (int j = i + 1; j < 4; ++j) {

				xj = botArr[j].x;
				yj = botArr[j].y;
				dist = calDist(xi, yi, xj, yj);
				botj_vx = botArr[j].lineSpeed[0];
				botj_vy = botArr[j].lineSpeed[1];
				face_to_j = botArr[j].faceTo;
				if (botArr[j].gootsId)
					rj = 0.53;
				else
					rj = 0.45;

				if (botj_vx * boti_vx + botj_vy * boti_vy > 0)		// �����ĵ����ͬ���˶����Ȳ���
					continue;

				double delta_vx = botj_vx - boti_vx, delta_vy = botj_vy - boti_vy;
				double deltax = xj - xi, deltay = yj - yi;

				if (delta_vx * deltax + delta_vy * deltay > 0)	// �����ĵ����������λ������������ٶ������Ƿ��򣨳ɶ۽ǣ���������˵���������������ڿ���
					continue;

				angleTowPoint = atan2(yj - yi, xj - xi);
				cosAngle = cos(angleTowPoint);
				sinAngle = sin(angleTowPoint);

				vi_par = boti_vx * cosAngle + boti_vy * sinAngle;	// i���������ߵ�ˮƽ�ٶ�
				vi_ver = boti_vx * sinAngle - boti_vy * cosAngle;	// i���������ߵĴ�ֱ�ٶ�
				vj_par = botj_vx * cosAngle + botj_vy * sinAngle;	// j���������ߵ�ˮƽ�ٶ�
				vj_ver = botj_vx * sinAngle - botj_vy * cosAngle;	// j���������ߵĴ�ֱ�ٶ�

				vij_par = vj_par - vi_par;	// ��i��j��j��iֱ�ӿ������ٶ�
				vij_ver = vj_ver - vi_ver;	// ��i��j��j��i��ֱԶ����ٶ�

				t_catch = dist / vij_par;

				if (rotate_flag[i] && rotate_flag[j]) {	// ���߶�û���������˶�
					if (fabs(t_catch) > f1_crash_t)	// ��ײʱ�仹ʣ�Ƚ϶�Ļ�����������
						continue;
				}
				else {	// ������һ�����������˶�
					if (fabs(t_catch) > 1 * f1_crash_t)	// ��ײʱ�仹ʣ�Ƚ϶�Ļ�����������
						continue;
				}

				if (fabs(vij_ver * t_catch) < (ri + rj)) {	// ˵����ʱ�����ǵ��ٶȱ��ֲ���Ļ�������t_catch��ʱ�䣬���Ǿͻ���ײ

					++crashNum[i];
					++crashNum[j];
					crashId[i][j] = 1;
					crashId[j][i] = 1;
					avoidCrashState[i][j] = 1;
					avoidCrashState[j][i] = 1;
					n_crash_t[i] = n_crash;
					n_crash_t[j] = n_crash;

					if (deltax * delta_vy - deltay * delta_vx < 0) {	// �����Ĳ����ֵΪ���������pi�Ľ��ٶ���ת
						rotate_dir[i][j] = true;
						rotate_dir[j][i] = true;
					}
					else {		// ֵΪ���������-pi�Ľ��ٶ���ת
						rotate_dir[i][j] = false;
						rotate_dir[j][i] = false;
					}
				}
				else {		// ����û�м�⵽��ײ�����ǰһ֡�������ڱ���״̬����ô���ڽ�����ϱ���״̬

					if (avoidCrashState[i][j] && n_crash_t[i]) {
						botArr[i].rotate(avoidCrashOmega[i]);
						botArr[j].rotate(avoidCrashOmega[j]);
						--n_crash_t[i];
						if (n_crash_t[i] == 0) {
							avoidCrashState[i][j] = 0;
							avoidCrashState[j][i] = 0;
						}
					}

				}
			}
		}

		// ��ײ����ģ��
		for (int k = 0; k < 4; ++k) {

			if (crashNum[k] == 0)	// û�м���Ҫ��ײ��Ŀ�꣬��������
				continue;

			botk_vx = botArr[k].lineSpeed[0];
			botk_vy = botArr[k].lineSpeed[1];

			if (crashNum[k] >= 2) {	// ������õ�����ײĿ�������ڵ���2����ֻ�����ٴ��������������
				botArr[k].forward(0.5 * sqrt(botk_vx * botk_vx + botk_vy * botk_vy));

				// ͬʱָʾ�����Ļ����ˣ����ؽ�k��Ϊ��ײĿ�꣬��������
				for (int m = 0; m < 4; ++m) {
					if (crashId[k][m] != -1) {
						--crashNum[m];
						--crashNum[k];
						crashId[k][m] = -1;
						crashId[m][k] = -1;
						avoidCrashState[k][m] = 0;
						avoidCrashState[m][k] = 0;
					}
				}
				continue;
			}

			// ��ʼ��ײ����
			for (int m = 0; m < 4; ++m) {
				if (crashId[k][m] != -1) {
					botm_vx = botArr[m].lineSpeed[0];
					botm_vy = botArr[m].lineSpeed[1];
					double vk = sqrt(botk_vx * botk_vx + botk_vy * botk_vy);
					double vm = sqrt(botm_vx * botm_vx + botm_vy * botm_vy);
					if (vk > 6)
						vk = 6;
					if (vm > 6)
						vm = 6;

					if (rotate_dir[k][m]) {
						if (rotate_flag[m]) {
							botArr[m].rotate(M_PI);
							botArr[m].forward(vm);
							avoidCrashOmega[m] = M_PI;
						}
						else {	// ���������ں��ˣ��û����˲����б���
							botArr[m].rotate(botArr[m].angleSpeed);
							botArr[m].forward(-vm);
							avoidCrashOmega[m] = botArr[m].angleSpeed;
						}
						if (rotate_flag[k]) {
							botArr[k].rotate(M_PI);
							botArr[k].forward(vk);
							avoidCrashOmega[k] = M_PI;
						}
						else {	// ���������ں��ˣ��û����˲����б���
							botArr[k].rotate(botArr[k].angleSpeed);
							botArr[k].forward(-vk);
							avoidCrashOmega[k] = botArr[k].angleSpeed;
						}
					}
					else {
						if (rotate_flag[m]) {
							botArr[m].rotate(-M_PI);
							botArr[m].forward(vm);
							avoidCrashOmega[m] = -M_PI;
						}
						else {	// ���������ں��ˣ��û����˲����б���
							botArr[m].rotate(botArr[m].angleSpeed);
							botArr[m].forward(-vm);
							avoidCrashOmega[m] = botArr[m].angleSpeed;
						}
						if (rotate_flag[k]) {
							botArr[k].rotate(-M_PI);
							botArr[k].forward(vk);
							avoidCrashOmega[k] = -M_PI;
						}
						else {	// ���������ں��ˣ��û����˲����б���
							botArr[k].rotate(botArr[k].angleSpeed);
							botArr[k].forward(-vk);
							avoidCrashOmega[k] = botArr[k].angleSpeed;
						}

					}
					crashId[k][m] = -1;
					crashId[m][k] = -1;
					--crashNum[m];
					--crashNum[k];
				}
			}

		}
	}

	void competeTar(int botId);

	/// <summary>
	/// �������·�
	/// </summary>
	virtual void afterReadMap();

	/// <summary>
	/// �������·�
	/// </summary>
	virtual void afterReadFrame(int frameId);
};

void strategy_2_baiyu_yng1::afterReadMap() {

	initDisV2AndWbTypeArr();
	n1 = wbTypeArr[1].size(), n2 = wbTypeArr[2].size(), n3 = wbTypeArr[3].size();
	n4 = wbTypeArr[4].size(), n5 = wbTypeArr[5].size(), n6 = wbTypeArr[6].size();
	n7 = wbTypeArr[7].size(), n8 = wbTypeArr[8].size(), n9 = wbTypeArr[9].size();
	//Sleep(2000);

}

void strategy_2_baiyu_yng1::afterReadFrame(int frameId) {

	initProductNeed(productNeed);

	if (H_GlobalContext.frameId == 750) {
		int a = 1;
	}

	for (int i = 0; i < 4; ++i) {

		hwRobot_2& bot = botArr[i];

		// ������в�Ʒ�������϶����Ի��߽���ĳ������̨ΪĿ�꣬���������ĵ�1����Ʒ����
		if (bot.gootsId > 0 && bot.gootsId < 7) // ������һ����û��update�ɹ���ֻҪ�����˲�Ʒ���ͻ����ĵ�1����Ʒ����
			--productNeed[bot.gootsId];
		// ��������в�Ʒ����֮ǰ�ܳɹ�����״̬����Ŀǰ��ͼȥ��ĳ����Ʒ(����)������Ҳ���ĵ�1����Ʒ����
		if (bot.gootsId == 0 && successUpdate[i]) // ���֮ǰ���ܳɹ�����״̬��˵������������Ʒ���Ҳ�����һ��Ŀ�꣬��botTarget[i]��Ȼ������������Ʒ�Ĺ���̨id
			--productNeed[wbArr[botTarId[i][1]].wbType];
		// ��̬����Ŀ��
		//competeTar(i);

		double botVx = bot.lineSpeed[0], botVy = bot.lineSpeed[1];
		if (fabs(botVx) > 0.5 || fabs(botVy) > 0.5)
			bot.vHistory = 0.8 * bot.vHistory + 0.2 * sqrt(botVx* botVx + botVy * botVy);

		double botX = bot.x, botY = bot.y;

		bool ifHaveProd = bot.gootsId > 0;
		if (botTarId[i][0] == 0) {
			bot.stickTime = 0;
		}
		else if (fabs(botVx) < 0.5 && fabs(botVy) < 0.5) {
			bot.stickTime = bot.stickTime + 1;
		}
		else {
			bot.stickTime = 0;
		}
		if (bot.stickTime > 25 && bot.avoidState != AVOIDED) {
			Point temPoint = Point(botX, botY);
			bot.mc->setPath(H_GlobalContext.pathPlan->getPath(temPoint, bot.mc->finalTarPoint, ifHaveProd));
		}

		int botLatticeX = -1, botLatticeY = -1;
		getLatticeIndex(botX, botY, botLatticeX, botLatticeY);
		bot.inNarrowZone = false;
		for (int y = botLatticeY - 2; y < botLatticeY + 3; ++y) {
			for (int x = botLatticeX - 2; x < botLatticeX + 3; ++x) {
				if (x < 0 || x > 99 || y < 0 || y > 99 || H_GlobalContext.gameMap[y][x] != '#') {
					continue;
				}
				else {
					bot.inNarrowZone = true;
					y = botLatticeY + 3;// ǿ���˳����ѭ��
					break;				// ���˳��ڲ�ѭ��
				}
			}
		}

		int n = bot.mc->nowPath.size();
		if (n == 0) {
			for (int j = 0; j <= H_GlobalContext.jamBufferSize - 1; ++j) {
				bot.jamDetectBuffer[j] = lattice(-1, -1);
				bot.jamDetectBufferPathId[j] = -1;
			}
			continue;
		}
		
		double minDist = 100000, curDist;
		int minPointId = bot.mc->nowPointId;
		for (int j = 0; j < n; ++j) {
			curDist = calDistSq(botX, botY, bot.mc->nowPath[j].x, bot.mc->nowPath[j].y);
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

	for (int i = 0; i < 4; ++i) {

		hwRobot_2& bot = botArr[i];

		if (botTarId[i][0] != 0) {		// �����ʱ������i��Ŀ��

			bot.mc->move();
			int state = bot.state;

			if (state == BOT_MOVE_STATE::HW_ARRIVE_TAR1) {	// �������Ŀ��
				if (botTarId[i][0] == 1) {		// ���Ŀǰ�������Ҫ�����Ŀ��
					if ((15000 - frameId) * 0.02 * bot.vHistory < wbDisHaveProd[botTarId[i][1]][botTarId[i][2]])
						continue;
					curWb[i] = wbArr[botTarId[i][1]];

					bot.buy();
					// ��ʱ������������������ֹͣ��������ȡ����Ʒ��Ӧ�ý����������ֶ���������̨��״̬��rpt��materialsState��ifHaveProduct��
					if (wbArr[botTarId[i][1]].getNeed() == 0 && wbArr[botTarId[i][1]].rpt == 0) {
						wbArr[botTarId[i][1]].materialsState = 0;
						if (wbArr[botTarId[i][1]].wbType == 6) {
							++productNeed[3];
							++productNeed[2];
							wbArr[botTarId[i][1]].rpt = 500;
						}
						else if (wbArr[botTarId[i][1]].wbType == 5) {
							++productNeed[3];
							++productNeed[1];
							wbArr[botTarId[i][1]].rpt = 500;
						}
						else if (wbArr[botTarId[i][1]].wbType == 4) {
							++productNeed[2];
							++productNeed[1];
							wbArr[botTarId[i][1]].rpt = 500;
						}
						else if (wbArr[botTarId[i][1]].wbType == 7) {
							++productNeed[6];
							++productNeed[5];
							++productNeed[4];
							wbArr[botTarId[i][1]].rpt = 1000;
						}
					}
					wbArr[botTarId[i][1]].ifHaveProduct = 0;
					wbIfTar[botTarId[i][1]][1] = 0;
					botTarId[i][0] = 2;
					botArr[i].curTarWbId = botTarId[i][2];
					// �ֶ����Ļ�����ӵ�л����״̬
					botArr[i].gootsId = curWb[i].wbType;
					bot.mc->finalTarPoint = Point(wbArr[botTarId[i][2]].x, wbArr[botTarId[i][2]].y);
					bot.mc->setPath(H_GlobalContext.pathPlan->getPath(curWb[i], wbArr[botTarId[i][2]], true));

					bot.mc->move();

				}
				else if (botTarId[i][0] == 2) {	// ���Ŀǰ�������Ҫ�۳���Ŀ��
					bot.sell();
					curWb[i] = wbArr[botTarId[i][2]];

					if (wbArr[bot.workbenchId].wbType != 9)
						wbArr[bot.workbenchId].materialsState ^= (1 << bot.gootsId);
					if (wbArr[botTarId[i][2]].getNeed() == 0 && wbArr[botTarId[i][2]].rpt == -1) {  // �����ʱ����̨����Ҫ�κ�ԭ�ϣ��ҿ�������
						wbArr[botTarId[i][2]].materialsState = 0;     // ������������
						if (wbArr[botTarId[i][2]].wbType == 6) {
							++productNeed[3];
							++productNeed[2];
							wbArr[botTarId[i][2]].rpt = 500;
						}
						else if (wbArr[botTarId[i][2]].wbType == 5) {
							++productNeed[3];
							++productNeed[1];
							wbArr[botTarId[i][2]].rpt = 500;
						}
						else if (wbArr[botTarId[i][2]].wbType == 4) {
							++productNeed[2];
							++productNeed[1];
							wbArr[botTarId[i][2]].rpt = 500;
						}
						else if (wbArr[botTarId[i][2]].wbType == 7) {
							++productNeed[6];
							++productNeed[5];
							++productNeed[4];
							wbArr[botTarId[i][2]].rpt = 1000;
						}
					}
					// �ͷŹ���̨��goodID�����
					if (wbArr[bot.workbenchId].wbType != 8 && wbArr[bot.workbenchId].wbType != 9)
						wbIfTar[botTarId[i][2]][0] ^= (1 << bot.gootsId);

					botTarId[i][0] = 0;
					// �ֶ����Ļ�����ӵ�л����״̬
					botArr[i].gootsId = 0;

					if (findNext(i) != 1) {		// ������һ�β��ң��ɹ������
						successUpdate[i] = true;
						--productNeed[wbArr[botTarId[i][1]].wbType];
						botArr[i].curTarWbId = botTarId[i][1];
						bot.mc->finalTarPoint = Point(wbArr[botTarId[i][1]].x, wbArr[botTarId[i][1]].y);
						bot.mc->setPath(H_GlobalContext.pathPlan->getPath(curWb[i], wbArr[botTarId[i][1]], false));
						
						bot.mc->move();
					}
					else {					// ��������Ҳ���Ŀ�꣬����ʱͣ����
						successUpdate[i] = false;
						botArr[i].curTarWbId = -1;
						bot.forward(0);
					}
				} 
			}
		}
		else {							// �����ʱ������iû��Ŀ��
			if (findNext(i) != 1) {		// ������һ�β��ң��ɹ������
				successUpdate[i] = true;
				--productNeed[wbArr[botTarId[i][1]].wbType];
				botArr[i].curTarWbId = botTarId[i][1];
				if (!NOT_IN_WB[i]) {	// �ڹ���̨����
					bot.mc->finalTarPoint = Point(wbArr[botTarId[i][1]].x, wbArr[botTarId[i][1]].y);
					bot.mc->setPath(H_GlobalContext.pathPlan->getPath(curWb[i], wbArr[botTarId[i][1]], false));
				}	
				else {
					bot.mc->finalTarPoint = Point(wbArr[botTarId[i][1]].x, wbArr[botTarId[i][1]].y);
					bot.mc->setPath(H_GlobalContext.pathPlan->getPath(botArr[i], wbArr[botTarId[i][1]]));
					NOT_IN_WB[i] = false;
				}

				bot.mc->move();
			}
			else {					// ��������Ҳ���Ŀ�꣬����ʱͣ����
				successUpdate[i] = false;
				botArr[i].curTarWbId = -1;
				bot.forward(0);
			}
		}
	}

	avoidCrash();

	for (int i = 0; i < 4; ++i) {

		hwRobot_2& bot = botArr[i];
		if (bot.stickTime > 50 && bot.avoidState != AVOIDED) {
			bot.forward(-2);
			bot.rotate(M_PI);
		}
	}

}

void strategy_2_baiyu_yng1::competeTar(int botId) {

	hwRobot_2& bot = botArr[botId];
	double penalty1 = 1.0;		// �����˵�Ŀ��ĳͷ�����
	double penalty2 = f1_penalty2;		// �ı��Լ��Ľ���Ŀ��ĳͷ�����
	double penalty3 = f1_penalty3;		// �ı��Լ��ĳ���Ŀ��ĳͷ�����
	int botIdid1 = botTarId[botId][1];	// botIdbuyĿ��id
	int botIdid2 = botTarId[botId][2];	// botIdsellĿ��id
	int goodId = wbArr[botIdid1].wbType;	// buy�Ļ�������

	if (botTarId[botId][0] == 1) {		// �����ʱ������botId��ǰ�������buyĿ��
		// ���ﲻ��Ҫ���productNeed����ΪbotId��һ�θ���ʱ���Ѿ�֪����goodId����Ҫ��

		bool robFlag = false;	// ��ʶ�����˵�Ŀ���Ƿ����ɹ���ֻ����Ļ����˵�buy��
		double botIdLastWeight = getWeight(botId, botIdid1, botIdid2, goodId);  // δ����Ŀ��ʱ��Ȩֵ
		double botIdCurWeight = botIdLastWeight;
		//int newTarId = findBestWbBuy(botId, goodId, botIdCurWeight);
		//if (newTarId != -1 && (botIdid1 != newTarId)) {	// ���ܷ��ҵ��µ�buyĿ�꣬�Ҳ����Լ����Լ�

		//	for (int i = 0; i < 4; ++i) {
		//		if (newTarId == botTarId[i][1]) {	// �������buy��Ŀ����û�б�����������ֻ���ܱ�һ���������������Ҳ������Լ���
		//			int wbId1 = -1, wbId2 = -1;
		//			double iLastWeight = getWeight(i, botTarId[i][1], botTarId[i][2], goodId);	// ��ʱ������i�϶�û�е���newTarId����Ϊ����һ������ͻ��ͷ����buyĿ��
		//			double iCurWeight = iLastWeight;

		//			findNext2(i, wbId1, wbId2, iCurWeight);	// �϶������Լ����Լ���Ŀ�꣬��Ϊ�Լ��Ѿ����Լ���Ŀ������ס��

		//			if (wbId1 != -1 && wbId2 != -1 && (botIdCurWeight - botIdLastWeight) > penalty1 * (iLastWeight - iCurWeight)) {	// Ȩֵ�������Ȩֵ˥��

		//				robFlag = true;

		//				int iId1 = botTarId[i][1];	// ������i��buy����̨
		//				int iId2 = botTarId[i][2];	// ������i��sell����̨

		//				// �ͷ�ԭ��������botId��buy����̨������ˣ���i��buy����̨������˺�sell����̨�������
		//				wbIfTar[botIdid1][1] = 0;								// �ͷŹ���̨botIdid1�������
		//				if (wbArr[iId2].wbType != 8 && wbArr[iId2].wbType != 9)
		//					wbIfTar[iId2][0] ^= (1 << goodId);					// �ͷŹ���̨iId2�������
		//				wbIfTar[iId1][1] = 0;									// �ͷŹ���̨iId1�������

		//				// ���»����˵�Ŀ�깤��̨
		//				botTarId[botId] = { 1,newTarId,botIdid2 };
		//				botTarId[i] = { 1,wbId1,wbId2 };

		//				// �����µĻ�����botId��buy����̨������ˣ���i��buy����̨������˺�sell����̨�������
		//				wbIfTar[newTarId][1] = 1;								// ��������̨newTarId�������
		//				if (wbArr[wbId2].wbType != 8 && wbArr[wbId2].wbType != 9)
		//					wbIfTar[wbId2][0] = (wbIfTar[wbId2][0] | (1 << wbArr[wbId1].wbType));	// ��������̨wbId2�������
		//				wbIfTar[wbId1][1] = 1;									// ��������̨wbId1�������

		//				// ���»������ƶ���Ŀ��
		//				botArr[botId].setMoveTarget(wbArr[botTarId[botId][1]]);
		//				botArr[botId].moveUntillArriveTarget_YNG1();
		//				botArr[i].setMoveTarget(wbArr[botTarId[i][1]]);
		//				botArr[i].moveUntillArriveTarget_YNG1();
		//			}
		//			break;

		//		}
		//	}

		//}

		if (robFlag == false) {		// ��������˵�Ŀ�겻�ɹ�������ͼ��û�л�����������buy����̨���Ҹ��º���С׬

			double x = botArr[botId].x, y = botArr[botId].y;
			double minDist = 1000.0;
			int tarId = -1;
			for (int i = 0; i < wbTypeArr[goodId].size(); ++i) {
				int id = wbTypeArr[goodId][i];
				if (wbArr[id].ifHaveProduct && !wbIfTar[id][1]) {    // �л����δ���������Լ���Ŀ���Ѿ����Լ������ˣ������л��ᱻѡ��
					double dist = calDist(x, y, wbArr[id].x, wbArr[id].y);
					if (dist < minDist) {
						tarId = id;
						minDist = dist;
					}
				}
			}
			if (tarId != -1) {

				double botIdCurWeight = getWeight(botId, tarId, botIdid2, goodId);
				if (botIdCurWeight <= penalty2 * botIdLastWeight)
					return;

				// �ͷ�ԭ���Ĺ���̨�������
				wbIfTar[botIdid1][1] = 0;
				// ������Ŀ�깤��̨�������
				wbIfTar[tarId][1] = 1;

				// ���»����˵�Ŀ�깤��̨
				botTarId[botId] = { 1,tarId,botIdid2 };

				// ���»������ƶ���Ŀ��
				/*botArr[botId].setMoveTarget(wbArr[botTarId[botId][1]]);
				botArr[botId].moveUntillArriveTarget_YNG1();*/

			}
		}
	}

	else if (botTarId[botId][0] == 2) {	// �����ʱ������botId��ǰ�������sellĿ��

		double botIdLastWeight = getWeight2(botId, botIdid2, goodId);  // δ����Ŀ��ʱ��Ȩֵ
		double botIdCurWeight = botIdLastWeight;
		int tarId = -1;
		findNext3(botId, goodId, tarId, botIdCurWeight);	// �϶������Լ����Լ���Ŀ�꣬��Ϊ�Լ��Ѿ����Լ���Ŀ������ס��
		if (tarId != -1) {

			double bot_vx = bot.lineSpeed[0], bot_vy = bot.lineSpeed[1];
			double wb_x = wbArr[tarId].x, wb_y = wbArr[tarId].y;
			double bot_x = bot.x, bot_y = bot.y;
			double deltax = wb_x - bot_x, deltay = wb_y - bot_y;
			double face_to_bot = botArr[botId].faceTo;
			double deltaAngle = fabs(atan2(deltay, deltax) - face_to_bot);
			if (deltaAngle > M_PI) {
				deltaAngle = 2 * M_PI - deltaAngle;
			}

			if (deltaAngle > M_PI_2) {	// ˵����ʱ����Ŀ���ڻ����˳���ı������������ĳͷ�����
				if (botIdCurWeight <= f1_penalty3_1 * botIdLastWeight) {
					return;
				}

			}

			if (botIdCurWeight <= penalty3 * botIdLastWeight)
				return;

			// �ͷ�ԭ���Ĺ���̨�������
			wbIfTar[botIdid2][0] ^= (1 << goodId);
			// ������Ŀ�깤��̨�������
			if (wbArr[tarId].wbType != 8 && wbArr[tarId].wbType != 9)
				wbIfTar[tarId][0] = (wbIfTar[tarId][0] | (1 << goodId));
			botTarId[botId][2] = tarId;
			// ���»������ƶ���Ŀ��
			/*botArr[botId].setMoveTarget(wbArr[tarId]);
			botArr[botId].moveUntillArriveTarget_YNG1();*/

		}
	}

}
