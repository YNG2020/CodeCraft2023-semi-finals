#pragma once

/// <summary>
/// bot����˶�״̬, 
/// ��Ŀ��, 
/// ��ǰ��Ŀ��һ��·��, ����Ŀ��һ, 
/// ��ǰ��Ŀ�����·��, ����Ŀ���, 
/// �ȴ���(���Կ��ܳ��ֵ�ԭ�صȴ����), 
/// ��ײ������(���Կ��ܳ��ֵ���ײ�������)
/// </summary>
enum BOT_MOVE_STATE {
	HW_NO_TARGET, HW_ON_THE_WAY_TO_TAR1, HW_ARRIVE_TAR1,
	HW_ON_THE_WAY_TO_TAR2, HW_ARRIVE_TAR2,
	HW_WAITING, HW_AVOIDING
};

/// <summary>
/// �ƶ����Ʋ���˶�״̬
/// ��ǰ��·��, �ڵ�����һ��·�����·��,����·����, ���·��,��·����(Ϊ�˿��ܷ������������)
/// </summary>
enum MOVE_CONTROL_STATE {
	MC_NO_PATH, MC_ON_THE_WAY, MC_ARRIVE_POINT, MC_COMPLETION_PATH, MC_OUT_OF_PATH
};

/// <summary>
/// �����˱���״̬
/// ���ڱ���, �ִ���õ㣬û���ڱ���
/// </summary>
enum BOT_AVOID_STATE {
	AVOIDING, AVOIDED, NOTAVOID
};

// fig1�ĳ�����
extern double f1_penalty2, f1_penalty3, f1_penalty3_1;
extern double f1_getMaxSpeed, f1_offset;
extern double f1_crash_t;
extern int f1_lastN[4];

// fig2�ĳ�����
extern double f2_penalty2, f2_penalty3, f2_penalty3_1;
extern double f2_getMaxSpeed, f2_offset;
extern double f2_crash_t;
extern int f2_lastN[4];

// fig3�ĳ�����
extern double f3_penalty2, f3_penalty3, f3_penalty3_1;
extern double f3_getMaxSpeed, f3_offset;
extern double f3_crash_t;
extern int f3_lastN[4];

// fig4�ĳ�����
extern double f4_penalty2, f4_penalty3, f4_penalty3_1;
extern double f4_getMaxSpeed, f4_offset;
extern double f4_crash_t;
extern int f4_lastN[4];

