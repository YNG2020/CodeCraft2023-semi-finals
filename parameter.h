#pragma once

/// <summary>
/// bot层的运动状态, 
/// 无目标, 
/// 在前往目标一的路上, 到达目标一, 
/// 在前往目标二的路上, 到达目标二, 
/// 等待中(用以可能出现的原地等待情况), 
/// 碰撞避免中(用以可能出现的碰撞避免情况)
/// </summary>
enum BOT_MOVE_STATE {
	HW_NO_TARGET, HW_ON_THE_WAY_TO_TAR1, HW_ARRIVE_TAR1,
	HW_ON_THE_WAY_TO_TAR2, HW_ARRIVE_TAR2,
	HW_WAITING, HW_AVOIDING
};

/// <summary>
/// 移动控制层的运动状态
/// 当前无路径, 在到达下一个路径点的路上,到达路径点, 完成路径,在路径外(为了可能发生的意外情况)
/// </summary>
enum MOVE_CONTROL_STATE {
	MC_NO_PATH, MC_ON_THE_WAY, MC_ARRIVE_POINT, MC_COMPLETION_PATH, MC_OUT_OF_PATH
};

/// <summary>
/// 机器人避让状态
/// 正在避让, 抵达避让点，没有在避让
/// </summary>
enum BOT_AVOID_STATE {
	AVOIDING, AVOIDED, NOTAVOID
};

// fig1的超参数
extern double f1_penalty2, f1_penalty3, f1_penalty3_1;
extern double f1_getMaxSpeed, f1_offset;
extern double f1_crash_t;
extern int f1_lastN[4];

// fig2的超参数
extern double f2_penalty2, f2_penalty3, f2_penalty3_1;
extern double f2_getMaxSpeed, f2_offset;
extern double f2_crash_t;
extern int f2_lastN[4];

// fig3的超参数
extern double f3_penalty2, f3_penalty3, f3_penalty3_1;
extern double f3_getMaxSpeed, f3_offset;
extern double f3_crash_t;
extern int f3_lastN[4];

// fig4的超参数
extern double f4_penalty2, f4_penalty3, f4_penalty3_1;
extern double f4_getMaxSpeed, f4_offset;
extern double f4_crash_t;
extern int f4_lastN[4];

