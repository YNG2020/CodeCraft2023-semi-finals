//#define DEBUG
// 注释掉上面这条后再上传!!!!!!!!************************************注释掉上面这条后再上传!!!!!!!!*****************************
  //                       .,!<i,.                        
  //                     ';><il!<<I`                      
  //                   ^l<>!l!!!l!><!"                    
  //                .,i<i!l!!!!!!!lli<>:.                 
  //              ';><!ll!!!!!!!!!!!!l!><I`               
  //            ^!<>!l!!!!!!!!!!!!!!!!!l!><!"             
  //         .:><ill!!!!!!!!!!!!!!!!!!!!!lli<>;'          
  //       `I<>!l!!!!!!!!!!!!!!!!!!!!!!!!!!!l!><l^        
  //     "!<>!l!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!l!i<i,.     
  //  ':><i!l!!!!ll!!!!!!!!!!!!!!!!!!!!!!!ll!!!!lli<>;'   
  //.:~~>iiiiiiii<>l!!!!!!!!!!!!!!!!!!!!!!i<iiiiiiii>~+I. 
  // '...........I+l!!!!!!!!!!!!!!!!!!!!!l~i...........'  
  //             ;+l!!!!!!!!!!!!!!!!!!!!!l~!              
  //             ;+l!!!!!!!!!!!!!!!!!!!!!l~!              
  //             ;+l!!!!!!!!!!!!!!!!!!!!!l~!              
  //             ;+l!!!!!!!!!!!!!!!!!!!!!l~!              
  //             ;+l!!!!!!!!!!!!!!!!!!!!!l~!              
  //             ;+l!!!!!!!!!!!!!!!!!!!!!l~!              
  //             ;+l!!!!!!!!!!!!!!!!!!!!!l~!              
  //             ;+l!!!!!!!!!!!!!!!!!!!!!l~!              
  //             ;_!!!!!!!!!!!!!!!!!!!!!!!+!              
  //             ^!IIIIIIIIIIIIIIIIIIIIIII!"                                                                                                                               
#include <iostream>
#include <vector>
#include "hwRobot_2.h"
#include "workbench.h"

#include "strategy_2.h"
#include "strategy_2_baiyu_yng_1.h"

#include"moveControl.h"
#include"moveControl_baiyu_yng1.h"

#include "basePathPlanning.h"
#include "BFSPathPlanning.h"
#include "Path.h"

//#include "hwcout.h"

using namespace std;

Strategy_2* stra = nullptr;
/////////////////

int readMap() {
    char c;
    int botId = 0;
    int wbId = 0;
    for (int y = 99; y >= 0; --y) {
        for (int x = 0; x < 100; ++x) {
            cin >> c;
            H_GlobalContext.gameMap[y][x] = c;
            if (c == '.') {
                continue;
            }   
            else if (c == 'A') {
                H_GlobalContext.botArr[botId].init(botId, x, y);
                ++botId;
            }
            else if (c == '#') {
                continue;
            }  
            else {
                H_GlobalContext.wbArr[wbId].init(wbId, x, y, c - '0');
                ++wbId;
            }
        }
    }

    cin >> c >> c;

    H_GlobalContext.wbNum = wbId;
    return wbId;
}

/// <summary>
/// 从输入读取每帧信息, 更新工作台和bot的成员变量
/// </summary>
/// <param name="K"></param>
void updateBeforeHandle(int K) {
    for (int i = 0; i < K; ++i) {
        H_GlobalContext.wbArr[i].update();
    }
    for (int i = 0; i < 4; ++i) {
        H_GlobalContext.botArr[i].updateBeforeHandle();
    }
}
/// <summary>
/// 在执行完handle之后, 再执行该函数.
/// 让用户在handle中只需要设置目标点, 而不用控制机器人的具体参数
/// </summary>
/// <param name="K"></param>
void updateAfterHandle() {
    for (int i = 0; i < 4; ++i) {
        H_GlobalContext.botArr[i].updateAfterHandle();
    }
}

/// <summary>
/// 读取每帧, 更新工作台与bot信息, 并处理格式问题
/// </summary>
/// <returns></returns>
int readFrame() {
    int tempI;
    int frameId;
    int money;
    int K;
    cin >> frameId;
    if (frameId == EOF || frameId < 0)
        return frameId;
    cin >> money >> K;
    H_GlobalContext.money = money;
    updateBeforeHandle(K);

    char tempC;
    cin >> tempC >> tempC;// OK

    cout << frameId << endl;

    H_GlobalContext.frameId = frameId;
    // 控制bot行为
    stra->afterReadFrame(frameId);

    updateAfterHandle();
    cout << "OK" << endl;
    return frameId;
}

void init() {
    //hwcout.setCoutBuf();
    //hwendl.setCoutBuf();
}


// todo : 当计算量不足以支撑帧率时, 可选择优化以下部分:
// 1 优化readFrame, 改为快读写法
// 2 优化handleFrame, 将其中遍历的部分改为更高效的查找, 避免重复计算
int main() {
    init();
    readMap();
    
    // 配置移动算法和策略算法先后无关

    // new必须执行在为H_GlobalContext赋值后, 否则会有无法预测的错误
    stra = new strategy_2_baiyu_yng1();
    // 为bot配置 移动控制算法
    for (int i = 0; i < 4; ++i) {
        H_GlobalContext.botArr[i].mc =
            new moveControl_baiyu_yng1(H_GlobalContext.botArr[i]);
    }
    H_GlobalContext.pathPlan = new BFSPathPlanning();
    H_GlobalContext.pathPlan->initPaths(H_GlobalContext.gameMap);

    stra->afterReadMap();
    cout << "OK" << flush;

    int frameId = 0;
    while (frameId >= 0) {
        frameId = readFrame();
    }

    delete stra;
    return 0;
}
