//#define DEBUG
// ע�͵��������������ϴ�!!!!!!!!************************************ע�͵��������������ϴ�!!!!!!!!*****************************
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
/// �������ȡÿ֡��Ϣ, ���¹���̨��bot�ĳ�Ա����
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
/// ��ִ����handle֮��, ��ִ�иú���.
/// ���û���handle��ֻ��Ҫ����Ŀ���, �����ÿ��ƻ����˵ľ������
/// </summary>
/// <param name="K"></param>
void updateAfterHandle() {
    for (int i = 0; i < 4; ++i) {
        H_GlobalContext.botArr[i].updateAfterHandle();
    }
}

/// <summary>
/// ��ȡÿ֡, ���¹���̨��bot��Ϣ, �������ʽ����
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
    // ����bot��Ϊ
    stra->afterReadFrame(frameId);

    updateAfterHandle();
    cout << "OK" << endl;
    return frameId;
}

void init() {
    //hwcout.setCoutBuf();
    //hwendl.setCoutBuf();
}


// todo : ��������������֧��֡��ʱ, ��ѡ���Ż����²���:
// 1 �Ż�readFrame, ��Ϊ���д��
// 2 �Ż�handleFrame, �����б����Ĳ��ָ�Ϊ����Ч�Ĳ���, �����ظ�����
int main() {
    init();
    readMap();
    
    // �����ƶ��㷨�Ͳ����㷨�Ⱥ��޹�

    // new����ִ����ΪH_GlobalContext��ֵ��, ��������޷�Ԥ��Ĵ���
    stra = new strategy_2_baiyu_yng1();
    // Ϊbot���� �ƶ������㷨
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
