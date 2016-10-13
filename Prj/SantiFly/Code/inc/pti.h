#ifndef _PTI_H_
#define _PTI_H_

#include "include.h"

#define FLY_FREQUENCY		50						// 电调频率
#define PTI_MIN				500						// 电调最小占空比
#define PTI_MAX				1000					// 电调最大占空比
#define PTI_CENTER			(PTI_MIN+PTI_MAX)/2		// 电调中间占空比

#define POWER_MIN			585						// 油门最小值
#define POWER_MAX			725						// 油门最大值
#define POWER_ATTITUDE		710						// 油门一键起飞时的高度

#define UNLOCK_DELAY		2000					// 解锁延时
#define TIMER_PERIOD		10						// 定时器周期

#define COM_DIRECTION_TIME	5						// 方向改变时正向持续时间（正向时间 + 反向时间 < 定时器周期）
#define DIS_DIRECTION_TIME	2						// 方向改变时反向持续时间（正向时间 + 反向时间 < 定时器周期）
#define COM_PTI_CHANGE		35						// 方向改变时正向电调的摆动范围
#define DIS_PTI_CHANGE		40						// 方向改变时反向电调的摆动范围


#define FTM_FLY				FTM0
#define CH_ROLL				FTM_CH0
#define CH_PITCH			FTM_CH1
#define CH_ATTITUDE			FTM_CH2
#define CH_YAW				FTM_CH3
#define CH_SURE_ATTITUDE	FTM_CH4
#define CH_SERVO			FTM_CH5

/* 飞行方向命令 */
typedef enum
{
	PTI_PITCH_FRONT,								// 俯仰方向前进
	PTI_PITCH_BACK,									// 俯仰方向后退
	PTI_PITCH_CENTER,								// 俯仰方向悬停
	
	PTI_ROLL_FRONT,									// 横滚方向前进
	PTI_ROLL_BACK,									// 横滚方向后退
	PTI_ROLL_CENTER,								// 横滚方向悬停
	
	PTI_YAW_FRONT,									// 偏航方向前进
	PTI_YAW_BACK,									// 偏航方向后退
	PTI_YAW_CENTER,									// 偏航方向悬停
	
	PTI_ALL_CENTER,									// 整机悬停
	PTI_PFRONT_RFRONT,								// 俯仰方向前进、横滚方向前进
	PTI_PFRONT_RBACK,								// 俯仰方向前进、横滚方向后退
	PTI_PBACK_RFRONG,								// 俯仰方向后退、横滚方向前进
	PTI_PBACK_RBACK									// 俯仰方向后退、横滚方向后退
} Direction;

/* 舵机操作命令 */
typedef enum
{
	SERVO_OPEN,										// 舵机开
	SERVO_CLOSE										// 舵机关
} ServoChange;

void ptiInit();										// PTI 初始化
void unlock();										// 电调解锁
void lock();										// 电调上锁
void controlFly(Direction direction);				// 控制飞行方向
void attitudeChange(int value);						// 改变油门值
void centerFly();									// 悬停
void stop();										// 紧急停机
void servoChange(ServoChange change);				// 改变舵机状态

#endif