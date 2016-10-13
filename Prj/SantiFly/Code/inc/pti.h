#ifndef _PTI_H_
#define _PTI_H_

#include "include.h"

#define FLY_FREQUENCY		50						// ���Ƶ��
#define PTI_MIN				500						// �����Сռ�ձ�
#define PTI_MAX				1000					// ������ռ�ձ�
#define PTI_CENTER			(PTI_MIN+PTI_MAX)/2		// ����м�ռ�ձ�

#define POWER_MIN			585						// ������Сֵ
#define POWER_MAX			725						// �������ֵ
#define POWER_ATTITUDE		710						// ����һ�����ʱ�ĸ߶�

#define UNLOCK_DELAY		2000					// ������ʱ
#define TIMER_PERIOD		10						// ��ʱ������

#define COM_DIRECTION_TIME	5						// ����ı�ʱ�������ʱ�䣨����ʱ�� + ����ʱ�� < ��ʱ�����ڣ�
#define DIS_DIRECTION_TIME	2						// ����ı�ʱ�������ʱ�䣨����ʱ�� + ����ʱ�� < ��ʱ�����ڣ�
#define COM_PTI_CHANGE		35						// ����ı�ʱ�������İڶ���Χ
#define DIS_PTI_CHANGE		40						// ����ı�ʱ�������İڶ���Χ


#define FTM_FLY				FTM0
#define CH_ROLL				FTM_CH0
#define CH_PITCH			FTM_CH1
#define CH_ATTITUDE			FTM_CH2
#define CH_YAW				FTM_CH3
#define CH_SURE_ATTITUDE	FTM_CH4
#define CH_SERVO			FTM_CH5

/* ���з������� */
typedef enum
{
	PTI_PITCH_FRONT,								// ��������ǰ��
	PTI_PITCH_BACK,									// �����������
	PTI_PITCH_CENTER,								// ����������ͣ
	
	PTI_ROLL_FRONT,									// �������ǰ��
	PTI_ROLL_BACK,									// ����������
	PTI_ROLL_CENTER,								// ���������ͣ
	
	PTI_YAW_FRONT,									// ƫ������ǰ��
	PTI_YAW_BACK,									// ƫ���������
	PTI_YAW_CENTER,									// ƫ��������ͣ
	
	PTI_ALL_CENTER,									// ������ͣ
	PTI_PFRONT_RFRONT,								// ��������ǰ�����������ǰ��
	PTI_PFRONT_RBACK,								// ��������ǰ��������������
	PTI_PBACK_RFRONG,								// ����������ˡ��������ǰ��
	PTI_PBACK_RBACK									// ����������ˡ�����������
} Direction;

/* ����������� */
typedef enum
{
	SERVO_OPEN,										// �����
	SERVO_CLOSE										// �����
} ServoChange;

void ptiInit();										// PTI ��ʼ��
void unlock();										// �������
void lock();										// �������
void controlFly(Direction direction);				// ���Ʒ��з���
void attitudeChange(int value);						// �ı�����ֵ
void centerFly();									// ��ͣ
void stop();										// ����ͣ��
void servoChange(ServoChange change);				// �ı���״̬

#endif