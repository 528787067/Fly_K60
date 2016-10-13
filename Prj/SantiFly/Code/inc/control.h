#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "include.h"
#include "pti.h"
#include "swave.h"

#define UART_HEAD_COM		0x08			// ����ͨ������ͷ
#define UART_FLY_COM		0x01			// ����ͨ�ŷ���״̬����β
#define UART_POWER_COM		0x02			// ����ͨ����������β
#define UART_ATTITUDE_COM	0x03			// ����ͨ�Ÿ߶�����β

#define TIMER_DELAY			TIMER_PERIOD	// ��ʱ������
#define TIMER_OVER			50				// ��ʱʱ�䣨TIMER_OVER ��ʱ�����ڣ�

#define SWAVE_ATTITUDE		100				// ���������߸߶�
#define SWAVE_ERROR			10				// �������������

#define BT_NO_CHAR			0x00			// û�н��յ��ַ�
#define BT_FLY_UNLOCK		0x01			// �ɿؽ���
#define BT_FLY_LOCK			0x02			// �ɿ�����
#define BT_FLY_STOP			0x03			// ����ͣ��
#define BT_ATTITUDE_ADD		0x04			// �߶ȼ�
#define BT_ATTITUDE_DOWN	0x05			// �߶ȼ�
#define BT_FLY_CENTER		0x06			// ��ͣ
#define BT_FLY_FRONT		0x07			// ��ǰ��
#define BT_FLY_BACK			0x08			// ����
#define BT_FLY_LEFT			0x09			// �����
#define BT_FLY_RIGHT		0x0a			// ���ҷ�
#define BT_THROW_BOLL		0x0b			// Ͷ��
#define BT_FLY_ATTITUDE		0x0c			// һ�����
#define BT_FLY_POWER_MODE	0x0d			// ���ſ���ģʽ
#define BT_FLY_SWAVE_MODE	0x0e			// ����������ģʽ
#define BT_FLY_STOP_QUICK   0x0f       	 	// ����ͣ��
#define BT_FLY_PITCH_CENTER	0x10			// ����������ͣ
#define BT_FLY_ROLL_CENTER	0x11			// ���������ͣ
#define BT_FLY_LEFT_FRONT	0x12			// ��ǰ
#define BT_FLY_LEFT_BACK	0x13			// ���
#define BT_FLY_RIGHT_FRONT	0x14			// ��ǰ
#define BT_FLY_RIGHT_BACK	0x15			// �Һ�

typedef enum
{
	FLY_LOCKED,								// �ɿ�����״̬
	FLY_STOP,								// �ɿ�ֹͣ״̬
	FLY_CENTER,								// ��ͣ
	FLY_FRONT,								// ��ǰ��
	FLY_BACK,								// ����
	FLY_LEFT,								// �����
	FLY_RIGHT,								// ���ҷ�
	FLY_LEFT_FRONT,							// ����ǰ��
	FLY_LEFT_BACK,							// ������
	FLY_RIGHT_FRONT,						// ����ǰ��
	FLY_RIGHT_BACK,							// ���Һ��
	FLY_PITCH_CENTER,						// ����������ͣ
	FLY_ROLL_CENTER							// ���������ͣ
} FlyState;									// ������״̬

typedef enum
{
	SWAVE_MODE,								// ����������ģʽ
	POWER_MODE								// �ֶ�����ģʽ
} ControlMode;	

void controlInit();

#endif