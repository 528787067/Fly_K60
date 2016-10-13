#include "pti.h"

/* PTI ��ʼ�� */
void ptiInit()
{
	ftm_pwm_init(FTM_FLY, CH_ATTITUDE, FLY_FREQUENCY, PTI_MIN);
	ftm_pwm_init(FTM_FLY, CH_PITCH, FLY_FREQUENCY, PTI_CENTER);
	ftm_pwm_init(FTM_FLY, CH_ROLL, FLY_FREQUENCY, PTI_CENTER);
	ftm_pwm_init(FTM_FLY, CH_YAW, FLY_FREQUENCY, PTI_CENTER);
	ftm_pwm_init(FTM_FLY, CH_SURE_ATTITUDE, FLY_FREQUENCY, PTI_MIN);
	ftm_pwm_init(FTM_FLY, CH_SERVO, FLY_FREQUENCY, PTI_MAX);
}

/* ������� */
void unlock()
{
	ftm_pwm_duty(FTM_FLY, CH_ATTITUDE, PTI_MIN);
	ftm_pwm_duty(FTM_FLY, CH_YAW, PTI_MAX);
	DELAY_MS(UNLOCK_DELAY);
	ftm_pwm_duty(FTM_FLY, CH_ATTITUDE, PTI_MIN);
	ftm_pwm_duty(FTM_FLY, CH_YAW, PTI_CENTER);
}

/* ������� */
void lock()
{
	ftm_pwm_duty(FTM_FLY, CH_ATTITUDE, PTI_MIN);
	ftm_pwm_duty(FTM_FLY, CH_YAW, PTI_MIN);
	DELAY_MS(UNLOCK_DELAY);
	ftm_pwm_duty(FTM_FLY, CH_ATTITUDE, PTI_MIN);
	ftm_pwm_duty(FTM_FLY, CH_YAW, PTI_CENTER);
}

/* ���Ʒ��з��� */
void controlFly(Direction direction)
{
	switch(direction)
	{
	/* ��������ǰ�� */
	case PTI_PITCH_FRONT:
		ftm_pwm_duty(FTM_FLY, CH_PITCH, PTI_CENTER + COM_PTI_CHANGE);
		DELAY_MS(COM_DIRECTION_TIME);
		ftm_pwm_duty(FTM_FLY, CH_PITCH, PTI_CENTER - DIS_PTI_CHANGE);
		DELAY_MS(DIS_DIRECTION_TIME);
		ftm_pwm_duty(FTM_FLY, CH_PITCH, PTI_CENTER);
		break;
		
	/* ����������� */
	case PTI_PITCH_BACK:
		ftm_pwm_duty(FTM_FLY, CH_PITCH, PTI_CENTER - COM_PTI_CHANGE);
		DELAY_MS(COM_DIRECTION_TIME);
		ftm_pwm_duty(FTM_FLY, CH_PITCH, PTI_CENTER + DIS_PTI_CHANGE);
		DELAY_MS(DIS_DIRECTION_TIME);
		ftm_pwm_duty(FTM_FLY, CH_PITCH, PTI_CENTER);
		break;
	
	/* ����������ͣ */	
	case PTI_PITCH_CENTER:
		ftm_pwm_duty(FTM_FLY, CH_PITCH, PTI_CENTER);
		break;
	

	/* �������ǰ�� */	
	case PTI_ROLL_FRONT:
		ftm_pwm_duty(FTM_FLY, CH_ROLL, PTI_CENTER + COM_PTI_CHANGE);
		DELAY_MS(COM_DIRECTION_TIME);
		ftm_pwm_duty(FTM_FLY, CH_ROLL, PTI_CENTER - DIS_PTI_CHANGE);
		DELAY_MS(DIS_DIRECTION_TIME);
		ftm_pwm_duty(FTM_FLY, CH_ROLL, PTI_CENTER);
		break;
		
	/* ���������� */
	case PTI_ROLL_BACK:
		ftm_pwm_duty(FTM_FLY, CH_ROLL, PTI_CENTER - COM_PTI_CHANGE);
		DELAY_MS(COM_DIRECTION_TIME);
		ftm_pwm_duty(FTM_FLY, CH_ROLL, PTI_CENTER + DIS_PTI_CHANGE);
		DELAY_MS(DIS_DIRECTION_TIME);
		ftm_pwm_duty(FTM_FLY, CH_ROLL, PTI_CENTER);
		break;
		
	/* ���������ͣ */
	case PTI_ROLL_CENTER:
		ftm_pwm_duty(FTM_FLY, CH_ROLL, PTI_CENTER);
		break;
		
	/* ƫ������ǰ�� */
	case PTI_YAW_FRONT:
		ftm_pwm_duty(FTM_FLY, CH_YAW, PTI_CENTER + COM_PTI_CHANGE);
		DELAY_MS(COM_DIRECTION_TIME);
		ftm_pwm_duty(FTM_FLY, CH_YAW, PTI_CENTER - DIS_PTI_CHANGE);
		DELAY_MS(DIS_DIRECTION_TIME);
		ftm_pwm_duty(FTM_FLY, CH_YAW, PTI_CENTER);
		break;
		
	/* ƫ��������� */
	case PTI_YAW_BACK:
		ftm_pwm_duty(FTM_FLY, CH_YAW, PTI_CENTER - COM_PTI_CHANGE);
		DELAY_MS(COM_DIRECTION_TIME);
		ftm_pwm_duty(FTM_FLY, CH_YAW, PTI_CENTER + DIS_PTI_CHANGE);
		DELAY_MS(DIS_DIRECTION_TIME);
		ftm_pwm_duty(FTM_FLY, CH_YAW, PTI_CENTER);
		break;
		
	/* ƫ��������ͣ */
	case PTI_YAW_CENTER:
		ftm_pwm_duty(FTM_FLY, CH_YAW, PTI_CENTER);
		break;
		
	/* ������ͣ */
	case PTI_ALL_CENTER:
		ftm_pwm_duty(FTM_FLY, CH_PITCH, PTI_CENTER);
		ftm_pwm_duty(FTM_FLY, CH_ROLL, PTI_CENTER);
		ftm_pwm_duty(FTM_FLY, CH_YAW, PTI_CENTER);
		break;
	
	/* ��������ǰ�����������ǰ�� */	
	case PTI_PFRONT_RFRONT:
		ftm_pwm_duty(FTM_FLY, CH_PITCH, PTI_CENTER + COM_PTI_CHANGE);
		ftm_pwm_duty(FTM_FLY, CH_ROLL, PTI_CENTER + COM_PTI_CHANGE);
		DELAY_MS(COM_DIRECTION_TIME);
		ftm_pwm_duty(FTM_FLY, CH_PITCH, PTI_CENTER - DIS_PTI_CHANGE);
		ftm_pwm_duty(FTM_FLY, CH_ROLL, PTI_CENTER - DIS_PTI_CHANGE);
		DELAY_MS(DIS_DIRECTION_TIME);
		ftm_pwm_duty(FTM_FLY, CH_PITCH, PTI_CENTER);
		ftm_pwm_duty(FTM_FLY, CH_ROLL, PTI_CENTER);
		break;
		
	/* ��������ǰ�������������� */	
	case PTI_PFRONT_RBACK:
		ftm_pwm_duty(FTM_FLY, CH_PITCH, PTI_CENTER + COM_PTI_CHANGE);
		ftm_pwm_duty(FTM_FLY, CH_ROLL, PTI_CENTER - COM_PTI_CHANGE);
		DELAY_MS(COM_DIRECTION_TIME);
		ftm_pwm_duty(FTM_FLY, CH_PITCH, PTI_CENTER - DIS_PTI_CHANGE);
		ftm_pwm_duty(FTM_FLY, CH_ROLL, PTI_CENTER + DIS_PTI_CHANGE);
		DELAY_MS(DIS_DIRECTION_TIME);
		ftm_pwm_duty(FTM_FLY, CH_PITCH, PTI_CENTER);
		ftm_pwm_duty(FTM_FLY, CH_ROLL, PTI_CENTER);
		break;
		
	/* ����������ˡ��������ǰ�� */	
	case PTI_PBACK_RFRONG:
		ftm_pwm_duty(FTM_FLY, CH_PITCH, PTI_CENTER - COM_PTI_CHANGE);
		ftm_pwm_duty(FTM_FLY, CH_ROLL, PTI_CENTER + COM_PTI_CHANGE);
		DELAY_MS(COM_DIRECTION_TIME);
		ftm_pwm_duty(FTM_FLY, CH_PITCH, PTI_CENTER + DIS_PTI_CHANGE);
		ftm_pwm_duty(FTM_FLY, CH_ROLL, PTI_CENTER - DIS_PTI_CHANGE);
		DELAY_MS(DIS_DIRECTION_TIME);
		ftm_pwm_duty(FTM_FLY, CH_PITCH, PTI_CENTER);
		ftm_pwm_duty(FTM_FLY, CH_ROLL, PTI_CENTER);
		break;
		
	/* ����������ˡ����������� */	
	case PTI_PBACK_RBACK:
		ftm_pwm_duty(FTM_FLY, CH_PITCH, PTI_CENTER - COM_PTI_CHANGE);
		ftm_pwm_duty(FTM_FLY, CH_ROLL, PTI_CENTER - COM_PTI_CHANGE);
		DELAY_MS(COM_DIRECTION_TIME);
		ftm_pwm_duty(FTM_FLY, CH_PITCH, PTI_CENTER + DIS_PTI_CHANGE);
		ftm_pwm_duty(FTM_FLY, CH_ROLL, PTI_CENTER + DIS_PTI_CHANGE);
		DELAY_MS(DIS_DIRECTION_TIME);
		ftm_pwm_duty(FTM_FLY, CH_PITCH, PTI_CENTER);
		ftm_pwm_duty(FTM_FLY, CH_ROLL, PTI_CENTER);
		break;
		
	default:
		break;
	}
}

/* �ı�����ֵ */
void attitudeChange(int value)
{
	value = (value > POWER_MAX) ? POWER_MAX : (value < POWER_MIN) ? POWER_MIN : value;
	ftm_pwm_duty(FTM_FLY, CH_ATTITUDE, value);
}


/* ��ͣ */
void centerFly()
{	
	ftm_pwm_duty(FTM_FLY, CH_PITCH, PTI_CENTER);
	ftm_pwm_duty(FTM_FLY, CH_ROLL, PTI_CENTER);
	ftm_pwm_duty(FTM_FLY, CH_YAW, PTI_CENTER);
}

/* ����ͣ�� */
void stop()
{
	ftm_pwm_duty(FTM_FLY, CH_ATTITUDE, POWER_MIN);
	ftm_pwm_duty(FTM_FLY, CH_PITCH, PTI_CENTER);
	ftm_pwm_duty(FTM_FLY, CH_ROLL, PTI_CENTER);
	ftm_pwm_duty(FTM_FLY, CH_YAW, PTI_CENTER);
}

/* �ı���״̬ */
void servoChange(ServoChange change)
{
	if(change == SERVO_OPEN)
		ftm_pwm_init(FTM_FLY, CH_SERVO, FLY_FREQUENCY, PTI_MIN);
	else
		ftm_pwm_init(FTM_FLY, CH_SERVO, FLY_FREQUENCY, PTI_MAX);
}