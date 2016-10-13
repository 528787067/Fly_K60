#include "pti.h"

/* PTI 初始化 */
void ptiInit()
{
	ftm_pwm_init(FTM_FLY, CH_ATTITUDE, FLY_FREQUENCY, PTI_MIN);
	ftm_pwm_init(FTM_FLY, CH_PITCH, FLY_FREQUENCY, PTI_CENTER);
	ftm_pwm_init(FTM_FLY, CH_ROLL, FLY_FREQUENCY, PTI_CENTER);
	ftm_pwm_init(FTM_FLY, CH_YAW, FLY_FREQUENCY, PTI_CENTER);
	ftm_pwm_init(FTM_FLY, CH_SURE_ATTITUDE, FLY_FREQUENCY, PTI_MIN);
	ftm_pwm_init(FTM_FLY, CH_SERVO, FLY_FREQUENCY, PTI_MAX);
}

/* 电调解锁 */
void unlock()
{
	ftm_pwm_duty(FTM_FLY, CH_ATTITUDE, PTI_MIN);
	ftm_pwm_duty(FTM_FLY, CH_YAW, PTI_MAX);
	DELAY_MS(UNLOCK_DELAY);
	ftm_pwm_duty(FTM_FLY, CH_ATTITUDE, PTI_MIN);
	ftm_pwm_duty(FTM_FLY, CH_YAW, PTI_CENTER);
}

/* 电调上锁 */
void lock()
{
	ftm_pwm_duty(FTM_FLY, CH_ATTITUDE, PTI_MIN);
	ftm_pwm_duty(FTM_FLY, CH_YAW, PTI_MIN);
	DELAY_MS(UNLOCK_DELAY);
	ftm_pwm_duty(FTM_FLY, CH_ATTITUDE, PTI_MIN);
	ftm_pwm_duty(FTM_FLY, CH_YAW, PTI_CENTER);
}

/* 控制飞行方向 */
void controlFly(Direction direction)
{
	switch(direction)
	{
	/* 俯仰方向前进 */
	case PTI_PITCH_FRONT:
		ftm_pwm_duty(FTM_FLY, CH_PITCH, PTI_CENTER + COM_PTI_CHANGE);
		DELAY_MS(COM_DIRECTION_TIME);
		ftm_pwm_duty(FTM_FLY, CH_PITCH, PTI_CENTER - DIS_PTI_CHANGE);
		DELAY_MS(DIS_DIRECTION_TIME);
		ftm_pwm_duty(FTM_FLY, CH_PITCH, PTI_CENTER);
		break;
		
	/* 俯仰方向后退 */
	case PTI_PITCH_BACK:
		ftm_pwm_duty(FTM_FLY, CH_PITCH, PTI_CENTER - COM_PTI_CHANGE);
		DELAY_MS(COM_DIRECTION_TIME);
		ftm_pwm_duty(FTM_FLY, CH_PITCH, PTI_CENTER + DIS_PTI_CHANGE);
		DELAY_MS(DIS_DIRECTION_TIME);
		ftm_pwm_duty(FTM_FLY, CH_PITCH, PTI_CENTER);
		break;
	
	/* 俯仰方向悬停 */	
	case PTI_PITCH_CENTER:
		ftm_pwm_duty(FTM_FLY, CH_PITCH, PTI_CENTER);
		break;
	

	/* 横滚方向前进 */	
	case PTI_ROLL_FRONT:
		ftm_pwm_duty(FTM_FLY, CH_ROLL, PTI_CENTER + COM_PTI_CHANGE);
		DELAY_MS(COM_DIRECTION_TIME);
		ftm_pwm_duty(FTM_FLY, CH_ROLL, PTI_CENTER - DIS_PTI_CHANGE);
		DELAY_MS(DIS_DIRECTION_TIME);
		ftm_pwm_duty(FTM_FLY, CH_ROLL, PTI_CENTER);
		break;
		
	/* 横滚方向后退 */
	case PTI_ROLL_BACK:
		ftm_pwm_duty(FTM_FLY, CH_ROLL, PTI_CENTER - COM_PTI_CHANGE);
		DELAY_MS(COM_DIRECTION_TIME);
		ftm_pwm_duty(FTM_FLY, CH_ROLL, PTI_CENTER + DIS_PTI_CHANGE);
		DELAY_MS(DIS_DIRECTION_TIME);
		ftm_pwm_duty(FTM_FLY, CH_ROLL, PTI_CENTER);
		break;
		
	/* 横滚方向悬停 */
	case PTI_ROLL_CENTER:
		ftm_pwm_duty(FTM_FLY, CH_ROLL, PTI_CENTER);
		break;
		
	/* 偏航方向前进 */
	case PTI_YAW_FRONT:
		ftm_pwm_duty(FTM_FLY, CH_YAW, PTI_CENTER + COM_PTI_CHANGE);
		DELAY_MS(COM_DIRECTION_TIME);
		ftm_pwm_duty(FTM_FLY, CH_YAW, PTI_CENTER - DIS_PTI_CHANGE);
		DELAY_MS(DIS_DIRECTION_TIME);
		ftm_pwm_duty(FTM_FLY, CH_YAW, PTI_CENTER);
		break;
		
	/* 偏航方向后退 */
	case PTI_YAW_BACK:
		ftm_pwm_duty(FTM_FLY, CH_YAW, PTI_CENTER - COM_PTI_CHANGE);
		DELAY_MS(COM_DIRECTION_TIME);
		ftm_pwm_duty(FTM_FLY, CH_YAW, PTI_CENTER + DIS_PTI_CHANGE);
		DELAY_MS(DIS_DIRECTION_TIME);
		ftm_pwm_duty(FTM_FLY, CH_YAW, PTI_CENTER);
		break;
		
	/* 偏航方向悬停 */
	case PTI_YAW_CENTER:
		ftm_pwm_duty(FTM_FLY, CH_YAW, PTI_CENTER);
		break;
		
	/* 整机悬停 */
	case PTI_ALL_CENTER:
		ftm_pwm_duty(FTM_FLY, CH_PITCH, PTI_CENTER);
		ftm_pwm_duty(FTM_FLY, CH_ROLL, PTI_CENTER);
		ftm_pwm_duty(FTM_FLY, CH_YAW, PTI_CENTER);
		break;
	
	/* 俯仰方向前进、横滚方向前进 */	
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
		
	/* 俯仰方向前进、横滚方向后退 */	
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
		
	/* 俯仰方向后退、横滚方向前进 */	
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
		
	/* 俯仰方向后退、横滚方向后退 */	
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

/* 改变油门值 */
void attitudeChange(int value)
{
	value = (value > POWER_MAX) ? POWER_MAX : (value < POWER_MIN) ? POWER_MIN : value;
	ftm_pwm_duty(FTM_FLY, CH_ATTITUDE, value);
}


/* 悬停 */
void centerFly()
{	
	ftm_pwm_duty(FTM_FLY, CH_PITCH, PTI_CENTER);
	ftm_pwm_duty(FTM_FLY, CH_ROLL, PTI_CENTER);
	ftm_pwm_duty(FTM_FLY, CH_YAW, PTI_CENTER);
}

/* 紧急停机 */
void stop()
{
	ftm_pwm_duty(FTM_FLY, CH_ATTITUDE, POWER_MIN);
	ftm_pwm_duty(FTM_FLY, CH_PITCH, PTI_CENTER);
	ftm_pwm_duty(FTM_FLY, CH_ROLL, PTI_CENTER);
	ftm_pwm_duty(FTM_FLY, CH_YAW, PTI_CENTER);
}

/* 改变舵机状态 */
void servoChange(ServoChange change)
{
	if(change == SERVO_OPEN)
		ftm_pwm_init(FTM_FLY, CH_SERVO, FLY_FREQUENCY, PTI_MIN);
	else
		ftm_pwm_init(FTM_FLY, CH_SERVO, FLY_FREQUENCY, PTI_MAX);
}