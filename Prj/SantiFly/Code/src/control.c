#include "control.h"

#define ATTITUDE_CHANGE		2							// ���ŵļӼ��仯ֵ
#define POWER_DELAY			200							// �ı�����ʱ�ļ��
#define SWAVE_ATT_TIMES		1000						// �������������Ÿı�����

static long timerFlag;
static FlyState flyState;
static char blueToothFlag;
static long attitude;
static ControlMode controlMode;
static int swaveTime;
static int currentAttitude;

static uint8 uartData[4];

void PIT0_IRQHandler();
void stopFly();

void controlInit()
{
	timerFlag = 0;
	swaveTime = 0;
	currentAttitude = 0;
	uartData[0] = UART_HEAD_COM;
	flyState = FLY_LOCKED;
	blueToothFlag = BT_NO_CHAR;
	attitude = PTI_MIN;
	controlMode = POWER_MODE;
	
	uart_init(UART2, 115200);							// ��ʼ������
 	pit_init_ms(PIT0, TIMER_DELAY);                    	// ��ʼ��PIT0����ʱʱ��Ϊ�� TIME
    set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);  // ����PIT0���жϷ�����Ϊ PIT0_IRQHandler
    enable_irq(PIT0_IRQn);                              // ʹ��PIT0�ж�
}

void PIT0_IRQHandler()
{
	PIT_Flag_Clear(PIT0);								// ��ն�ʱ���жϱ�־	
	
	if(controlMode == SWAVE_MODE)
		currentAttitude = getSwaveDistance();			// ��ȡ��ǰ�������߶�
	else
		currentAttitude = SWAVE_ATTITUDE;
	
	if(attitude == POWER_MIN)
	{
		uartData[1] = FLY_STOP/100;
		uartData[2] = FLY_STOP%100;
		uartData[3] = UART_FLY_COM;
	}
	else if(flyState == FLY_CENTER && controlMode == POWER_MODE)
	{
		uartData[1] = attitude/100;
		uartData[2] = attitude%100;
		uartData[3] = UART_POWER_COM;
	}
	else if(flyState == FLY_CENTER && controlMode == SWAVE_MODE)
	{
		uartData[1] = currentAttitude/100;
		uartData[2] = currentAttitude%100;
		uartData[3] = UART_ATTITUDE_COM;
	}
	else //if(controlMode == POWER_MODE)
	{
		uartData[1] = flyState/100;
		uartData[2] = flyState%100;
		uartData[3] = UART_FLY_COM;
	}
	uart_putbuff(UART2, uartData, 4);					// ����λ������ָ��	
	
	/* ����������ģʽ */
	if(++swaveTime > SWAVE_ATT_TIMES/TIMER_DELAY && controlMode == SWAVE_MODE && flyState != FLY_LOCKED && flyState != FLY_STOP)
	{
		swaveTime = 0;
		/* �߶ȳ��� 5m ��ȫͣ�������� */
		if(currentAttitude > 500)
		{
			stopFly();
			lock();
			flyState = FLY_LOCKED;
			return;
		}
		/* ������߶�������� */
		if(currentAttitude > SWAVE_ATTITUDE+SWAVE_ERROR)
		{
			if(--attitude < POWER_MIN)
				attitude = POWER_MIN;
			attitudeChange(attitude);
		}
		/* û�дﵽ��߶�������� */
		else if(currentAttitude < SWAVE_ATTITUDE-SWAVE_ERROR)
		{
			if(++attitude > POWER_MAX)
				attitude = POWER_MAX;	
			attitudeChange(attitude);
		}
		/* �߶��ڷ��Ϸ�Χ�� */
		else;
	}
	
	timerFlag++;
	blueToothFlag = BT_NO_CHAR;
	/* �������յ����� */
	if(uart_querychar(UART2, &blueToothFlag) != BT_NO_CHAR)
	{
		timerFlag = 0;
		// û�������ҽ��յ��Ĳ��ǽ���������ֱ�ӷ���
		if(flyState == FLY_LOCKED && blueToothFlag != BT_FLY_UNLOCK)
			return;
		
		switch(blueToothFlag)
		{
		case BT_FLY_UNLOCK:								// ����
			if(flyState != FLY_LOCKED)
				break;
			disable_irq(PIT0_IRQn);
			unlock();
			flyState = FLY_STOP;
			attitude = POWER_MIN;
			servoChange(SERVO_CLOSE);
			enable_irq(PIT0_IRQn);
			break;
		case BT_FLY_LOCK:								// ����
			if(flyState == FLY_LOCKED)
				break;
			disable_irq(PIT0_IRQn);
			stopFly();
			lock();
			flyState = FLY_LOCKED;
			attitude = PTI_MIN;
			servoChange(SERVO_CLOSE);
			enable_irq(PIT0_IRQn);
			break;
		case BT_FLY_STOP:								// ͣ��
			if(attitude <= POWER_MIN)
				break;
			disable_irq(PIT0_IRQn);
			stopFly();
			servoChange(SERVO_CLOSE);
			enable_irq(PIT0_IRQn);
			break;
		case BT_ATTITUDE_ADD:							// �߶ȼ�
			if(controlMode == SWAVE_MODE || attitude >= POWER_MAX)
				break;
			attitude += ATTITUDE_CHANGE;
			if(attitude > POWER_MAX)
				attitude = POWER_MAX;
			attitudeChange(attitude);
			if(attitude > POWER_MIN && flyState == FLY_STOP)
				flyState = FLY_CENTER;
			break;
		case BT_ATTITUDE_DOWN:							// �߶ȼ�
			if(controlMode == SWAVE_MODE || attitude <= POWER_MIN)
				break;
			attitude -= ATTITUDE_CHANGE;
			if(attitude < POWER_MIN)
				attitude = POWER_MIN;
			attitudeChange(attitude);
			if(attitude == POWER_MIN)
				flyState = FLY_STOP;
			break;
		case BT_FLY_CENTER:								// ��ͣ
			if(attitude <= POWER_MIN)
				break;
			centerFly();
			flyState = FLY_CENTER;
			break;
		case BT_FLY_FRONT:								// ��ǰ��
			if(attitude <= POWER_MIN)
				break;
			controlFly(PTI_PITCH_FRONT);
			flyState = FLY_FRONT;
			break;
		case BT_FLY_BACK:								// ����
			if(attitude <= POWER_MIN)
				break;
			controlFly(PTI_PITCH_BACK);
			flyState = FLY_BACK;			
			break;
		case BT_FLY_LEFT:								// �����
			if(attitude <= POWER_MIN)
				break;
			controlFly(PTI_ROLL_BACK);
			flyState = FLY_LEFT;
			break;
		case BT_FLY_RIGHT:								// ���ҷ�
			if(attitude <= POWER_MIN)
				break;
			controlFly(PTI_ROLL_FRONT);
			flyState = FLY_RIGHT;
			break;
		case BT_THROW_BOLL:								// Ͷ��
			servoChange(SERVO_OPEN);
			break;
		case BT_FLY_ATTITUDE:							// һ�����
			disable_irq(PIT0_IRQn);
			if(flyState != FLY_CENTER && attitude > POWER_MIN)
				centerFly();
			while(attitude < POWER_ATTITUDE)
			{
				if(attitude > 695)
				{
					attitude += 2;
					if(attitude > POWER_ATTITUDE)
						attitude = POWER_ATTITUDE;
					attitudeChange(attitude);
					DELAY_MS(POWER_DELAY+100);
				}
				else
				{
					attitude += 3;
					if(attitude > POWER_ATTITUDE)
						attitude = POWER_ATTITUDE;
					attitudeChange(attitude);
					DELAY_MS(POWER_DELAY);
				}
			}
			while(attitude > POWER_ATTITUDE)
			{
				attitude -= 5;
				if(attitude < POWER_ATTITUDE)
					attitude = POWER_ATTITUDE;
				attitudeChange(attitude);
				DELAY_MS(POWER_DELAY);
			}
			flyState = FLY_CENTER;	
			enable_irq(PIT0_IRQn);
			break;
		case BT_FLY_POWER_MODE:							// ���ſ���ģʽ
			controlMode = POWER_MODE;
			break;
		case BT_FLY_SWAVE_MODE:							// ����������ģʽ
			controlMode = SWAVE_MODE;
			break;
		case BT_FLY_STOP_QUICK:							// ����ͣ��
			stop();
			attitude = POWER_MIN;
			flyState = FLY_STOP;
			lock();
			flyState = FLY_LOCKED;
			servoChange(SERVO_CLOSE);
			break;
		
		case BT_FLY_PITCH_CENTER:						// ����������ͣ
			if(attitude <= POWER_MIN)
				break;
			controlFly(PTI_PITCH_CENTER);
			flyState = FLY_PITCH_CENTER;
			break;
		case BT_FLY_ROLL_CENTER:						// ���������ͣ
			if(attitude <= POWER_MIN)
				break;
			controlFly(PTI_ROLL_CENTER);
			flyState = FLY_ROLL_CENTER;
			break;
		case BT_FLY_LEFT_FRONT:							// ��ǰ
			if(attitude <= POWER_MIN)
				break;
			controlFly(PTI_PFRONT_RBACK);
			flyState = FLY_LEFT_FRONT;
			break;
		case BT_FLY_LEFT_BACK:							// ���
			if(attitude <= POWER_MIN)
				break;
			controlFly(PTI_PBACK_RBACK);
			flyState = FLY_LEFT_BACK;
			break;
		case BT_FLY_RIGHT_FRONT:						// ��ǰ
			if(attitude <= POWER_MIN)
				break;
			controlFly(PTI_PFRONT_RFRONT);
			flyState = FLY_RIGHT_FRONT;
			break;
		case BT_FLY_RIGHT_BACK:							// �Һ�
			if(attitude <= POWER_MIN)
				break;
			controlFly(PTI_PBACK_RFRONG);
			flyState = FLY_RIGHT_BACK;
			break;
		
		default:
			break;
		}
	}
	else
	{
		if(timerFlag > TIMER_OVER)						// TIMER_OVER ����ʱ����û���յ����ݲ��ҷɿ��Ѿ��������䲢����
		{
			timerFlag = 0;
			if(flyState != FLY_LOCKED)
			{
				disable_irq(PIT0_IRQn);
				stopFly();
				lock();
				flyState = FLY_LOCKED;
				attitude = PTI_MIN;
				enable_irq(PIT0_IRQn);
			}
		}
	}
}


/* �ɿ�ͣ�� */
void stopFly()
{
	if(flyState != FLY_CENTER && attitude > POWER_MIN)
		centerFly();
	while(attitude > 695)
	{
		attitudeChange(--attitude);
		DELAY_MS(POWER_DELAY);
	}
	while(attitude > 690)
	{
		attitude -= ATTITUDE_CHANGE;
		attitudeChange(attitude);
		DELAY_MS(POWER_DELAY);
	}
	while(attitude > POWER_MIN)
	{
		attitude -= 10;
		if(attitude < POWER_MIN)
			attitude = POWER_MIN;
		attitudeChange(attitude);
		DELAY_MS(POWER_DELAY);
	}
	flyState = FLY_STOP;
}





















