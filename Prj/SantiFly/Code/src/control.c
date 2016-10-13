#include "control.h"

#define ATTITUDE_CHANGE		2							// 油门的加减变化值
#define POWER_DELAY			200							// 改变油门时的间隔
#define SWAVE_ATT_TIMES		1000						// 超声波定高油门改变周期

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
	
	uart_init(UART2, 115200);							// 初始化串口
 	pit_init_ms(PIT0, TIMER_DELAY);                    	// 初始化PIT0，定时时间为： TIME
    set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);  // 设置PIT0的中断服务函数为 PIT0_IRQHandler
    enable_irq(PIT0_IRQn);                              // 使能PIT0中断
}

void PIT0_IRQHandler()
{
	PIT_Flag_Clear(PIT0);								// 清空定时器中断标志	
	
	if(controlMode == SWAVE_MODE)
		currentAttitude = getSwaveDistance();			// 获取当前超声波高度
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
	uart_putbuff(UART2, uartData, 4);					// 向上位机发送指令	
	
	/* 超声波定高模式 */
	if(++swaveTime > SWAVE_ATT_TIMES/TIMER_DELAY && controlMode == SWAVE_MODE && flyState != FLY_LOCKED && flyState != FLY_STOP)
	{
		swaveTime = 0;
		/* 高度超过 5m 安全停机并上锁 */
		if(currentAttitude > 500)
		{
			stopFly();
			lock();
			flyState = FLY_LOCKED;
			return;
		}
		/* 超过额定高度则减油门 */
		if(currentAttitude > SWAVE_ATTITUDE+SWAVE_ERROR)
		{
			if(--attitude < POWER_MIN)
				attitude = POWER_MIN;
			attitudeChange(attitude);
		}
		/* 没有达到额定高度则加油门 */
		else if(currentAttitude < SWAVE_ATTITUDE-SWAVE_ERROR)
		{
			if(++attitude > POWER_MAX)
				attitude = POWER_MAX;	
			attitudeChange(attitude);
		}
		/* 高度在符合范围内 */
		else;
	}
	
	timerFlag++;
	blueToothFlag = BT_NO_CHAR;
	/* 蓝牙接收到命令 */
	if(uart_querychar(UART2, &blueToothFlag) != BT_NO_CHAR)
	{
		timerFlag = 0;
		// 没解锁并且接收到的不是解锁命令则直接返回
		if(flyState == FLY_LOCKED && blueToothFlag != BT_FLY_UNLOCK)
			return;
		
		switch(blueToothFlag)
		{
		case BT_FLY_UNLOCK:								// 解锁
			if(flyState != FLY_LOCKED)
				break;
			disable_irq(PIT0_IRQn);
			unlock();
			flyState = FLY_STOP;
			attitude = POWER_MIN;
			servoChange(SERVO_CLOSE);
			enable_irq(PIT0_IRQn);
			break;
		case BT_FLY_LOCK:								// 上锁
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
		case BT_FLY_STOP:								// 停机
			if(attitude <= POWER_MIN)
				break;
			disable_irq(PIT0_IRQn);
			stopFly();
			servoChange(SERVO_CLOSE);
			enable_irq(PIT0_IRQn);
			break;
		case BT_ATTITUDE_ADD:							// 高度加
			if(controlMode == SWAVE_MODE || attitude >= POWER_MAX)
				break;
			attitude += ATTITUDE_CHANGE;
			if(attitude > POWER_MAX)
				attitude = POWER_MAX;
			attitudeChange(attitude);
			if(attitude > POWER_MIN && flyState == FLY_STOP)
				flyState = FLY_CENTER;
			break;
		case BT_ATTITUDE_DOWN:							// 高度减
			if(controlMode == SWAVE_MODE || attitude <= POWER_MIN)
				break;
			attitude -= ATTITUDE_CHANGE;
			if(attitude < POWER_MIN)
				attitude = POWER_MIN;
			attitudeChange(attitude);
			if(attitude == POWER_MIN)
				flyState = FLY_STOP;
			break;
		case BT_FLY_CENTER:								// 悬停
			if(attitude <= POWER_MIN)
				break;
			centerFly();
			flyState = FLY_CENTER;
			break;
		case BT_FLY_FRONT:								// 向前飞
			if(attitude <= POWER_MIN)
				break;
			controlFly(PTI_PITCH_FRONT);
			flyState = FLY_FRONT;
			break;
		case BT_FLY_BACK:								// 向后飞
			if(attitude <= POWER_MIN)
				break;
			controlFly(PTI_PITCH_BACK);
			flyState = FLY_BACK;			
			break;
		case BT_FLY_LEFT:								// 向左飞
			if(attitude <= POWER_MIN)
				break;
			controlFly(PTI_ROLL_BACK);
			flyState = FLY_LEFT;
			break;
		case BT_FLY_RIGHT:								// 向右飞
			if(attitude <= POWER_MIN)
				break;
			controlFly(PTI_ROLL_FRONT);
			flyState = FLY_RIGHT;
			break;
		case BT_THROW_BOLL:								// 投球
			servoChange(SERVO_OPEN);
			break;
		case BT_FLY_ATTITUDE:							// 一键起飞
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
		case BT_FLY_POWER_MODE:							// 油门控制模式
			controlMode = POWER_MODE;
			break;
		case BT_FLY_SWAVE_MODE:							// 超声波定高模式
			controlMode = SWAVE_MODE;
			break;
		case BT_FLY_STOP_QUICK:							// 紧急停机
			stop();
			attitude = POWER_MIN;
			flyState = FLY_STOP;
			lock();
			flyState = FLY_LOCKED;
			servoChange(SERVO_CLOSE);
			break;
		
		case BT_FLY_PITCH_CENTER:						// 俯仰方向悬停
			if(attitude <= POWER_MIN)
				break;
			controlFly(PTI_PITCH_CENTER);
			flyState = FLY_PITCH_CENTER;
			break;
		case BT_FLY_ROLL_CENTER:						// 横滚方向悬停
			if(attitude <= POWER_MIN)
				break;
			controlFly(PTI_ROLL_CENTER);
			flyState = FLY_ROLL_CENTER;
			break;
		case BT_FLY_LEFT_FRONT:							// 左前
			if(attitude <= POWER_MIN)
				break;
			controlFly(PTI_PFRONT_RBACK);
			flyState = FLY_LEFT_FRONT;
			break;
		case BT_FLY_LEFT_BACK:							// 左后
			if(attitude <= POWER_MIN)
				break;
			controlFly(PTI_PBACK_RBACK);
			flyState = FLY_LEFT_BACK;
			break;
		case BT_FLY_RIGHT_FRONT:						// 右前
			if(attitude <= POWER_MIN)
				break;
			controlFly(PTI_PFRONT_RFRONT);
			flyState = FLY_RIGHT_FRONT;
			break;
		case BT_FLY_RIGHT_BACK:							// 右后
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
		if(timerFlag > TIMER_OVER)						// TIMER_OVER 个定时周期没有收到数据并且飞控已经解锁则降落并锁定
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


/* 飞控停机 */
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





















