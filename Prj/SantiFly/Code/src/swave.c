#include "swave.h"

void swaveInit()
{
#if SWAVE_WORK
	gpio_init(TRIQ, GPO, 0);
	gpio_init(ECHO,GPI,0);
#endif
}

int getSwaveDistance()
{
#if SWAVE_WORK
	int swaveTime = 0;
	
	gpio_set(TRIQ, 1);
	DELAY_US(15);
	gpio_set(TRIQ, 0);
	
	while(gpio_get(ECHO) == 0);
	pit_time_start(PIT1);
	while(gpio_get(ECHO) == 1);
	swaveTime = pit_time_get_us(PIT1);
	
	if(swaveTime == ~0)
		swaveTime = 0;
	return (int)(0.017 * swaveTime + 0.5);
#else
	return SWAVE_ATTITUDE;
#endif
}

/*
void RASING_HANDLER();
void FALLING_HANDLER();

static uint32 waveTime;

void swaveInit()
{
	waveTime = 0;
	gpio_init(TRIQ, GPO, 1);
	DELAY_US(15);
	gpio_set(TRIQ, 0);
	
	port_init(ECHO, ALT1 | IRQ_RISING  | PULLDOWN | PF );	// 上升缘触发中断
	updateSwave();
}

void updateSwave()
{	
	gpio_set(TRIQ, 1);
	DELAY_US(15);
	gpio_set(TRIQ, 0);
	
	disable_irq(PORTB_IRQn);
	PORTB_ISFR = ~0;
	port_init_NoALT(ECHO, IRQ_RISING  | PULLDOWN | PF );	// 上升缘触发中断
	set_vector_handler(PORTB_VECTORn, RASING_HANDLER);
	enable_irq(PORTB_IRQn);
}

int getSwaveDistance()
{
	return (int)(0.017 * waveTime + 0.5);
}

void RASING_HANDLER()
{
	disable_irq(PORTB_IRQn);
	PORTB_ISFR = ~0;
	port_init_NoALT(ECHO, IRQ_FALLING | PULLUP | PF );		// 下降缘触发中断
	set_vector_handler(PORTB_VECTORn, FALLING_HANDLER);
	enable_irq(PORTB_IRQn);
	pit_time_start(PIT1);
	//lptmr_time_start_ms();
}

void FALLING_HANDLER()
{
	disable_irq(PORTB_IRQn);
	PORTB_ISFR = ~0;
	waveTime = pit_time_get_us(PIT1);
	//waveTime = lptmr_time_get_ms();
	if(waveTime == ~0)
		waveTime = 0;
	updateSwave();
}
*/