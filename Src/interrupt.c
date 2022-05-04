/*
 * interrupt.c
 *
 *  Created on: Mar 31, 2020
 *      Author: Ali Yazdanifar
 */
# include "A_gpio_f407_drv.h"
# include <string.h>

void delay(void)
{
for(uint32_t i=0 ; i<=100000 ; i++);
}

void EXTI15_10_IRQHandler(void)
{
	uint8_t ali;
	GPIO_Toggle_PIN(GPIOA,0);
	ali=Event_Pending(pin_10);
	//GPIO_IRQHandler(pin_6);
	delay();


}

int main(void)
{
	GPIO_Clock_CONTROL(GPIOA,ON);
//	GPIO_Clock_CONTROL(GPIOG,ON);
	GPIO_Clock_CONTROL(GPIOC,ON);

	GPIO_HANDLER gpio_handler , gpio_1_handler , gpio_2_handler;

	memset(&gpio_handler,0,sizeof(gpio_handler));
	memset(&gpio_1_handler,0,sizeof(gpio_1_handler));
//	memset(&gpio_2_handler,0,sizeof(gpio_2_handler));

	gpio_handler.pGPIOx=GPIOA;
	gpio_handler.GPIO_PIN_CONF.Pin=pin_0;
	gpio_handler.GPIO_PIN_CONF.Mode=GPIO_MODE_OUTPUT_PP;
	gpio_handler.GPIO_PIN_CONF.Pull=GPIO_PIN_NO_PD_PU;
	gpio_handler.GPIO_PIN_CONF.Speed=GPIO_SPEED_LOW;


	gpio_1_handler.pGPIOx=GPIOC;
	gpio_1_handler.GPIO_PIN_CONF.Pin=pin_10;
	gpio_1_handler.GPIO_PIN_CONF.Mode=GPIO_MODE_EVT_FT;
	gpio_1_handler.GPIO_PIN_CONF.Pull=GPIO_PIN_PULL_UP;


	//gpio_2_handler.pGPIOx=GPIOC;
	//gpio_2_handler.GPIO_PIN_CONF.Pin=pin_10;
	//gpio_2_handler.GPIO_PIN_CONF.Mode=GPIO_MODE_EVT_FT;
	//gpio_2_handler.GPIO_PIN_CONF.Pull=GPIO_PIN_PULL_UP;





	GPIO_Init(&gpio_handler);
	GPIO_Init(&gpio_1_handler);
	//GPIO_Init(&gpio_2_handler);

	//GPIO_IRQConf(EXTI15_10_IRQn,IRQ_Priority_15,ON);


	//SOFT_IRQ_Request(pin_6);
	while(1)

	{
		if(Event_Pending(pin_10)==1)
			while(1);

	}

	return 0;
}


