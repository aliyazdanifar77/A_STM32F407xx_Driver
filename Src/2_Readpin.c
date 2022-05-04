/*
 * 2_Readpin.c
 *
 *  Created on: Mar 29, 2020
 *      Author: Ali
 */


# include <A_gpio_f407_drv.h>


void delay(void)
{
for(uint32_t i=0 ; i<=100000 ; i++);
}

int main (void)
{
GPIO_HANDLER gpio_1_handler , gpio_2_handler;


//configuration of output pin A0
gpio_1_handler.pGPIOx=GPIOA;
gpio_1_handler.GPIO_PIN_CONF.Pin=pin_0;
gpio_1_handler.GPIO_PIN_CONF.Mode=GPIO_MODE_OUTPUT_PP;
gpio_1_handler.GPIO_PIN_CONF.Pull=GPIO_PIN_NO_PD_PU;
gpio_1_handler.GPIO_PIN_CONF.Speed=GPIO_SPEED_LOW;

//configuration of input pin A0
gpio_2_handler.pGPIOx=GPIOA;
gpio_2_handler.GPIO_PIN_CONF.Pin=pin_1;
gpio_2_handler.GPIO_PIN_CONF.Mode=GPIO_MODE_INPUT;
gpio_2_handler.GPIO_PIN_CONF.Pull=GPIO_PIN_PULL_DOWN;

GPIO_Clock_CONTROL(GPIOA,ON);
GPIO_Init(&gpio_1_handler);
GPIO_Init(&gpio_2_handler);


while(1)
{
	while(GPIO_Read(GPIOA,0b0000000000000001)==ON)
	{
		GPIO_Toggle_PIN(GPIOA,pin_0);
		delay();
	}

}

return 0;
}

