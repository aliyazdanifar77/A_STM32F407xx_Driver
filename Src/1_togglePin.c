/*
 * 1_togglePin.c
 *
 *  Created on: Mar 28, 2020
 *      Author: Ali
 */
# include "A_gpio_f407_drv.h"


void delay(void)
{
for(uint32_t i=0 ; i<=100000 ; i++);
}

int main (void)
{
GPIO_HANDLER gpio_handler;

gpio_handler.pGPIOx=GPIOA;
gpio_handler.GPIO_PIN_CONF.Pin=pin_0;
gpio_handler.GPIO_PIN_CONF.Mode=GPIO_MODE_OUTPUT_PP;
gpio_handler.GPIO_PIN_CONF.Pull=GPIO_PIN_NO_PD_PU;
gpio_handler.GPIO_PIN_CONF.Speed=GPIO_SPEED_LOW;

GPIO_Clock_CONTROL(GPIOA,ON);
GPIO_Init(&gpio_handler);


while(1)
{
GPIO_Toggle_PIN(GPIOA,pin_0);
delay();
}

return 0;
}
