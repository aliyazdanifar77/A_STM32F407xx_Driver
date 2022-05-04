/*
 * A_gpio_f407_drv.h
 *
 *  Created on: Mar 23, 2020
 *      Author: Ali Yazdanifar
 */

#ifndef INC_A_GPIO_F407_DRV_H_
#define INC_A_GPIO_F407_DRV_H_

#include "A_stm32f407xx.h"

typedef struct
{
	  uint8_t Pin;      	/*!< Specifies the GPIO pins to be configured.*/
	  int8_t Mode;    		/*!< Specifies the operating mode for the selected pins.*/
	  uint8_t Pull;     	/*!< Specifies the Pull-up or Pull-Down activation for the selected pins.*/
	  uint8_t Speed;     	/*!< Specifies the speed for the selected pins.*/
	  uint8_t Alternate; 	/*!< Peripheral to be connected to the selected pins.*/

}PIN_CONF;


typedef struct
{
	GPIO_REG_STR *pGPIOx;
	PIN_CONF GPIO_PIN_CONF;

}GPIO_HANDLER;

//macros for pin
#define pin_0							0
#define pin_1							1
#define pin_2							2
#define pin_3							3
#define pin_4							4
#define pin_5							5
#define pin_6							6
#define pin_7							7
#define pin_8							8
#define pin_9							9
#define pin_10							10
#define pin_11							11
#define pin_12							12
#define pin_13							13
#define pin_14							14
#define pin_15							15

//macro for mode
#define GPIO_MODE_INPUT					0
#define GPIO_MODE_OUTPUT_PP				-2
#define GPIO_MODE_OUTPUT_OD				-1
#define GPIO_MODE_ALT_FUNC				2
#define GPIO_MODE_ANALOG				3

#define GPIO_MODE_IT_FT					4
#define GPIO_MODE_IT_RT					5
#define GPIO_MODE_IT_FRT				6

#define GPIO_MODE_EVT_FT				7
#define GPIO_MODE_EVT_RT				8
#define GPIO_MODE_EVT_FRT				9

// macro for speed
#define GPIO_SPEED_LOW 					0
#define GPIO_SPEED_MEDIUM 				1
#define GPIO_SPEED_FAST 				2
#define GPIO_SPEED_HIGH 				3

// macro for pulldown_pullup
#define GPIO_PIN_NO_PD_PU				0
#define GPIO_PIN_PULL_UP				1
#define GPIO_PIN_PULL_DOWN				2




#define ON 								1
#define OFF								0



// interrupt




// APIs prototype

void GPIO_Clock_CONTROL(GPIO_REG_STR *pdata , uint8_t EnDi);

void GPIO_Init(GPIO_HANDLER *pdata);

void GPIO_Deinit(GPIO_REG_STR *pdata);

void GPIO_Write_PIN(GPIO_REG_STR *pdata	 ,  uint8_t PinNum , uint8_t PinVal);

void GPIO_Write_PORT(GPIO_REG_STR *pdata ,  uint16_t PinVal);

uint16_t GPIO_Read(GPIO_REG_STR *pdata , uint16_t PinAdd);

void GPIO_Toggle_PIN(GPIO_REG_STR *pdata , uint8_t PinNum);

void GPIO_IRQConf(uint8_t IRQNum , uint8_t IRQPrio , uint8_t EnDi);

void GPIO_IRQHandler(uint8_t PinNum);

void SOFT_IRQ_Request(uint8_t PinNum);

uint8_t Event_Pending(uint8_t PinNum);





#endif /* INC_A_GPIO_F407_DRV_H_ */
