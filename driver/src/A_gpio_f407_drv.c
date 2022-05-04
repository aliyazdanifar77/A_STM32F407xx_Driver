/*
 * A_gpio_f407_drv.c
 *
 *  Created on: Mar 23, 2020
 *      Author: Ail_Yazdanifar
 */

#include "A_gpio_f407_drv.h"
//#include <ali_stm32f407xx.h>

//



/*******************************************************
 * @fn								- GPIO_Clock_CONTROL
 *
 * @brief							- clock enable or disable function
 *
 * @param[in]						-
 *
 *
 * @return							-None
 *
 * @Note
 */

void GPIO_Clock_CONTROL(GPIO_REG_STR *pdata , uint8_t EnDi)
{
	if(EnDi==1)
		GPIO_Clock_CONTROL_EN(pdata);
	else
		GPIO_Clock_CONTROL_DI(pdata);
}





/*******************************************************
 * @fn								- GPIO_Init
 *
 * @brief							- configure pin of micro in interrupt/event/input/output/alternate func/analog mode
 * 										this func config pin in this way
 * 										1-1: configuring mode of pin not interrupt mode
 * 										1-2: else interrupt/event mode configuration <<<<<peripheral side>>>>>
 * 										2:	configuring speed of pin
 * 										3:  configuring pull of pin
 * 										4:  configuring alternate function of pin
 * @param[in]						-
 *
 *
 * @return							-None
 *
 * @Note
 */


void GPIO_Init(GPIO_HANDLER *pdata)
{

	// configuring mode of pin not interrupt mode
	if(pdata->GPIO_PIN_CONF.Mode <= GPIO_MODE_ANALOG)
	{
		if(pdata->GPIO_PIN_CONF.Mode <= GPIO_MODE_OUTPUT_OD)
		{


			pdata->pGPIOx->MODER &= ~(0x3<<pdata->GPIO_PIN_CONF.Pin);
			pdata->pGPIOx->MODER |= (0x1<<(2 * pdata->GPIO_PIN_CONF.Pin));


			pdata->GPIO_PIN_CONF.Mode +=(uint8_t )2;
			pdata->pGPIOx->OTYPER &= ~(0x1 <<pdata->GPIO_PIN_CONF.Pin);
			pdata->pGPIOx->OTYPER |= (pdata->GPIO_PIN_CONF.Mode<<pdata->GPIO_PIN_CONF.Pin);
		}
		else
		{

			pdata->pGPIOx->MODER &= ~(0x3<<pdata->GPIO_PIN_CONF.Pin);
			pdata->pGPIOx->MODER |= (pdata->GPIO_PIN_CONF.Mode<<(2 * pdata->GPIO_PIN_CONF.Pin));
		}

	}
	// else interrupt/event mode configuration <<<<<peripheral side>>>>>
	else
	{
		if(pdata->GPIO_PIN_CONF.Mode >= GPIO_MODE_EVT_FT)
		{

			// 1. configuration of kind of trigger
			if(pdata->GPIO_PIN_CONF.Mode == GPIO_MODE_EVT_FT)
			{
				EXTI->FTSR |= (1 << pdata->GPIO_PIN_CONF.Pin);
				EXTI->RTSR &= ~(1<< pdata->GPIO_PIN_CONF.Pin);
			}
			else if(pdata->GPIO_PIN_CONF.Mode == GPIO_MODE_EVT_RT)
			{
				EXTI->RTSR |= (1 << pdata->GPIO_PIN_CONF.Pin);
				EXTI->FTSR &= ~(1<< pdata->GPIO_PIN_CONF.Pin);
			}
			else
			{
				EXTI->RTSR |= (1 << pdata->GPIO_PIN_CONF.Pin);
				EXTI->FTSR |= (1<< pdata->GPIO_PIN_CONF.Pin);
			}

			// 2. Setting the SYSCFG registers for choosing pin
			uint8_t portcode = Gpio_Port_Code(pdata->pGPIOx);
			SYSCFG_CLK_EN();
			SYSCFG->EXTICR[((pdata->GPIO_PIN_CONF.Pin)/4)] &= ~(0xF << 4 * ((pdata->GPIO_PIN_CONF.Pin)% 4));
			SYSCFG->EXTICR[((pdata->GPIO_PIN_CONF.Pin)/4)] |= (portcode<<(4 * (pdata->GPIO_PIN_CONF.Pin % 4)));

			// 3. Enabling EXTI block for event mode
			EXTI->EMR |= (1<<pdata->GPIO_PIN_CONF.Pin);


		}

		else
		{

			// 1. configuration of kind of trigger
				if(pdata->GPIO_PIN_CONF.Mode == GPIO_MODE_IT_FT	)
				{
					EXTI->FTSR |= (1 << pdata->GPIO_PIN_CONF.Pin);
					EXTI->RTSR &= ~(1<< pdata->GPIO_PIN_CONF.Pin);
				}
				else if(pdata->GPIO_PIN_CONF.Mode == GPIO_MODE_IT_RT)
				{
					EXTI->RTSR |= (1 << pdata->GPIO_PIN_CONF.Pin);
					EXTI->FTSR &= ~(1<< pdata->GPIO_PIN_CONF.Pin);
				}
				else if(pdata->GPIO_PIN_CONF.Mode == GPIO_MODE_IT_FRT)
				{
					EXTI->RTSR |= (1 << pdata->GPIO_PIN_CONF.Pin);
					EXTI->FTSR |= (1<< pdata->GPIO_PIN_CONF.Pin);
				}


				// 2. Setting the SYSCFG registers for choosing pin
				uint8_t portcode = Gpio_Port_Code(pdata->pGPIOx);
				SYSCFG_CLK_EN();
				SYSCFG->EXTICR[((pdata->GPIO_PIN_CONF.Pin)/4)] &= ~(0xF << (4 * ((pdata->GPIO_PIN_CONF.Pin)% 4)));
				SYSCFG->EXTICR[((pdata->GPIO_PIN_CONF.Pin)/4)] |= (portcode <<(4 * (pdata->GPIO_PIN_CONF.Pin % 4)));

				// 3. Enabling EXTI block for interrupt mode
				EXTI->IMR |= (1<<pdata->GPIO_PIN_CONF.Pin);
		}

	}

	// configuring speed of pin

	pdata->pGPIOx->OSPEEDR &= ~(0x3<< pdata->GPIO_PIN_CONF.Pin);
	pdata->pGPIOx->OSPEEDR |= (pdata->GPIO_PIN_CONF.Speed << (2 * pdata->GPIO_PIN_CONF.Pin));


	// configuring pull of pin
	pdata->pGPIOx->PUPDR &= ~(0x3 <<pdata->GPIO_PIN_CONF.Pin);
	pdata->pGPIOx->PUPDR |= (pdata->GPIO_PIN_CONF.Pull << (2 * pdata->GPIO_PIN_CONF.Pin));


	// configuring alternate function of pin
	uint8_t temp1,temp2;
	temp1=(pdata->GPIO_PIN_CONF.Pin)/8;
	temp2=(pdata->GPIO_PIN_CONF.Pin)%8;
	pdata->pGPIOx->AFR[temp1] &= ~(0xF <<(4 * temp2));
	pdata->pGPIOx->AFR[temp1] |=  (pdata->GPIO_PIN_CONF.Alternate <<(4 * temp2));

}

// de-init function for reset the gpio to the normal state



/*******************************************************
 * @fn								- GPIO_Deinit
 *
 * @brief							-
 *
 * @param[in]						-
 *
 *
 * @return							-None
 *
 * @Note
 */

void GPIO_Deinit(GPIO_REG_STR *pdata)
{
	GPIO_REG_RESET(pdata);
}


/*******************************************************
 * @fn								- GPIO_Read
 *
 * @brief							-
 *
 * @param[in]						-
 *
 *
 * @return							-None
 *
 * @Note							if we want to return a value of specific pin we should use
 * 									(bool )((pdata->IDR >> pinnumber)&(0x00000001));
 */


uint16_t GPIO_Read(GPIO_REG_STR *pdata , uint16_t PinAdd)
{
return (uint32_t )(pdata->IDR)&(PinAdd);
}



/*******************************************************
 * @fn								- GPIO_Write_PIN
 *
 * @brief							-
 *
 * @param[in]						-
 *
 *
 * @return							-None
 *
 * @Note
 */


void GPIO_Write_PIN(GPIO_REG_STR *pdata	, uint8_t PinNum , uint8_t PinVal)
{
	pdata->ODR &= ~(1 << PinNum);
	pdata->ODR |= (PinVal << PinNum);
}



/*******************************************************
 * @fn								- GPIO_Write_PORT
 *
 * @brief							-
 *
 * @param[in]						-
 *
 *
 * @return							-None
 *
 * @Note
 */


void GPIO_Write_PORT(GPIO_REG_STR *pdata ,  uint16_t PortVal)
{
	pdata->ODR = PortVal;
	// we can use blow code for doing this opration
	///////pdata->ODR &= 0xFFFF;
	///////pdata->ODR |= PinVal;
}


/*******************************************************
 * @fn								- GPIO_Toggle_PIN
 *
 * @brief							-
 *
 * @param[in]						-
 *
 *
 * @return							-None
 *
 * @Note
 */

void GPIO_Toggle_PIN(GPIO_REG_STR *pdata , uint8_t PinNum)
{
	pdata->ODR ^=(1<<PinNum);
}


/*******************************************************
 * @fn								- GPIO_IRQConf
 *
 * @brief							-
 *
 * @param[in]						-
 *
 *
 * @return							-None
 *
 * @Note
 */


void GPIO_IRQConf(uint8_t IRQNum , uint8_t IRQPrio , uint8_t EnDi)
{
	uint8_t temp1 = IRQNum / 32;
	uint8_t temp2 = IRQNum % 32;
	uint8_t temp3 = IRQNum / 4 ;
	uint8_t temp4 = IRQNum	% 4 ;

// set periority of interrupt
*(NVIC_IPR_BASE_ADD + temp3) |= (IRQPrio << ((8 * (temp4)) + 4));

// Enabling interrupt in core side
if(EnDi==ON)
*(NVIC_ISER_BASE_ADD + temp1) |= (1<< temp2);
else
*(NVIC_ICER_BASE_ADD + temp1) |= (1<< temp2);


}


/*******************************************************
 * @fn								- GPIO_IRQHandler
 *
 * @brief							-
 *
 * @param[in]						-
 *
 *
 * @return							-None
 *
 * @Note
 */

void GPIO_IRQHandler(uint8_t PinNum)
{
	if(EXTI->PR & (1<< PinNum))
	{
		EXTI->PR |= (1<<PinNum);
	}
}


/*******************************************************
 * @fn								- SOFT_IRQ_Request
 *
 * @brief							-If interrupt are enabled on line x in the EXTI_IMR register, calling this function, thus resulting in an
									interrupt request generation.
 *
 * @param[in]						-PinNum : the number of pin // pin_0 ,pin_ 1 , .....
 *
 *
 * @return							-None
 *
 * @Note							-this generate interrupt request on which EXTI line that configures in GPIO_Init func
 */
void SOFT_IRQ_Request(uint8_t PinNum)
{
EXTI->SWIER |= (1<<PinNum);
}

/*******************************************************
 * @fn								- SOFT_IRQ_Request
 *
 * @brief							-If interrupt are enabled on line x in the EXTI_IMR register, calling this function, thus resulting in an
									interrupt request generation.
 *
 * @param[in]						-PinNum : the number of pin // pin_0 ,pin_ 1 , .....
 *
 *
 * @return							-None
 *
 * @Note							-this generate interrupt request on which EXTI line that configures in GPIO_Init func
 */


uint8_t Event_Pending(uint8_t PinNum)
{
	uint8_t Event_Pending = (EXTI->PR >> PinNum)&(0b0000000000000001);
	//GPIO_IRQHandler(PinNum);
	return Event_Pending;
}




