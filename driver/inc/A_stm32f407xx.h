/*
 * ali_stm32f407xx.h
 *
 *  Created on: Mar 23, 2020
 *      Author: Ali Yazdanifar
// */

#ifndef INC_ALI_STM32F407XX_H_
#define INC_ALI_STM32F407XX_H_

#include <stdint.h>
#define _vo volatile

// defining arm cortex m4 register
#define NVIC_ISER_BASE_ADD			((_vo uint32_t *)0xE000E100)
#define NVIC_ICER_BASE_ADD			((_vo uint32_t *)0xE000E180)
#define NVIC_IPR_BASE_ADD			((_vo uint32_t *)0xE000E400)

// defining base add of memory
#define FALSH_BASE_ADD			0x08000000U
#define SRAM1_BASE_ADD			0x20000000U  //112kb
#define SRAM2_BASE_ADD			0x2001C000U
#define SYS_MEM_BASE_ADD		0x1FFF0000U
#define OTP_BASE_ADD			0x1FFF7800U
#define SRAM					SRAM1_BASE_ADD

// defining base add of bus peripheral
#define APB1_BASE_ADD			0x40000000U
#define APB2_BASE_ADD			0x40010000U
#define AHB1_BASE_ADD			0x40020000U
#define AHB2_BASE_ADD			0x50000000U
#define AHB3_BASE_ADD			0xA0000000U


// defining base add of gpio peripheral

#define GPIOA_BASE_ADD			(AHB1_BASE_ADD+0x0000)
#define GPIOB_BASE_ADD			(AHB1_BASE_ADD+0x0400)
#define GPIOC_BASE_ADD			(AHB1_BASE_ADD+0x0800)
#define GPIOD_BASE_ADD			(AHB1_BASE_ADD+0x0C00)
#define GPIOE_BASE_ADD			(AHB1_BASE_ADD+0x1000)
#define GPIOF_BASE_ADD			(AHB1_BASE_ADD+0x1400)
#define GPIOG_BASE_ADD			(AHB1_BASE_ADD+0x1800)
#define GPIOH_BASE_ADD			(AHB1_BASE_ADD+0x1C00)
#define GPIOI_BASE_ADD			(AHB1_BASE_ADD+0x2000)

// defining base add of USART peripheral
//				on APB1 BUS
#define USART2_BASE_ADD				(APB1_BASE_ADD+0x4400)
#define USART3_BASE_ADD				(APB1_BASE_ADD+0x4800)
#define UART4_BASE_ADD				(APB1_BASE_ADD+0x4C00)
#define UART5_BASE_ADD				(APB1_BASE_ADD+0x5000)
#define UART7_BASE_ADD				(APB1_BASE_ADD+0x7800)
#define UART8_BASE_ADD				(APB1_BASE_ADD+0x7C00)
// 				on APB2 BUS
#define USART1_BASE_ADD				(APB2_BASE_ADD+0x1000)
#define USART6_BASE_ADD				(APB2_BASE_ADD+0x1400)



// defining base add of EXTI peripheral
#define EXTI_BASE_ADD             	(APB2_BASE_ADD + 0x3C00)



// defining base add of SYSCFG peripheral
#define SYSCFG_BASE_ADD          	(APB2_BASE_ADD + 0x3800)



// defining base add of I2C peripheral
#define I2C1_BASE_ADD             	(APB1_BASE_ADD + 0x5400)
#define I2C2_BASE_ADD           	(APB1_BASE_ADD + 0x5800)
#define I2C3_BASE_ADD           	(APB1_BASE_ADD + 0x5C00)



// defining base add of SPI peripheral
#define SPI1_BASE_ADD            	(APB2_BASE_ADD + 0x3000)
#define SPI2_BASE_ADD           	(APB1_BASE_ADD + 0x3800)
#define SPI3_BASE_ADD           	(APB1_BASE_ADD + 0x3C00)



// defining base add of RCC peripheral
#define RCC_BASE_ADD            	(AHB1_BASE_ADD + 0x3800)




typedef struct
{
  _vo uint32_t CR;            /*!< RCC clock control register,                                  Address offset: 0x00 */
  _vo uint32_t PLLCFGR;       /*!< RCC PLL configuration register,                              Address offset: 0x04 */
  _vo uint32_t CFGR;          /*!< RCC clock configuration register,                            Address offset: 0x08 */
  _vo uint32_t CIR;           /*!< RCC clock interrupt register,                                Address offset: 0x0C */
  _vo uint32_t AHB1RSTR;      /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
  _vo uint32_t AHB2RSTR;      /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
  _vo uint32_t AHB3RSTR;      /*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                                    */
  _vo uint32_t APB1RSTR;      /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
  _vo uint32_t APB2RSTR;      /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                               */
  _vo uint32_t AHB1ENR;       /*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
  _vo uint32_t AHB2ENR;       /*!< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
  _vo uint32_t AHB3ENR;       /*!< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                                    */
  _vo uint32_t APB1ENR;       /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
  _vo uint32_t APB2ENR;       /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                               */
  _vo uint32_t AHB1LPENR;     /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
  _vo uint32_t AHB2LPENR;     /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
  _vo uint32_t AHB3LPENR;     /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                                    */
  _vo uint32_t APB1LPENR;     /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
  _vo uint32_t APB2LPENR;     /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                               */
  _vo uint32_t BDCR;          /*!< RCC Backup domain control register,                          Address offset: 0x70 */
  _vo uint32_t CSR;           /*!< RCC clock control & status register,                         Address offset: 0x74 */
  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                               */
  _vo uint32_t SSCGR;         /*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
  _vo uint32_t PLLI2SCFGR;    /*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */

} RCC_REG_STR;




typedef struct {

_vo uint32_t MODER;
_vo uint32_t OTYPER;
_vo uint32_t OSPEEDR;
_vo uint32_t PUPDR;
_vo uint32_t IDR;
_vo uint32_t ODR;
_vo uint32_t BSRR;
_vo uint32_t LCKR;
_vo uint32_t AFR[2];

} GPIO_REG_STR;



typedef struct
{
  _vo uint32_t IMR;    /*!< EXTI Interrupt mask register,            Address offset: 0x00 */
  _vo uint32_t EMR;    /*!< EXTI Event mask register,                Address offset: 0x04 */
  _vo uint32_t RTSR;   /*!< EXTI Rising trigger selection register,  Address offset: 0x08 */
  _vo uint32_t FTSR;   /*!< EXTI Falling trigger selection register, Address offset: 0x0C */
  _vo uint32_t SWIER;  /*!< EXTI Software interrupt event register,  Address offset: 0x10 */
  _vo uint32_t PR;     /*!< EXTI Pending register,                   Address offset: 0x14 */

} EXTI_REG_STR;


typedef struct
{
  _vo uint32_t MEMRMP;       /*!< SYSCFG memory remap register,                      Address offset: 0x00      */
  _vo uint32_t PMC;          /*!< SYSCFG peripheral mode configuration register,     Address offset: 0x04      */
  _vo uint32_t EXTICR[4];    /*!< SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 */
  uint32_t      RESERVED[2];  /*!< Reserved, 0x18-0x1C                                                          */
  _vo uint32_t CMPCR;        /*!< SYSCFG Compensation cell control register,         Address offset: 0x20      */
} SYSCFG_REG_STR;


#define EXTI0_IRQn                  6      /*!< EXTI Line0 Interrupt                                              */
#define EXTI1_IRQn                  7      /*!< EXTI Line1 Interrupt                                              */
#define EXTI2_IRQn                  8      /*!< EXTI Line2 Interrupt                                              */
#define EXTI3_IRQn                  9      /*!< EXTI Line3 Interrupt                                              */
#define EXTI4_IRQn                  10     /*!< EXTI Line4 Interrupt 												*/
#define EXTI9_5_IRQn                23     /*!< External Line[9:5] Interrupts                                     */
#define EXTI15_10_IRQn              40     /*!< External Line[15:10] Interrupts                                   */


#define IRQ_Priority_0				0
#define IRQ_Priority_1				1
#define IRQ_Priority_2				2
#define IRQ_Priority_3				3
#define IRQ_Priority_4				4
#define IRQ_Priority_5				5
#define IRQ_Priority_6				6
#define IRQ_Priority_7				7
#define IRQ_Priority_8				8
#define IRQ_Priority_9				9
#define IRQ_Priority_10				10
#define IRQ_Priority_11				11
#define IRQ_Priority_12				12
#define IRQ_Priority_13				13
#define IRQ_Priority_14				14
#define IRQ_Priority_15				15


#define GPIOA						((GPIO_REG_STR *)GPIOA_BASE_ADD)
#define GPIOB						((GPIO_REG_STR *)GPIOB_BASE_ADD)
#define GPIOC						((GPIO_REG_STR *)GPIOC_BASE_ADD)
#define GPIOD						((GPIO_REG_STR *)GPIOD_BASE_ADD)
#define GPIOE						((GPIO_REG_STR *)GPIOE_BASE_ADD)
#define GPIOF						((GPIO_REG_STR *)GPIOF_BASE_ADD)
#define GPIOG						((GPIO_REG_STR *)GPIOG_BASE_ADD)
#define GPIOH						((GPIO_REG_STR *)GPIOH_BASE_ADD)
#define GPIOI						((GPIO_REG_STR *)GPIOI_BASE_ADD)
#define RCC                 		((RCC_REG_STR *)RCC_BASE_ADD)
#define EXTI						((EXTI_REG_STR *)EXTI_BASE_ADD)
#define SYSCFG						((SYSCFG_REG_STR *)SYSCFG_BASE_ADD)

// reset GPIOx register

#define GPIO_REG_RESET(pdata)		do{ (RCC->AHB1RSTR |= (1<<(((uint32_t )pdata - APB1_BASE_ADD)/0x400))); \
										(RCC->AHB1RSTR &= ~(1<<(((uint32_t )pdata - APB1_BASE_ADD)/0x400))); 	}while(0)






// disable and enable clock of peripheral

#define GPIO_Clock_CONTROL_EN(pdata)		(RCC->AHB1ENR |= (1<<(((uint32_t )pdata - AHB1_BASE_ADD)/0x400)))
#define GPIO_Clock_CONTROL_DI(pdata)		(RCC->AHB1ENR &= ~(1<<(((uint32_t )pdata - AHB1_BASE_ADD)/0x400)))

# define GPIOA_CLK_EN()				(RCC->AHB1ENR |= (1<<0))
# define GPIOB_CLK_EN()				(RCC->AHB1ENR |= (1<<1))
# define GPIOC_CLK_EN()				(RCC->AHB1ENR |= (1<<2))
# define GPIOD_CLK_EN()				(RCC->AHB1ENR |= (1<<3))
# define GPIOE_CLK_EN()				(RCC->AHB1ENR |= (1<<4))
# define GPIOF_CLK_EN()				(RCC->AHB1ENR |= (1<<5))
# define GPIOG_CLK_EN()				(RCC->AHB1ENR |= (1<<6))
# define GPIOH_CLK_EN()				(RCC->AHB1ENR |= (1<<7))
# define GPIOI_CLK_EN()				(RCC->AHB1ENR |= (1<<8))

#define SYSCFG_CLK_EN()				(RCC->APB2ENR |= (1<<14))
#define SYSCFG_CLK_DI()				(RCC->APB2ENR &= ~(1<<14))

# define GPIOA_CLK_DI()				(RCC->AHB1ENR &= ~(1<<0))
# define GPIOB_CLK_DI()				(RCC->AHB1ENR &= ~(1<<1))
# define GPIOC_CLK_DI()				(RCC->AHB1ENR &= ~(1<<2))
# define GPIOD_CLK_DI()				(RCC->AHB1ENR &= ~(1<<3))
# define GPIOE_CLK_DI()				(RCC->AHB1ENR &= ~(1<<4))
# define GPIOF_CLK_DI()				(RCC->AHB1ENR &= ~(1<<5))
# define GPIOG_CLK_DI()				(RCC->AHB1ENR &= ~(1<<6))
# define GPIOH_CLK_DI()				(RCC->AHB1ENR &= ~(1<<7))
# define GPIOI_CLK_DI()				(RCC->AHB1ENR &= ~(1<<8))

# define I2C1_CLK_EN()				(RCC->APB1ENR |= (1<<21))
# define I2C2_CLK_EN()				(RCC->APB1ENR |= (1<<22))
# define I2C3_CLK_EN()				(RCC->APB1ENR |= (1<<23))

# define I2C1_CLK_DI()				(RCC->APB1ENR &= ~(1<<21))
# define I2C2_CLK_DI()				(RCC->APB1ENR &= ~(1<<22))
# define I2C3_CLK_DI()				(RCC->APB1ENR &= ~(1<<23))

# define SPI1_CLK_EN()				(RCC->APB2ENR |= (1<<12))
# define SPI2_CLK_EN()				(RCC->APB1ENR |= (1<<14))
# define SPI3_CLK_EN()				(RCC->APB1ENR |= (1<<15))
# define SPI4_CLK_EN()				(RCC->APB2ENR |= (1<<13))

# define SPI1_CLK_DI()				(RCC->APB2ENR &= ~(1<<12))
# define SPI2_CLK_DI()				(RCC->APB1ENR &= ~(1<<14))
# define SPI3_CLK_DI()				(RCC->APB1ENR &= ~(1<<15))
# define SPI4_CLK_DI()				(RCC->APB2ENR &= ~(1<<13))

# define USART1_CLK_EN()				(RCC->APB2ENR |= (1<<4))
# define USART2_CLK_EN()				(RCC->APB1ENR |= (1<<17))
# define USART3_CLK_EN()				(RCC->APB1ENR |= (1<<18))
# define UART4_CLK_EN()				(RCC->APB1ENR |= (1<<19))
# define UART5_CLK_EN()				(RCC->APB1ENR |= (1<<20))
# define USART6_CLK_EN()				(RCC->APB1ENR |= (1<<5))
# define UART7_CLK_EN()				(RCC->APB1ENR |= (1<<30))
# define UART8_CLK_EN()				(RCC->APB1ENR |= (1<<31))

# define USART1_CLK_DI()				(RCC->APB2ENR &= ~(1<<4))
# define USART2_CLK_DI()				(RCC->APB1ENR &= ~(1<<17))
# define USART3_CLK_DI()				(RCC->APB1ENR &= ~(1<<18))
# define UART4_CLK_DI()				(RCC->APB1ENR &= ~(1<<19))
# define UART5_CLK_DI()				(RCC->APB1ENR &= ~(1<<20))
# define USART6_CLK_DI()				(RCC->APB1ENR &= ~(1<<5))
# define UART7_CLK_DI()				(RCC->APB1ENR &= ~(1<<30))
# define UART8_CLK_DI()				(RCC->APB1ENR &= ~(1<<31))


#define Gpio_Port_Code(x)       ((x==GPIOA)?0:\
								(x==GPIOB)?1:\
								(x==GPIOC)?2:\
								(x==GPIOD)?3:\
								(x==GPIOE)?4:\
								(x==GPIOF)?5:\
								(x==GPIOG)?6:\
								(x==GPIOH)?7:\
								(x==GPIOI)?8:0)

// we can use blow macro for the same operation
//#define Gpio_Port_Code(pdata)        (((uint32_t )pdata - APB1_BASE_ADD)/0x400)



















#endif /* INC_ALI_STM32F4 7XX_H_ */
