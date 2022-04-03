#ifndef MAIN_H
#define MAIN_H

#include "stm32f0xx.h"

int APB_clock = 8000000;
int AHB_clock = 8000000;



#define A 0
#define B 1
#define C 2
#define D 3
#define F 5

#define IN 0
#define OUT 1
#define ALTERNATIVE 2
#define ANALOG 3

#define AF0 0
#define AF1 1
#define AF2 2
#define AF3 3
#define AF4 4
#define AF5 5
#define AF6 6
#define AF7 7

#define NO_PUPDR 0
#define PULL_UP 1
#define PULL_DOWN 2

#define LOW_SPEED 0
#define MEDIUM_SPEED 1
#define HIGH_SPEED 3

#define FLASH_LATENCY_1WS()    { FLASH->ACR |= FLASH_ACR_LATENCY; }   //Set 1 wait state
#define FLASH_LATENCY_0WS()    { FLASH->ACR &=~FLASH_ACR_LATENCY; }   //Set 0 wait state                                                                      
#define FLASH_PREFETCH_EN()    { FLASH->ACR |= FLASH_ACR_PRFTBE;  }   //Turn On Flash prefetch(12 bytes per read)
#define FLASH_PREFETCH_OFF()   { FLASH->ACR &=~FLASH_ACR_PRFTBE;  }   //Turn Off Flash prefetch

#define HSI_START() { RCC->CR |= RCC_CR_HSION; while(!(RCC->CR & RCC_CR_HSIRDY)); }
#define HSI_STOP() { RCC->CR &= ~RCC_CR_HSION;}

#define HSE_START() { RCC->CR |= RCC_CR_HSEON; while(!(RCC->CR & RCC_CR_HSERDY));}
#define HSE_STOP() { RCC->CR &=~RCC_CR_HSEON;}

//Division SYSCLK clock for AHB
#define SET_AHB_PRESCALLER(AHB_DIV)      {MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE,   AHB_DIV); }
//Division AHB clock for APB1/APB2
#define SET_APB_PRESCALLER(APB_DIV)      {MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE,   APB_DIV); }

//Main PLL multiplication factor should be in range [2;16]
#define PLL_MUL(MUL) { MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLMUL, (((MUL-2UL)&0xF)<<RCC_CFGR_PLLMUL_Pos));}
//HSE Division factor should be in range [1;16]
#define HSE_PREDIV(DIV) { MODIFY_REG(RCC->CFGR2, RCC_CFGR2_PREDIV, ((DIV-1UL)&0xF) );}

#define PLLSourseHSI_DIV2() {RCC->CFGR &=~RCC_CFGR_PLLSRC; }
#define PLLSourseHSE_PREDIV() {RCC->CFGR |= RCC_CFGR_PLLSRC_HSE_PREDIV; }
#define PLL_START() {RCC->CR |= RCC_CR_PLLON; while(!(RCC->CR & RCC_CR_PLLRDY)); }

#define APB_DIV1 RCC_CFGR_PPRE_DIV1
#define APB_DIV2 RCC_CFGR_PPRE_DIV2
#define APB_DIV4 RCC_CFGR_PPRE_DIV4
#define APB_DIV8 RCC_CFGR_PPRE_DIV8
#define APB_DIV16 RCC_CFGR_PPRE_DIV16

#define AHB_DIV1 RCC_CFGR_HPRE_DIV1
#define AHB_DIV2 RCC_CFGR_HPRE_DIV2
#define AHB_DIV4 RCC_CFGR_HPRE_DIV4
#define AHB_DIV8 RCC_CFGR_HPRE_DIV8
#define AHB_DIV16 RCC_CFGR_HPRE_DIV16
#define AHB_DIV64 RCC_CFGR_HPRE_DIV64
#define AHB_DIV128 RCC_CFGR_HPRE_DIV128
#define AHB_DIV256 RCC_CFGR_HPRE_DIV256
#define AHB_DIV512 RCC_CFGR_HPRE_DIV512

#define RCC_GPIOA() {RCC->AHBENR |= RCC_AHBENR_GPIOAEN;}
#define RCC_GPIOB() {RCC->AHBENR |= RCC_AHBENR_GPIOBEN;}
#define RCC_GPIOC() {RCC->AHBENR |= RCC_AHBENR_GPIOCEN;}
#define RCC_GPIOD() {RCC->AHBENR |= RCC_AHBENR_GPIODEN;}
#define RCC_GPIOF() {RCC->AHBENR |= RCC_AHBENR_GPIOFEN;}


#define PLL_MUL_TACT(MULTIPLIC, AHB_DIV, APB_DIV) ({FLASH_LATENCY_1WS();\
  FLASH_PREFETCH_EN();\
  HSE_START();\
  SET_AHB_PRESCALLER(AHB_DIV);\
  SET_APB_PRESCALLER(APB_DIV);\
  PLL_MUL(MULTIPLIC);\
  PLLSourseHSI_DIV2();\
  PLL_START();\
  RCC->CFGR |= RCC_CFGR_SW_PLL;\
  SystemCoreClockUpdate();\
  AHB_clock = 8000000 / 2 * MULTIPLIC;\
  APB_clock = 8000000 / 2 * MULTIPLIC;\
})

  
#define GPIOx ((GPIO_TypeDef *)(AHB2PERIPH_BASE + 0x00000400 * PORT))

#define GPIO_MODER_IN(PORT, PIN) {GPIOx->MODER &= ~(3 << (PIN * 2));}
#define GPIO_MODER_OUT(PORT, PIN) {GPIOx->MODER &= (3 << (PIN * 2)); GPIOx->MODER |= (1 << (PIN * 2));}
#define GPIO_MODER_ALTERNATIVE(PORT, PIN) {GPIOx->MODER &= (3 << (PIN * 2)); GPIOx->MODER |= (2 << (PIN * 2));}
#define GPIO_MODER_ANALOG(PORT, PIN) {GPIOx->MODER &= (3 << (PIN * 2)); GPIOx->MODER |= (3 << (PIN * 2));}

#define GPIO_BSRR_UP(PORT, PIN) {GPIOx->BSRR |= (1 << PIN);}
#define GPIO_BSRR_DOWN(PORT, PIN) {GPIOx->BSRR |= (1 << (16 + PIN));}

void SET_ALTERNATIVE_FUNC(int PORT, int PIN, int FUNC)
{
  if(PIN >= 8)
  {
    GPIOx->AFR[1] &= ~(15 << ((PORT - 8) * 4));
    GPIOx->AFR[1] |= (FUNC << ((PORT - 8) * 4));
  }
  else
  {
    GPIOx->AFR[0] &= ~(15 << ((PORT) * 4));
    GPIOx->AFR[0] |= (FUNC << ((PORT) * 4));
  }
}

int DIGITAL_READ(int PORT, int PIN) //IDR
{
  return (GPIOx->IDR & (1 << PIN));
}

void GPIO_MODER_PIN(int PORT, int PIN, int FUNC_OF_PIN)
{
  switch(FUNC_OF_PIN)
  {
  case 0:
    GPIO_MODER_IN(PORT, PIN);
    break;
  case 1:
    GPIO_MODER_OUT(PORT, PIN);
    break;
  case 2:
    GPIO_MODER_ALTERNATIVE(PORT, PIN);
    break;
  case 3:
    GPIO_MODER_ANALOG(PORT, PIN);
    break;
  default:
    break;
  }
}

void GPIO_BSRR_PIN(int PORT, int PIN, int VAL)
{
  if(VAL)
  {
    GPIO_BSRR_UP(PORT, PIN);
  }
  else
  {
    GPIO_BSRR_DOWN(PORT, PIN);
  }
}

void PUPDR_PIN(int PORT, int PIN, int VAL)
{
  if(!VAL)
  {
    GPIOx->PUPDR &= ~(3 << (PORT * 2));
  }
  else
  {
    GPIOx->PUPDR &= ~(3 << (PORT * 2));
    GPIOx->PUPDR |= (VAL << (PORT * 2));
  }
}

void SET_PIN_SPEED(int PORT, int PIN, int SPEED)
{
  if(!SPEED)
  {
    GPIOx->OSPEEDR &= ~(3 << (PORT * 2));
  }
  else
  {
    GPIOx->OSPEEDR &= ~(3 << (PORT * 2));
    GPIOx->OSPEEDR |= (SPEED << (PORT * 2));
  }
}

#define RCC_SYSTICK() { ;}

void delay_msk(int msk) 
{
  SysTick->LOAD = SystemCoreClock/8000000*msk;
  SysTick->VAL = 0;
  SysTick->CTRL = 1;
  while(!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)){}
  SysTick->CTRL = 0;
}










#define RCC_ADC {RCC->APB2ENR |= RCC_APB2ENR_ADCEN;}


#endif