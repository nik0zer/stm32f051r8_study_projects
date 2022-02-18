 #include "stm32f0xx.h"

int main(void){
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
  RCC->CR |= RCC_CR_HSION;
  GPIOA->MODER |= (GPIO_MODER_MODER0_0) ;
  GPIOA->ODR |= GPIO_ODR_0;
  
  GPIOA->PUPDR |= GPIO_PUPDR_PUPDR1_1;
  
  while(1)
  {
    if(GPIOA->IDR&GPIO_IDR_1)
      GPIOA->ODR |= GPIO_ODR_0;
    else
      GPIOA->ODR &= ~GPIO_ODR_0;
  }
  
  
  return 0;
}
