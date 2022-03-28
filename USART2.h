#ifndef USART2_H
#define USART2_H
#include "Main.h"

#define RCC_USART2 {RCC->APB1ENR|=RCC_APB1ENR_USART2EN;}

void usart2_change_speed(uint32_t speed)
{
  USART2->BRR=SystemCoreClock/speed;
}

void usart2_send_byte(uint8_t ch) {
  while(!(USART2->SR&USART_SR_TXE)) {}
  USART2 -> DR = ch;
}

char usart2_receive_byte()
{
  while(!(USART2->SR&USART_SR_RXNE)) {}
  return USART2->DR;
}

void usart2_init(uint32_t speed)
{
  RCC_GPIOA();
  RCC_USART2();
  uint32_t tmp=GPIOA->CRL;
  tmp &= ~0xFF00;
  tmp |= 0x4D00;
  GPIOA->CRL=tmp;

  usart2_change_speed(speed);

  USART2->CR1 |= USART_CR1_UE;
  USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;
    
  USART2->DR;
}







#endif
