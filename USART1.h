////////////////////////////////////////////
#ifndef USART1_H
#define USART1_H
#include <stdlib.h>
#include "System.h"

#define ALTERNATIVE_USART 1

int size_of_unread_USART1_data = 0;
char* USART1_data = NULL;

enum ERRORS
{
  OK = 0,
  VALUE_OUT_OF_RANGE = 1
};

int USART1_speed = 9600;

int oversampling = 16;

void set_halfduplex_USART1()
{
  USART1->CR2 &= ~(USART_CR2_LINEN | USART_CR2_CLKEN);
  USART1->CR3 &= ~(USART_CR3_SCEN  | USART_CR3_IREN);
  USART1->CR3 |= USART_CR3_HDSEL; 
}

int set_oversampling_USART1(int new_oversampling)
{
  if(new_oversampling == 16 || new_oversampling == 8)
  {
    oversampling = new_oversampling;
    int bit = (oversampling == 8) ? 1 : 0;
    USART1->CR1 &= ~(1 << 15);
    USART1->CR1 |= (bit << 15);
    return OK;
  }
  else
  {
    return VALUE_OUT_OF_RANGE;
  }
}


int USART1_halfduplex_init(int speed)
{
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
  /* (1) Oversampling by 16, 9600 baud */
  /* (2) Single-wire half-duplex mode */
  /* (3) 8 data bit, 1 start bit, 1 stop bit, no parity, reception and
  transmission enabled */
  USART1_speed = speed;
  set_oversampling_USART1(oversampling);
  USART1->BRR = 16 / oversampling * APB_clock / USART1_speed; /* (1) */
  set_halfduplex_USART1();
  USART1->CR1 = USART_CR1_TE | USART_CR1_RXNEIE
  | USART_CR1_RE | USART_CR1_UE; /* (3) */
  /* Polling idle frame Transmission */
  while ((USART1->ISR & USART_ISR_TC) != USART_ISR_TC)
  {
    delay_msk(10);
  }
  USART1->ICR |= USART_ICR_TCCF; /* Clear TC flag */
  USART1->TDR = ((int)('0'));
  
  NVIC_EnableIRQ(USART1_IRQn);
  return OK;
}

int pin_set_USART1_halfduplex_mode(int PORT, int pin)
{
  GPIO_MODER_PIN(PORT, pin, ALTERNATIVE);
  SET_ALTERNATIVE_FUNC(PORT, pin, ALTERNATIVE_USART);
  return OK;
}

int USART1_transfer_byte(char byte)
{
  while((USART1->ISR & USART_ISR_TC) != USART_ISR_TC){}
  USART1->TDR = ((int)byte);
  return OK;
}

int USART1_transfer_bytes(char* bytes, int size)
{
  int send_bytes = 0;
  while(send_bytes < size)
  {
    while((USART1->ISR & USART_ISR_TC) != USART_ISR_TC){}
    USART1->TDR = ((int)((char)bytes[send_bytes]));
    send_bytes++;
  }
  return OK;
}

void USART1_IRQHandler()
{
  if((USART1->ISR & USART_ISR_RXNE) == USART_ISR_RXNE)
  {
    char data = (char)((uint8_t)(USART1->RDR));
    size_of_unread_USART1_data++;
    USART1_data = (char*)realloc(USART1_data, size_of_unread_USART1_data);
    USART1_data[size_of_unread_USART1_data - 1] = data;
  }
  else
  {
    if((USART1->ISR & USART_ISR_TC) == USART_ISR_TC)
    {
      USART1->ICR |= USART_ICR_TCCF;
    }
    else
    {
      
    }
  }
}

int USART1_change_speed(int new_speed)
{
  if(USART1_speed != new_speed)
  {
    USART1_speed = new_speed;
    NVIC_DisableIRQ(USART1_IRQn);
    USART1->CR1 &= ~USART_CR1_UE;
    USART1->BRR = 16 / oversampling * APB_clock / USART1_speed;
    USART1->CR1 |= USART_CR1_UE;
    NVIC_EnableIRQ(USART1_IRQn);
  }
  return OK;
}

int USART1_init(int speed)
{
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
  USART1_speed = speed;
  set_oversampling_USART1(oversampling);
  USART1->BRR = 16 / oversampling * APB_clock / USART1_speed;
  USART1->CR1 = USART_CR1_TE | USART_CR1_RXNEIE
  | USART_CR1_RE | USART_CR1_UE;
  USART1->ICR |= USART_ICR_TCCF; /* Clear TC flag */
  USART1->TDR = ((int)('0'));
  NVIC_EnableIRQ(USART1_IRQn);
  return OK;
}

int set_TX_RX_pin_USART1(int PORT1, int pin1, int PORT2, int pin2)
{
  GPIO_MODER_PIN(PORT1, pin1, ALTERNATIVE);
  SET_ALTERNATIVE_FUNC(PORT1, pin1, ALTERNATIVE_USART);
  GPIO_MODER_PIN(PORT2, pin2, ALTERNATIVE);
  SET_ALTERNATIVE_FUNC(PORT2, pin2, ALTERNATIVE_USART);
  return OK;
}


#endif
