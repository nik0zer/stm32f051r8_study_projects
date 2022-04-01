////////////////////////////////////////////
#include "Main.h"
#include "stm32f0xx.h"

#define ALTERNATIVE_USART 1

enum ERRORS
{
  OK = 0,
  VALUE_OUT_OF_RANGE = 1
};

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
  /* (1) Oversampling by 16, 9600 baud */
  /* (2) Single-wire half-duplex mode */
  /* (3) 8 data bit, 1 start bit, 1 stop bit, no parity, reception and
  transmission enabled */
  set_oversampling_USART1(oversampling);
  USART1->BRR = 16 / oversampling * APB_clock / speed; /* (1) */
  set_halfduplex_USART1();
  USART1->CR1 = USART_CR1_TE | USART_CR1_RXNEIE
  | USART_CR1_RE | USART_CR1_UE; /* (3) */
  /* Polling idle frame Transmission */
  while ((USART1->ISR & USART_ISR_TC) != USART_ISR_TC)
  {
    delay_ms(10);
  }
  USART1->ICR |= USART_ICR_TCCF; /* Clear TC flag */
  USART1->CR1 |= USART_CR1_TCIE; /* Enable TC interrupt */
  NVIC_EnableIRQ(USART1_IRQn);
  return OK;
}

int pin_set_USART1_halfduplex_mode(int PORT, int pin, int speed)
{
  USART1_halfduplex_init(speed);
  GPIO_MODER_PIN(PORT, pin, ALTERNATIVE);
  SET_ALTERNATIVE_FUNC(PORT, pin, ALTERNATIVE_USART);
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
  
}