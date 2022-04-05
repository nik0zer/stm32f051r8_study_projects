#include "Main.h"
#include "USART1.h"
#include "oneWireUSART1.h"
#include "DS18B20.h"

char* bytes = "12345";
int size_of_bytes = 5;

int main()
{
  PLL_MUL_TACT(12, AHB_DIV1, APB_DIV1);
  USART1_init(9600);
  RCC_GPIOA();
  RCC_GPIOC();
  set_TX_RX_pin_USART1(A, 9, A, 10);
  while(1)
  {
    
  }
}