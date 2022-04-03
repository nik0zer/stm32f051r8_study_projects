#ifndef ONEWIREUSART2_H
#define ONEWIREUSART2_H

#include "Main.h"
#include <stdlib.h>
#include "USART1.h"

enum oneWire_returns
{
  NO_DEVISES = 0,
  DEVISES_FOUND = 1
};

enum oneWire_read_bit
{
  BIT_0 = 0,
  BIT_1 = 1
};

#define oneWire_RESET_speed 9600
#define oneWire_SEND_speed 115200

#define oneWire_RESET 0xF0
#define oneWire_BIT_1 0xFF
#define oneWire_BIT_0 0x00

int oneWire_USART1_init(int PORT, int pin)
{
  PUPDR_PIN(PORT, pin, PULL_UP);
  USART1_halfduplex_init(9600);
  pin_set_USART1_halfduplex_mode(PORT, pin);
  return OK;
}

int reset_oneWire_USART1()
{
  USART1_change_speed(oneWire_RESET_speed);
  USART1_transfer_byte(oneWire_RESET);
  while(USART1_data == NULL){delay_msk(50);}
  int return_int = 0;
  if(USART1_data[size_of_unread_USART1_data - 1] != oneWire_RESET)
  {
    return_int = DEVISES_FOUND;
  }
  else
  {
    return_int = NO_DEVISES;
  }
  free(USART1_data);
  size_of_unread_USART1_data = 0;
  USART1_data = NULL;
  return return_int;
}

int send_bit(int bit)
{
  USART1_change_speed(oneWire_SEND_speed);
  if(bit == 1)
  {
    USART1_transfer_byte(oneWire_BIT_1);
  }
  else
  {
    if(bit == 0)
    {
      USART1_transfer_byte(oneWire_BIT_0);
    }
    else
    {
      return VALUE_OUT_OF_RANGE;
    }
  }
  return OK;
}

int read_bit()
{
  USART1_change_speed(oneWire_SEND_speed);
  USART1_transfer_byte(oneWire_BIT_1);
  int return_int = 0;
  if(USART1_data[size_of_unread_USART1_data - 1] == oneWire_BIT_1)
  {
    return_int = BIT_1;
  }
  else
  {
    return_int = BIT_0;
  }
  free(USART1_data);
  size_of_unread_USART1_data = 0;
  USART1_data = NULL;
  return return_int;
}



#endif