#ifndef DS18B20_H
#define DS18B20_H
#include "System.h"
#include "oneWireUSART1.h"

#define CONVERT_T 0x44
#define WRITE_IN_MEMORY 0x4E
#define READ_MEMORY 0xBE

enum DS18B20_RETURN_CODES
{
  NO_DEVISE = 1
};

int DS18B20_init(int PORT, int pin)
{
  init_oneWire_USART1(PORT, pin);
  if(reset_oneWire_USART1() == NO_DEVISES)
  {
    return NO_DEVISE;
  }
  send_byte_oneWire_USART1(SKIP_ROM);
  send_byte_oneWire_USART1(CONVERT_T);
  return OK;
}




#endif