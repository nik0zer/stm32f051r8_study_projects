#ifndef ONEWIRE_H
#define ONEWIRE_H
#include "Main.h"
#include "USART2.h"
#include "stm32f0xx.h"

#define OW_RESET            0xF0
#define OW_W0               0x00
#define OW_W1               0xFF
#define OW_R                0xFF

#define OW_NO_DEVICE        1
#define OW_OK               0



#define OW_RESET_SPEED      9600
#define OW_TRANSFER_SPEED   115200

int ow_reset()
{
    usart2_change_speed(OW_RESET_SPEED);
    
    usart2_send_byte(OW_RESET);
    
    char r = usart2_receive_byte();
    
    usart2_change_speed(OW_TRANSFER_SPEED);

    return (r == OW_RESET) ? OW_NO_DEVICE : OW_OK;
}

uint8_t ow_send_bit(uint8_t b)
{
    usart2_send_byte((b&1) ? OW_W1 : OW_W0);
    return (usart2_receive_byte()==OW_W1) ? 1 : 0;
}

uint8_t ow_send(uint8_t b)
{
    int r=0;
    for(int i=0; i<8; i++, b>>=1)
    {
        r>>=1;
        if(ow_send_bit(b))
            r|=0x80;
    }
    return r;
}

void ow_send(const void *buf, unsigned int size)
{
    const uint8_t *p=(const uint8_t *)buf;
    for(unsigned int i=0; i<size; i++, p++)
        ow_send(*p);
}

inline uint8_t ow_receive()
{
    return ow_send(0xFF);
}

void ow_receive(void *buf, unsigned int size)
{
    uint8_t *p=(uint8_t *)buf;
    for(unsigned int i=0; i<size; i++, p++)
        *p=ow_send(0xFF);
}


int ow_start()
{
  DELAY_MS(1000);
  usart2_init(OW_TRANSFER_SPEED);
  if(ow_reset()!=OW_OK)
    return OW_NO_DEVICE;
  return OW_OK;
}








#endif