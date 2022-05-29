/**
  ******************************************************************************
  *          GPIO.h
  *          A.Chiffa Dev.
  *          Header file for GPIO module(No Source file).
  *          For STM32F05x MCU 
  *          V1.0
  * @attention
  *          Redistributions of source code must retain the above copyright notice,
  *          this list of conditions and the following disclaimer.
  *          Commercial use is possible only by agreement with the developers
  * @Mail    ArsChiffa@gmail.com
  *
  ******************************************************************************
*/
#ifndef GPIO_H
#define GPIO_H 

#include "stm32f0xx.h"
#define POSITION_VAL(VAL)     (__CLZ(__RBIT(VAL)))  

//------------------------------------------------------------------------------
//                            GPIO PINs definitions 
//------------------------------------------------------------------------------

#define PA0     (GPIO_BSRR_BS_0  | (0x0000<<16))
#define PA1     (GPIO_BSRR_BS_1  | (0x0000<<16))
#define PA2     (GPIO_BSRR_BS_2  | (0x0000<<16))
#define PA3     (GPIO_BSRR_BS_3  | (0x0000<<16))
#define PA4     (GPIO_BSRR_BS_4  | (0x0000<<16))
#define PA5     (GPIO_BSRR_BS_5  | (0x0000<<16))
#define PA6     (GPIO_BSRR_BS_6  | (0x0000<<16))
#define PA7     (GPIO_BSRR_BS_7  | (0x0000<<16))
#define PA8     (GPIO_BSRR_BS_8  | (0x0000<<16))
#define PA9     (GPIO_BSRR_BS_9  | (0x0000<<16))
#define PA10    (GPIO_BSRR_BS_10 | (0x0000<<16))
#define PA11    (GPIO_BSRR_BS_11 | (0x0000<<16))
#define PA12    (GPIO_BSRR_BS_12 | (0x0000<<16))
#define PA13    (GPIO_BSRR_BS_13 | (0x0000<<16))
#define PA14    (GPIO_BSRR_BS_14 | (0x0000<<16))
#define PA15    (GPIO_BSRR_BS_15 | (0x0000<<16))

#define PB0     (GPIO_BSRR_BS_0  | (0x0400<<16))
#define PB1     (GPIO_BSRR_BS_1  | (0x0400<<16))
#define PB2     (GPIO_BSRR_BS_2  | (0x0400<<16))
#define PB3     (GPIO_BSRR_BS_3  | (0x0400<<16))
#define PB4     (GPIO_BSRR_BS_4  | (0x0400<<16))
#define PB5     (GPIO_BSRR_BS_5  | (0x0400<<16))
#define PB6     (GPIO_BSRR_BS_6  | (0x0400<<16))
#define PB7     (GPIO_BSRR_BS_7  | (0x0400<<16))
#define PB8     (GPIO_BSRR_BS_8  | (0x0400<<16))
#define PB9     (GPIO_BSRR_BS_9  | (0x0400<<16))
#define PB10    (GPIO_BSRR_BS_10 | (0x0400<<16))
#define PB11    (GPIO_BSRR_BS_11 | (0x0400<<16))
#define PB12    (GPIO_BSRR_BS_12 | (0x0400<<16))
#define PB13    (GPIO_BSRR_BS_13 | (0x0400<<16))
#define PB14    (GPIO_BSRR_BS_14 | (0x0400<<16))
#define PB15    (GPIO_BSRR_BS_15 | (0x0400<<16))

#define PC0     (GPIO_BSRR_BS_0  | (0x0800<<16))
#define PC1     (GPIO_BSRR_BS_1  | (0x0800<<16))
#define PC2     (GPIO_BSRR_BS_2  | (0x0800<<16))
#define PC3     (GPIO_BSRR_BS_3  | (0x0800<<16))
#define PC4     (GPIO_BSRR_BS_4  | (0x0800<<16))
#define PC5     (GPIO_BSRR_BS_5  | (0x0800<<16))
#define PC6     (GPIO_BSRR_BS_6  | (0x0800<<16))
#define PC7     (GPIO_BSRR_BS_7  | (0x0800<<16))
#define PC8     (GPIO_BSRR_BS_8  | (0x0800<<16))
#define PC9     (GPIO_BSRR_BS_9  | (0x0800<<16))
#define PC10    (GPIO_BSRR_BS_10 | (0x0800<<16))
#define PC11    (GPIO_BSRR_BS_11 | (0x0800<<16))
#define PC12    (GPIO_BSRR_BS_12 | (0x0800<<16))
#define PC13    (GPIO_BSRR_BS_13 | (0x0800<<16))
#define PC14    (GPIO_BSRR_BS_14 | (0x0800<<16))
#define PC15    (GPIO_BSRR_BS_15 | (0x0800<<16))

//------------------------------------------------------------------------------
//                            GPIO parameters definitions 
//------------------------------------------------------------------------------

#define  MODE_INPUT                (0x00000000U)               /*!< Select input mode */
#define  MODE_OUTPUT               GPIO_MODER_MODER0_0         /*!< Select output mode */
#define  MODE_ALTERNATE            GPIO_MODER_MODER0_1         /*!< Select alternate function mode */
#define  MODE_ANALOG               GPIO_MODER_MODER0           /*!< Select analog mode */

#define  PULL_NO                   (0x00000000U)               /*!< Select I/O no pull */
#define  PULL_UP                   GPIO_PUPDR_PUPDR0_0         /*!< Select I/O pull up */
#define  PULL_DOWN                 GPIO_PUPDR_PUPDR0_1         /*!< Select I/O pull down */

#define  SPEED_LOW                 (0x00000000U)               /*!< Select I/O low output speed    */
#define  SPEED_MEDIUM              GPIO_OSPEEDER_OSPEEDR0_0    /*!< Select I/O medium output speed */
#define  SPEED_HIGH                GPIO_OSPEEDER_OSPEEDR0_1    /*!< Select I/O fast output speed   */
#define  SPEED_VERY_HIGH           GPIO_OSPEEDER_OSPEEDR0      /*!< Select I/O high output speed   */

#define  TYPE_PUSHPULL             (0x00000000U)               /*!< Select push-pull as output type */
#define  TYPE_OPENDRAIN            GPIO_OTYPER_OT0            /*!< Select open-drain as output type */

#define  AF0                       (0x0000000U)     /*!< Select alternate function 0 (NO AF) */
#define  AF1                       (0x0000001U)     /*!< Select alternate function 1 */
#define  AF2                       (0x0000002U)     /*!< Select alternate function 2 */
#define  AF3                       (0x0000003U)     /*!< Select alternate function 3 */
#define  AF4                       (0x0000004U)     /*!< Select alternate function 4 */
#define  AF5                       (0x0000005U)     /*!< Select alternate function 5 */
#define  AF6                       (0x0000006U)     /*!< Select alternate function 6 */
#define  AF7                       (0x0000007U)     /*!< Select alternate function 7 */
#define  AF8                       (0x0000008U)     /*!< Select alternate function 8 */
#define  AF9                       (0x0000009U)     /*!< Select alternate function 9 */
#define  AF10                      (0x000000AU)     /*!< Select alternate function 10 */
#define  AF11                      (0x000000BU)     /*!< Select alternate function 11 */
#define  AF12                      (0x000000CU)     /*!< Select alternate function 12 */
#define  AF13                      (0x000000DU)     /*!< Select alternate function 13 */
#define  AF14                      (0x000000EU)     /*!< Select alternate function 14 */
#define  AF15                      (0x000000FU)     /*!< Select alternate function 15 */

//------------------------------------------------------------------------------
//                            GPIO MACRO definitions 
//------------------------------------------------------------------------------

#define GET_PORT(PIN)     ((GPIO_TypeDef *)( ((uint32_t)GPIOA)+(PIN>>16) ))
#define GET_PIN(PIN)      (PIN&0xFFFF)

//------------------------------------------------------------------------------
//                             GPIO_Set     (function)
//------------------------------------------------------------------------------

__FORCEINLINE void GPIO_PinSet(int32_t Pin){
  
 GET_PORT(Pin)->BSRR=GET_PIN(Pin);
}
/*!  GPIO_PinSet(Pin) Sets HIGH level on Pin
 *   @Arguments GPIO_PinSet(Pin):
 *   & Ref. to Pin definitions in GPIO.h
 */

//------------------------------------------------------------------------------
//                            GPIO_Reset    (function)
//------------------------------------------------------------------------------

__FORCEINLINE void GPIO_PinReset(uint32_t Pin){
  
 GET_PORT(Pin)->BSRR=GET_PIN(Pin)<<16;  
}
/*!  GPIO_PinReset(Pin) Sets LOW level on Pin
 *   @Arguments GPIO_PinReset(Pin):
 *   & Ref. to Pin definitions in GPIO.h
 */

//------------------------------------------------------------------------------
//                            GPIO_Toggle  (function)
//------------------------------------------------------------------------------

__FORCEINLINE void GPIO_PinToggle(uint32_t Pin){
  
 GET_PORT(Pin)->ODR = (GET_PORT(Pin)->ODR)^GET_PIN(Pin);  
}
/*!  GPIO_PinToggle( Pin ) INVERTS level on Pin
 *   @Arguments GPIO_PinToggle( Pin ):
 *   & Ref. to Pin definitions in GPIO.h
 */

//------------------------------------------------------------------------------
//                            GPIO_IsPinSet    (function)
//------------------------------------------------------------------------------

__FORCEINLINE uint32_t GPIO_IsPinSet(uint32_t Pin){
  
 return (  ((GET_PORT(Pin)->IDR) & GET_PIN(Pin)) == GET_PIN(Pin)  );
}
/*!  GPIO_IsPinSet( Pin )  checks the Input Data Register
 *   @Return current level on Pin(IDR whith pin mask)
 *   @Arguments GPIO_IsPinSet( Pin):
 *   & Ref. to Pin definitions in GPIO.h
 */

//------------------------------------------------------------------------------
//                            GPIO_ReadPort     (function)
//------------------------------------------------------------------------------

__FORCEINLINE uint32_t GPIO_ReadPort(GPIO_TypeDef *GPIOx){
  
 return (uint32_t)(GPIOx->IDR);  
}
/*!   GPIO_ReadPort(GPIOx) checks the Input Data Register of GPIOx
 *    @Return IDR full data
 *    @Arguments GPIO_ReadPort(GPIOx):
 *    & Ref. to stm32f415xx.h
 *    GPIOA
 *    GPIOB
 *    GPIOC
 *    GPIOD
 *    GPIOE
 *    GPIOF
 *    GPIOG
 *    GPIOH 
 *    GPIOI
 */

//------------------------------------------------------------------------------
//                            GPIO_WritePort     (function)
//------------------------------------------------------------------------------

__FORCEINLINE void GPIO_WritePort(GPIO_TypeDef *GPIOx, uint32_t PortValue){
  
 GPIOx->ODR = PortValue;  
}    
/*!   GPIO_WritePort(GPIOx, PortValue) Write  PortValue to ODR of GPIOx
 *    @Arguments GPIO_WritePort(GPIOx, PortValue):
 *    & Ref. to stm32f415xx.h
 *    GPIOA        
 *    GPIOB        PortValue - Data for  Output Data Register
 *    GPIOC
 *    GPIOD
 *    GPIOE
 *    GPIOF
 *    GPIOG
 *    GPIOH
 *    GPIOI
 */

//------------------------------------------------------------------------------
//                            GPIO_Pin        (function)
//------------------------------------------------------------------------------

__FORCEINLINE void GPIO_Pin(uint32_t Pin, uint32_t Mode, uint32_t Pull, uint32_t Alternate){
  
 uint32_t PinOnly = GET_PIN(Pin); 
 uint32_t Position = POSITION_VAL(PinOnly);
 RCC->AHBENR |= RCC_AHBENR_GPIOAEN<<(Pin>>26);     //GPIO clock
 while(PinOnly !=0){ 
   MODIFY_REG(GET_PORT(Pin)->PUPDR, (0x03UL<<(Position * 2U)), (Pull<<(Position * 2U))); //Pull config
   MODIFY_REG(GET_PORT(Pin)->MODER, (0x03UL<<(Position * 2U)), (Mode<<(Position * 2U))); //Mode config  
   MODIFY_REG(GET_PORT(Pin)->AFR[(Position/8)], (0x0FUL<<((Position%8UL) * 4U)), (Alternate<<((Position%8UL) * 4U))); //AF config 
   MODIFY_REG(GET_PORT(Pin)->OSPEEDR, (0x03UL<<(Position * 2U)), (SPEED_HIGH<<(Position * 2U))); //Speed config  
   PinOnly &= ~(1U<<Position);
   Position = POSITION_VAL(PinOnly);  
 }  
}
/*!   GPIO_Pin(Pin, Mode, Pull, Alternate) config Mode, Pulling and Alternate Functions
 *    of corresponding pins. It can work with seweral Pins the same PORT  (e.c. PA1|PA3|PA7 )
 *    & Clocks GPIO Port 
 *    IT sets push and speed to PUSHPULL and SPEED_HIGH as default  
 *    @Arguments for  GPIO_Pin(Pin, Mode, Pull, Alternate):
 *    & Ref. to Pin definitions in GPIO.h
 *          MODE_INPUT         PULL_NO     AF0                      
 *          MODE_OUTPUT        PULL_UP     AF1                       
 *          MODE_ALTERNATE     PULL_DOWN   AF2                       
 *          MODE_ANALOG                    AF3                       
 *                                         AF4                       
 *                                         AF5                       
 *                                         AF6                       
 *                                         AF7                       
 *                                         AF8                       
 *                                         AF9                       
 *                                         AF10                      
 *                                         AF11                      
 *                                         AF12                      
 *                                         AF13                      
 *                                         AF14                      
 *                                         AF15                      
 */

//------------------------------------------------------------------------------
//                            GPIO_AF          (function)
//------------------------------------------------------------------------------

__FORCEINLINE void GPIO_AF(uint32_t Pin, uint32_t Alternate){
  
 uint32_t PinOnly = GET_PIN(Pin); 
 uint32_t Position = POSITION_VAL(PinOnly);
 while(PinOnly !=0){ 
   MODIFY_REG(GET_PORT(Pin)->AFR[(Position/8)], (0x0FUL<<((Position%8UL) * 4U)), (Alternate<<((Position%8UL) * 4U)));  
   PinOnly &= ~(1U<<Position);
   Position = POSITION_VAL(PinOnly);  
 }  
}
 /*!   GPIO_AF(Pin, Alternate) config Alternate Functions of corresponding pins.
 *     It can work with seweral Pins the same PORT  (e.c. PA1|PA3|PA7 )
 *     @Arguments for  GPIO_AF(Pin, Alternate):
 *     & Ref. to Pin definitions in GPIO.h
 *                                 AF0                      
 *                                 AF1                       
 *                                 AF2                       
 *                                 AF3                       
 *                                 AF4                       
 *                                 AF5                       
 *                                 AF6                       
 *                                 AF7                       
 *                                 AF8                       
 *                                 AF9                       
 *                                 AF10                      
 *                                 AF11                      
 *                                 AF12                      
 *                                 AF13                      
 *                                 AF14                      
 *                                 AF15                      
 */

//------------------------------------------------------------------------------
//                           GPIO_Pull        (function)
//------------------------------------------------------------------------------

__FORCEINLINE void GPIO_Pull(uint32_t Pin, uint32_t Pull){
  
 uint32_t PinOnly = GET_PIN(Pin); 
 uint32_t Position = POSITION_VAL(PinOnly);
 while(PinOnly !=0){    
   MODIFY_REG(GET_PORT(Pin)->PUPDR, (0x03UL<<(Position * 2U)), (Pull<<(Position * 2U)));
   PinOnly &= ~(1U<<Position);
   Position = POSITION_VAL(PinOnly);  
 } 
}
/*!    GPIO_Pull(Pin, Pull) config PULL of corresponding pins.
 *     It can work with seweral Pins the same PORT  (e.c. PA1|PA3|PA7 )
 *     @Arguments for  GPIO_Pull(Pin, Pull):
 *     & Ref. to Pin definitions in GPIO.h
 *               PULL_NO   
 *               PULL_UP   
 *               PULL_DOWN 
 */

//------------------------------------------------------------------------------
//                          GPIO_Mode          (function)
//------------------------------------------------------------------------------

__FORCEINLINE void GPIO_Mode(uint32_t Pin, uint32_t Mode){
  
 uint32_t PinOnly = GET_PIN(Pin); 
 uint32_t Position = POSITION_VAL(PinOnly); 
 while(PinOnly !=0){    
   MODIFY_REG(GET_PORT(Pin)->MODER, (0x03UL<<(Position * 2U)), (Mode<<(Position * 2U)));
   PinOnly &= ~(1U<<Position);
   Position = POSITION_VAL(PinOnly);  
 }
}
/*!    GPIO_Mode(Pin, Mode) config MODE of corresponding pins.
 *     It can work with seweral Pins the same PORT  (e.c. PA1|PA3|PA7 )
 *     @Arguments for  GPIO_Mode(Pin, Mode):
 *     & Ref. to Pin definitions in GPIO.h
 *               MODE_INPUT
 *               MODE_OUTPUT
 *               MODE_ALTERNATE
 *               MODE_ANALOG
 */

//------------------------------------------------------------------------------
//                         GPIO_Speed          (function)
//------------------------------------------------------------------------------

__FORCEINLINE void GPIO_Speed(uint32_t Pin, uint32_t  Speed){
  
 uint32_t PinOnly = GET_PIN(Pin); 
 uint32_t Position = POSITION_VAL(PinOnly);
 while(PinOnly !=0){    
   MODIFY_REG(GET_PORT(Pin)->OSPEEDR, (0x03UL<<(Position * 2U)), (Speed<<(Position * 2U)));
   PinOnly &= ~(1U<<Position);
   Position = POSITION_VAL(PinOnly);  
 } 
}
/*!    GPIO_Speed(Pin, Speed) config SPEED of corresponding pins.
 *     It can work with seweral Pins the same PORT  (e.c. PA1|PA3|PA7 )
 *     @Arguments for  GPIO_Speed(Pin, Speed):
 *     & Ref. to Pin definitions in GPIO.h
 *               SPEED_LOW
 *               SPEED_MEDIUM
 *               SPEED_HIGH
 *               SPEED_VERY_HIGH
 */

//------------------------------------------------------------------------------
//                        GPIO_Type           (function)
//------------------------------------------------------------------------------

__FORCEINLINE void GPIO_Type(uint32_t Pin, uint32_t Type){
  
 uint32_t PinOnly = GET_PIN(Pin); 
 uint32_t Position = POSITION_VAL(PinOnly);
 while(PinOnly !=0){    
   MODIFY_REG(GET_PORT(Pin)->OTYPER, GET_PIN(Pin), (Type * (GET_PIN(Pin))));
   PinOnly &= ~(1U<<Position);
   Position = POSITION_VAL(PinOnly);  
 }       
}
/*!    GPIO_Type(Pin, Type) config TYPE of corresponding pins.
 *     It can work with seweral Pins the same PORT  (e.c. PA1|PA3|PA7 )
 *     @Arguments for  GPIO_Type(Pin, Type):
 *     & Ref. to Pin definitions in GPIO.h
 *               TYPE_PUSHPULL
 *               TYPE_OPENDRAIN
 */


#endif 