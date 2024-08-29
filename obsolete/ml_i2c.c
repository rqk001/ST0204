/**
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/*
 *  Copyright (c) 2019 BAE Systems
 *
 *  Author(s):  Roman Kostin  <rqk@stlport.com>
 *
 */  

/**
 *  Standard Compliant Low Light Level Camera
 *
 *  I2C Slave side communications as per Interface Control Document (ICD)
 *
 *  Have to respond to two kinds of queries
 *
 *  Register write:
 *  -<S>-<DAW>-<R0LO>-<R0HI>-<D0LO>-<D0HI>- [ -<DnLO>-<DnHI>- ] -<P>-
 *
 *  S - START
 *  DAW - I2C slave address (R bit reset)
 *  R0{LO,HI} - first register address, LSB and MSB
 *  Dx{LO,HI} - data for register x
 *  P - STOP
 *
 *  Register read:
 *  -<S>-<DAW>-<R0LO>-<R0HI>-<P>-+
 *                               |
 *  +----------------------------+
 *  |
 *  +-<S>-<DAR>-<D0LO>-<D0HI>- [ -<DnLO>-<DnHI>- ] -<P>-  
 *
 *  S - START
 *  DAW - I2C slave address (R bit reset)
 *  DAR - I2C slave address (R bit set)
 *  R0{LO,HI} - first register address, LSB and MSB
 *  Dx{LO,HI} - data from register x
 *  P - STOP
 *
 *
 *  The State Machine:
 *
 *    +-----+  +----------------------------------------------------+
 *    |     |  |                                                    |
 *    |     |  |                                                    |
 *    |     V  V                                                    |
 *    |     IDLE                                                    |
 *    |       | (ADDR_IT)                                           |
 *    |       |                                                     |
 *    |       V                                                     |
 *    |    WADDR0                                                   |
 *    |       | (RXNE_IT)                                           |
 *    |       | <RX_DATA>                                           |
 *    |       V                                                     |
 *    |    WADDR1                                                   |
 *    |       | (RXNE_IT)                                           |
 *    |       | <RX_DATA>                                           |
 *    |       V            (STOP_IT)                                |
 *    |    WADDRDONE------------------->READREG                     |
 *    |       | (RXNE_IT)                  |                        |
 *    |       | <RX_DATA>                  |                        |
 *    |       |                            V                        |
 *    |       |                        READREGP                     |
 *    |       |                            | (ADDR_IT)              |
 *    |  +--+ |                            | <DFETCH>               |
 *    |  |  V V                            V                        |
 *    |  | WDATA1                      READREGP2                    |
 *    |  |    |  (RXNE_IT)                 | (DFETCH_DONE)          |
 *    |  |    |  <RX_DATA>                 |                        |
 *    |  |    V                            |                        |
 *    |  | WDATA1P                         |                        |
 *    |  |    |  <DSTORE>                  |                        |
 *    |  |    |                    +-----+ | +--------------+       |
 *    |  |    V    (STOP_IT)       |     V V V              |       |
 *    |  | WDATA0--------------+   |     RDATA0             |       |
 *    |  |    | (RNXE_IT)      |   |       | (TXE_IT)       |       |
 *    |  |    | <RX_DATA>      |   |       | <TX_DATA>      |       |
 *    |  +----+                |   |       V                |       |
 *    +------------------------+   |     RDATA1             |       |
 *                                 |       | (TXE_IT)       |       |
 *                                 |       | <TX_DATA>      |       |
 *                                 |       | <MASK_TXE>     |       |
 *       +-------------------------+       | <WAIT>         |       |
 *       |                                 V                |       |
 *       |                              RDATA1P             |       |
 *       |                                 | <DFETCH>       |       |
 *       |                                 |                |       |
 *       |                                 |                |       |
 *       |                  (WAIT_DONE)    |                |       |
 *       |      +--------------------------+                |       |
 *       |      |                          | (DFETCH_DONE)  |       |
 *       |      V                          |                |       |
 *       |  RDATAWAIT1        (STOP_IT)    V                |       |
 *       |      | (DFETCH_DONE)  +-------RDATAWAIT2         |       |
 *       |      | <UNMASK_TXE>   |         | (WAIT_DONE)    |       | 
 *       |      |                |         | <UNMASK_TXE>   |       |
 *       +------+                |         +----------------+       |
 *                               +----------------------------------+
 *
 *    (ITEM) in parentheses denotes event or condition
 *    <ITEM> in brackets denotes action
 *
 *    (ADDR_IT) - I2C bus address interrupt
 *    (RXNE_IT) - Receiver nonempty interrupt
 *    <RX_DATA> - Read from receiver HW
 *    (STOP_IT) - STOP condition interrupt
 *    <DFETCH> - Retrieve next register value
 *    (DFETCH_DONE) - Next register value ready
 *    (TXE_IT) - Transmitter empty interrupt
 *    <TX_DATA> - Write to transmitter HW
 *    <MASK_TXE> - Disable TXE interrupt
 *    <UNMASK_TXE> - Enable TXE interrupt
 *    <WAIT> - Schedule a timeout
 *    (WAIT_DONE) - Timeout triggered
 *
 */


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ml_i2c.h"
#include "stm32l4xx_ll_i2c.h"

I2CS_State_Typedef  I2CS_State = I2CS_IDLE;

volatile int      i;
#define RESET_DELAY   20

int       RI;                                 // Register index
union {uint8_t RA8[2]; uint16_t RA16; } RA;   // Register #
union {uint8_t DA8[2]; uint16_t DA16; } DA;   // Data

void TIM2_Timeout_us(void (*cb)(void *), void *arg, int us);

#define SM_INSTR
// XXX primitive instrumentation
#ifdef  SM_INSTR
typedef struct {
  int       l;
  I2CS_State_Typedef    s;
} trans_t;

#define LOGMAX  128
trans_t   itl[LOGMAX];
int       iti;

#endif  // SM_INSTR

/* Private function prototypes -----------------------------------------------*/
void     SystemClock_Config(void);
void     LED_Init(void);
void     LED_On(void);
void     LED_Off(void);
void     LED_Blinking(uint32_t Period);
void     Configure_I2C_Slave(void);

void    SPI_Write(uint16_t reg, uint16_t val);
void    SPI_Read(uint16_t reg);
/**
  * @brief  This function configures I2C3 in Slave mode.
  * @note   This function is used to :
  *         -1- Enables GPIO clock and configures the I2C3 pins.
  *         -2- Enable the I2C3 peripheral clock and I2C3 clock source.
  *         -3- Configure NVIC for I2C3.
  *         -4- Configure I2C3 functional parameters.
  *         -5- Enable I2C3.
  *         -6- Enable I2C3 address match/error interrupts.
  * @note   Peripheral configuration is minimal configuration from reset values.
  *         Thus, some useless LL unitary functions calls below are provided as
  *         commented examples - setting is default configuration from reset.
  * @param  None
  * @retval None
  */
void Configure_I2C_Slave(void)
{
  uint32_t timing = 0;

  /* (1) Enables GPIO clock and configures the I2C3 pins **********************/
  /*    (SCL on PC.0, SDA on PC.1)                     **********************/

  /* Enable the peripheral clock of GPIOC */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);

  /* Configure SCL Pin as : Alternate function, High Speed, Open drain, Pull up */
  LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_0, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOC, LL_GPIO_PIN_0, LL_GPIO_AF_4);
  LL_GPIO_SetPinSpeed(GPIOC, LL_GPIO_PIN_0, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_0, LL_GPIO_OUTPUT_OPENDRAIN);
  LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_0, LL_GPIO_PULL_UP);

  /* Configure SDA Pin as : Alternate function, High Speed, Open drain, Pull up */
  LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_1, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOC, LL_GPIO_PIN_1, LL_GPIO_AF_4);
  LL_GPIO_SetPinSpeed(GPIOC, LL_GPIO_PIN_1, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_1, LL_GPIO_OUTPUT_OPENDRAIN);
  LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_1, LL_GPIO_PULL_UP);

  /* (2) Enable the I2C3 peripheral clock and I2C3 clock source ***************/

  /* Enable the peripheral clock for I2C3 */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C3);

  /* Set I2C3 clock source as SYSCLK */
  LL_RCC_SetI2CClockSource(LL_RCC_I2C3_CLKSOURCE_SYSCLK);

  /* (3) Configure NVIC for I2C3 **********************************************/

  /* Configure Event IT:
   *  - Set priority for I2C3_EV_IRQn
   *  - Enable I2C3_EV_IRQn
   */
  NVIC_SetPriority(I2C3_EV_IRQn, 7);  
  NVIC_EnableIRQ(I2C3_EV_IRQn);

  /* Configure Error IT:
   *  - Set priority for I2C3_ER_IRQn
   *  - Enable I2C3_ER_IRQn
   */
  NVIC_SetPriority(I2C3_ER_IRQn, 0);  
  NVIC_EnableIRQ(I2C3_ER_IRQn);

  /* (4) Configure I2C3 functional parameters *********************************/

  /* Disable I2C3 prior modifying configuration registers */
  LL_I2C_Disable(I2C3);

  /* Configure the SDA setup, hold time and the SCL high, low period */
  /* Timing register value is computed with the STM32CubeMX Tool,
    * Fast Mode @400kHz with I2CCLK = 80 MHz,
    * rise time = 100ns, fall time = 10ns
    * Timing Value = (uint32_t)0x00F02B86
    */
  timing = __LL_I2C_CONVERT_TIMINGS(0x0, 0xF, 0x0, 0x2B, 0x86);
  LL_I2C_SetTiming(I2C3, timing);

  /* Configure the Own Address1 :
   *  - OwnAddress1 is SLAVE_OWN_ADDRESS
   *  - OwnAddrSize is LL_I2C_OWNADDRESS1_7BIT
   *  - Own Address1 is enabled
   */
  LL_I2C_SetOwnAddress1(I2C3, SLAVE_OWN_ADDRESS, LL_I2C_OWNADDRESS1_7BIT);
  LL_I2C_EnableOwnAddress1(I2C3);

  /* Enable Clock stretching */
  /* Reset Value is Clock stretching enabled */
  //LL_I2C_EnableClockStretching(I2C3);

  /* Configure Digital Noise Filter */
  /* Reset Value is 0x00            */
  //LL_I2C_SetDigitalFilter(I2C3, 0x00);

  /* Enable Analog Noise Filter           */
  /* Reset Value is Analog Filter enabled */
  //LL_I2C_EnableAnalogFilter(I2C3);

  /* Enable General Call                  */
  /* Reset Value is General Call disabled */
  //LL_I2C_EnableGeneralCall(I2C3);

  /* Configure the 7bits Own Address2               */
  /* Reset Values of :
   *     - OwnAddress2 is 0x00
   *     - OwnAddrMask is LL_I2C_OWNADDRESS2_NOMASK
   *     - Own Address2 is disabled
   */
  //LL_I2C_SetOwnAddress2(I2C3, 0x00, LL_I2C_OWNADDRESS2_NOMASK);
  //LL_I2C_DisableOwnAddress2(I2C3);

  /* Enable Peripheral in I2C mode */
  /* Reset Value is I2C mode */
  //LL_I2C_SetMode(I2C3, LL_I2C_MODE_I2C);

  /* (5) Enable I2C3 **********************************************************/
  LL_I2C_Enable(I2C3);

  /* (6) Enable I2C3 address match/error interrupts:
   *  - Enable Address Match Interrupt
   *  - Enable Not acknowledge received interrupt
   *  - Enable Error interrupts
   *  - Enable Stop interrupt
   */
  LL_I2C_EnableIT_ADDR(I2C3);
  LL_I2C_EnableIT_RX(I2C3);
  LL_I2C_EnableIT_NACK(I2C3);
  LL_I2C_EnableIT_ERR(I2C3);
  LL_I2C_EnableIT_STOP(I2C3);
}

// XXX Hackety hack
void
Reg_Write()
{
  SPI_Write(RA.RA16 + RI, DA.DA16);
}

void
Reg_WriteDone(uint16_t reg, uint16_t *val)
{
}

#ifdef  SM_INSTR

#define SetState(st) do { itl[iti].l = __LINE__; itl[iti].s = (st); ++iti; iti %= LOGMAX; I2CS_State = (st); } while (0)

#else

#define SetState(st) do { I2CS_State = (st); } while (0)

#endif

void
Reg_ReadDone(uint16_t reg, uint16_t *val)
{
  DA.DA16 = *val;
  switch (I2CS_State) {
  case I2CS_RDATAWAIT1:
  case I2CS_READREGP2:
    SetState(I2CS_RDATA0);
    LL_I2C_EnableIT_TX(I2C3);
    break;

  case I2CS_RDATA1P:
    SetState(I2CS_RDATAWAIT2);
    break;
  }
}
 
void
Reg_Read()
{
  // XXX Async
  SPI_Read(RA.RA16 + RI);
}

void
I2CS_StopWaitDoneCallout(void *arg)
{
  // Only if we're still waiting,
  // keep sending data
  if (I2CS_State == I2CS_RDATAWAIT2) {
    SetState(I2CS_RDATA0);
    LL_I2C_EnableIT_TX(I2C3);
  } else {
    SetState(I2CS_RDATAWAIT1);
  }
}

void
I2CS_ResetWaitDoneCallout(void *arg)
{
  SetState(I2CS_RESET);
}

void
I2CS_Check(I2C_TypeDef *hi2c)
{
static I2CS_State_Typedef last = I2CS_IDLE;
  /*
   *
   *  Along with ML_I2CS_EV_IRQHadler() implements a state machine
   *  for I2C Slave interface
   *
   */

  switch (I2CS_State) {
  case I2CS_RESETWAIT:
    TIM2_Timeout_us(I2CS_ResetWaitDoneCallout, 0, 2);
    break;

  case I2CS_RESET:
//    LL_I2C_EnableIT_TX(I2C3);
    LL_I2C_Enable(I2C3);
    SetState(I2CS_IDLE);
    break;

  case I2CS_READREGP:
    Reg_Read();
    SetState(I2CS_READREGP2);
    break;

  case I2CS_RDATA0:
  case I2CS_RDATA1:
    LL_I2C_EnableIT_TX(I2C3);
    break;

  case I2CS_RDATA1P:
    ++RI;
    Reg_Read();
    TIM2_Timeout_us(I2CS_StopWaitDoneCallout, 0, 10);
    break;

  case I2CS_WDATA1P:
    Reg_Write();
    ++RI;
    SetState(I2CS_WDATA0);
    break;

  default:
    if (last != I2CS_State) {
      last = I2CS_State;
      SetState(I2CS_State);
    }
    break;
  }
}

/*
 *  When I2C ADDR is sent on the bus
 *
 */
void
ML_I2CS_ADDR_Callback(I2C_TypeDef *hi2c)
{

  switch (I2CS_State) {
  case I2CS_IDLE:
    if (LL_I2C_GetAddressMatchCode(hi2c) == SLAVE_OWN_ADDRESS) {
      RI = 0;
      if (LL_I2C_GetTransferDirection(hi2c) == LL_I2C_DIRECTION_READ) {
        SetState(I2CS_RDATA);
      } else {
        // Transition to I2CS_WADDR
        SetState(I2CS_WADDR0);
      }
    }
    break;

  // Get back to process thread to read data
  case I2CS_READREG:
    LL_I2C_DisableIT_TX(hi2c);
    SetState(I2CS_READREGP);
    break;

  default:
    SetState(I2CS_State);
    break;
  }
}

void
ML_I2CS_RXNE_Callback(I2C_TypeDef *hi2c)
{

  switch (I2CS_State) {
  case I2CS_WADDR0:
    RA.RA8[0] = LL_I2C_ReceiveData8(hi2c);  // Clears the flag
    SetState(I2CS_WADDR1);
    break;

  case I2CS_WADDR1:
    RA.RA8[1] = LL_I2C_ReceiveData8(hi2c);  // Clears the flag
    // XXX
    RA.RA8[1] = 0;
    SetState(I2CS_WADDRDONE);
    break;

  case I2CS_WADDRDONE:
  case I2CS_WDATA0:
    DA.DA8[0] = LL_I2C_ReceiveData8(hi2c);
    SetState(I2CS_WDATA1);
    break;

  case I2CS_WDATA1:
    DA.DA8[1] = LL_I2C_ReceiveData8(hi2c);
    SetState(I2CS_WDATA1P);
    break;

  default:
    SetState(I2CS_State);
    break;
  }
}

void
ML_I2CS_STOP_Callback(I2C_TypeDef *hi2c)
{
  switch (I2CS_State) {
  case I2CS_WDATA0:
  case I2CS_RDATA1:
  case I2CS_RDATA1P:
  case I2CS_RDATAWAIT2:
    LL_I2C_DisableIT_TX(I2C3);
    SetState(I2CS_IDLE);
    break;

  case I2CS_WADDRDONE:
    SetState(I2CS_READREG);
    break;

  default:
    SetState(I2CS_RESETWAIT);
    LL_I2C_Disable(I2C3);
    LL_I2C_ClearFlag_TXE(I2C3);
    break;
  }
}

void
ML_I2CS_NACK_Callback(I2C_TypeDef *hi2c)
{
  SetState(I2CS_RESETWAIT);
  LL_I2C_DisableIT_TX(I2C3);
  LL_I2C_Disable(I2C3);
  LL_I2C_ClearFlag_TXE(I2C3);
}

void
ML_I2CS_TXE_Callback(I2C_TypeDef *hi2c)
{

  switch (I2CS_State) {
  case I2CS_RDATA0:
    LL_I2C_TransmitData8(hi2c, DA.DA8[0]);
    SetState(I2CS_RDATA1);
    break;

  case I2CS_RDATA1:
    LL_I2C_TransmitData8(hi2c, DA.DA8[1]);
    SetState(I2CS_RDATA1P);
    break;

  default:
    SetState(I2CS_State);
    break;
  }
}

void
ML_I2CS_EV_IRQHandler(I2C_TypeDef *hi2c)
{
  
  /*
   *
   *  Together with I2CS_Check() implements a state machine
   *  for I2C Slave interface
   *
   */

  if (LL_I2C_IsActiveFlag_ADDR(hi2c)) {
    ML_I2CS_ADDR_Callback(hi2c);
    LL_I2C_ClearFlag_ADDR(hi2c);
  }
    
  if (LL_I2C_IsActiveFlag_RXNE(hi2c)) {
    ML_I2CS_RXNE_Callback(hi2c);
  }
    
  if (LL_I2C_IsActiveFlag_TXE(hi2c)) {
    LL_I2C_DisableIT_TX(hi2c);
    ML_I2CS_TXE_Callback(hi2c);
  }
    
  if (LL_I2C_IsActiveFlag_STOP(hi2c)) {
    ML_I2CS_STOP_Callback(hi2c);
    LL_I2C_ClearFlag_STOP(hi2c);
  }
}


/**
  * @brief  Function called in case of error detected in I2C IT Handler
  * @param  None
  * @retval None
  */
void
ML_I2CS_ER_IRQHandler(I2C_TypeDef *hi2c)
{
  /* Disable I2C3_EV_IRQn */
  NVIC_DisableIRQ(I2C3_EV_IRQn);

  /* Disable I2C3_ER_IRQn */
  NVIC_DisableIRQ(I2C3_ER_IRQn);

  /* Unexpected event : Set LED2 to Blinking mode to indicate error occurs */
  LED_Blinking(LED_BLINK_ERROR);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
