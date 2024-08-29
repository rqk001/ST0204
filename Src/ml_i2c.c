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
 *  -<S>-<DA>-<R0LO>-<R0HI>-<D0LO>-<D0HI>- [ -<DnLO>-<DnHI>- ] -<P>-
 *
 *  S - START
 *  DA - I2C slave address (R bit reset)
 *  R0{LO,HI} - first register address, LSB and MSB
 *  Dx{LO,HI} - data for register x
 *  P - STOP
 *
 *  Register read:
 *  -<S>-<DA>-<R0LO>-<R0HI>-[<P>]-+
 *                                 |
 *  +------------------------------+
 *  |
 *  +-<S>-<DA>-<D0LO>-<D0HI>- [ -<DnLO>-<DnHI>- ] -<P>-  
 *
 *  Stop condition after R0HI is optional.
 *
 *  S - START
 *  DA - I2C slave address (R bit reset)
 *  DA - I2C slave address (R bit set)
 *  R0{LO,HI} - first register address, LSB and MSB
 *  Dx{LO,HI} - data from register x
 *  P - STOP
 *
 *
 *  Unlike on the client (master) side, we don't know the number of
 *  items to be read or written. This is deduced:
 *
 *  On Register Write: After the last byte is sent, a STOP
 *  condition is raised
 *
 *  On Register Read: After the last byte is received, a NACK
 *  is sent instead of an ACK - this conforms to I2C Spec
 *  (https://www.nxp.com/docs/en/user-guide/UM10204.pdf)
 *  p. 3.1.6 #5:
 *  "A master-receiver must signal the end of the transfer to
 *  the slave transmitter  ...  leads to the generation of NACK"
 *
 *  This will be implemented as a state machine with the following
 *  logic:
 *
 *  I2C Slave Side State Machine:
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
 *    |       | <DFETCH>                                            |
 *    |       |                                                     |
 *    |       V             (ADDR_IT)                               |
 *    |    WADDRDONE-------------------->READREG2                   |
 *    |       |      \                     |                        |
 *    |       |       \ (DFETCH_DONE)      | (DFETCH_DONE)          |
 *    |       |        |<TX_DATA>          | <TX_DATA>              |
 *    |       |        V     (ADDR_IT)     |                        |
 *    |       |      READREG1------------+ |                        |
 *    |       |           |              | |                        |
 *    |       | (RXNE_IT) |              | |                        |
 *    |       |          /               | |                        |
 *    |       |         / <TX_FLUSH>     | |                        |
 *    |  +--+ | +------+                 | |                        |
 *    |  |  V V V                        V V                        |
 *    |  |  WDATA1               +----->RDATA1                      |
 *    |  |    |  (RXNE_IT)       |         | (TXE_IT)               |
 *    |  |    |  <RX_DATA>       |         | <TX_DATA>              |
 *    |  |    V                  |         | <DFETCH>               |
 *    |  | WDATA1P               |         V           (NACK_IT)    | 
 *    |  |    |  <DSTORE>        |      RDATA1P---------------------+
 *    |  |    |                  |         |          <TX_FLUSH>
 *    |  |    |                  |         | (DFETCH_DONE)
 *    |  |    V    (STOP_IT)     |         |
 *    |  | WDATA0----------+     |         V 
 *    |  |    | (RNXE_IT)  |     |      RDATA0 
 *    |  |    | <RX_DATA>  |     |         | (TXE_IT)
 *    |  +----+            |     |         | <TX_DATA>
 *    +--------------------+     |         | <DFETCH>
 *                               |         V
 *                               |      RDATA0P
 *                               |         |
 *                               |         | (DFETCH_DONE)
 *                               +---------+
 *                
 *
 *    (ITEM) in parentheses denotes event or condition
 *    <ITEM> in brackets denotes action
 *
 *    (ADDR_IT) - I2C bus address interrupt
 *    (RXNE_IT) - Receiver nonempty interrupt
 *    (TXE_IT) - Transmitter empty interrupt
 *    (STOP_IT) - STOP Condition interrupt
 *    (NACK_IT) - NACK interrupt
 *    <RX_DATA> - Read from receiver HW
 *    (STOP_IT) - STOP condition interrupt
 *    <DFETCH> - Request next register value read
 *    (DFETCH_DONE) - Next register value ready
 *    <DSTORE> - Request next register write
 *    <TX_DATA> - Write to transmitter HW
 *    <MASK_TXE> - Disable TXE interrupt
 *    <UNMASK_TXE> - Enable TXE interrupt
 *
 */


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ml_i2c.h"
#include "stm32l4xx_ll_i2c.h"

I2CS_State_Typedef  I2CS_State = I2CS_IDLE;

volatile int      i;
#define RESET_DELAY   20

#define RDATA_ASYNC 1
// #undef RDATA_ASYNC

int       RI;                                 // Register index
union {uint8_t RA8[2]; uint16_t RA16; } RA;   // Register #
union {uint8_t DA8[2]; uint16_t DA16; } DA;   // Data

void TIM2_Timeout_us(void (*cb)(void *), void *arg, int us);
#if NOTYET
void SPI_Read(void);
void SPI_Write(uint16_t reg, uint16_t val);
#else
void MEM_Read(void);
void MEM_Write(void);
#endif
void Reg_ReadCmplt(void *arg);

// Local Memory
static uint16_t *A = (uint16_t *)0x10000000;

#define MEM_READ_DELAY_US 100

/********************** Instrumentation ************************/
/*
 *  Simple State Machine instrumentation for debugging
 *
 *  Keeping the last LOGMAX state transitions in a circular buffer.
 *  Record source line number and new state.
 *
 */

// Turn it on
#define SM_INSTR
// #undef SM_INSTR

#ifdef  SM_INSTR
typedef struct {
  int       l;
  I2CS_State_Typedef    s;
} trans_t;

#define LOGMAX  128
trans_t   tl[LOGMAX];
int       ti;

#define SetState(st) do { tl[ti].l = __LINE__; tl[ti].s = (st); ++ti; ti %= LOGMAX; I2CS_State = (st); } while (0)

#else // SM_INSTR

#define SetState(st) do { I2CS_State = (st); } while (0)

#endif  // SM_INSTR

/* Private function prototypes -----------------------------------------------*/
void     SystemClock_Config(void);
void     LED_Init(void);
void     LED_On(void);
void     LED_Off(void);
void     LED_Blinking(uint32_t Period);
void     Configure_I2C_Slave(void);

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
  NVIC_SetPriority(I2C3_EV_IRQn, 0);  
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
   *  - Enable Receiver Nonempty interrupt
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

// Just stick around and let's debug it
void
Error_Handler()
{
  for (;;);
}

// Empty callback
void
Reg_WriteDone(uint16_t reg, uint16_t *val)
{
}

#ifdef  NO_SPI_YET
/*************************** SPI interface ****************************/
// XXX Hackety hack
void
Reg_Write()
{
  SPI_Write(RA.RA16 + RI, DA.DA16);
}

void
Reg_Read()
{
  // XXX Async
  SPI_Read(RA.RA16 + RI);
}

#else // NO_SPI_YET

void
MEM_Write()
{
uint16_t    *A = (uint16_t *)0x10000000;

  A[RA.RA16 + RI] = DA.DA16;
}

void
MEM_Read()
{
#ifdef  RDATA_ASYNC
  // XXX Async
  TIM2_Timeout_us(Reg_ReadCmplt, (void *)(&A[RA.RA16 + RI]), MEM_READ_DELAY_US);
#else
  Reg_ReadCmplt((void *)(&A[RA.RA16 + RI]));
#endif
}

// XXX Hackety hack
void
Reg_Write()
{
  MEM_Write();
}

void
Reg_Read()
{
  // XXX Async
  MEM_Read();
}

#endif // NO_SPI_YET

void
DFetch()
{
  Reg_Read();
}

/**************************** Read Callback *************************/
void
Reg_ReadCmplt(void *val)
{
  DA.DA16 = *(uint16_t *)val;

  // To eliminate races
  NVIC_DisableIRQ(I2C3_EV_IRQn);

/*************Critical - IRQ disabled*******************/
  switch (I2CS_State) {
  case I2CS_READREG2:
    ++RI;
    SetState(I2CS_RDATA1);
    LL_I2C_TransmitData8(I2C3, DA.DA8[0]);
    LL_I2C_EnableIT_TX(I2C3);
    break;

  case I2CS_RDATA1P:
    ++RI;
    SetState(I2CS_RDATA0);
    LL_I2C_EnableIT_TX(I2C3);
    break;

  case I2CS_WADDRDONE:
    ++RI;
    SetState(I2CS_READREG1);
    LL_I2C_TransmitData8(I2C3, DA.DA8[0]);
    LL_I2C_EnableIT_TX(I2C3);
    break;

  case I2CS_RDATA0P:
    ++RI;
    SetState(I2CS_RDATA1);
    LL_I2C_EnableIT_TX(I2C3);
    break;

  default:
//    Error_Handler();
    break;
  }
/**************End critical - IRQ enabled*****************/

  // Reenable interrupt
  NVIC_EnableIRQ(I2C3_EV_IRQn);
}
 
/*
 *  To eliminate the race, we just delay the callback 100 us
 *
 */
void
Reg_ReadDone(uint16_t reg, uint16_t *val)
{
  Reg_ReadCmplt(val);
}

/************************* Main Loop Check ****************************/

void
I2CS_Check(I2C_TypeDef *hi2c)
{
  /*
   *
   *  Along with ML_I2CS_EV_IRQHadler() implements a state machine
   *  for I2C Slave interface
   *
   */

  // To eliminate races
  NVIC_DisableIRQ(I2C3_EV_IRQn);

/*************Critical - IRQ disabled*******************/
  switch (I2CS_State) {
  case I2CS_READREG:
    // ignore
    break;

  case I2CS_WDATA1P:
    Reg_Write();
    SetState(I2CS_WDATA0);
    break;
  }
/**************End critical - IRQ enabled*****************/

  // Reenable interrupt
  NVIC_EnableIRQ(I2C3_EV_IRQn);
}


/************************** Device ADDR on the bus ******************/

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
      RI = RI = 0;
      if (LL_I2C_GetTransferDirection(hi2c) == LL_I2C_DIRECTION_READ) {
        Error_Handler();
      } else {
        // Transition to I2CS_WADDR
        SetState(I2CS_WADDR0);
      }
    }
    break;

  // Get back to process thread to read data 
  case I2CS_WADDRDONE:
    LL_I2C_EnableIT_TX(hi2c);
    SetState(I2CS_READREG2);
    break;

  case I2CS_READREG1:
    LL_I2C_EnableIT_TX(hi2c);
    SetState(I2CS_RDATA1);
    break;

  default:
    Error_Handler();
    break;
  }
}

/************************* Receiver nonempty ********************/

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
    RA.RA16 = RA.RA16;
    Reg_Read();
    SetState(I2CS_WADDRDONE);
    break;

  case I2CS_WADDRDONE:
    DA.DA8[0] = LL_I2C_ReceiveData8(hi2c);
    SetState(I2CS_WDATA1);
    break;

  case I2CS_READREG1:
    DA.DA8[0] = LL_I2C_ReceiveData8(hi2c);
    LL_I2C_ClearFlag_TXE(I2C3);
    SetState(I2CS_WDATA1);
    break;

  case I2CS_WDATA0:
    ++RI;
    DA.DA8[0] = LL_I2C_ReceiveData8(hi2c);
    SetState(I2CS_WDATA1);
    break;

  case I2CS_WDATA1:
    DA.DA8[1] = LL_I2C_ReceiveData8(hi2c);
    SetState(I2CS_WDATA1P);
    break;

  default:
    SetState(I2CS_State);
    Error_Handler();
    break;
  }
}

void
ML_I2CS_STOP_Callback(I2C_TypeDef *hi2c)
{
  switch (I2CS_State) {
  case I2CS_WDATA0:
    SetState(I2CS_IDLE);
    break;

  case I2CS_RDATA1:
  case I2CS_RDATA1P:
    LL_I2C_DisableIT_TX(I2C3);
    LL_I2C_ClearFlag_TXE(I2C3);
    SetState(I2CS_IDLE);
    break;

  case I2CS_WDATA1P:
  case I2CS_READREG1:
  case I2CS_WADDRDONE:
  case I2CS_IDLE:
    // Ignore
    break;

  default:
    Error_Handler();
    break;
  }
}

void
ML_I2CS_NACK_Callback(I2C_TypeDef *hi2c)
{
  switch (I2CS_State) {
  case I2CS_RDATA1:
  case I2CS_RDATA1P:
    LL_I2C_DisableIT_TX(I2C3);
    LL_I2C_ClearFlag_TXE(I2C3);
    SetState(I2CS_IDLE);
    break;

  default:
    Error_Handler();
    break;
  }
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
    LL_I2C_DisableIT_TX(hi2c);
    LL_I2C_TransmitData8(hi2c, DA.DA8[1]);
    DFetch();
    SetState(I2CS_RDATA1P);
    break;

  case I2CS_IDLE:
    Error_Handler();
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

  if (LL_I2C_IsEnabledIT_NACK(hi2c)  &&  LL_I2C_IsActiveFlag_NACK(hi2c)) {
    ML_I2CS_NACK_Callback(hi2c);
    LL_I2C_ClearFlag_NACK(hi2c);
    return;
  }
    
  if (LL_I2C_IsEnabledIT_STOP(hi2c)  &&  LL_I2C_IsActiveFlag_STOP(hi2c)) {
    ML_I2CS_STOP_Callback(hi2c);
    LL_I2C_ClearFlag_STOP(hi2c);
    return;
  }

  if (LL_I2C_IsEnabledIT_ADDR(hi2c)  &&  LL_I2C_IsActiveFlag_ADDR(hi2c)) {
    ML_I2CS_ADDR_Callback(hi2c);
    LL_I2C_ClearFlag_ADDR(hi2c);
    return;
  }
    
  if (LL_I2C_IsEnabledIT_RX(hi2c)  &&  LL_I2C_IsActiveFlag_RXNE(hi2c)) {
    ML_I2CS_RXNE_Callback(hi2c);
    return;
  }
    
  if (LL_I2C_IsEnabledIT_TX(hi2c)  &&  LL_I2C_IsActiveFlag_TXE(hi2c)) {
    ML_I2CS_TXE_Callback(hi2c);
    return;
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

  Error_Handler();

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
