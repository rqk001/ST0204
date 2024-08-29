/*
 *  Copyright (c) 2019 BAE Systems
 *
 *  Author(s):  Roman Kostin  <rqk@stlport.com>
 *
 */

/**
 *  Standard Compliant Low Light Level Camera
 *
 *  SPI backend for I2C register file server
 *
 */

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_spi.h"
#include "stdint.h"
#include "main.h"

typedef enum {
	SPI_IDLE,
	SPI_WRITEADDR,
	SPI_READADDR,
  SPI_WRITEREADY,
  SPI_READWAIT,
  SPI_WRITESENT,
  SPI_READSENT,
  SPI_WRITEDONE,
  SPI_READDONE,
  SPI_ERROR
} SPI_StateTypeDef;

void Reg_WriteDone(uint16_t reg, uint16_t *val);
void Reg_ReadDone(uint16_t reg, uint16_t *val);

#define MASTER_BOARD

SPI_HandleTypeDef SpiHandle;

uint16_t  SPI_rdreg;
uint16_t  SPI_wrreg;
uint16_t  SPI_rdval;
uint16_t  SPI_wrval;

/* transfer state */
SPI_StateTypeDef SPI_State = SPI_IDLE;

#define SM_INSTR
// XXX primitive instrumentation
#ifdef  SM_INSTR
typedef struct {
  int       l;
  SPI_StateTypeDef    s;
} trans_t;

#define LOGMAX  128
trans_t   stl[LOGMAX];
int       sti;

#define SetState(st) do { stl[sti].l = __LINE__; stl[sti].s = (st); ++sti; sti %= LOGMAX; SPI_State = (st); } while (0)

#else

#define SetState(st) do { SPI_State = (st); } while (0)

#endif  // SM_INSTR

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Error_Handler(void);

/* Private functions ---------------------------------------------------------*/

static void
SendReadAddress(uint16_t reg)
{

  SetState(SPI_READADDR);

  // Set RD bit
  SPI_rdreg = reg | 0x8000;
  if (HAL_SPI_Transmit_IT(&SpiHandle, (uint8_t *)&SPI_rdreg, sizeof(uint16_t)) != HAL_OK) {
    Error_Handler();
  }
}

static void
SendWriteAddress(uint16_t reg)
{

  SetState(SPI_WRITEADDR);
  SPI_wrreg = reg;

  // Reset RD bit
  reg &= 0x7fff;
  if (HAL_SPI_Transmit_IT(&SpiHandle, (uint8_t *)&reg, sizeof(uint16_t)) != HAL_OK) {
    Error_Handler();
  }
}

static void
SendRegData(void)
{
  if (HAL_SPI_Transmit_IT(&SpiHandle, (uint8_t *)&SPI_wrval, sizeof(uint16_t)) != HAL_OK) {
    Error_Handler();
  }
}

static void
RcvRegData(void)
{
  if (HAL_SPI_Receive_IT(&SpiHandle, (uint8_t *)&SPI_rdval, sizeof(uint16_t)) != HAL_OK) {
    Error_Handler();
  }
}

void
SPI_Read(uint16_t reg)
{
  SendReadAddress(reg);
}

void
SPI_Write(uint16_t reg, uint16_t val)
{
  SPI_wrval = val;
  SendWriteAddress(reg);
}

void
SPI_Check(void)
{
  HAL_NVIC_DisableIRQ(SPIx_IRQn);
/******************* Start Critical ******************/
  switch (SPI_State) {
  case SPI_READWAIT:
    SetState(SPI_READSENT);
    RcvRegData();
    break;
  case SPI_WRITEREADY:
    SetState(SPI_WRITESENT);
    SendRegData();
    break;
  case SPI_READDONE:
    SetState(SPI_IDLE);
    Reg_ReadDone(SPI_rdreg, &SPI_rdval);
    break;
  case SPI_WRITEDONE:
    SetState(SPI_IDLE);
    Reg_WriteDone(SPI_wrreg, &SPI_wrval);
    break;
  default:
    break;
  }
/******************* End Critical ******************/
  HAL_NVIC_EnableIRQ(SPIx_IRQn);
}

void
ML_SPI_Init(void)
{

  /* Set the SPI parameters */
//  SpiHandle.Instance               = SPIx;
  SpiHandle.Instance               = SPI1;
  SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
  SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
  SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
  SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
  SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
  SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  SpiHandle.Init.CRCPolynomial     = 7;
  SpiHandle.Init.CRCLength         = SPI_CRC_LENGTH_8BIT;
  SpiHandle.Init.NSS               = SPI_NSS_SOFT;
  SpiHandle.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
  SpiHandle.Init.Mode              = SPI_MODE_MASTER;

  if(HAL_SPI_Init(&SpiHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = PLL (MSI)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            PLL_M                          = 1
  *            PLL_N                          = 40
  *            PLL_R                          = 2
  *            PLL_P                          = 7
  *            PLL_Q                          = 4
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
#if 0
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* MSI is enabled after System reset, activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLP = 7;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
}
#endif  // 0

/**
  * @brief  This function handles SPI interrupt request.
  * @param  None
  * @retval None
  */
void SPIx_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&SpiHandle);
}


/**
  * @brief  TxRx Transfer completed callback.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report end of Interrupt TxRx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  switch (SPI_State) {
  case SPI_READSENT:
    SetState(SPI_READDONE);
    break;
  default:
    SetState(SPI_ERROR);
    break;
  }
}

/**
  * @brief  TxRx Transfer completed callback.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report end of Interrupt TxRx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{

  switch (SPI_State) {
  case SPI_READADDR:
    SetState(SPI_READWAIT);
    break;
  case SPI_WRITEADDR:
    SetState(SPI_WRITEREADY);
    break;
  case SPI_WRITESENT:
    SetState(SPI_WRITEDONE);
    break;
  default:
    SetState(SPI_ERROR);
    break;
  }
}

/**
  * @brief  TxRx Transfer completed callback.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report end of Interrupt TxRx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  SetState(SPI_ERROR);
}

/**
  * @brief  SPI error callbacks.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  SetState(SPI_ERROR);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  while(1)
  {
    HAL_Delay(1000);
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

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
