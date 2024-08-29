/*
 *  Standard Compliant Low Light Level (SCL3C) Camera
 *
 *  Copyright (c) 2019 BAE Systems
 *
 *  Author(s):  Roman Kostin  <rqk@stlport.com>
 *
 */

#include "stm32l4xx_hal_gpio.h"
#include "stm32l4xx_hal_rcc.h"
#include "stm32l4xx_it.h"
#include "stm32l4xx_ll_tim.h"

#define TXNE_DELAY  24

/**
 *  Timeout callbacks
 *
 */
// Callback type
typedef void (*CBType)(void *);

// Timeout entry
typedef struct {
  uint32_t        id;
  CBType          cb;
  void            *arg;
} TO_entry;

// Should be an entry per timer
TO_entry Timeout;

/*
 *  Microsecond timeout
 *
 */
void
TIM2_Timeout_us(CBType cb, void *arg, uint32_t us)
{
  Timeout.id = (uint32_t)TIM2;
  Timeout.cb = cb;
  Timeout.arg = arg;

  LL_TIM_DisableCounter(TIM2);
  LL_TIM_SetPrescaler(TIM2, (SystemCoreClock / 1000000) - 1); // 1 MHz
  LL_TIM_SetAutoReload(TIM2, us);

  /* TIM IT enable */
  LL_TIM_EnableIT_UPDATE(TIM2);
  LL_TIM_EnableCounter(TIM2);
}

/*
 *  Millisecond timeout
 *
 */
void
TIM2_Timeout_ms(CBType cb, void *arg, uint32_t ms)
{
  Timeout.id = (uint32_t)TIM2;
  Timeout.cb = cb;
  Timeout.arg = arg;

  LL_TIM_DisableCounter(TIM2);
  LL_TIM_SetPrescaler(TIM2, (SystemCoreClock / 1000) - 1); // 1 MHz
  LL_TIM_SetAutoReload(TIM2, ms);

  /* TIM IT enable */
  LL_TIM_EnableIT_UPDATE(TIM2);
  LL_TIM_EnableCounter(TIM2);
}

void
TIM2_TimeoutCallback(void)
{
TO_entry      TO = Timeout;

  Timeout.id = 0;
  Timeout.cb = 0;
  Timeout.arg = 0;

  LL_TIM_ClearFlag_UPDATE(TIM2);
  LL_TIM_DisableIT_UPDATE(TIM2);
  LL_TIM_DisableCounter(TIM2);

  (*TO.cb)(TO.arg);
}

void
TIM2_Update_Callback()
{
  if (Timeout.id != 0) {
    TIM2_TimeoutCallback();
  }
}
 
void
TIM2_IRQHandler(void)
{
  if (LL_TIM_IsActiveFlag_UPDATE(TIM2)) {
    LL_TIM_ClearFlag_UPDATE(TIM2);
  }    
  TIM2_Update_Callback();
}

void
TIM2_Init(void)
{
  /* Enable the TIM2 gloabal Interrupt */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);

  /* TIM2 clock enable */
  __HAL_RCC_TIM2_CLK_ENABLE();

  LL_TIM_DisableCounter(TIM2);
  LL_TIM_SetPrescaler(TIM2, (SystemCoreClock / 1000000) - 1); // 1 MHz
  LL_TIM_SetOnePulseMode(TIM2, TIM_CR1_OPM);
  LL_TIM_SetAutoReload(TIM2, 1);

}


