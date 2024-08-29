#include "stdint.h"
#include "main.h"

#define INNER_LOOP 100
#define OUTER_LOOP 15000

int8_t T1[INNER_LOOP];
int8_t T2[INNER_LOOP];
int16_t PFPN[INNER_LOOP];

int16_t w1 = 100; //Determined by characterization
int16_t w2 = 99;  //Determined by characterization

int16_t pfpn_k = 0;
int16_t pfpn_i = 0;
int16_t temp = 0;

void PFPN_Init()
{
}

void PFPN_Check()
{

  if (pfpn_k >= OUTER_LOOP)
    return;

  //Simulate PFPN calculation
  if (T1[pfpn_i] == -128 || T2[pfpn_i] == -128)
    temp = 0x20;
  else
    temp = ((T1[pfpn_i] * w1) + (T2[pfpn_i] * w2)) >> 8;
  PFPN[pfpn_i] = temp;
  
  ++pfpn_i;
  if (pfpn_i >= INNER_LOOP) {
    pfpn_i = 0;
    ++pfpn_k;
  }
}


