#ifndef _DISPLAY_H_21_01_2017
#define _DISPLAY_H_21_01_2017

#ifdef __cplusplus
 extern "C" {
#endif
   
#include <stdint.h>

   
// Limit 13 numeric symbols
#define DISPLAY_LIMIT_SHOW (12349999L)

// Declare display functions
void Display_ShowTime(int32_t seconds);

// Use defines to link the hardware configurations to the correct numbers
#define COMMON_CATHODE 0
#define COMMON_ANODE 1
#define N_TRANSISTORS 2
#define P_TRANSISTORS 3
#define NP_COMMON_CATHODE 1
#define NP_COMMON_ANODE 0

#ifdef __cplusplus
}
#endif

#endif //_DISPLAY_H_21_01_2017