#ifndef __BOARD_H__
#define __BOARD_H__

#include "cmsis.h"

#ifdef  __cplusplus
extern "C" {
#endif

#define _RED_SHIFT        (1 << 18)                   // RGB_LED Red   LED
#define _GREEN_SHIFT      (1 << 19)                   // RGB_LED Green LED
#define _BLUE_SHIFT       (1 << 1)                    // RGB_LED Blue  LED

#define _LED_D1_SHIFT    (1 << 8)                     // USR LED D1
#define _LED_D2_SHIFT    (1 << 9)                     // USR LED D2
#define _LED_D3_SHIFT    (1 << 10)                    // USR LED D3  

#define _SPI0_CS_SHIFT         (1 << 0)  
#define _SPI1_CS_SHIFT         (1 << 4)    

// SPI bus SPI1_PCS and Diagnostic Pins //    
#define _PTE2_SHIFT      (1 << 2)                     // PTE2 for DIAG2 flag
#define _PTE3_SHIFT      (1 << 3)                     // PTE3 for DIAG3 flag
#define _PTE4_SHIFT      (1 << 4)                     // PTE4 for SPI1_CS manual control
#define _PTE5_SHIFT      (1 << 5)                     // PTE4 for SPI1_CS manual control  

#define _SPI0_CS_LOW     (FPTD->PDOR &= ~(1 << 0));    // Set PTD0 pin LOW
#define _SPI0_CS_HIGH    (FPTD->PDOR |=  (1 << 0));    // Set PTD0 pin HIGH

#define _SPI1_CS_LOW     (FPTE->PDOR &= ~(1 << 4));    // Set PTE4 pin LOW
#define _SPI1_CS_HIGH    (FPTE->PDOR |=  (1 << 4));    // Set PTE4 pin HIGH  

#define _AMBLIGHT_EN_SHIFT     (1 << 5)               // Mask to turn-on Light Sensor

// MAX8856 Battery Charger Mode Settings
#define _PWR_EN1_SHIFT   (1 << 2)                     // Charge Mode Select pin EN1
#define _PWR_EN2_SHIFT   (1 << 3)                     // Charge Mode Select pin EN2

#define _CHRG_SNS_EN_SHIFT     (1 << 2)               // Mask to turn-on Battery measurement    

#define _BATT_MEASUREMENT_OFF  (FPTC->PDOR |=  _CHRG_SNS_EN_SHIFT);   // 1 = Turn-OFF Batt.measurement voltage-divider FET    

// uSD Power Enable
#define _uSD_PEN_DISABLE   (FPTE->PDOR &= ~(1 << 3));    // Set PTE3 pin LOW
#define _uSD_PEN_ENABLE    (FPTE->PDOR |=  (1 << 3));    // Set PTE3 pin HIGH  

#define _SPI0_CS_LOW     (FPTD->PDOR &= ~(1 << 0));    // Set PTD0 pin LOW
#define _SPI0_CS_HIGH    (FPTD->PDOR |=  (1 << 0));    // Set PTD0 pin HIGH  

// uSD SPI CS
#define _uSD_CS_LOW     (FPTE->PDOR &= ~(1 << 5));    // Set PTE5 pin LOW
#define _uSD_CS_HIGH    (FPTE->PDOR |=  (1 << 5));    // Set PTE5 pin HIGH  

#define _WL_MOD_ENABLE_SHIFT   (1 << 13)              // Mask to turn-on Wi-Fi module  

#define _WL_MOD_ENABLE_OFF     (FPTA->PDOR &= ~_WL_MOD_ENABLE_SHIFT);    
#define _WL_MOD_ENABLE_ON      (FPTA->PDOR |=  _WL_MOD_ENABLE_SHIFT);  

void board_init();

#ifdef  __cplusplus
}
#endif // __cplusplus

#endif // __BOARD_H__
