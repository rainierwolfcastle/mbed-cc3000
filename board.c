#include "board.h"

void USR_GPIO_init() {
    // MAX8856 EN1,EN2: turn on clock to portA, portB, portC, portD, portE for GPIO
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK ;
    // MAX8856 EN1,EN2: set Pin Mux to GPIO mode for PTB2, PTB3
    PORTB->PCR[2]  = PORT_PCR_MUX(1);
    PORTB->PCR[3]  = PORT_PCR_MUX(1);
    PORTD->PCR[5]  = PORT_PCR_MUX(1);
    // MAX8856 EN1,EN2: set PortB LED outputs to 1, MAX8856 EN2 and EN1 set to 1,0 (for 500mA)
    FPTB->PDOR |= _PWR_EN2_SHIFT & ~_PWR_EN1_SHIFT;
    // MAX8856 EN1,EN2: set PortB pins direction to output
    FPTB->PDDR |= _PWR_EN1_SHIFT | _PWR_EN2_SHIFT;
    
    // Ensure PTA4 pin (NMI input) is configured for GPIO output function and not NMI ! 
    PORTA->PCR[4]  = PORT_PCR_MUX(1);    // GPIO is alt1 function for PTA4 pin
    PORTA->PCR[4] |= PORT_PCR_IRQC(0);   // Disable interrupts on PTA4 pin
    
    // Ambient Light Sensor: set PortD AMB_LIGHT_EN to 1
    FPTD->PDOR |= _AMBLIGHT_EN_SHIFT;
    // Ambient Light Sensor: set PortD AMB_LIGHT_EN direction to output
    FPTD->PDDR |= _AMBLIGHT_EN_SHIFT;
  
    // Batt.measurement voltage-divider FET: set Pin Mux to GPIO mode for PTC2
    PORTC->PCR[2]  = PORT_PCR_MUX(1);
    // Batt.measurement voltage-divider FET: set PortC CHRG_SNS_EN to 0 to turn-ON measurement
    _BATT_MEASUREMENT_OFF;  //  1 = Turn-OFF Batt.measurement voltage-divider FET  
    // Batt.measurement voltage-divider FET: set PortC CHRG_SNS_EN direction to output
    FPTC->PDDR |= _CHRG_SNS_EN_SHIFT;

    // Wi-Fi Module Enable: PTA13 port mode = GPIO
    PORTA->PCR[13] |= PORT_PCR_MUX(1);

    // Wi-Fi Module Enable: set PortA WL_MOD_ENABLE direction to output
    FPTA->PDDR |= _WL_MOD_ENABLE_SHIFT;      
    // Wi-Fi Module Enable: Turn-off WL_MOD_ENABLE initially 
    _WL_MOD_ENABLE_OFF;
    
    // Diagnostic pins and manually controlled SPI_CS pins
    PORTE->PCR[5]  = PORT_PCR_MUX(1);                        // PTE5 = GPIO uSD CS (user controlled)
    PORTE->PCR[3]  = PORT_PCR_MUX(1);                        // PTE3 = GPIO uSD Power Enable pin
    _uSD_PEN_DISABLE;
    PORTE->PCR[4]  = PORT_PCR_MUX(1);                        // PTE4 = GPIO, SPI1_CS (user controlled)
    PORTD->PCR[0]  = PORT_PCR_MUX(1);                        // PTD0 = GPIO, SPI0_CS (user controlled)
    PORTE->PCR[2]  = PORT_PCR_MUX(1) | 0x3;                  // PTD2 = GPIO + Pullup enabled, uSD Card Detect
    
    _SPI0_CS_HIGH; _SPI1_CS_HIGH;                              // Startup state of User controlled SPI pins  
    _uSD_CS_HIGH;

    FPTE->PDDR |= _PTE3_SHIFT | _PTE4_SHIFT | _PTE5_SHIFT;     // Set PTE4, PTE3, PTE2 to outputs    
    FPTD->PDDR |= _SPI0_CS_SHIFT;                            // Set PTD0 to output       
}
