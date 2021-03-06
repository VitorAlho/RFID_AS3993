/*
  Section: Included Files
*/
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include <xc.h>

#include "mcc_generated_files/system.h"
#include "mcc_generated_files/pin_manager.h"
#include "mcc_generated_files/spi1.h"

#include "bsp.h"
#include "as3993.h"
#include "gen2.h"
#include "delay.h"

/*
  Section: Global variables 
*/

uint8_t readerInitStatus;

uint8_t num_of_tags;

/*
                         Main application
 */
int main(void)
{
    // initialize the device
    SYSTEM_Initialize();
    
    RFID_AS3993_load_callbacks(SPI1_Exchange8bitBuffer,
                               delay_ms,
                               delay_us,
                               enableExtInterrupt,
                               disableExtInterrupt,
                               clearExtInterrupt,
                               setAS3993_enablePin,
                               AS3993_isEnabled,
                               setAS3993_SPI_enablePin);
    
    // Seleciona a antena 1
    SEL_BBA_SetHigh();
    SEL_A1_4_SetHigh();
    SEL_A1_2_SetHigh();
    SEL_A3_4_SetLow();
    
    readerInitStatus = as3993Initialize(915000ULL);
        
    while(1){     
        LIGA_PA_SetHigh();
        num_of_tags = inventoryGen2();
        if(num_of_tags>=1){
            int i=0;
            LED_TAG_SetHigh();
            i=1;
        }
        LIGA_PA_SetLow();
        delay_ms(10);
        LED_TAG_SetLow();
    } 
    
    return 0;
}
/**
 End of File
*/

