/**
  Generated main.c file from MPLAB Code Configurator

  @Company
    Microchip Technology Inc.

  @File Name
    main.c

  @Summary
    This is the generated main.c using PIC24 / dsPIC33 / PIC32MM MCUs.

  @Description
    This source file provides main entry point for system initialization and application code development.
    Generation Information :
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.170.0
        Device            :  PIC24FJ256DA210
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.61
        MPLAB 	          :  MPLAB X v5.45
*/

/*
    (c) 2020 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

/*
  Section: Defines
*/

/*
  Section: Included Files
*/
#include <stdio.h>
#include <string.h>
#include <xc.h>
#include "platform.h"
#include "as3993_public.h"
#include "appl_commands.h"
#include "spi1.h"
#include "mcc_generated_files/system.h"
#include "mcc_generated_files/pin_manager.h"
#include <stdint.h>

#ifndef FCY
#define FCY SYSCLK/2
#endif
#include <libpic30.h>
        
/*
  Section: Global variables 
*/

uint8_t readerInitStatus;

extern Freq Frequencies;

uint8_t num_of_tags;

/*
                         Main application
 */
int main(void)
{
    // initialize the device
    SYSTEM_Initialize();
    
    // Seleciona a antena 1
    SEL_BBA_SetHigh();
    SEL_A1_4_SetHigh();
    SEL_A1_2_SetHigh();
    SEL_A3_4_SetLow();
    
    Frequencies.freq[0] = 915000;
    Frequencies.numFreqs = 1;
        readerInitStatus = as3993Initialize(Frequencies.freq[0]);
        
    while(1){     
        LIGA_PA_SetHigh();
        num_of_tags = inventoryGen2();
        if(num_of_tags>=1){
            int i=0;
            LED_TAG_SetHigh();
            i=1;
        }
        LIGA_PA_SetLow();
        __delay_ms(10);
        LED_TAG_SetLow();
    } 
    
    return 0;
}
/**
 End of File
*/

