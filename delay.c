/*
 * File:   delay.c
 * Author: vitor
 *
 * Created on January 20, 2021, 2:01 AM
 */

#include "delay.h"
#include "mcc_generated_files/system.h"
#include <libpic30.h>

 void delay_ms(uint16_t delay){
     __delay_ms(delay);
 }
 
 void delay_us(uint16_t delay){
     __delay_us(delay);
 }
