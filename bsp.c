/*
 * File:   bsp.c
 * Author: vitor
 *
 * Created on January 20, 2021, 1:55 AM
 */

#include "bsp.h"
#include "mcc_generated_files/ext_int.h"
#include "as3993.h"
#include <xc.h>

 void EX_INT1_CallBack(void){
     as3993Isr();
 }
 
 void enableExtInterrupt(void){
     _INT1IE = 1;
 } 
 
 void disableExtInterrupt(void){
     _INT1IE = 0;
 } 
 
 void clearExtInterrupt(void){
     _INT1IF = 0;
 } 
 
 void setAS3993_enablePin(uint8_t val){
     _LATB2 = val;
 }
 
 uint8_t AS3993_isEnabled(void){
     return _LATB2;
 }
 
 void setAS3993_SPI_enablePin(uint8_t val){
     _LATF8 = val;
 }
