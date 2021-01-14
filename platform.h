
#ifndef PLATFORM_H_
#define PLATFORM_H_

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
//#include <p24Fxxxx.h>
#include "p24FJ256DA210.h"
#include "Compiler.h"
#include "global.h"
#include "as3993_config.h"
#include <stdint.h>


/*
******************************************************************************
* DEFINES
******************************************************************************
*/
/** define for stopMode parameter of writeReadAS3993() */
#define STOP_NONE               0
/** define for stopMode parameter of writeReadAS3993() */
#define STOP_SGL                1
/** define for stopMode parameter of writeReadAS3993() */
#define STOP_CONT               2

/** map as3993Isr to _INT1Interrupt */
#define as3993Isr EX_INT1_CallBack

/*! map timer2Isr to _T3Interrupt */
#define timer3Isr TMR3_CallBack
#define timer2Isr _T2Interrupt


/** Macro for enable external IRQ */
#define ENEXTIRQ()                _INT1IE = 1;

/** Macro for disable external IRQ */
#define DISEXTIRQ()               _INT1IE = 0

/** Macro for clearing external IRQ flag*/
#define CLREXTIRQ()               _INT1IF = 0

/** Macro for setting enable pin */
#define EN(x)                     ENABLE=(x)

/** Macro for setting DCDC on/off */
#define DCDC(x)                   DCDCPIN=(x)

/** Macro for setting PA on/off */
//#define EN_PA(x)                  EN_PAPIN(x)

/** Macro for setting NCS pin, serial enable line */
#define NCS(x)                    NCSPIN=(x)
/** Macro for activating AS3993 for SPI communication */
#define NCS_SELECT()              NCS(0)
/** Macro for deactivating AS3993 for SPI communication */
#define NCS_DESELECT()            NCS(1)

#ifdef TUNER
#define SEN_TUNER_CIN(x)          //SEN_CINPIN(x)
#define SEN_TUNER_CLEN(x)         //SEN_CLENPIN(x)
#define SEN_TUNER_COUT(x)         //SEN_COUTPIN(x)
#endif

/** Definition for the MCU Status LED */
//#define MCULED(x)                 MCULEDPIN(x)

/** Definition of the general status LEDs */
#define NUMLEDS                   BOARD_NUMLEDS

#define LEDON                     0
#define LEDOFF                    1

/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/
/*------------------------------------------------------------------------- */
/** This function initializes uController I/O ports for the configured board
  *
  * This function does not take or return a parameter.
  */
void platformInit(void);

/*------------------------------------------------------------------------- */
/** This function initializes SPI communication.
  *
  * This function does not take or return a parameter.
  */
void spiInit(void);

/*------------------------------------------------------------------------- */
/** This function talks with the AS3993 chip.
  */
void writeReadAS3993( const uint8_t* wbuf, uint8_t wlen, uint8_t* rbuf, uint8_t rlen, uint8_t stopMode, uint8_t doStart );

/** This function talks with the AS3993 chip from ISR.
  */
void writeReadAS3993Isr( const uint8_t *wbuf, uint8_t wlen, uint8_t* rbuf, uint8_t rlen );

/*------------------------------------------------------------------------- */
/** \attention Do not use, not implemented.
  *  This function sets the interface to the AS3993 for accessing it via
  * direct mode. 
  */
void setPortDirect();

/*------------------------------------------------------------------------- */
/** \attention Do not use, not implemented.
  *  This function sets the interface to the AS3993 for accessing it via
  * normal SPI mode using writeReadAS3993()
  */
void setPortNormal();

//marte
void sel_antena (int16_t);
void sel_cap_cin(int16_t);
void sel_cap_cout(int16_t);
void sel_cap_clen(int16_t);
void desliga_saida_pa (void);
void liga_saida_pa (void);
void ld_pa (int16_t);
void antena_ok (uint8_t);
void auto_sintonia (void);

/*
******************************************************************************
* INTERNAL DEFINES
* Not to be used directly, use the macros at the top of this file
******************************************************************************
*/

#if FERMI
/** Definition for the serial enable pin */
#define NCSPIN                    _LATF8 //_LATB15
/** Definition for the Direct data mode Pin*/

/** Definition for the enable pin */
#define ENABLE                    _LATB2 //_LATB13

//----------------------- novo -----------------------------------------
    #define SAIDA_RS(x)                 _LATD11=(x)     //SAIDA RS
    #define SAIDA_ELCD(x)               _LATD0=(x)      //SAIDA ELCD
    #define SAIDA_BD4(x)                _LATD6=(x)      //SAIDA BD4
    #define SAIDA_BD5(x)                _LATD7=(x)      //SAIDA BD5
    #define SAIDA_BD6(x)                _LATD13=(x)     //SAIDA BD6
    #define SAIDA_BD7(x)                _LATD12=(x)     //SAIDA BD7
    #define SAIDA_DIR(x)                _LATG13=(x)     //SAIDA DIR

    #define LED_A1(x)                   _LATE0=(x)      //SAIDA LED_A1
    #define LED_A2(x)                   _LATE1=(x)      //SAIDA LED A2
    #define LED_A3(x)                   _LATE2=(x)      //SAIDA LED A3
    #define LED_A4(x)                   _LATE3=(x)      //SAIDA LED_A4
    #define LED_A5(x)                   _LATE4=(x)      //SAIDA LED A5
    #define LED_A6(x)                   _LATE5=(x)      //SAIDA LED A6

    #define LED_A7(x)                   _LATA1=(x)      //SAIDA LED A7
    #define LED_A8(x)                   _LATA6=(x)      //SAIDA LED A8
    //Definicoes para placas anterioes a 7.15
    //#define LED_A7(x)                   _LATD3=(x)      //SAIDA LED A7
    //#define LED_A8(x)                   _LATD2=(x)      //SAIDA LED A8

    #define LED_TAG(x)                  _LATE8=(x)      //SAIDA LED TAG
    #define LIGA_PA(x)                  _LATE9=(x)      //SAIDA LIGA PA

    #define SEL_BBA(x)                  _LATB10=(x)     //SAIDA SEL_BBA
    #define SEL_A1_4(x)                 _LATB11=(x)     //SAIDA SEL_A1-4
    #define SEL_B5_8(x)                 _LATB12=(x)     //SAIDA SEL_B5-8
    #define SEL_A1_2(x)                 _LATB13=(x)     //SAIDA SEL_A1-2
    #define SEL_A3_4(x)                 _LATB15=(x)     //SAIDA SEL_A3-4
    #define SEL_B5_6(x)                 _LATF13=(x)     //SAIDA SEL_B5-6
    #define SEL_B7_8(x)                 _LATF12=(x)     //SAIDA SEL_B7-8
    #define TUNE_CAP3(x)                _LATF4=(x)      //SAIDA TUNE_CAP3

    #define SAI_1(x)                    _LATG15=(x)     //SAIDA SAI_1
    #define SAI_2(x)                    _LATB3=(x)      //SAIDA SAI_2
    #define SAI_3(x)                    _LATC4=(x)      //SAIDA SAI_3
    #define SAI_4(x)                    _LATC3=(x)      //SAIDA SAI_4
    #define SAI_5(x)                    _LATC2=(x)      //SAIDA SAI_5
    #define SAI_6(x)                    _LATC1=(x)      //SAIDA SAI_6
    #define SAI_7(x)                    _LATB6=(x)      //SAIDA SAI_6
    #define TUNE_CAP1(x)                _LATD15=(x)     //SAIDA TUNE_CAP1
    #define TUNE_CAP2(x)                _LATD14=(x)     //SAIDA TUNE_CAP2

    #define GP_0(x)                     _LATB4=(x)      //SAIDA GP_0
    #define GP_1(x)                     _LATC13=(x)      //SAIDA GP_1
    #define GP_2(x)                     _LATF2=(x)      //SAIDA GP_2
    #define GP_3(x)                     _LATA4=(x)      //SAIDA GP_3
    #define GP_4(x)                     _LATA5=(x)      //SAIDA GP_4
    #define GP_5(x)                     _LATA2=(x)      //SAIDA GP_5
    #define GP_6(x)                     _LATG12=(x)      //SAIDA GP_6
    #define GP_7(x)                     _LATA9=(x)      //SAIDA GP_7
    #define GP_8(x)                     _LATA0=(x)      //SAIDA GP_8
    #define GP_9(x)                     _LATC14=(x)      //SAIDA GP_9
    #define GP_10(x)                     _LATF0=(x)      //SAIDA GP_10
    #define GP_11(x)                     _LATF1=(x)      //SAIDA GP_11
    #define GP_12(x)                     _LATA1=(x)      //SAIDA GP_12
    #define GP_13(x)                     _LATD10=(x)      //SAIDA GP_13
    #define GP_14(x)                     _LATA6=(x)      //SAIDA GP_14
    #define GP_15(x)                     _LATA7=(x)      //SAIDA GP_15
    #define GP_16(x)                     _LATB5=(x)      //SAIDA GP_16

    #define LED_ZIG(x)                   _LATA10=(x)     //SAIDA LED_ZIG
    #define LED_WIFI                     _LATA10
    #define LED_WF(x)                    _LATG1=(x)//_LATG14=(x)     //SAIDA RIO4
    #define LED_3G(x)                    _LATG14=(x)//_LATG0=(x)      //SAIDA LED0
    #define LED_GPS(x)                   _LATG0=(x)//_LATG1=(x)      //SAIDA LED1

//----------------------------------------------------------------------

#define PA_ON                     HIGH
#define PA_OFF                    LOW

#endif

#ifdef FEMTO2
/** Definition for the serial enable pin */
#define NCSPIN                    _LATB15

/** Definition for the enable pin */
#define ENABLE                    _LATB8

#define MCULEDPIN(x)              _LATB4=(x)

#define BOARD_NUMLEDS             2
#define LED1PIN(x)                _LATA4=(x)
#define LED2PIN(x)                _LATA2=(x)
#define LED3PIN(x)
#define LED4PIN(x)

/** Antenna switch is used to toggle antenna port 1 and 2 */
#define SWITCH_ANTENNA(x)       _LATB2 = ((x)==2?SWITCH_ANT_P2:SWITCH_ANT_P1)
#define SWITCH_ANT_P1           0x0
#define SWITCH_ANT_P2           0x1

#define SEN_CINPIN(x)           _LATB5=(x)
#define SEN_CLENPIN(x)
#define SEN_COUTPIN(x)          _LATB9=(x)

#define PA_ON                     HIGH
#define PA_OFF                    LOW

#elif FEMTO2_1
/** Definition for the serial enable pin */
#define NCSPIN                    _LATB15

/** Definition for the enable pin */
#define ENABLE                    _LATB8

#define MCULEDPIN(x)              _LATB1=(x) //changed from V2.0 to V2.1

#define BOARD_NUMLEDS             2
#define LED1PIN(x)                _LATB0=(x) //changed from V2.0 to V2.1
#define LED2PIN(x)                _LATA2=(x)
#define LED3PIN(x)
#define LED4PIN(x)

/** Antenna switch is used to toggle antenna port 1 and 2 */
#define SWITCH_ANTENNA(x)       _LATB2 = ((x)==2?SWITCH_ANT_P2:SWITCH_ANT_P1)
#define SWITCH_ANT_P1           0x0
#define SWITCH_ANT_P2           0x1

#define SEN_CINPIN(x)           _LATB5=(x)
#define SEN_CLENPIN(x)
#define SEN_COUTPIN(x)          _LATB9=(x)

#define PA_ON                     HIGH
#define PA_OFF                    LOW

#elif MEGA
/** Definition for the serial enable pin */
#define NCSPIN                    _LATD9
/** Definition for the Direct data mode Pin*/

/** Definition for the enable pin */
#define ENABLE                    _LATA15
/** Definition for the PA enable Pin, this is a dummy pin as the
 * fermi reader pa is always enabled  */
#define EN_PAPIN(x)                  _LATA0=(x)

#define MCULEDPIN(x)              _LATG6=(x)

#define BOARD_NUMLEDS             3
#define LED1PIN(x)                _LATB8=(x)
#define LED2PIN(x)                _LATD15=(x)
#define LED3PIN(x)                _LATF5=(x)
#define LED4PIN(x)

#define PA_ON                     HIGH
#define PA_OFF                    LOW

#elif RADON
/** Definition for the serial enable pin */
#define NCSPIN                    _LATC5
/** Definition for the Direct data mode Pin*/

/** Definition for the enable pin */
#define ENABLE                    _LATC3
/** Definition for the PA enable Pin, this is a dummy pin as the
 * Radon reader pa is always enabled, use LED5  */
#define EN_PAPIN                  LED2PIN

#define MCULEDPIN(x)              LED1PIN(x)
#define ANT1LEDPIN(x)             LED6PIN(x)
#define ANT2LEDPIN(x)             LED5PIN(x)

#define BOARD_NUMLEDS             7
/* A1 is LED5(schematic) aka D13 */
#define LED1PIN(x)                _LATA1=(x)
/* A8 is LED4(schematic) aka D8 */
#define LED2PIN(x)                _LATA8=(x)
/* B4 is LED3(schematic) aka D9 */
#define LED3PIN(x)                _LATB4=(x)
/* A4 is LED2(schematic) aka D10 */
#define LED4PIN(x)                _LATA4=(x)
/* A9 is LED1(schematic) aka D7 */
#define LED5PIN(x)                _LATA9=(x)
/* C6 is LED6(schematic) aka D11 */
#define LED6PIN(x)                _LATC6=(x)

/** Antenna switch is used to toggle antenna port 1 and 2 */
#define SWITCH_ANTENNA(x)       (_LATC7 = ((x)==2?SWITCH_ANT_P2:SWITCH_ANT_P1), ANT2LEDPIN((x)==2), ANT1LEDPIN((x)!=2))
#define SWITCH_ANT_P1           0x0
#define SWITCH_ANT_P2           0x1

#define SEN_CINPIN(x)           _LATC0=(x)
#define SEN_CLENPIN(x)          _LATC1=(x)
#define SEN_COUTPIN(x)          _LATC2=(x)

#define SEN_ANT1_CINPIN(x)      _LATB2=(x)
#define SEN_ANT1_COUTPIN(x)     _LATB3=(x)


#define PA_ON                     LOW
#define PA_OFF                    HIGH
#endif


#endif /* PLATFORM_H_ */
