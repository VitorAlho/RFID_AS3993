/*
 *****************************************************************************
 * Copyright by ams AG                                                       *
 * All rights are reserved.                                                  *
 *                                                                           *
 * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
 * THE SOFTWARE.                                                             *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
 *****************************************************************************
 */
/*! \file
 *
 *  \author U.Herrmann ( based on work by E.Grubmueller
 *  \author T. Luecker (Substitute)
 *  \author Bernhard Breinbauer
 *
 *   \brief Declaration of low level functions provided by the as3993 series chips
 *
 *   Declares low level functions provided by the as3993 series chips. All 
 *   higher level functions are provided by as3993_public.h and protocol
 *   work is contained in gen2.h and iso6b.h
 */

#ifndef _AS3993_H_
#define _AS3993_H_


//#include "ams_types.h"
#include <stdint.h>

///////////////////////////////////////////////////////
//                      ERRORS
///////////////////////////////////////////////////////

/*!
 * Error codes to be used within the application.
 * They are represented by an int8_t
 */
#define ERR_NONE     0 /*!< no error occured */
#define ERR_NOMEM   -1 /*!< not enough memory to perform the requested operation */
#define ERR_BUSY    -2 /*!< device or resource busy */
#define ERR_IO      -3 /*!< generic IO error */
#define ERR_TIMEOUT -4 /*!< error due to timeout */
#define ERR_REQUEST -5 /*!< invalid request or requested function can't be executed at the moment */
#define ERR_NOMSG   -6 /*!< No message of desired type */
#define ERR_PARAM   -7 /*!< Parameter error */

#define ERR_LAST_ERROR -32

#define ERR_FIRST_AS3993_ERROR (ERR_LAST_ERROR-1)

/* Errors primarily raised by AS3993 itself, however codes like ERR_CHIP_CRCERROR can be reused by ISO6B */
#define ERR_CHIP_NORESP               (ERR_FIRST_AS3993_ERROR - 0)
#define ERR_CHIP_HEADER               (ERR_FIRST_AS3993_ERROR - 1)
#define ERR_CHIP_PREAMBLE             (ERR_FIRST_AS3993_ERROR - 2)
#define ERR_CHIP_RXCOUNT              (ERR_FIRST_AS3993_ERROR - 3)
#define ERR_CHIP_CRCERROR             (ERR_FIRST_AS3993_ERROR - 4)
#define ERR_CHIP_FIFO                 (ERR_FIRST_AS3993_ERROR - 5)

#define ERR_REFLECTED_POWER           (ERR_FIRST_AS3993_ERROR - 16)

/* Upper level protocol errors, e.g. a Write can fail when access command has failed... */
#define ERR_GEN2_SELECT               (ERR_FIRST_AS3993_ERROR - 32)
#define ERR_GEN2_ACCESS               (ERR_FIRST_AS3993_ERROR - 33)
#define ERR_GEN2_REQRN                (ERR_FIRST_AS3993_ERROR - 34)
#define ERR_GEN2_CHANNEL_TIMEOUT      (ERR_FIRST_AS3993_ERROR - 35)


/* ISO6b errors */
#define ERR_ISO6B_NACK                (ERR_FIRST_AS3993_ERROR - 64)

///////////////////////////////////////////////////////
//                     END ERRORS
///////////////////////////////////////////////////////

#define AS3993_REG_STATUSCTRL           0X00
#define AS3993_REG_PROTOCOLCTRL         0X01
#define AS3993_REG_TXOPTIONS            0X02
#define AS3993_REG_RXOPTIONS            0X03
#define AS3993_REG_TRCALHIGH            0X04
#define AS3993_REG_TRCALLOW             0X05
#define AS3993_REG_AUTOACKTIMER         0X06
#define AS3993_REG_RXNORESPONSEWAITTIME 0X07
#define AS3993_REG_RXWAITTIME           0X08
#define AS3993_REG_RXFILTER             0X09
#define AS3993_REG_RXMIXERGAIN          0X0A
#define AS3993_REG_REGULATORCONTROL     0X0B
#define AS3993_REG_RFOUTPUTCONTROL      0X0C
#define AS3993_REG_MISC1                0X0D
#define AS3993_REG_MISC2                0X0E
#define AS3993_REG_MEASUREMENTCONTROL   0X10
#define AS3993_REG_VCOCONTROL           0X11
#define AS3993_REG_CPCONTROL            0X12
#define AS3993_REG_MODULATORCONTROL1    0X13
#define AS3993_REG_MODULATORCONTROL2    0X14
#define AS3993_REG_MODULATORCONTROL3    0X15
#define AS3993_REG_MODULATORCONTROL4    0X16
#define AS3993_REG_PLLMAIN1             0X17
#define AS3993_REG_PLLMAIN2             0X18
#define AS3993_REG_PLLMAIN3             0X19
#define AS3993_REG_PLLAUX1              0X1A
#define AS3993_REG_PLLAUX2              0X1B
#define AS3993_REG_PLLAUX3              0X1C
#define AS3993_REG_ICD                  0X1D
#define AS3993_REG_MIXOPTS              0X22
#define AS3993_REG_TEST1                0X23
#define AS3993_REG_TEST2                0X24
#define AS3993_REG_TEST3                0X25
#define AS3993_REG_TEST4                0X26
#define AS3993_REG_TXRESHAPE            0X27 /* ? */
#define AS3993_REG_STATUSPAGE           0X29
#define AS3993_REG_AGCANDSTATUS         0X2A
#define AS3993_REG_RSSILEVELS           0X2B
#define AS3993_REG_AGL                  0X2C
#define AS3993_REG_ADC                  0X2D
#define AS3993_REG_COMMANDSTATUS        0X2E
#define AS3993_REG_DEVICEVERSION        0X33
#define AS3993_REG_IRQMASK1             0X35
#define AS3993_REG_IRQMASK2             0X36
#define AS3993_REG_IRQSTATUS1           0X37
#define AS3993_REG_IRQSTATUS2           0X38
#define AS3993_REG_FIFOSTATUS           0X39
#define AS3993_REG_RXLENGTHUP           0X3A
#define AS3993_REG_RXLENGTHLOW          0X3B
#define AS3993_REG_TXSETTING            0X3C
#define AS3993_REG_TXLENGTHUP           0X3D
#define AS3993_REG_TXLENGTHLOW          0X3E
#define AS3993_REG_FIFO                 0X3F

//Reader commands-------------------------------------------
#define AS3993_CMD_IDLE                  0x80
#define AS3993_CMD_DIRECT_MODE           0x81
#define AS3993_CMD_SOFT_INIT             0x83
#define AS3993_CMD_HOP_TO_MAIN_FREQUENCY 0x84
#define AS3993_CMD_HOP_TO_AUX_FREQUENCY  0x85
#define AS3993_CMD_TRIGGERADCCON         0x87
#define AS3993_CMD_TRIG_RX_FILTER_CAL    0x88
#define AS3993_CMD_INC_RX_FILTER_CAL     0x89
#define AS3993_CMD_DEC_RX_FILTER_CAL     0x8A
#define AS3993_CMD_TRANSMCRC             0x90
#define AS3993_CMD_TRANSMCRCEHEAD        0x91
#define AS3993_CMD_TRANSMNOCRC           0x92
#define AS3993_CMD_DELAY_TRANSMIT_CRC    0x93 /* ? */
#define AS3993_CMD_DELAY_TRANSMIT_NO_CRC 0x94 /* ? */
#define AS3993_CMD_CLOSE_SLOT_SEQUENCE   0x95 /* ? */
#define AS3993_CMD_BLOCKRX               0x96
#define AS3993_CMD_ENABLERX              0x97
#define AS3993_CMD_QUERY                 0x98
#define AS3993_CMD_QUERYREP              0x99
#define AS3993_CMD_QUERYADJUSTUP         0x9A
#define AS3993_CMD_QUERYADJUSTNIC        0x9B
#define AS3993_CMD_QUERYADJUSTDOWN       0x9C
#define AS3993_CMD_ACK                   0x9D
#define AS3993_CMD_NAK                   0x9E
#define AS3993_CMD_REQRN                 0x9F
#define AS3993_CMD_SUPPLY_AUTO_LEVEL     0xA2
#define AS3993_CMD_SUPPLY_MANUAL_LEVEL   0xA3
#define AS3993_CMD_VCO_AUTO_RANGE        0xA4
#define AS3993_CMD_VCO_MANUAL_RANGE      0xA5
#define AS3993_CMD_AGL_ON                0xA6
#define AS3993_CMD_AGL_OFF               0xA7
#define AS3993_CMD_STORE_RSSI            0xA8
#define AS3993_CMD_CLEAR_RSSI            0xA9
#define AS3993_CMD_ANTI_COLL_ON          0xAA
#define AS3993_CMD_ANTI_COLL_OFF         0xAB

/*IRQ Mask register */
/** AS3993 interrupt: no response bit */
#define AS3993_IRQ1_NORESP               0x01
/** AS3993 interrupt: Auto ACK finished */
#define AS3993_IRQ1_AUTOACK              0x02
/** AS3993 interrupt: header bit / 2bytes */
#define AS3993_IRQ1_HEADER               0x08
/** AS3993 IRQ status register: receive error */
#define AS3993_IRQ1_RXERR                0x10
/** AS3993 interrupt: fifo bit */
#define AS3993_IRQ1_FIFO                 0x20
/** AS3993 IRQ status register: receive complete */
#define AS3993_IRQ1_RX                   0x40
/** AS3993 IRQ status register: transmit complete */
#define AS3993_IRQ1_TX                   0x80

/** AS3993 interrupt: error 3 bit: preamble/fifo overflow */
#define AS3993_IRQ2_PREAMBLE             0x01
/** AS3993 interrupt: error 2 bit: rxcount */
#define AS3993_IRQ2_RXCOUNT              0x02
/** AS3993 interrupt: error1 bit: CRC/autohop */
#define AS3993_IRQ2_CRCERROR             0x04
/** AS3993 interrupt: cmd bit: end of command */
#define AS3993_IRQ2_END_CMD              0x40
/** AS3993 interrupt: ana bit: change of OSC, PLL, RF field */
#define AS3993_IRQ2_END_ANA              0x80

#define AS3993_IRQ1_MASK_ALL             0xfb
#define AS3993_IRQ2_MASK_ALL             0xc7

/*AS3993_REG_FIFO STATUS register */
/** AS3993 AS3993_REG_FIFO status register: FIFO overflow */
#define AS3993_FIFOSTAT_OVERFLOW              0x20

#define RESP_NORESINTERRUPT AS3993_IRQ1_NORESP   
#define RESP_AUTOACK        AS3993_IRQ1_AUTOACK 
#define RESP_HEADERBIT      AS3993_IRQ1_HEADER   
#define RESP_RXERR          AS3993_IRQ1_RXERR  
#define RESP_FIFO           AS3993_IRQ1_FIFO  
#define RESP_RXIRQ          AS3993_IRQ1_RX   
#define RESP_TXIRQ          AS3993_IRQ1_TX  
#define RESP_PREAMBLEERROR (AS3993_IRQ2_PREAMBLE << 8)
#define RESP_RXCOUNTERROR  (AS3993_IRQ2_RXCOUNT  << 8)
#define RESP_CRCERROR      (AS3993_IRQ2_CRCERROR << 8)
#define RESP_END_CMD       (AS3993_IRQ2_END_CMD  << 8)
#define RESP_END_ANA       (AS3993_IRQ2_END_ANA  << 8)

#if RUN_ON_AS3980 || RUN_ON_AS3981   //as3980/81 produces irq_err2 (without irq_err) if new epc is read 500ms after last one.
#define RESP_RXDONE_OR_ERROR  (RESP_RXIRQ | RESP_AUTOACK | RESP_RXERR | RESP_NORESINTERRUPT | RESP_RXCOUNTERROR)
#else
#define RESP_RXDONE_OR_ERROR  (RESP_RXIRQ | RESP_AUTOACK | RESP_RXERR | RESP_NORESINTERRUPT)
#endif
/* RESP_FIFOOVERFLOW does not work reliably */
#define RESP_ANY    (AS3993_IRQ1_MASK_ALL | (AS3993_IRQ2_MASK_ALL << 8))

#define RSSI_MODE_REALTIME                  0x00
#define RSSI_MODE_PILOT                     0x04
#define RSSI_MODE_2NDBYTE                   0x06
#define RSSI_MODE_PEAK                      0x08

#define AS3993_NOM_SENSITIVITY              87


/* at 40kHz BLF one gen2 slot takes ~40ms, we are going to wait
 * 50ms (in as3993WaitForResponse()) to be on the safe side. */
/** delay in us which will be used as3993WaitForResponse() loop */
#define WAITFORRESPONSEDELAY    50
/** max delay in us which we will wait for an response from AS3993 */
#define WAITFORRESPONSETIMEOUT  50000
/** number of loop iterations in as3993WaitForResponse() */
#define WAITFORRESPONSECOUNT    WAITFORRESPONSETIMEOUT/WAITFORRESPONSEDELAY


extern volatile uint16_t as3993Response;
extern uint32_t as3993CurrentBaseFreq;

/*------------------------------------------------------------------------- */
/** Sends only one command to the AS3993. \n
  * @param command The command which is send to the AS3993.
  */
void as3993SingleCommand(uint8_t command);

/*------------------------------------------------------------------------- */
/** Reads data from a address and some following addresses from the
  * AS3993. The len parameter defines the number of address read. \n
  * @param address addressbyte
  * @param len Length of the buffer.
  * @param *readbuf Pointer to the first byte of the array where the data has to be stored in.
  */
void as3993ContinuousRead(uint8_t address, int8_t len, uint8_t *readbuf);

/** Reads data from the fifo and some following addresses from the
  * AS3993. The len parameter defines the number of address read.
  * @param len Length of the buffer.
  * @param *readbuf Pointer to the first byte of the array where the data has to be stored in.
  */
void as3993FifoRead(int8_t len, uint8_t *readbuf);

/** Single Read -> reads one byte from one address of the AS3993.  \n
  * @param address The addressbyte
  * @return The databyte read from the AS3993
  */
uint8_t as3993SingleRead(uint8_t address);

/*------------------------------------------------------------------------- */
/** Continuous Write -> writes several bytes to subsequent addresses of the AS3993.  \n
  * @param address addressbyte
  * @param *buf Pointer to the first byte of the array.
  * @param len Length of the buffer.
  */
void as3993ContinuousWrite(uint8_t address, uint8_t *buf, int8_t len);

/** Single Write -> writes one byte to one address of the AS3993.  \n
  * @param address The addressbyte
  * @param value The databyte
  */
void as3993SingleWrite(uint8_t address, uint8_t value);

/*------------------------------------------------------------------------- */
/** Sends first some commands to the AS3993. The number of
  * commands is specified with the parameter com_len. Then it sets
  * the address where the first byte has to be written and after that
  * every byte from the buffer is written to the AS3993. \n
  * @param *command Pointer to the first byte of the command buffer
  * @param com_len Length of the command buffer.
  * @param address addressbyte
  * @param *buf Pointer to the first byte of the data array.
  * @param buf_len Length of the buffer.

  */
void as3993CommandContinuousAddress(uint8_t *command, uint8_t com_len,
                             uint8_t address, uint8_t *buf, uint8_t buf_len);

/*------------------------------------------------------------------------- */
/** This function waits for the specified response(IRQ).
  */
void as3993WaitForResponse(uint16_t waitMask);

/*------------------------------------------------------------------------- */
/** This function waits for the specified response(IRQ).
  */
void as3993WaitForResponseTimed(uint16_t waitMask, uint16_t ms);

#if 0
/*------------------------------------------------------------------------- */
/** This function gets the current response
  */
uint16_t as3993GetResponse(void);

/*------------------------------------------------------------------------- */
/** This function clears the response bits according to mask
  */
uint16_t as3993ClrResponseMask(uint16_t mask);

/*------------------------------------------------------------------------- */
/** This function clears all responses
  */
uint16_t as3993ClrResponse(void);
#else
#define as3993GetResponse() as3993Response
#define as3993ClrResponseMask(mask) as3993Response&=~(mask)
#define as3993ClrResponse() as3993Response=0
#endif

/*!
 *****************************************************************************
 *  \brief  Enter the direct mode
 *
 *  The direct mode is needed since the AS3993 doesn't support ISO18000-6B
 *  directly. Direct mode means that the MCU directly issues the commands.
 *  The AS3993 modulates the serial stream from the MCU and sends it out.
 *
 *****************************************************************************
 */
void as3993EnterDirectMode();

/*!
 *****************************************************************************
 *  \brief  Leave the direct mode
 *
 *  After calling this function the AS3993 is in normal mode again.
 *
 *****************************************************************************
 */
void as3993ExitDirectMode();

/*!
 *****************************************************************************
 *  \brief  Enter the power down mode by setting EN pin to low, saving
 *  registers beforehand
 *
 *****************************************************************************
 */
void as3993EnterPowerDownMode();

/*!
 *****************************************************************************
 *  \brief  Exit the power down mode by setting EN pin to high, restoring
 *  registers afterwards.
 *
 *****************************************************************************
 */
void as3993ExitPowerDownMode();

/*!
 *****************************************************************************
 *  \brief  Enter the normal power mode: EN is high, stby and rf_on bits are low
 *
 *****************************************************************************
 */
void as3993EnterPowerNormalMode();

/*!
 *****************************************************************************
 *  \brief  Exit the normal power mode.
 *
 *****************************************************************************
 */
void as3993ExitPowerNormalMode();

/*!
 *****************************************************************************
 *  \brief  Enter the normal power mode with rf on.
 *  EN is high, stby bit is low and rf_on bit is high.
 *
 *****************************************************************************
 */
void as3993EnterPowerNormalRfMode();

/*!
 *****************************************************************************
 *  \brief  Exit the normal power mode with rf on.
 *
 *****************************************************************************
 */
void as3993ExitPowerNormalRfMode();

/*!
 *****************************************************************************
 *  \brief  Enter the standby power down mode:
 *  EN is high, stby is high and rf_on bit is low.
 *
 *****************************************************************************
 */
void as3993EnterPowerStandbyMode();

/*!
 *****************************************************************************
 *  \brief  Exit the standby power down mode.
 *
 *****************************************************************************
 */
void as3993ExitPowerStandbyMode();

extern uint8_t as3993ChipVersion;

/*!
 *****************************************************************************
 *  \brief  after EN goes high the IRQ register has to be read after osc_ok
 * goes high.
 *****************************************************************************
 */
void as3993WaitForStartup(void);

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
#define timer3Isr //TMR3_CallBack

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

/** Macro for setting NCS pin, serial enable line */
#define NCS(x)                    NCSPIN=(x)
/** Macro for activating AS3993 for SPI communication */
#define NCS_SELECT()              NCS(0)
/** Macro for deactivating AS3993 for SPI communication */
#define NCS_DESELECT()            NCS(1)

/** Definition for the serial enable pin */
#define NCSPIN                    _LATF8 //_LATB15
/** Definition for the Direct data mode Pin*/

/** Definition for the enable pin */
#define ENABLE                    _LATB2 //_LATB13

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
#endif /* _AS3993_H_ */
