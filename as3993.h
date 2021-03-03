
#ifndef _AS3993_H_
#define _AS3993_H_

#include <stdint.h>

///** Definition of the maximum frequencies for the hopping */
#define MAXFREQ                 53
///** Definition of the maximum tune entries in tuning table */
#define MAXTUNE                 52

/** Definition of the maximum number of tags, which can be read in 1 round */
#define MAXTAG 12//20//120

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

#define AS3993_ERR_NORES      ERR_CHIP_NORESP
#define AS3993_ERR_PREAMBLE   ERR_CHIP_PREAMBLE
#define AS3993_ERR_HEADER     ERR_CHIP_HEADER
#define AS3993_ERR_FIFO_OVER  ERR_CHIP_FIFO
#define AS3993_ERR_RXCOUNT    ERR_CHIP_RXCOUNT
#define AS3993_ERR_CRC        ERR_CHIP_CRCERROR
#define AS3993_ERR_RXERR      ERR_CHIP_RXERR

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
#define AS3993_CMD_DEC_RX_FILTER_CAL     0x89
#define AS3993_CMD_INC_RX_FILTER_CAL     0x8A
#define AS3993_CMD_TRANSMCRC             0x90
#define AS3993_CMD_TRANSMCRCEHEAD        0x91
#define AS3993_CMD_TRANSMNOCRC           0x92
//#define AS3993_CMD_DELAY_TRANSMIT_CRC    0x93 /* ? */
//#define AS3993_CMD_DELAY_TRANSMIT_NO_CRC 0x94 /* ? */
//#define AS3993_CMD_CLOSE_SLOT_SEQUENCE   0x95 /* ? */
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

#define RESP_RXDONE_OR_ERROR  (RESP_RXIRQ | RESP_AUTOACK | RESP_RXERR | RESP_NORESINTERRUPT)

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

/** value for #readerPowerDownMode. Activates power down mode. (EN low)*/
#define POWER_DOWN              0
/** value for #readerPowerDownMode. Activates normal mode. (EN high, RF off, stdby 0)*/
#define POWER_NORMAL            1
/** value for #readerPowerDownMode. Activates normal mode with rf field on. (EN high, RF off, stdby 0)*/
#define POWER_NORMAL_RF         2
/** value for #readerPowerDownMode. Activates standby mode. (EN high, RF off, stdby 1)*/
#define POWER_STANDBY           3

/** define for stopMode parameter of writeReadAS3993() */
#define STOP_NONE               0
/** define for stopMode parameter of writeReadAS3993() */
#define STOP_SGL                1
/** define for stopMode parameter of writeReadAS3993() */
#define STOP_CONT               2

/** Macro for enable external IRQ */
//#define ENEXTIRQ()                _INT1IE = 1;

/** Macro for disable external IRQ */
//#define DISEXTIRQ()               _INT1IE = 0

/** Macro for clearing external IRQ flag*/
//#define CLREXTIRQ()               _INT1IF = 0

/** Definition for the serial enable pin */
//#define NCSPIN                     //_LATB15
/** Definition for the Direct data mode Pin*/

/** Definition for the enable pin */
//#define ENABLE                    _LATB2 //_LATB13

/** Macro for setting enable pin */
//#define EN(x)                     ENABLE=(x)

/** Macro for activating AS3993 for SPI communication */
//#define NCS_SELECT()               _LATF8 = 0
/** Macro for deactivating AS3993 for SPI communication */
//#define NCS_DESELECT()             _LATF8 = 1

extern volatile uint16_t as3993Response;

extern uint32_t as3993CurrentBaseFreq;

/** If rssi measurement is above this threshold the channel is regarded as
    used and the system will hop to the next frequency. Otherwise this frequency is used */
extern int8_t rssiThreshold;

/** Will be set to 1 if a cyclic inventory is performed, see callStartStop(). */
extern uint8_t cyclicInventory;

/** If set to 0 inventory round commandos will be triggered by the FW, otherwise
 * the autoACK feature of the reader will be used which sends the required Gen2
 * commands automatically.*/
extern uint8_t autoAckMode;

/** If set to 0 normal inventory round will be performed, if set to 1 fast inventory rounds will be performed.
 * The value is set in callStartStop() and callInventoryGen2().
 * For details on normal/fast inventory rounds see parameter singulate of gen2SearchForTags().*/
extern uint8_t fastInventory;

/** Value for register #AS3993_REG_STATUSPAGE. This defines what RSSI value is sent
 * to the host along with the tag data. The value is set in callStartStop() and callInventoryGen2(). */
extern uint8_t rssiMode;

///** To be communicated to GUI, basically result of hopFrequencies(), having information on skipped rounds */
extern int8_t inventoryResult;

/** Currently used protocol, valid values are: #SESSION_GEN2 and #SESSION_ISO6B. */
extern uint8_t currentSession;      // start with invalid session (neither Gen2 nor ISO 6b)

/** @struct Frequencies_
  * This struct stores the list of frequencies which are used for hopping.
  * For tuning enabled boards the struct also stores the tuning settings for
  * each frequency.
  *
  */
typedef struct
{
    /** Number of frequencies in freq. */
    uint8_t numFreqs;

    /** List of frequencies which are used for hopping. */
    uint32_t freq[MAXFREQ];

    /** Counts how often this freq has been used in hopping. Only available on tuner enabled boards. */
    uint16_t countFreqHop[MAXFREQ];

} Freq;

typedef struct
{
    /** number of entries in the table */
    uint8_t tableSize;
    /** currently active entry in the table. */
    uint8_t currentEntry;
    /** frequency which is assigned to this tune parameters. */
    unsigned long freq[MAXTUNE];
    /** tune enable for antenna. Valid values: 1 = antenna 1, 2 = antenna 2, 3 = antenna 1+2 */
    uint8_t      tuneEnable[MAXTUNE];
    /** Cin tuning parameter for antenna 1 + 2. */
    uint8_t           cin[2][MAXTUNE];
    /** Clen tuning parameter for antenna 1 + 2. */
    uint8_t          clen[2][MAXTUNE];
    /** Cout tuning parameter for antenna 1 + 2. */
    uint8_t          cout[2][MAXTUNE];
    /** Reflected power which was measured after last tuning. */
    uint16_t      tunedIQ[2][MAXTUNE];
} TuningTable;

/** This function initialises the AS3993. A return value greater 0 indicates an error.\n
 *
 * @param baseFreq the base frequency to set at initialization time, may be 
 *        changed later on. 
 * @return       error code
 * @return 0     : if everything went o.k.
 * @return 1     : writing + reading SPI failed.
 * @return 2     : Reset via EN low + high failed.
 * @return 3     : IRQ line failed.
 * @return 4     : Crystal not stable (Bit0 in AGC and Internal Status register 0x2A)
 * @return 5     : PLL not locked (Bits1 in AGC and Internal Status register 0x2A)
 */
uint16_t as3993Initialize(uint32_t baseFreq);

/** This function reads the version register of the AS3993 and
  * returns the value. \n
  * @return Returns the value of the AS3993 version register.
  */
uint8_t as3993ReadChipVersion(void);

/*------------------------------------------------------------------------- */
/** This function sets the frequency in the approbiate register
  * @param regs Which register is being used either AS3993_REG_PLLMAIN or AS3993_REG_PLLAUX
  * @param frequency frequency in kHz
  */
void as3993SetBaseFrequency(uint8_t regs, uint32_t frequency);

/*------------------------------------------------------------------------- */
/** This function turns the antenna power on or off. 
  * @param on boolean to value to turn it on (1) or off (0).
  */
void as3993AntennaPower( uint8_t on);

/*------------------------------------------------------------------------- */
/**  This function gets the RSSI (ReceivedSignalStrengthIndicator) of the
  *  environment in dBm. For measuring the antenna will be switched off, 
  *  int the time num_of_ms_to_scan measurements performed and the antenna
  *  if necessary turned back on.
  *  
  *  The returned value is the highest RSSI value expressed in dBm. -128 
  *  stands for no signal.
  *  
  *  Use this function to implement LBT (Listen Before Talk) at least for European
  *  based readers. Use like this
  *  \code
  *  as3993SetBaseFrequency( f + 100);
  *  as3993SaveSensitivity();
  *  as3993SetSensitivity(-50);
  *  as3993GetRSSI(10,&iq,&dBm)
  *  as3993RestoreSensitivity();
  *  if (dBm < -40) { 
  *     //use frequency f for operation
  *  }else{
  *     // try to find another freqency
  *  }
  *  \endcode
  */
void as3993GetRSSI( uint16_t num_of_ms_to_scan, uint8_t *rawIQ, int8_t *dBm );

/*------------------------------------------------------------------------- */
/**  This function stores the current sensitivity registers. After that 
  *  as3993SetSensitivity can be called to change sensitivty for subsequent 
  *  as3993GetRSSI() calls. as3993RestoreSensitivity() restores it again.
  */
void as3993SaveSensitivity( );

/*------------------------------------------------------------------------- */
/**  This function restores the sensitivity previousley saved using
  *  as3993SaveSensitivity().
  */
void as3993RestoreSensitivity( );

/*------------------------------------------------------------------------- */
/**  This function tries to set the sensitivity to a level to allow detection 
  *  of signals using as3993GetRSSI() with a level higher than miniumSignal(dBm).
  *  \return the level offset. Sensitivity was set to minimumSignal - (returned_val).
  *  negative values mean the sensitivity could not be reached.
  */
int8_t as3993SetSensitivity( int8_t minimumSignal );

/*------------------------------------------------------------------------- */
/**  This function gets the current rx sensitivity level.
  */
int8_t as3993GetSensitivity( void );

/*------------------------------------------------------------------------- */
/** This function gets the values for the reflected power. For measuring the 
  * antenna will be switched on, a measurement performed and the antenna
  * if necessary turned back off. The two measured 8-bit values are returned as
  * one 16-bit value, the lower byte containing receiver-A (I) value, the 
  * higher byte containing receiver-B (Q) value. The two values are signed 
  * values and will be converted from sign magnitude representation to natural.
  * Note: To get a more accurate result the I and Q values of
  * as3993GetReflectedPowerNoiseLevel() should be subtracted from the result of
  * as3993GetReflectedPower()
  */
uint16_t as3993GetReflectedPower( void );

/*------------------------------------------------------------------------- */
/**
  * This function measures and returns the noise level of the reflected power
  * measurement of the active antenna. The returned value is of type uint16_t and
  * contains the 8 bit I and Q values of the reflected power. The I value is
  * located in the lower 8 bits, the Q value in the upper 8 bits. \n
  * See as3993GetReflectedPower() fore more info.
  */
uint16_t as3993GetReflectedPowerNoiseLevel( void );

/*------------------------------------------------------------------------- */
/** This function reads in the current register settings (configuration only), 
  * performs a reset by driving the enable line first low, then high and then
  * writes back the register values.
  */
void as3993Reset(void);

/*------------------------------------------------------------------------- */
/** This function performs a reset by driving the enable line first low,
  * then high.
  */
void as3993ResetDoNotPreserveRegisters(void);

/*------------------------------------------------------------------------- */
/** This function is for generic sending and receiving gen2 commands through 
    the FIFO.
    The parameters are:
    \param cmd: One of the AS3993_CMD_* commands wich transmits something.
    \param txbuf: buffer to be tranmitted.
    \param txbits: the size of txbuf in bits
    \param rxbuf: a buffer getting the response of the tag
    \param rxbits: the maximum number of expected rx bits (size of rxbuf in bits)
                      The value will be written to rxlength register. If 0 rxlength is
                      automatically calculated by the reader and no boundary check for rxbuf is done.
    \param norestime: value for register AS3993_REG_RXNORESPONSEWAITTIME
                      0xff means 26 ms. Other values are much shorter.
    \param followCmd: Command to be sent after rxdone interrupt but before 
           emptying FIFO. This is needed for not violating T2 timing requirement.
    \param waitTxIrq: If 0 tx irq will not be handled after transmitting command,
           otherwise FW will wait for Tx irq after transmitting command.

    Certain values may be set to 0/NULL thus changing the behavior. Examples:

    - as3993TxRxGen2Bytes(AS3993_CMD_TRANSMCRCEHEAD, buf_, 50, buf_, &rxbits, 0xff, 0) with rxbits=33
      can be used to send WRITE command and retrieve after max 20ms a reply expecting a header.
    - as3993TxRxGen2Bytes(AS3993_CMD_TRANSMCRC, buf_, len, NULL, &rxbits, 0, 0) with rxbits=0
      is a transmit-only used for sending SELECT. With *rxbits==0 and rxbuf==NULL RX will be turned off immediately
      after TX-done interrupt
    - as3993TxRxGen2Bytes(AS3993_CMD_QUERY,buf_,16,tag->rn16,&rxlen,gen2IntConfig.no_resp_time,AS3993_CMD_ACK) with rxlen==16
      sends QUERY command, waits for tx-done, then waits for rx-done, then 
      sends ACK command and only as the last step reads RN16 from FIFO
    - as3993TxRxGen2Bytes(0,buf_,0,buf_,&rxlen,gen2IntConfig.no_resp_time,fast?0:AS3993_CMD_REQRN) with rxlen==sizeof(buf_)*8
      does not send anything (ACK was executed via followCmd as previous example),
      waits for tx-done, waits for rx-done, sends REQRN, reads EPC from FIFO

  */
int8_t as3993TxRxGen2Bytes(uint8_t cmd, uint8_t *txbuf, uint16_t txbits, 
                               uint8_t *rxbuf, uint16_t *rxbits,
                               uint8_t norestime, uint8_t followCmd,
                               uint8_t waitTxIrq);

/**
 * This function handles the rx-ed data for an command expecting a header bit.
 * On AS3980 we cannnot use the built in TRANSMCRCEHEADER command because
 * the header bit interrupt is hidden by the 2nd byte interrupt, which cannot be
 * disabled. Instead we use the the command TRANSMCRC and handle the header bit
 * in this function.\n
 * If a header bit is set the function will return ERR_CHIP_HEADER. \n
 * The content of the buffer will be shifted by 1 bit to remove the header bit.\n
 * As this function might ignore potential CRC errors, it checks if the received
 * data contains a the same handle as the handle parameter.
 * @param ret The return value of as399xTxRxGen2Bytes after sending a access command with TRANSMCRC
 * @param destbuf The received data. Will be shifted by 1 bit.
 * @param rxbits Number of received data bits.
 * @param handle Handle which should be inside the received data.
 * @return Returns the corrected error code.
 */
int8_t as3980HandleResponseWithHeaderBit(int8_t ret, uint8_t *destbuf, uint16_t rxbits, uint8_t const * handle);

/**
  * This functions performs an ADC conversion and returns the signed result in machine coding */
int8_t as3993GetADC( void );

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

#define as3993GetResponse() as3993Response

#define as3993ClrResponseMask(mask) as3993Response&=~(mask)

#define as3993ClrResponse() as3993Response=0

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

/*!
 *****************************************************************************
 *  \brief  after EN goes high the IRQ register has to be read after osc_ok
 * goes high.
 *****************************************************************************
 */
void as3993WaitForStartup(void);

void powerDownReader(void);

void powerUpReader(void);

void as3993Isr(void);

void RFID_AS3993_load_callbacks(void* spi_write,
                                void* delay_ms,
                                void* delay_us,
                                void* enableExtInterrupt,
                                void* disableExtInterrupt,
                                void* clearExtInterrupt,
                                void* setEnablePin,
                                void* isEnabledPin,
                                void* setSpiEnablePin);

void RFID_AS3993_delay_ms(uint16_t delay);

void RFID_AS3993_delay_us(uint16_t delay);

//----------------------------------------------------------------------
#endif /* _AS3993_H_ */
