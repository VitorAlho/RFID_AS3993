
#include "as3993_config.h"
#include "platform.h"
#include "as3993.h"
#include "as3993_public.h"
#include "global.h"
#include "timer.h"
#include "gen2.h"
#include "stdlib.h"
#include "string.h"
#include "Compiler.h"

/** Definition protocol read bit */
#define READ                    0x40

/** This variable is used as flag to signal an data reception.
  *  It is a bit mask of the RESP_TXIRQ, RESP_RXIRQ, RESP_* values
  */
volatile uint16_t as3993Response = 0;

/** temporary buffer for as3993SaveSensitivity() and as3993RestoreSensitivity() */
static uint8_t as3993SavedSensRegs[1];

/** Restore registers 0x00 to 0x1D + 5 registers (0x22, 0x29, 0x35, 0x36 and 0x3A)
 *  after power up */
static uint8_t as3993PowerDownRegs[AS3993_REG_ICD+6];

/** Will be set to 0 if version register is > 0x60. Silicon revision 0x60 needs
 specific handling in some functions. */
static uint8_t gChipRevisionZero = 1;

/*------------------------------------------------------------------------- */
uint16_t as3993Initialize(uint32_t baseFreq)
{
    uint8_t myBuf[4];

    as3993ResetDoNotPreserveRegisters();

    // check SPI communication
    myBuf[0] = 0x55;
    myBuf[1] = 0xAA;
    myBuf[2] = 0xFF;
    myBuf[3] = 0x00;
    as3993ContinuousWrite(AS3993_REG_MODULATORCONTROL1, myBuf, 4);
    memset(myBuf, 0x33, sizeof(myBuf));
    as3993ContinuousRead(AS3993_REG_MODULATORCONTROL1, 4,myBuf);
    if ((myBuf[0]!=0x55) || 
        (myBuf[1]!=0xAA) || 
        (myBuf[2]!=0xFF) ||
        (myBuf[3]!=0x00))
    {
        //LOG("%hhx %hhx %hhx %hhx\n", myBuf[0], myBuf[1], myBuf[2], myBuf[3]);
        return 1; // data bus interface pins not working
    }

    // check EN pin + SPI communication
    as3993ResetDoNotPreserveRegisters();
    as3993ContinuousRead(AS3993_REG_MODULATORCONTROL1, 4, myBuf);
    if ((myBuf[0]==0x55) || 
        (myBuf[1]==0xAA) || 
        (myBuf[2]==0xFF) ||
        (myBuf[3]==0x00))
    {
        //LOG("EN pin failed\n");
        return 2; /* enable pin not working */
    }

    // check IRQ line
    delay_ms(1);
    as3993SingleWrite(AS3993_REG_IRQMASK1, 0x20);
    // set up 48Byte transmission, but we supply less, therefore a fifo underflow IRQ is produced
    as3993SingleWrite(AS3993_REG_TXLENGTHUP, 0x03);
    as3993SingleCommand(AS3993_CMD_TRANSMCRC);
    as3993ContinuousWrite(AS3993_REG_FIFO,myBuf,4);
    as3993ContinuousWrite(AS3993_REG_FIFO,myBuf,4);
    as3993ContinuousWrite(AS3993_REG_FIFO,myBuf,4);
    as3993ContinuousWrite(AS3993_REG_FIFO,myBuf,4);
    as3993ContinuousWrite(AS3993_REG_FIFO,myBuf,4);
    as3993ContinuousWrite(AS3993_REG_FIFO,myBuf,4);

    as3993WaitForResponse(RESP_FIFO);
    if ( !(as3993GetResponse() & RESP_FIFO) )
    {
        return 3;
    }
    
    as3993ClrResponse();

    as3993ResetDoNotPreserveRegisters();
    as3993SingleCommand(AS3993_CMD_HOP_TO_MAIN_FREQUENCY);

    /* chip status control 0x00 */
    /*STBY - - - - AGC REC RF */
    /* 0   0 0 0 0  0   1  0  = 0x02 */
    as3993SingleWrite(AS3993_REG_STATUSCTRL, 0x02);

    /*protocl control register 0x01 */
    /*RX_CRC_N DIR_MODE AutoACK1 AutoACK2 - PROT1 PROT0  */
    /*   0         0       0         0    0   0     0 = 0x00 */
    as3993SingleWrite(AS3993_REG_PROTOCOLCTRL, 0x00);

    /* TX options 0x02 */
    /* - - TxOne1 TxOne0 - Tari2 Tari1 Tari0 */

    /* 0 0   1      1    0   0     0     0   = 0x30 */
    as3993SingleWrite(AS3993_REG_TXOPTIONS, 0x30);

    /* RX options register is set by gen2 configuration */
    /* TRcal high + low register is set by gen2 configuration */

    /* PLL Main REgister 0x17*/
    /* use 100kHz as pll reference as we use ETSI as default */
    //as3993SingleWrite(AS3993_REG_PLLMAIN1, 0x54);
    as3993SingleWrite(AS3993_REG_PLLMAIN1, 0x41);   //125kHz
    as3993SingleWrite(AS3993_REG_PLLMAIN2, 0xA0);   //e B = 104
    as3993SingleWrite(AS3993_REG_PLLMAIN3, 0x76);   //e A = 118

    as3993SingleWrite(AS3993_REG_MEASUREMENTCONTROL, 0x00);

    /*MISC1 0x0D */
    /*hs_output hs_oad miso_pd2 miso_pd1 open_dr s_mix iadd_sink2 iadd_sink1 */
    /*     1       0       0        0        0     0       0          0 = 0xC0 */
    as3993SingleWrite(AS3993_REG_MISC1, 0x00 | 0x80);

    /*REGULATOR and PA Bias 0x0B */
    /*pa_bias1-0 rvs_rf2-1 rvs2-0  */
    /*   0 0       0 1 1    0 1 1   = 0x1B */
    //as3993SingleWrite(AS3993_REG_REGULATORCONTROL, 0x1B);
    as3993SingleWrite(AS3993_REG_REGULATORCONTROL, 0x3F);

    /*RF Output and LO Control Register 0x0C */
    /*eTX7 - eTX0  */
    /*  0 0 0 0 0 0 1 0   = 0x58 */
    /* int pa, PA 14mA */
    as3993SingleWrite(AS3993_REG_RFOUTPUTCONTROL, 0x22);

    /*Modulator Control Register 1 0x13 */
    /* - MAIN_MOD AUXMOD tpreset use_corr ELFP ASKRATE1 ASKRATE0 */
    /* 0    0       1       0       0       0      0        0      =0x20 */
    myBuf[0] = 0x20;
    /*Modulator Control Register 2 0x14 */

    /*ook_ask PR_ASK MOD_DEP5-MOD_DEP0 */
    /*    1      1     0 1     1 1 0 1   = 0xDD */
    myBuf[1] = 0xDD;

    /*Modulator Control Register 3 0x15 */
    /*TRFON1 TRFON0 lin_mode TXLEVEL4-TXLEVEL0 */
    /*   0     0       0      0       0 1 1 1  =0x07 */

    myBuf[2] = 0x00;    //potencia saida em 0dbm (max)

    as3993ContinuousWrite(AS3993_REG_MODULATORCONTROL1, myBuf, 3);

    /*RF Output and LO Control Register 0x0C */
    /*LF_R3<2-1> LF_C3<5-3> cp<2-0>  */
    /*    0 0       1 1 0    1 0 1   = 0x25 */
    /* 30kOhm, 160pF, 1500uA */    
        as3993SingleWrite(AS3993_REG_CPCONTROL, 0x35);

    as3993SingleWrite(AS3993_REG_MISC2, 0x00);      // no clsys

    /*Enable Interrupt Register Register 1 0x35 */
    /*e_irq_TX e_irq_RX e_irq_fifo e_irq_err e_irq_header RFU e_irq_AutoAck e_irq_noresp  */
    /*    1        1          1         1          1       1        1             1     = 0xFF */
    as3993SingleWrite(AS3993_REG_IRQMASK1, 0xFF);
    /*Enable Interrupt Register Register 2 0x35 */
    /*e_irq_ana e_irq_cmd RFU RFU RFU e_irq_err1 e_irq_err2 e_irq_err3 */
    /*    0         0      0   0   0      1          1          1     = 0x07 */
    as3993SingleWrite(AS3993_REG_IRQMASK2, 0x07);
    
    /*RX Length Register 1 0x3A */
    /*RX_crc_n2 fifo_dir_irq2 rep_irg2 auto_errcode_RXl RXl11-RXl8 */
    /*    0           0           0            1         0 0 0 0  = 0x10 */
    as3993SingleWrite(AS3993_REG_RXLENGTHUP, 0x10);

    /* Give base freq. a reasonable value */
    as3993SetBaseFrequency(AS3993_REG_PLLMAIN1, baseFreq);
    as3993SetSensitivity(-AS3993_NOM_SENSITIVITY);

    /* Now that the chip is configured with correct ref frequency the PLL 
       should lock */
    delay_ms(20);
    myBuf[0] = as3993SingleRead(AS3993_REG_AGCANDSTATUS);
    if (!(myBuf[0] & 0x03))
    {
        return 4; /* Crystal not stable */
    }
    if (!(myBuf[0] & 0x02)) 
    { 
       return 5; /* PLL not locked */
    }

    return 0;
}

/*------------------------------------------------------------------------- */
/** External Interrupt Function
  * The AS3993 uses the external interrupt to signal the uC that
  * something happened. The interrupt function reads the AS3993
  * IRQ status registers.
  * The interrupt function sets the flags if an event occours.
  */
#if RUN_ON_AS3993 || RUN_ON_AS3980 || RUN_ON_AS3994 || RUN_ON_AS3981
void as3993Isr(void)
{
    uint8_t regs[2];
    static uint8_t addr = READ | AS3993_REG_IRQSTATUS1;
    if (gChipRevisionZero)
        delay_us(30);

    writeReadAS3993Isr(&addr, 1, regs, 2);
    as3993Response |= (regs[0] | (regs[1] << 8));

    CLREXTIRQ();
    //LOG("isr: %hx\n", as3993Response);
}
#endif

/*------------------------------------------------------------------------- */
uint8_t as3993ReadChipVersion(void)
{
    uint8_t version;
    version = as3993SingleRead(AS3993_REG_DEVICEVERSION);
    if (version > 0x60)
        gChipRevisionZero = 0;
    return version;
}

/*------------------------------------------------------------------------- */
void as3993SingleCommand(uint8_t command)
{
    DISEXTIRQ();
    writeReadAS3993( &command, 1, 0 , 0 , STOP_SGL, 1);
    ENEXTIRQ();
}
/*------------------------------------------------------------------------- */
void as3993SingleWriteNoStop(uint8_t address, uint8_t value)
{
    uint8_t buf[2];
    buf[0] = address;
    buf[1] = value;
    writeReadAS3993( buf, 2, 0 , 0 , STOP_NONE, 1);
}

/*------------------------------------------------------------------------- */
void as3993ContinuousRead(uint8_t address, int8_t len, uint8_t *readbuf)
{
    DISEXTIRQ();
    address |= READ;
    writeReadAS3993( &address, 1, readbuf , len , STOP_CONT, 1);
    ENEXTIRQ();
}

/*------------------------------------------------------------------------- */
void as3993FifoRead(int8_t len, uint8_t *readbuf)
{
    static uint8_t address = AS3993_REG_FIFO | READ ;
    DISEXTIRQ();
    writeReadAS3993( &address, 1, readbuf , len , STOP_CONT, 1);
    ENEXTIRQ();
}

/* Function is called from interrupt and normal level, therefore this 
   function must be forced to be reentrant on Keil compiler. */
uint8_t as3993SingleRead(uint8_t address)
{
    uint8_t readdata;

    DISEXTIRQ();
    address |= READ;
    writeReadAS3993( &address, 1, &readdata , 1 , STOP_SGL, 1);

    ENEXTIRQ();
    return(readdata);
}

void as3993ContinuousWrite(uint8_t address, uint8_t *buf, int8_t len)
{
    DISEXTIRQ();
    writeReadAS3993( &address, 1, 0 , 0 , STOP_NONE, 1);
    writeReadAS3993( buf, len, 0 , 0 , STOP_CONT, 0);
    ENEXTIRQ();
}

void as3993SingleWrite(uint8_t address, uint8_t value)
{
    uint8_t buf[2];
    buf[0] = address;
    buf[1] = value;
    DISEXTIRQ();
    writeReadAS3993( buf, 2, 0 , 0 , STOP_SGL, 1);
    ENEXTIRQ();
}

/*------------------------------------------------------------------------- */
void as3993CommandContinuousAddress(uint8_t *command, uint8_t com_len,
                             uint8_t address, uint8_t *buf, uint8_t buf_len)
{
    DISEXTIRQ();
    writeReadAS3993( command, com_len, 0 , 0 , STOP_NONE, 1);
    writeReadAS3993( &address, 1, 0 , 0 , STOP_NONE, 0);
    writeReadAS3993( buf, buf_len, 0 , 0 , STOP_CONT, 0);
    ENEXTIRQ();
}

void as3993WaitForResponseTimed(uint16_t waitMask, uint16_t counter)
{
    while (((as3993Response & waitMask) == 0) && (counter))
    {
        delay_ms(1);
        counter--;
    }
    if (counter==0)
    {
#if !USE_UART_STREAM_DRIVER
        //LOG("TI O T %x %x\n", as3993Response, waitMask);
#endif
        as3993Reset();
        as3993Response = RESP_NORESINTERRUPT;
    }

}

void as3993WaitForResponse(uint16_t waitMask)
{
    uint16_t counter;
    counter=0;
    while (((as3993Response & waitMask) == 0) && (counter < WAITFORRESPONSECOUNT))
    {
        counter++;
        delay_us(WAITFORRESPONSEDELAY);
    }
    if (counter >= WAITFORRESPONSECOUNT)
    {
#if !USE_UART_STREAM_DRIVER
        //LOG("TI O response: %x, mask: %x\n", as3993Response, waitMask);
#endif
        as3993Reset();
        as3993Response = RESP_NORESINTERRUPT;
    }
//    else
//    	LOG("TI ok response: %x, mask: %x\n", as3993Response, waitMask);
}

void as3993SelectLinkFrequency(uint8_t a)
{
    uint8_t reg;

    reg = as3993SingleRead(AS3993_REG_RXOPTIONS);

    reg &= ~0xf0;
    reg |= (a<<4);
    as3993SingleWrite(AS3993_REG_RXOPTIONS, reg);
}

static void as3993LockPLL(void)
{
    uint8_t buf;
    uint8_t var;
    uint16_t i;

    uint8_t vco_voltage;
    buf = as3993SingleRead(AS3993_REG_STATUSPAGE);
    buf &= ~0x30;
    buf |= 0x10; /* have vco_ri in aglstatus */
    as3993SingleWrite(AS3993_REG_STATUSPAGE, buf);

    buf = as3993SingleRead(AS3993_REG_VCOCONTROL);
    buf |= 0x80; /* set mvco bit */
    as3993SingleWrite(AS3993_REG_VCOCONTROL, buf);
    delay_ms(1); /* give PLL some settling time, should be around 500us */

    vco_voltage = as3993SingleRead(AS3993_REG_AGL) & 0x07;

    buf &= ~0x80; /* reset mvco bit */
    as3993SingleWrite(AS3993_REG_VCOCONTROL, buf);

    if ( vco_voltage <= 1 || vco_voltage >= 6 )
    {
        i=0;
        do
        {
            i++;
            as3993SingleCommand(AS3993_CMD_VCO_AUTO_RANGE);
            delay_ms(10);  /* Please keep in mind, that the Auto Bit procedure will take app. 6 ms whereby the locktime of PLL is just 400us */
            var=as3993SingleRead(AS3993_REG_AGCANDSTATUS);
        } while ( (var & 0x02)==0 && (i<3));/* wait for PLL to be locked and give a few attempts */
    }
}

/*------------------------------------------------------------------------- */
void as3993SetBaseFrequency(uint8_t regs, uint32_t frequency)
{
    //TODO: use autohop command instead?
    uint8_t buf[3];
    uint8_t statusreg;
    uint16_t ref=0, i, j, x, reg_A,reg_B;
    uint32_t divisor;

    statusreg = as3993SingleRead(AS3993_REG_STATUSCTRL);
    as3993SingleWrite(AS3993_REG_STATUSCTRL, statusreg & 0xfe);
    if (regs == AS3993_REG_PLLMAIN1)
    {
        as3993ContinuousRead(AS3993_REG_PLLMAIN1, 3, buf);
        switch (buf[0]& 0x70)
        {
/*
        case 0x00: {
            ref=500;
        } break;
        case 0x10: {
            ref=250;
        } break;
*/
        case 0x40: {
            ref=125;
        } break;
        case 0x50: {
            ref=100;
        } break;
        case 0x60: {
            ref=50;
        } break;
        case 0x70: {
            ref=25;
        } break;
        default: {
            ref=0;
        }
        }
    }
    divisor=frequency/ref;

    i = 0x3FFF & (divisor >> 6); /* division by 64 */
    x = (i<<6)+ i;
    if (divisor > x)
    {
        x += 65;
        i++;
    }
    x -= divisor;
    j = i;
    do
    {
        if (x >= 33)
        {
            i--;
            x -= 33;
        }
        if (x >= 32)
        {
            j--;
            x -= 32;
        }
    } while (x >= 32);
    if (x > 16) 
    {            /* subtract 32 from x if it is larger than 16 */
        x -= 32; /* this yields more closely matched A and B values */
        j--;
    }

    reg_A = i - x;
    reg_B = j + x;
    if (regs==AS3993_REG_PLLMAIN1)
    {
        buf[0] = (buf[0] & 0xF0) | ((uint8_t)((reg_B >> 6) & 0x0F));
        buf[1] = (uint8_t)((reg_B << 2) & 0xFC) |  (uint8_t)((reg_A >> 8) & 0x03);
        buf[2] = (uint8_t)reg_A;

        as3993ContinuousWrite(AS3993_REG_PLLMAIN1, buf, 3);
    }
    as3993LockPLL();
    as3993SingleWrite(AS3993_REG_STATUSCTRL, statusreg);
}

void as3993EnterPowerDownMode()
{
    uint8_t i;
    int count;
    
    if (!ENABLE) return;

    DISEXTIRQ();
    CLREXTIRQ();
    /* Switch off antenna */
    as3993PowerDownRegs[0] = as3993SingleRead(AS3993_REG_STATUSCTRL);
    as3993SingleWrite(0, as3993PowerDownRegs[0] & (~0x03));
    for (i=1; i<AS3993_REG_ICD; i++)
    {
        if (i!=0x0F)
            as3993PowerDownRegs[i] = as3993SingleRead(i);
    }
    as3993PowerDownRegs[AS3993_REG_ICD+0] = as3993SingleRead(AS3993_REG_MIXOPTS);
    as3993PowerDownRegs[AS3993_REG_ICD+1] = as3993SingleRead(AS3993_REG_STATUSPAGE);
    as3993PowerDownRegs[AS3993_REG_ICD+2] = as3993SingleRead(AS3993_REG_IRQMASK1);
    as3993PowerDownRegs[AS3993_REG_ICD+3] = as3993SingleRead(AS3993_REG_IRQMASK2);
    as3993PowerDownRegs[AS3993_REG_ICD+4] = as3993SingleRead(AS3993_REG_TXSETTING);
    as3993PowerDownRegs[AS3993_REG_ICD+5] = as3993SingleRead(AS3993_REG_RXLENGTHUP);
    /* Wait for antenna being switched off */
    count = 500;
    while(count-- && (as3993SingleRead(AS3993_REG_AGCANDSTATUS) & 0x04))
    {
        delay_ms(1);
    }
    EN(LOW);
}

void as3993ExitPowerDownMode()
{
    uint8_t i;
    uint8_t buf[2];
    
    if (ENABLE) return;

    EN(HIGH);
    delay_us(10);
    as3993WaitForStartup();

    /* Do not switch on antenna before PLL is locked.*/
    as3993SingleWrite(0, as3993PowerDownRegs[0] & (~0x03));
    for (i=1; i<AS3993_REG_ICD; i++)
    {
        if (i!=0x0F)
            as3993SingleWrite(i, as3993PowerDownRegs[i]);
    }
    as3993SingleWrite(AS3993_REG_MIXOPTS,    as3993PowerDownRegs[AS3993_REG_ICD+0]);
    as3993SingleWrite(AS3993_REG_STATUSPAGE, as3993PowerDownRegs[AS3993_REG_ICD+1]);
    as3993SingleWrite(AS3993_REG_IRQMASK1,   as3993PowerDownRegs[AS3993_REG_ICD+2]);
    as3993SingleWrite(AS3993_REG_IRQMASK2,   as3993PowerDownRegs[AS3993_REG_ICD+3]);
    as3993SingleWrite(AS3993_REG_TXSETTING,  as3993PowerDownRegs[AS3993_REG_ICD+4]);
    as3993SingleWrite(AS3993_REG_RXLENGTHUP, as3993PowerDownRegs[AS3993_REG_ICD+5] & 0xF0);
    delay_us(300);
    as3993LockPLL();
    as3993SingleWrite(AS3993_REG_STATUSCTRL, as3993PowerDownRegs[0]);

    buf[0] = as3993SingleRead(AS3993_REG_AGCANDSTATUS);
    if (!(buf[0] & 0x03))
    {
        //LOG("/******** Crystal not stable */\n");
    }
    if (!(buf[0] & 0x02))
    {
       //LOG("/********* PLL not locked */\n");
    }
    CLREXTIRQ();
    ENEXTIRQ();
}

void as3993Reset(void)
{
    as3993EnterPowerDownMode();
    delay_ms(1);
    as3993ExitPowerDownMode();
}

void as3993ResetDoNotPreserveRegisters(void)
{
    EN(LOW);
    delay_ms(1);
    EN(HIGH);
    delay_us(10);
    as3993WaitForStartup();
}

void as3993EnterPowerNormalMode()
{
    uint8_t stat;

    as3993ExitPowerDownMode();      //ensure that EN is high
    stat = as3993SingleRead(AS3993_REG_STATUSCTRL);
    stat &= 0x7F;
    as3993SingleWrite(AS3993_REG_STATUSCTRL, stat);
    as3993AntennaPower(0);
}

void as3993ExitPowerNormalMode()
{
}

void as3993EnterPowerNormalRfMode()
{
    uint8_t stat;

    as3993ExitPowerDownMode();      //ensure that EN is high
    stat = as3993SingleRead(AS3993_REG_STATUSCTRL);
    stat &= 0x7F;
    as3993SingleWrite(AS3993_REG_STATUSCTRL, stat);
    as3993AntennaPower(1);
}

void as3993ExitPowerNormalRfMode()
{
}

void as3993EnterPowerStandbyMode()
{
    uint8_t stat;

    as3993ExitPowerDownMode();      //ensure that EN is high
    as3993AntennaPower(0);
    stat = as3993SingleRead(AS3993_REG_STATUSCTRL);
    stat |= 0x80;
    as3993SingleWrite(AS3993_REG_STATUSCTRL, stat);
}

void as3993ExitPowerStandbyMode()
{
    uint8_t stat;

    as3993ExitPowerDownMode();      //ensure that EN is high
    stat = as3993SingleRead(AS3993_REG_STATUSCTRL);
    stat &= ~0x80;
    as3993SingleWrite(AS3993_REG_STATUSCTRL, stat);
}

void as3993WaitForStartup(void)
{
    uint8_t osc_ok, version;
    uint8_t myBuf[2];
    uint16_t count = 0;

    do
    {
        version = as3993ReadChipVersion() & 0x60;   //ignore revision
        osc_ok = as3993SingleRead(AS3993_REG_AGCANDSTATUS);
        count++;
    }
    while (!((version == 0x60 && osc_ok & 0x01) || count > 250));    //wait for startup
    delay_us(500);
    as3993ContinuousRead(AS3993_REG_IRQSTATUS1, 2, &myBuf[0]);    // ensure that IRQ bits are reset
    as3993ClrResponse();
    delay_ms(2);            // give AS3993 some time to fully initialize
}

void as3993AntennaPower( uint8_t on)
{
    uint8_t val;
    unsigned count;
    val = as3993SingleRead(AS3993_REG_STATUSCTRL);

    if (on)
    {
        if ((val & 0x03) == 0x03)
            return;
        val |= 3;

        delay_us(300);
        as3993SingleWrite(AS3993_REG_STATUSCTRL, val);
    }
    else
    {
        if ((val & 0x03) != 0x03)
            return;
        val &= ~3;
        as3993SingleWrite(AS3993_REG_STATUSCTRL, val);
        /* Wait for antenna being switched off */
        count = 500;
        while(count-- && (as3993SingleRead(AS3993_REG_AGCANDSTATUS) & 0x04))
        {
            delay_us(100);
        }
        //MCULED(LEDOFF);
        //MCULED(LEDON);
        //desliga_saida_pa();
        //delay_us(300);
#ifdef EXTPA
        //EN_PA(PA_OFF);
#endif
#if 0
        DCDC(HIGH);  
#endif
    }

    if(on) delay_ms(6); /* according to standard we have to wait 1.5ms before issuing commands  */
}

static uint8_t as3993GetRawRSSI( void )
{
    uint8_t value;

    as3993SingleCommand(AS3993_CMD_BLOCKRX);
    as3993SingleCommand(AS3993_CMD_ENABLERX);
    delay_us(500);/* According to architecture note we have to wait here at least 100us
                    however experiments show 350us to be necessary on AS3992 */
    value = as3993SingleRead(AS3993_REG_RSSILEVELS);
    as3993SingleCommand(AS3993_CMD_BLOCKRX);
    return value;
}

void as3993SaveSensitivity( )
{
    as3993SavedSensRegs[0] = as3993SingleRead(AS3993_REG_RXMIXERGAIN);
}

void as3993RestoreSensitivity( )
{
    as3993SingleWrite(AS3993_REG_RXMIXERGAIN, as3993SavedSensRegs[0]);
}

int8_t as3993GetSensitivity( )
{
    int8_t sensitivity = 0;
    uint8_t reg0a, reg0d, gain;

    reg0d = as3993SingleRead(AS3993_REG_MISC1);
    reg0a = as3993SingleRead(AS3993_REG_RXMIXERGAIN);

    if ((reg0d & 0x04))
    { /* single ended input mixer*/
        sensitivity -= AS3993_NOM_SENSITIVITY;
        if ( (reg0a & 0x03) == 0x03 ) /* 6dB increase */
        {
            sensitivity -= 6;
        }
        else if ( (reg0a & 0x03) == 0x00 ) /* 6dB decrease */
        {
            sensitivity += 6;
        }
    }
    else
    { /* differential input mixer */
        sensitivity -= AS3993_NOM_SENSITIVITY;
        if ( reg0a & 0x01 )  /* mixer attenuation */
        {
            sensitivity += 8;
        }
        if ( reg0a & 0x02)  /* differential mixer gain increase */
        {
            sensitivity -= 10;
        }
    }
    gain = (reg0a >> 6) * 3;
    if ( reg0a & 0x20 )
    { /* RX Gain direction: increase */
        sensitivity -= gain;
    }
    else
    {
        sensitivity += gain;
    }
    return sensitivity;
}

int8_t as3993SetSensitivity( int8_t minimumSignal )
{
    uint8_t reg0d, reg0a, gain;
      
    reg0a = as3993SingleRead(AS3993_REG_RXMIXERGAIN);
    reg0d = as3993SingleRead(AS3993_REG_MISC1);
        
    reg0a &= 0x1C;
    if ((reg0d & 0x04))
    { /* single ended input mixer*/
        minimumSignal += AS3993_NOM_SENSITIVITY;
        if ( minimumSignal >= 6 )
        {
            minimumSignal -= 6; /* 6dB gain decrease */
            reg0a |= 0;
        }
        else if ( minimumSignal <= -6 )
        {
            minimumSignal += 6; /* 6dB gain increase */
            reg0a |= 3;
        }
        else
            reg0a |= 1;         /* nominal gain */
    }
    else
    { /* differential input mixer */
        minimumSignal += AS3993_NOM_SENSITIVITY;
        if ( minimumSignal >= 8 )   /* mixer attenuation */
        {
            minimumSignal -= 8;
            reg0a |= 1;
        }
        if ( minimumSignal <= -10 ) /* differential mixer gain increase */
        {
            minimumSignal += 10;
            reg0a |= 2;
        }
    }
    if ( minimumSignal > 0)
    {
        reg0a &= ~0x20;             /* RX Gain direction: decrease */
        gain = minimumSignal / 3;
        if ( gain>3 ) gain = 3;
        minimumSignal -= gain*3;
        
    }
    else
    {
        reg0a |= 0x20;              /* RX Gain direction: increase */
        gain = (-minimumSignal+2) / 3;
        if ( gain>3 ) gain = 3;
        minimumSignal += gain*3;
    }
    reg0a |= (gain<<6);
    as3993SingleWrite(AS3993_REG_RXMIXERGAIN, reg0a);
        
    return minimumSignal;
}

void as3993GetRSSI( uint16_t num_of_ms_to_scan, uint8_t *rawIQ, int8_t* dBm )
{
    uint8_t val;
    uint8_t rawVal, singleInput;
    uint8_t regStatus, regFilter;
    uint16_t num_of_reads = num_of_ms_to_scan/2; /* as3993GetRawRSSI() delays 500us*/
    uint8_t gainSettings;
    int8_t sum;

    if (num_of_reads == 0) num_of_reads = 1;
    regStatus = as3993SingleRead(AS3993_REG_STATUSCTRL);
    as3993SingleWrite(AS3993_REG_STATUSCTRL, 2 ); /* Receiver on and transmitter are off */
    val = as3993SingleRead(AS3993_REG_STATUSPAGE);
    val &= 0xF0;                                    //set real time rssi in rssi register
    as3993SingleWrite(AS3993_REG_STATUSPAGE, val);
    regFilter = as3993SingleRead(AS3993_REG_RXFILTER);
    as3993SingleWrite(AS3993_REG_RXFILTER, 0xff ); /* Optimal filter settings */
    if(!(regStatus & 0x02)) delay_ms(10); /* rec_on needs about 6ms settling time, to be sure wait 10 ms */

    sum = 0;
    while (num_of_reads--)
    {
        uint8_t temp;
        rawVal = as3993GetRawRSSI();
        temp = (rawVal&0x0f) + (rawVal>>4);
        if (temp > sum)
        {
            sum = temp;
            *rawIQ = rawVal;
        }
    }

    if (sum == 0)
    { /* short exit, below formula does not work for 0 value */
        *dBm   = -128;
        *rawIQ = 0;
        goto end;
    }

    sum += (sum>>3); /* ~2.2*meanRSSI, here we calculate 2.25*meanRSSI, should not matter with a range 0..30 */
    singleInput = as3993SingleRead(AS3993_REG_MISC1) & 0x04;
    gainSettings = as3993SingleRead(AS3993_REG_RXMIXERGAIN);
    if (singleInput)
    { /* single ended input mixer*/
        sum -= 75;
        if ((gainSettings & 0x01)) sum += 5; /* attenuation */
    }
    else
    { /* differential input mixer */
        sum -= 71;
        if ((gainSettings & 0x02)) sum -= 10; /* differential mixer gain increase */
        if ((gainSettings & 0x01)) sum += 8; /* attenuation */
    }
    /* Gain in/de-crease */
    if (gainSettings & 0x20)
    {
        sum -= 3*(gainSettings>>6);
    }
    else
    {
        sum += 3*(gainSettings>>6);
    }
    *dBm = sum;

end:
    as3993SingleWrite(AS3993_REG_RXFILTER, regFilter ); /* Restore filter */
    as3993SingleWrite(AS3993_REG_STATUSCTRL, regStatus);
    if(regStatus & 1) delay_ms(2); /* according to standard we have to wait 1.5ms before issuing commands  */
    return;
}

/* ADC Values are in sign magnitude representation -> convert */
#define CONVERT_ADC_TO_NAT(A) (((A)&0x80)?((A)&0x7f):(0 - ((A)&0x7f)))
int8_t as3993GetADC( void )
{
    int8_t val;
    as3993SingleCommand(AS3993_CMD_TRIGGERADCCON);
    delay_us(20); /* according to spec */
    val = as3993SingleRead(AS3993_REG_ADC);
    val = CONVERT_ADC_TO_NAT(val);
    return val;
}

uint16_t as3993GetReflectedPower( void )
{
    uint16_t value;
    int8_t adcVal;
    uint8_t regMeas, regStatus;
    regStatus = as3993SingleRead(AS3993_REG_STATUSCTRL);
    regMeas = as3993SingleRead(AS3993_REG_MEASUREMENTCONTROL);
    as3993SingleWrite(AS3993_REG_MEASUREMENTCONTROL, regMeas & ~0xC0);  /* disable the OAD pin outputs */

    as3993SingleWrite(AS3993_REG_STATUSCTRL, regStatus | 0x03 ); /* Receiver and transmitter are on */
    as3993SingleCommand(AS3993_CMD_BLOCKRX); /* Reset the receiver - otherwise the I values seem to oscillate */
    as3993SingleCommand(AS3993_CMD_ENABLERX);
    as3993SingleWrite(AS3993_REG_MEASUREMENTCONTROL, 0x01); /* Mixer A DC */
    delay_us(300); /* settling time */
    value = as3993GetADC();
    as3993SingleWrite(AS3993_REG_MEASUREMENTCONTROL, 0x02); /* Mixer B DC */
    delay_us(300); /* settling time */
    adcVal = as3993GetADC();
    as3993SingleCommand(AS3993_CMD_BLOCKRX); /* Disable the receiver since we enabled it before */

    /* mask out shifted in sign bits */
    value = (value & 0xff) | (adcVal << 8);
    as3993SingleWrite(AS3993_REG_MEASUREMENTCONTROL, regMeas);
    as3993SingleWrite(AS3993_REG_STATUSCTRL, regStatus);
    return value;
}

uint16_t as3993GetReflectedPowerNoiseLevel( void )
{
    uint16_t value;
    int8_t i_0, q_0;
    uint8_t regMeas, regStatus;
    regStatus = as3993SingleRead(AS3993_REG_STATUSCTRL);
    regMeas = as3993SingleRead(AS3993_REG_MEASUREMENTCONTROL);
    as3993SingleWrite(AS3993_REG_MEASUREMENTCONTROL, regMeas & ~0xC0);  /* disable the OAD pin outputs */

    /* First measure the offset which might appear with disabled antenna */
    as3993SingleWrite(AS3993_REG_STATUSCTRL, (regStatus | 2) & (~1) ); /* Field off, receiver on */
    as3993SingleCommand(AS3993_CMD_BLOCKRX); /* Reset the receiver - otherwise the I values seem to oscillate */
    as3993SingleCommand(AS3993_CMD_ENABLERX);
    as3993SingleWrite(AS3993_REG_MEASUREMENTCONTROL, 0x01); /* Mixer A DC */
    delay_us(300); /* settling time */
    i_0 = as3993GetADC();
    as3993SingleWrite(AS3993_REG_MEASUREMENTCONTROL, 0x02); /* Mixer B DC */
    delay_us(300); /* settling time */
    q_0 = as3993GetADC();

    value = (i_0 & 0xff) | (q_0 << 8);
    as3993SingleWrite(AS3993_REG_MEASUREMENTCONTROL, regMeas);
    as3993SingleWrite(AS3993_REG_STATUSCTRL, regStatus);
    return value;

}

int8_t as3993TxRxGen2Bytes(uint8_t cmd, uint8_t *txbuf, uint16_t txbits, 
                               uint8_t *rxbuf, uint16_t *rxbits,
                               uint8_t norestime, uint8_t followCmd,
                               uint8_t waitTxIrq)
{
    static uint8_t currCmd;
    uint8_t buf[2];
    uint8_t rxbytes = (*rxbits + 7) / 8;
    uint8_t checkRxLength = 0;
    uint16_t rxs = 0;
    uint8_t rxed = 0;
    if (rxbits)
    {
        rxs = *rxbits;
        *rxbits = 0;
        checkRxLength = 1;
    }
    if(rxbuf || rxs)
    {
        as3993SingleWrite(AS3993_REG_RXNORESPONSEWAITTIME, norestime);
    }
    if (rxs)
    {
        buf[1] = rxs & 0xff;
        buf[0] = ((rxs>>8) & 0x0F) | 0x10;    // set auto_errcode_rxl
        as3993ContinuousWrite(AS3993_REG_RXLENGTHUP,buf,2);
    }
    if (txbits)
    {
        uint8_t txbytes = (txbits + 7)/8;
        uint8_t totx = txbytes;
        if (totx > 24) totx = 24;

        /* set up two bytes tx length register */
        buf[1] = (txbits % 8) << 1;
        buf[1] |= (txbits / 8) << 4;
        buf[0] = txbits / 128;
        as3993ContinuousWrite(AS3993_REG_TXLENGTHUP,buf,2);

        as3993CommandContinuousAddress(&cmd, 1, AS3993_REG_FIFO, txbuf, totx);
        txbytes -= totx;
        txbuf += totx;
        while (txbytes)
        {
            totx = txbytes;
            if (totx > 16) totx = 18;

            as3993WaitForResponse(RESP_FIFO);
            as3993ClrResponseMask(RESP_FIFO);
            as3993ContinuousWrite(AS3993_REG_FIFO,txbuf,totx);
            txbytes -= totx;
            txbuf += totx;
        }
        currCmd = cmd;
    }
    else if (cmd)
    {
        as3993SingleCommand(cmd);
        currCmd = cmd;
    }
    if (currCmd && waitTxIrq)
    {
        as3993WaitForResponse(RESP_TXIRQ);
        as3993ClrResponseMask(RESP_TXIRQ | RESP_FIFO);
    }
    if (rxbits && !rxs && !rxbuf)
    {   /* we do not want to receive any data */
        as3993SingleCommand(AS3993_CMD_BLOCKRX);
        buf[1] = 0;
        buf[0] = 0;
        as3993ContinuousWrite(AS3993_REG_RXLENGTHUP,buf,2);
    }
    if (rxbuf)
    {
        uint8_t count;
        uint16_t resp;
        if (0xff == norestime)
        {
            as3993WaitForResponseTimed(RESP_RXDONE_OR_ERROR | RESP_FIFO, 30);
        }
        else
        {
            as3993WaitForResponse(RESP_RXDONE_OR_ERROR | RESP_FIFO);
        }
        while (as3993GetResponse() & RESP_FIFO &&
            !(as3993GetResponse() & RESP_RXIRQ))
        {
            count = 18;
            if (checkRxLength && count > rxbytes)
            {
#if AS3993DEBUG
                //LOG("limiting1 %hhx %hhx resp %hx\n",count,rxbytes,as3993GetResponse());
#endif
                count = rxbytes;
            }
            as3993FifoRead(count, rxbuf);
            rxbuf += count;
            rxbytes -= count;
            rxed += count;
            as3993ClrResponseMask(RESP_FIFO);
            if (checkRxLength && rxbytes == 0)   //we do not want to receive more data
                return AS3993_ERR_RXCOUNT;
            as3993WaitForResponse(RESP_RXDONE_OR_ERROR | RESP_FIFO);
        }
        as3993WaitForResponse(RESP_RXDONE_OR_ERROR);
        resp = as3993GetResponse();
        as3993ClrResponse();
        if (followCmd && !(resp & (RESP_NORESINTERRUPT | RESP_RXERR)))
        {
            as3993SingleCommand(followCmd);
            currCmd = followCmd;
        }
        count = as3993SingleRead(AS3993_REG_FIFOSTATUS) & 0x1F; /*get the number of bytes */
        if (checkRxLength && count > rxbytes)
        {
#if AS3993DEBUG
            //LOG("limiting %hhx %hhx\n",count,rxbytes);
#endif
            count = rxbytes;
            resp |= RESP_RXCOUNTERROR;
        }
        if (count)
            as3993FifoRead(count, rxbuf);
        rxbytes -= count;
        rxed += count;
        if (rxbits)
            *rxbits = 8 * rxed;

#if RUN_ON_AS3980 || RUN_ON_AS3981   /* on AS3980/81 header IRQ is actually 2nd-byte IRQ -> ignore if no
                                        command which expects header bit was sent */
        if (currCmd != AS3993_CMD_TRANSMCRCEHEAD)
            resp &= ~RESP_HEADERBIT;
#endif
        if(resp & (RESP_NORESINTERRUPT | RESP_RXERR | RESP_HEADERBIT | RESP_RXCOUNTERROR))
        {
#if AS3993DEBUG
            if(resp & RESP_RXERR)
            {   //log errors (except no response)
                //LOG("rx error=%hx , rxed=%hhx , rxlen=%hx\n",resp, rxed);
                //LOGDUMP(rxbuf, rxed);
            }
#endif
            if (resp & RESP_NORESINTERRUPT)
                return AS3993_ERR_NORES;
            if (resp & RESP_HEADERBIT && currCmd == AS3993_CMD_TRANSMCRCEHEAD)
                return AS3993_ERR_HEADER;
            //if (resp & RESP_FIFOOVERFLOW)   return AS3993_ERR_FIFO_OVER;
            if (resp & RESP_RXCOUNTERROR)
            {
                if (!(resp & RESP_RXERR))   //as3980 produces irq_err2 (without irq_rxerr) if new epc is read 500ms after last one.
                {
                   // WaitForAS3980();        // Wait for tx to be re-enabled.
                }
                return AS3993_ERR_RXCOUNT;
            }
            if (resp & RESP_PREAMBLEERROR)  return AS3993_ERR_PREAMBLE;
            if (resp & RESP_CRCERROR)       return AS3993_ERR_CRC;
            return -1;
        }
    }
    return ERR_NONE;
}
