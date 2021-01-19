
/*
 ******************************************************************************
 * INCLUDES
 ******************************************************************************
 */

#include "gen2.h"
#include "as3993.h"
#include "as3993_public.h"
#include "appl_commands.h"
#include "string.h"
#include <limits.h>
#include <stdint.h>
#include "mcc_generated_files/system.h"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

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

/** Define this to 1 if you want to have logging for appl commands. */
//#define APPLDEBUG               0

/** Identifier for gen2 protocol session */
#define SESSION_GEN2            1
/** Identifier for ISO6B protocol session */
#define SESSION_ISO6B           2

/** Maximum of consecutive select commands currently supported. */
#define MAX_SELECTS             2

/** value for #readerPowerDownMode. Activates power down mode. (EN low)*/
#define POWER_DOWN              0
/** value for #readerPowerDownMode. Activates normal mode. (EN high, RF off, stdby 0)*/
#define POWER_NORMAL            1
/** value for #readerPowerDownMode. Activates normal mode with rf field on. (EN high, RF off, stdby 0)*/
#define POWER_NORMAL_RF         2
/** value for #readerPowerDownMode. Activates standby mode. (EN high, RF off, stdby 1)*/
#define POWER_STANDBY           3

/*
 ******************************************************************************
 * LOCAL VARIABLES
 ******************************************************************************
 */

/** Default Gen2 configuration, can be changed callConfigGen2(). */
static struct gen2Config gen2Configuration = {TARI_125, GEN2_LF_256, GEN2_COD_MILLER4, TREXT_OFF, 0, GEN2_SESSION_S0, 0};
/** Start value for Q for Gen2 inventory rounds, can be changed callConfigGen2(). */
uint8_t gen2qbegin = 4;

/** Internal number of currently configured select commands. */
static int num_selects;
/** Parameters for select commands, can be changed via callSelectTag()*/
static struct gen2SelectParams selParams[MAX_SELECTS];

/** Internal variable which contains the number of received tags in the last inventory round. */
static uint8_t num_of_tags;
/** New tag data has been received in last inventory round and can be sent in sendTagData() now.*/
static uint8_t tagDataAvailable;

/** AS3993 init status. This is the return value of as3993Initialize() */
//uint16_t readerInitStatus;

/** Array of Structures which stores all necessary Information about the Tags.
 */
Tag __attribute__((far)) tags_[MAXTAG];

/**Pointer to data of currently selected Tag.*/
Tag *selectedTag;

/**Contains the list of used frequencies.*/
Freq Frequencies;

/** If rssi measurement is above this threshold the channel is regarded as
    used and the system will hop to the next frequency. Otherwise this frequency is used */
static int8_t rssiThreshold;
/** Will be set to 1 if a cyclic inventory is performed, see callStartStop(). */
static uint8_t cyclicInventory;
/** If set to 0 inventory round commandos will be triggered by the FW, otherwise
 * the autoACK feature of the reader will be used which sends the required Gen2
 * commands automatically.*/
static uint8_t autoAckMode;
/** If set to 0 normal inventory round will be performed, if set to 1 fast inventory rounds will be performed.
 * The value is set in callStartStop() and callInventoryGen2().
 * For details on normal/fast inventory rounds see parameter singulate of gen2SearchForTags().*/
static uint8_t fastInventory;
/** Value for register #AS3993_REG_STATUSPAGE. This defines what RSSI value is sent
 * to the host along with the tag data. The value is set in callStartStop() and callInventoryGen2(). */
static uint8_t rssiMode;
/** Currently configured power down mode of reader. Available modes are: #POWER_DOWN, #POWER_NORMAL,
 *  #POWER_NORMAL_RF and #POWER_STANDBY */
static uint8_t readerPowerDownMode;
/** To be communicated to GUI, basically result of hopFrequencies(), having information on skipped rounds */
static int8_t inventoryResult;

/** Currently used protocol, valid values are: #SESSION_GEN2 and #SESSION_ISO6B. */
static uint8_t currentSession = 0;      // start with invalid session (neither Gen2 nor ISO 6b)

void insertBitStream(uint8_t *dest, uint8_t const *source, uint8_t len, uint8_t bitpos)
{
    int16_t i;
    uint8_t mask0 = (1<<bitpos)-1;
    uint8_t mask1 = (1<<(8-bitpos))-1;

    for (i=0; i<len; i++)
    {
        dest[i+0] &= (~mask0);
        dest[i+0] |= ((source[i]>>(8-bitpos)) & mask0);
        dest[i+1] &= ((~mask1) << bitpos);
        dest[i+1] |= ((source[i] & mask1) << bitpos);
    }
}

static uint8_t continueCheckTimeout( ) 
{
        
    return 1;
}

/** This funcition checks the current session, if necessary closes it
and opens a new session. Valid session values are: #SESSION_GEN2 and #SESSION_ISO6B. */
static void checkAndSetSession( uint8_t newSession)
{
    if (currentSession == newSession) return;
    switch (currentSession)
    {
        case SESSION_GEN2:
            gen2Close();
            break;
        case SESSION_ISO6B:

            break;
    }
    switch (newSession)
    {
        case SESSION_GEN2:
            gen2Open(&gen2Configuration);
            break;
        case SESSION_ISO6B:

            break;
    }
    currentSession = newSession;
}

void performSelects()
{
    int i;
    for (i = 0; i<num_selects; i++)
    {
        gen2Select(selParams + i);
        /* We would have to wait T4=2*RTcal here (max 140us). Logic analyzer showed enough time without delaying */
    }
}

/**
 * This function is called by callInventoryGen2() and doCyclicInventory() and performs
 * the actual Gen2 inventory round.
 */
void callInventoryGen2Internal()
{
    uint8_t foundTags;
    if (tagDataAvailable)       // wait until all tag data has been sent before starting next inventory round
    {
        return;
    }
    foundTags = inventoryGen2();
  
    //if (foundTags)ld_saidas (2, 1);

    tagDataAvailable = 1;
    //APPLOG("inventory Gen2, found tags: %hhx\n", foundTags);
}

uint8_t inventoryGen2(void)
{
    int8_t result;

    powerUpReader();
    as3993AntennaPower(1);
    delay_ms(1);
    
    result = 0;// hopFrequencies();
    
    if( !result )
    {
        checkAndSetSession(SESSION_GEN2);
        as3993SingleWrite(AS3993_REG_STATUSPAGE, rssiMode);
        if (rssiMode == RSSI_MODE_PEAK)      //if we use peak rssi mode, we have to send anti collision commands
            as3993SingleCommand(AS3993_CMD_ANTI_COLL_ON);

        num_of_tags = 0;
        //total_tags = 0;
        performSelects();
        if( !autoAckMode )
            num_of_tags = gen2SearchForTags(tags_, MAXTAG, gen2qbegin, continueCheckTimeout, fastInventory?0:1, 1);
        else
            num_of_tags = gen2SearchForTagsAutoAck(tags_, MAXTAG, gen2qbegin, continueCheckTimeout, fastInventory?0:1, 1);
        //total_tags = num_of_tags;
        if (rssiMode == RSSI_MODE_PEAK)      //if we use peak rssi mode, we have to send anti collision commands
            as3993SingleCommand(AS3993_CMD_ANTI_COLL_OFF);
    }
    inventoryResult = result;
    
    as3993AntennaPower(0);
    powerDownReader();    

    //APPLOG("end inventory, found tags: %hhx\n", num_of_tags);
    return num_of_tags;
}

void initCommands(void)
{
    currentSession = 0;
    cyclicInventory = 0;
    fastInventory = 0;
    autoAckMode = 1;
//    rssiMode = RSSI_MODE_PEAK;
    rssiMode = 0x06;    //rssi at 2nd byte

    rssiThreshold = -40;
    readerPowerDownMode = POWER_NORMAL;

    tagDataAvailable = 0;
    num_of_tags = 0;
    
    as3993AntennaPower(0);
    powerDownReader();
}

/**
 * This function is called periodically from main() loop. If a cyclic inventory has
 * been started callInventoryGen2Internal() is called.
 */
void doCyclicInventory(void)
{
    if (cyclicInventory)
    {
        callInventoryGen2Internal();
    }
}

/** Handles the configured power down mode of the reader. The power down mode
 * is define in readerPowerDownMode variable and can be changed via callReaderConfig().
 * Available modes are: #POWER_DOWN, #POWER_NORMAL, #POWER_NORMAL_RF and #POWER_STANDBY
 */
static void powerDownReader(void)
{
    switch (readerPowerDownMode)
    {
        case POWER_DOWN:
            as3993EnterPowerDownMode();
            break;
        case POWER_NORMAL:
            as3993EnterPowerNormalMode();
            break;
        case POWER_NORMAL_RF:
            as3993EnterPowerNormalRfMode();
            break;
        case POWER_STANDBY:
            as3993EnterPowerStandbyMode();
            break;
        default:
            as3993EnterPowerDownMode();
    }
}

/**
 * Handles power up of reader. Basically reverts changes done by powerDownReader().
 */
static void powerUpReader(void)
{
    switch (readerPowerDownMode)
    {
        case POWER_DOWN:
            as3993ExitPowerDownMode();
            break;
        case POWER_NORMAL:
            as3993ExitPowerNormalMode();
            break;
        case POWER_NORMAL_RF:
            as3993ExitPowerNormalRfMode();
            break;
        case POWER_STANDBY:
            as3993ExitPowerStandbyMode();
            break;
        default:
            as3993ExitPowerDownMode();
    }
}
