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
 *  \author U.Herrmann ( based on work by E.Grubmueller )
 *  \author T. Luecker (Substitute)
 *  \author B. Bernhard
 *
 *   \brief Configuration file for all AS99x firmware
 *
 *   Do configuration of the driver in here.
 */
#include "global.h"

#ifndef _AS3993_CONFIG_H
#define _AS3993_CONFIG_H

/***************************************************************************/
/********************** configuration area *********************************/
/***************************************************************************/

/** Define this to 1 if you have a FERMI board */
#define FERMI 1

/** Define this to 1 if you have an (ams internal) eval board */
#define EVALBOARD 0

/** Define this to 1 if you have a FEMTO v2 board */
//#define FEMTO2 0

/** Define this to 1 if you have a FEMTO v2.1 board */
#define FEMTO2_1 0

/** Define this to 1 if you have a MEGA board */
#define MEGA 0

/** Define this to 1 if you have a RADON board */
#define RADON 0

/** Define this to 1 to enable AS3993 support. */
#define RUN_ON_AS3993 1

/** Define this to 1 to enable AS3994 support. */
#define RUN_ON_AS3994 0

/** Define this to 1 to enable AS3980 support. */
#define RUN_ON_AS3980 0

/** Define this to 1 to enable AS3980 support. */
#define RUN_ON_AS3981 0

/** Define this to 1 if as3993Initialize() should perform a proper selftest,
  testing connection AS3993, crystal, pll, antenna */
#define AS3993_DO_SELFTEST 1

/** Define this to 1 if you want debug messages in as3993.c or want to have
 bebug output on OAD pins*/
#define AS3993DEBUG 0

/** If set to 1 reader intialization will be more verbose. */
#define VERBOSE_INIT 0

/** Set this to 1 to enable iso6b support. Not supported atm. */
#define ISO6B 0

/** Baudrate used if uart communication is enabled at compile time. */
#define BAUDRATE 115200UL

/***************************************************************************/
/******************** private definitions, not to be changed ***************/
/***************************************************************************/
#define EVAL_HW_ID          1
#define FERMI_HW_ID         2
#define FERMI_BIG_HW_ID     3
#define FEMTO_1V0_HW_ID     4
#define FEMTO_1V1_HW_ID     5
#define FEMTO_2V0_HW_ID     6
#define MEGA_HW_ID          7
#define RADON_HW_ID         8
#define FEMTO_2V1_HW_ID     9

#if RUN_ON_AS3993
#define CHIP                "AS3993"
#endif

#define FIRMWARE_ID         CHIP" Reader Firmware"
#define FIRMWARE_VERSION    0x020208UL;

#if FERMI
#define HARDWARE_ID         CHIP" FERMI Reader Hardware 1.0"
#define HARDWARE_ID_NUM     FERMI_HW_ID
#endif

#define WRONG_CHIP_ID       "caution wrong chip"

/** define to identify Cin cap of tuner. */
#define TUNER_CIN       0x01
/** define to identify Clen cap of tuner. */
#define TUNER_CLEN      0x02
/** define to identify Cout cap of tuner. */
#define TUNER_COUT      0x04

/* Set the following configuration switches depending on the board setup:
 * INTVCO or EXTVCO
 * INTPA or EXTPA
 * SINGLEINP or BALANCEDINP
 * ANTENNA_SWITCH if 2 antenna ports are available
 * TUNER if antenna tuning is available
 * TUNER_CONFIG configures which tuner caps are available (bitmask)
*/

#if FERMI
#define INTVCO
#define EXTPA
#define BALANCEDINP
//#define ANTENNA_SWITCH
#define TUNER
#define TUNER_CONFIG (TUNER_CIN | TUNER_CLEN | TUNER_COUT)

#endif
#endif
