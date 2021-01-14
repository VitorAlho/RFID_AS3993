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
/** @file
  * @brief This file includes all functionality to use some hardware timers
  *
  * @author Ulrich Herrmann
  * @author Bernhard Breinbauer
  */

//#include <p24Fxxxx.h>
#include "p24FJ256DA210.h"
#include "Compiler.h"
#include "as3993_config.h"
#include "global.h"
#include "platform.h"
#include "timer.h"

void tick(void);
static volatile uint16_t slowTimerMsValue;

void slowTimerStart( )
{
    T3CONbits.TON = 0;
    slowTimerMsValue = 1;   //start with 1ms to get immediate stop at 0ms delays
    TMR3 = 0;
    _T3IF = 0;
    _T3IE = 1;
    T3CONbits.TON = 1;
}

uint16_t slowTimerValue( )
{
    return slowTimerMsValue;
}

void slowTimerStop( )
{
    _T3IE = 0;
    T3CONbits.TON = 0;
}

void timerInit()
{
    // Timer3 is slow-running timer used for timing of longer periods
    // prescaler 1:64, period 10ms
    T3CON = 0x00;
    T3CONbits.TON = 0;
    T3CONbits.TCKPS = 2;        // prescaler 1:64
    _T3IP = 2; // Timer3 interrupt priority 2 (low)
    PR3 = (SYSCLK / 64) / 100;
    // do not enable T3 interrupt here, they will be enabled in slowTimerStart()
}

void timer3Isr(void)	// interrupt handler for Timer3
{					// overflow
    _T3IF = 0;
    slowTimerMsValue += 10;             // increase ms counter by 10 as period is 10ms
}


void ligaTimer2 (void)
{
    T2CONbits.TON = 0;

    //T2CON = 0x0000;
    PR2 = 16000;
    TMR2 = 0;
    _T2IF = 0;
    _T2IE = 1;
    T2CONbits.TON = 1;

}


void INTERRUPT timer2Isr (void)    // interrupt handler for Timer2
{
    
    //tick();
    _T2IF = 0;
    //_LATC8 = !_LATC8;
}

