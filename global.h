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
 * FOR A PARTICULAR 
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
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
  * @brief This file provides declarations for global helper functions.
  *
  * @author Ulrich Herrmann
  */


//#define INTERFACE_DE_AJUSTES 1 //Para ajustes com o software AjustesMaisPortalMI.py

//#define SANKYU 1//CLIENTE
#define ARCELOR 1//CLIENTE
//#define BRUNAUER 1//CLIENTE
//#define CONTROL_PLUS 1//CLIENTE 

#ifdef SANKYU

//#define SANKYU_SAO_PAULO 1//LUGAR
#define SANKYU_JOAO_MONLEVADE 1//LUGAR

#ifdef SANKYU_JOAO_MONLEVADE
// **** DEFINE A SER USADO **** //
#define SANKYU_JOAO_MONLEVADE_EMPILHADEIRA_7_5_TONS 1
#elif SANKYU_SAO_PAULO
// **** DEFINE A SER USADO **** //
#define SANKYU_SAO_PAULO_EMPILHADEIRA_2_5_TONS 1
#endif




#elif ARCELOR //CLIENTE

#define JUIZ_DE_FORA  1//LUGAR
#ifdef JUIZ_DE_FORA

//#define PORTAL 1//PRODUTO
#define EMPILHADEIRA 1//PRODUTO

#ifdef PORTAL
#define DUAS_ANTENAS 1//CONFIGURACAO
#define QUATRO_ANTENAS 1//CONFIGURACAO
#ifdef DUAS_ANTENAS
// **** DEFINE A SER USADO **** //
#define ARCELOR_JUIZ_DE_FORA_PORTAL_DUAS_ANTENAS 1
#elif QUATRO_ANTENAS
// **** DEFINE A SER USADO **** //
#define ARCELOR_JUIZ_DE_FORA_PORTAL_QUATRO_ANTENAS 1
#endif

#elif EMPILHADEIRA
// **** DEFINE A SER USADO **** //
#define ARCELOR_JUIZ_DE_FORA_EMPILHADEIRA_TRADMAQ_3_5_TONS 1
#endif

#endif

#elif BRUNAUER //CLIENTE
#define JOAO_MONLEVADE 1 //LUGAR 
#ifdef JOAO_MONLEVADE
// **** DEFINE A SER USADO **** //
#define BRUNAUER_JOAO_MONLEVADE_EMPILHADEIRA_2_5_TONS 1
#endif 

#elif CONTROL_PLUS
#define CONTROL_PLUS_AUTOTESTE
#define PORTAL_WIEGAND
#define PORTAO_RFID
#endif 

//#define EMPILHADEIRA

//#define BARBACENA 1


#ifndef __GLOBAL_H__
#define __GLOBAL_H__

/** Defines uC used clock frequency */
#define SYSCLK_16MHZ                16000000ULL
#define SYSCLK_12MHZ                12000000ULL
#define SYSCLK_8MHZ                 8000000ULL
#define SYSCLK_4MHZ                 4000000ULL
#if 1
#define SYSCLK    SYSCLK_16MHZ
#else
#define SYSCLK    tbd
#endif
#define FCY    (SYSCLK)

#include "as3993_config.h"
#include "ams_types.h"

/** Definition high */
#define HIGH                      1

/** Definition all bits low */
#define LOW                       0

#define BIT0	0x01
#define BIT1	0x02
#define BIT2	0x04
#define BIT3	0x08
#define BIT4	0x10
#define BIT5	0x20
#define BIT6	0x40
#define BIT7	0x80

void bin2Chars(int value, unsigned char *destbuf);
void bin2Hex(char value, unsigned char *destbuf);
void u32ToEbv(u32 value, u8 *ebv, u8 *len);
void insertBitStream(u8 *dest, u8 const *source, u8 len, u8 bitpos);
u32 readU32FromLittleEndianBuffer(u8 const *buffer);
void recebeOperador (u8 dado);
//void zeraPonteiroDoBufferNaMaquinaDeEstados_EventosDeParadaes (void);
void bloqueia_frente(void);
void bloqueia_reh(void);
void ld_saidas(int saida, int status);
void sel_led(int led, int status);
//void escolhe_antena(void);

extern volatile int operador_atual_outro;


/** Definition for the maximal EPC length */
#define EPCLENGTH              12 //32  // number of bytes for EPC, standard allows up to 62 bytes
/** Definition for the PC length */
#define PCLENGTH                2
/** Definition for the CRC length */
#define CRCLENGTH               2
/** Definition of the maximum frequencies for the hopping */
#define MAXFREQ                 53
/** Definition of the maximum tune entries in tuning table */
#define MAXTUNE                 52
/** Definition of the maximum number of tags, which can be read in 1 round */
//#define MAXTAG 255
#define MAXTAG 12//20//120

#endif
