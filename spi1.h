/* 
 * File:   spi1.h
 * Author: Vitor
 *
 * Created on 8 de Maio de 2020, 14:14
 */

#ifndef SPI1_H
#define	SPI1_H

#ifdef	__cplusplus
extern "C" {
#endif

#ifndef FCY
    #define FCY 16000000ULL
#endif
    
#include <xc.h>
#include <libpic30.h>
    
void InitSPI1(void);
int WriteReadSPI1(const unsigned char *txdata, unsigned char *rxdata, unsigned int lenght);

#ifdef	__cplusplus
}
#endif

#endif	/* SPI1_H */

