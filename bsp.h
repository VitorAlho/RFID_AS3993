/* 
 * File:   bsp.h
 * Author: Vitor
 *
 * Created on 11 de Maio de 2020, 13:04
 */

#ifndef BSP_H
#define	BSP_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <xc.h>
//#include <PPS.h>

    void bspInit(void);
    
    void AS3993_Enable(void);
    void AS3993_Disable(void);
    void AS3993_EnableSpiSlaveSelect(void);
    void AS3993_DisableSpiSlaveSelect(void);
    
#ifdef	__cplusplus
}
#endif

#endif	/* BSP_H */

