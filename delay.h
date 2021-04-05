
/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef _DELAY_H_
#define	_DELAY_H_

#include <xc.h> // include processor files - each processor file is guarded.  

 void delay_ms(uint16_t delay);
 
 void delay_us(uint16_t delay);

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* _DELAY_H_ */

