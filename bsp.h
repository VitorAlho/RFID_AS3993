
/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef _BSP_H_
#define	_BSP_H_

#include <xc.h> // include processor files - each processor file is guarded.  

///////////////////////////////////////////////////
// REGION: Hardware resources to use AS3993
///////////////////////////////////////////////////

 void EX_INT1_CallBack(void);               // External interrupt used by AS3993
 
 void enableExtInterrupt(void);             // Enable external interrupt
 
 void disableExtInterrupt(void);            // Disable external interrupt
 
 void clearExtInterrupt(void);              // Clear external interrupt flag
 
 void setAS3993_enablePin(uint8_t val);     // AS3993 enable pin
 
 uint8_t AS3993_isEnabled(void);            // Get AS3993 enable pin state
 
 void setAS3993_SPI_enablePin(uint8_t val); // AS3993 SPI enable pin
 
///////////////////////////////////////////////////
// END REGION: Hardware resources to use AS3993
///////////////////////////////////////////////////

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* _BSP_H_ */

