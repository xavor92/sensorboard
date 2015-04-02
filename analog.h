/*
 * analog.h
 *
 *  Created on: 03.04.2015
 *      Author: olli
 */

#ifndef ANALOG_H_
#define ANALOG_H_

/*
 *	Includes
 */
#include <avr/io.h>

/*
 *	Defines
 */


/*
 *	Global Functions (Declarations, extern)
 */

extern void ANALOG_Init();
extern void ANALOG_Service();
extern void ANALOG_Trigger();
extern unsigned int ANALOG_GetValues();

#endif /* ANALOG_H_ */
