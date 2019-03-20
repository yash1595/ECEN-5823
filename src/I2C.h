/*
 * I2C.h
 *
 *  Created on: Mar 19, 2019
 *      Author: yashm
 */

#ifndef SRC_I2C_H_
#define SRC_I2C_H_

#include "LETIMER.h"

typedef enum {i2cisDone=0,TurnPowerOff=2,TurnOnPower=3,StartWrite=4,Idle=5,Incomplete=6,Sleep=7,Wait=8,Sleep1=9}Event;
typedef enum {PreWrite=0,PostWrite=1,Error=2}State;
State current;
Event event;

void I2C_Startup(void);

void I2C_TempConvertBLE(void);
/*********************************************************************************
 * @func :	Disables the I2C pins
 * @brief:	GPIO_PinOutClear() is used to disable the SDA-SCL and Enable pins.
 * @param:	None
 * @return:	None
 *********************************************************************************/

void I2C_ShutDown(void);

void Event_Handler(void);



#endif /* SRC_I2C_H_ */
