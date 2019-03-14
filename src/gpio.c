/*
 * gpio.c
 *
 *  Created on: Dec 12, 2018
 *      Author: Dan Walkes
 */
#include "gpio.h"
#include "log.h"
#include "em_gpio.h"
#include <string.h>



/**
 * TODO: define these.  See the radio board user guide at https://www.silabs.com/documents/login/user-guides/ug279-brd4104a-user-guide.pdf
 * and GPIO documentation at https://siliconlabs.github.io/Gecko_SDK_Doc/efm32g/html/group__GPIO.html
 */
#define	LED0_port gpioPortF
#define LED0_pin  (6)
#define LED1_port gpioPortF
#define LED1_pin  (5)

#define ButtonPort	gpioPortF
#define ButtonPin		(6)

void gpioInit()
{

	GPIO_PinModeSet(LED1_port, LED1_pin, gpioModePushPull, false);
	GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthStrongAlternateStrong);
	gpioLed1SetOff();

}

void Button0_Init(void)
{
	GPIO_PinModeSet(ButtonPort, ButtonPin, gpioModeInput, false);
	GPIO_IntClear(GPIO_IntGet());
	GPIO_IntConfig(ButtonPort, ButtonPin, true, true, true);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
}

void gpioLed0SetOn()
{
	GPIO_PinOutSet(LED0_port,LED0_pin);
}
void gpioLed0SetOff()
{
	GPIO_PinOutClear(LED0_port,LED0_pin);
}
void gpioLed1SetOn()
{
	GPIO_PinOutSet(LED1_port,LED1_pin);
}
void gpioLed1SetOff()
{
	GPIO_PinOutClear(LED1_port,LED1_pin);
}
void gpioEnableDisplay()
{
	GPIO_PinOutSet(gpioPortD, 15);//Check
}
void gpioSetDisplayExtcomin(bool high)
{
	if(high)
	{
		GPIO_PinOutSet(gpioPortD,13);
	}

	else
	{
		GPIO_PinOutClear(gpioPortD,13);
	}


}

