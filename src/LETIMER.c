/*
 * LETIMER.c
 *
 *  Created on: Jan 28, 2019
 *      Author: yashm
 */
#include "LETIMER.h"


const LETIMER_Init_TypeDef letimer=	//Structure
 {
 		.enable=true,
 		.debugRun=false,
 		.comp0Top=true,
 		.bufTop=false,
 		.out0Pol=false,
 		.out1Pol=false,
		.ufoa0= false,         /* PWM output on output 0 */
		.ufoa1= false,       /* PWM output on output 1*/
 		.repMode= letimerRepeatFree
  };

 uint8_t interrupt_flip=0;

void TimerInit(void)
{
	  CMU_OscillatorEnable(CLOCK_OSC,true,true);	//cmuOsc_LFXO
	  CMU_ClockSelectSet	(cmuClock_LFA,CLOCK_SEL);	//cmuSelect_ULFRCO


	  CMU_ClockEnable(cmuClock_CORELE, true);

	  CMU_ClockEnable(cmuClock_LETIMER0, true);
	  CMU_ClockDivSet(cmuClock_LETIMER0,CLOCK_DIV);	//Divide by 4
	  CMU_ClockEnable(cmuClock_GPIO, true);

	  LETIMER_Enable(LETIMER0,false);				//Disable the timer
	  LETIMER_Init(LETIMER0,&letimer);
	  LETIMER_CompareSet(LETIMER0,0,Comp0_Cal());	//Comp0 3.00s
	  LETIMER_IntSet(LETIMER0,LETIMER_IFC_COMP0);	//0x1->COMP0
	  NVIC_EnableIRQ(LETIMER0_IRQn);
	  LETIMER_IntEnable(LETIMER0,LETIMER_IFC_COMP0);//LETIMER_IFC_COMP0

	  LETIMER_Enable(LETIMER0,true);	//Enable the timer


 }


void SleepModeSel(void)
{
	switch(SLEEP_MODE)
	{
	case sleepEM0: while(1);
				   break;
	case sleepEM1: __WFI();
				   break;
	case sleepEM2: EMU_EnterEM2(true);
				   break;
	case sleepEM3: EMU_EnterEM3(false);
				   break;
	}
}

/*********************************************************************************
 * @func :	LETIMER0 IRQ Handler
 * @brief:	Sets an Event for scheduler to process in main loop
 *********************************************************************************/
void LETIMER0_IRQHandler(void)
{
	LETIMER_IntClear(LETIMER0,_LETIMER_IF_COMP0_MASK);
	Event=Set;
	LETIMER_CompareSet(LETIMER0,0,Comp0_Cal());
}


/************************************************************************
 * @func :	Initializes the Sensor, initiates 12C communication and takes
 * 			the reading.
 * @param:	None.
 ***********************************************************************/

void I2C_TempInit(void)
{
	  LETIMER_IntDisable(LETIMER0,_LETIMER_IF_COMP0_MASK); //Disable Interrupts

	  I2C_Startup();
	  timerWaitUs(80000);
	  I2CPM_TempMeasure();
	  I2C_ShutDown();
	  timerWaitUs(80000);

	  LETIMER_IntEnable(LETIMER0,_LETIMER_IF_COMP0_MASK);
}

/************************************************************************
 * @func :	Sets up the ports for SCl-SDA and Enable pin.
 * 			Selects appropriate pins and assigns them to the structure
 * 			I2CSPM_Init_TypeDef.
 * @param:	None
 * @return:	None
 ***********************************************************************/
void I2C_Startup(void)
{
	  I2CSPM_Init_TypeDef i2cInit=
	  {
			  .sclPin=10,
			  .sdaPin=11,
			  .portLocationScl=14,
			  .portLocationSda=16
	  };
	  I2CSPM_Init(&i2cInit);
	  GPIO_PinModeSet(gpioPortD, 15, gpioModePushPull, 1);	//Enable the I2C module
}


/*********************************************************************************
 * @func :	Measures the temperature
 * @brief:	Sets values for WRITE command to Sensor by modifying the structure
 * 			I2C_TransferSeq_TypeDef. Once I2CTransfer is done, READ command is
 * 			initiated.The final value is 16bits which is converted to float using
 * 			formula given in the datasheet.
 * @param:	None
 * @return:	None
 *********************************************************************************/
void I2CPM_TempMeasure(void)
{
	  uint16_t addr = (0x80);
	  uint8_t command = 0xE3;
	  I2C_TransferSeq_TypeDef    seq;
	  I2C_TransferReturn_TypeDef ret;
	  uint8_t                    i2c_read_data[2];
	  uint8_t                    i2c_write_data[1];
	  uint16_t data;
	  float temp;

	  seq.addr  = addr;
	  seq.flags = I2C_FLAG_WRITE;
	  // Select command to issue
	  i2c_write_data[0] = command;
	  seq.buf[0].data   = i2c_write_data;
	  seq.buf[0].len    = 1;

	  ret = I2CSPM_Transfer(I2C0, &seq);

	  if(ret!= i2cTransferDone)
	  {
		  LOG_ERROR("Error:%x during write\n",ret);
		  return;
	  }
	  seq.flags = I2C_FLAG_READ;
	  seq.buf[0].data   = i2c_read_data;
	  seq.buf[0].len    = 2;

	  ret = I2CSPM_Transfer(I2C0, &seq);

	  if(ret!=i2cTransferDone)
	  {
		  LOG_ERROR("Error:%x during read.\n",ret);
		  return;
	  }

	  data |= (i2c_read_data[0]<<8)|(i2c_read_data[1]) ;
	  temp=((175.72*data)/65535)-46.85;
	  LOG_INFO("%f",temp);


}
/*********************************************************************************
 * @func :	Disables the I2C pins
 * @brief:	GPIO_PinOutClear() is used to disable the SDA-SCL and Enable pins.
 * @param:	None
 * @return:	None
 *********************************************************************************/

void I2C_ShutDown(void)
{
		  GPIO_PinOutClear(gpioPortD,15);
		  GPIO_PinOutClear(gpioPortC,10);
		  GPIO_PinOutClear(gpioPortC,11);
}

/************************************************************************
 * @func :	Calculates the delay using LETIMER0 ticks as a reference.
 * @param:	Delay in micro-seconds.
 ***********************************************************************/

void timerWaitUs(uint32_t us_wait)	//80ms delay needed
{
		uint32_t ticks=LETIMER_CounterGet(LETIMER0);
		uint32_t counter = CounterGet(us_wait);				//Calculates the delay using LFA clock(#defined in LETIMER.h)
		while((ticks-LETIMER_CounterGet(LETIMER0))<counter);
}
