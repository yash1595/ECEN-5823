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
		.ufoa0= letimerUFOAPulse,//false,         /* PWM output on output 0 */
		.ufoa1= letimerUFOAPulse,//false,       /* PWM output on output 1*/
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
	 // LETIMER_RepeatSet(LETIMER0,0,0X01);
	 // LETIMER_RepeatSet(LETIMER0,1,0X01);
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
	if(LETIMER_IntGet(LETIMER0)&LETIMER_IFC_COMP0)
	{
		LETIMER_IntClear(LETIMER0,_LETIMER_IF_COMP0_MASK);
		event=TurnOnPower;
		LETIMER_CompareSet(LETIMER0,0,Comp0_Cal());
		SLEEP_SleepBlockBegin(sleepEM2);
		++RollOver;
	}

	else
	{
		LETIMER_IntClear(LETIMER0,_LETIMER_IF_COMP1_MASK);
		//LETIMER_IntDisable(LETIMER0,_LETIMER_IF_COMP1_MASK);
		event=StartWrite;
		SLEEP_SleepBlockBegin(sleepEM2);
	}


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
		if(us_wait==0)counter=1;							//Ensures that if us_wait is 0, the delay runs for atleast 1 clock cycle
		NVIC_DisableIRQ(LETIMER0_IRQn);						//Disabling interrupts before setting value in the COMP1 register
		LETIMER_CompareSet(LETIMER0,1,counter);				//COMP1 set to calculated counter value
		LETIMER_IntSet(LETIMER0,LETIMER_IFC_COMP0|LETIMER_IFC_COMP1);	//Setting the interupt for COMP1
		LETIMER_IntEnable(LETIMER0,LETIMER_IFC_COMP0|LETIMER_IFC_COMP1);//Enabling interrupt for COMP1.
		NVIC_EnableIRQ(LETIMER0_IRQn);						//Enabling the LETIMER interrupts
		event=Sleep;										//Event is Sleep. Remains in Sleep till COMP1 interrupt triggers
		/*	if(counter>0)
		{
			while((ticks-LETIMER_CounterGet(LETIMER0))<counter);
		}
	*/
}
/******************************************************************/
void Event_Handler(void)
{
	switch(event)
	{
		/**************************************************************************
		 * @event: TurnOnPower-This event initializes the I2C using I2CSPM_Init().
		 *         It then triggers the TimerWaitUs() function
		 * @previous: The previous state is Idle.
		 * @next:  The state transitioned from this state is into Sleep till the COMP1
		 * 		   interrupt triggers
		 **************************************************************************/
		case TurnOnPower:
		LOG_INFO("Time Stamp:%d\t",loggerGetTimestamp(RollOver));	//Provides the current time
		LOG_INFO("Entered TurnOnPower state\n");
		I2C_Startup();
		timerWaitUs(80000);
		break;

		/**************************************************************************
		 * @event: StartWrite-This event initiates the write command to the I2C temperature
		 * 		   sensor.
		 * @previous: It transitions here from Sleep state called by the TurnOnPower event.
		 * 			  After an ISR for COMP1 triggers, the event is set to StartWrite.
		 * @next:  The next state is determined by the 12C ISR return value.
		 **************************************************************************/
		case StartWrite:
		LOG_INFO("Time Stamp:%d\t",loggerGetTimestamp(RollOver));
		LOG_INFO("Entered StartWrite state\n");
		current = PreWrite;
	  	seq.addr  = addr;
	  	seq.flags = I2C_FLAG_WRITE;
	  	i2c_write_data[0] = command;
	  	seq.buf[0].data   = i2c_write_data;
	  	seq.buf[0].len    = 1;
	  	NVIC_EnableIRQ(I2C0_IRQn);
	  	ret = I2C_TransferInit(I2C0, &seq);
	  	event=Sleep1;//EMU_EnterEM1();
	  	break;

	  	/**************************************************************************
	  	* @event: i2cisDone-This event is called by the I2C ISR if the I2C Transfer was
	  	* 		  completed. There are 2 cases for I2CTransfer complete, Pre-Write and
	  	* 		  Post-Write.
	  	* @previous: StartWrite was the previous event if the I2C returned i2cisDone if
	  	* 			 not the previous event would be Incomplete
	  	* @next:  The next state is determined by the 12C ISR return value.
	  	***************************************************************************/
	  	case i2cisDone:
	  	LOG_INFO("Time Stamp:%d\t",loggerGetTimestamp(RollOver));
	  	LOG_INFO("Entered i2cisDone state\n");
	  	/*****************************************************************************
	  	 * Pre-Write - This is condition entered if the write transfer is completed
	  	 * 			   successfully. Once entered, the current state is changed to
	  	 * 			   Post-Write.
	  	 *****************************************************************************/
	  	if(current==PreWrite)
	  	{
	  		LOG_INFO("Entered PreWrite\n");
	  		current=PostWrite;
	  		seq.flags = I2C_FLAG_READ;
	  		seq.buf[0].data   = i2c_read_data;
	  		seq.buf[0].len    = 2;
	  		ret = I2C_TransferInit(I2C0,&seq);	//Initiate Read
	  		event=Sleep1;//EMU_EnterEM1();//EMU_EnterEM1();
	  	}
	  	/*****************************************************************************
	  	 * Post-Write - This is condition entered if the read transfer is completed
	  	 * 			   successfully. Once entered, the current state is changed to
	  	 * 			   Pre-Write.
	  	 *****************************************************************************/
	  	else if(current==PostWrite)
	  	{
	  		LOG_INFO("Entered PostWrite\n");
	  		data |= (i2c_read_data[0]<<8)|(i2c_read_data[1]) ;
	  		temp=((175.72*data)/65535)-46.85;
	  		LOG_INFO("Temp:%f\n",temp);
	  		current=PreWrite;
	  		I2C_ShutDown();
	  		NVIC_DisableIRQ(I2C0_IRQn);
	  		event=Sleep;
	  	}
	  	break;

	  	/**************************************************************************
	  	* @event: Idle-This event is the first at the time of initialization.
	  	* @previous: There is no previous state for this
	  	* @next:  The next event is TurnOnPower. Called by the COMP0 ISR
	  	***************************************************************************/
	  	case Idle:
	  	LOG_INFO("Time Stamp:%d\t",loggerGetTimestamp(RollOver));
	  	LOG_INFO("Entered Idle State\n");
	  	break;

	  	/**************************************************************************
	  	* @event: Incomplete - This event is called if an I2C transfer is incomplete.
	  	* @previous: StartWrite would be the previous state(ideally) or Incomplete
	  	* 			 could be the previous state if I2C ISR had returned Incomplete
	  	* 			 previously.
	  	* @next:  The next event is determined by I2C ISR Handler. If the transfer is
	  	* 		  successful the next event will be i2cisDone. If not, it would be
	  	* 		  incomplete or Error
	  	***************************************************************************/
	  	case Incomplete:
	  	LOG_INFO("Time Stamp:%d\t",loggerGetTimestamp(RollOver));
	  	LOG_INFO("Entered Incomplete\n");
	  	//EMU_EnterEM1();
	  	break;
	  	/**************************************************************************
	  	* @event: Sleep-This event enters the EM3 sleep mode.
	  	* @previous: Can be invoked from either from TurnOnPower or from i2cisDone.
	  	* @next:  The next state if invoked from TurnOnPower(COMP1 ISR), will be StartWrite.
	  	*         If invoked from i2cisDone the next state would be TurnOnPower.
	  	***************************************************************************/
	  	case Sleep:
	  	LOG_INFO("Time Stamp:%d\t",loggerGetTimestamp(RollOver));
	  	SLEEP_SleepBlockEnd(sleepEM2);
	  	EMU_EnterEM2(true);
	  	break;

	  	/**************************************************************************
	  	* @event: Error - This event is called if I2C Transfer was complete but with
	  	* 		  errors. The system diables the I2C0 interrupts and enters EM3 mode.
	  	* @previous: i2cisDone or StartWrite.
	  	* @next:  The next state would be TurnOnPower.
	  	***************************************************************************/
	  	case Error:
	  	LOG_INFO("Entered Error\n");
	  	NVIC_DisableIRQ(I2C0_IRQn);
	  	event=Sleep;
	  	break;

	  	case Sleep1:
	  	EMU_EnterEM1();
	  	break;

	  	default : break;
	}
}

/**************************************************************************************
 * @irq: Checks if transfer is complete. If not, sets event= incomplete.
 * 		 This causes the system to enter EM1. If the transfer is complete it sets event,
 * 		 I2cisDone or Error depending on whether the transfer returned 0 or not.
 ************************************************************8**************************/
void I2C0_IRQHandler(void)
{
	ret = I2C_Transfer(I2C0);
	if(ret!=i2cTransferInProgress)
	{
		if(ret==i2cTransferDone)
		event = i2cisDone;
		else event=Error;
	}
	else
	{
		event = Sleep1;
	}

  NVIC_ClearPendingIRQ(I2C0_IRQn);

}

void Init_Globals(void)
{
	event = Idle;
	current = PreWrite;
	addr = 0x80;
	command = 0xE3;
	SleepMode = sleepEM0;
	RollOver = 0;
}

/*void LoggerTimeStamp(void)
{
	LOG_INFO("Time Stamp:%f\t",(float)((3.0*RollOver)+(((Comp0_Cal()-LETIMER_CounterGet(LETIMER0))*(4.0))/32768)));
}*/
