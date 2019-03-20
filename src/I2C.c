#include "LETIMER.h"
#include "I2C.h"

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
	  GPIO_PinModeSet(gpioPortD, 15, gpioModePushPull, 1);
	  GPIO_PinOutSet(gpioPortD, 15);	  	  	  	  	  	  	  	  	  //Enable the I2C module
	  timerWaitUs(80000);//timerwaitus(80000);// event=StartWrite;
}

void I2C_TempConvertBLE(void)
{
	  uint8_t HTM_BUFF[5]; /* Stores the temperature data in the Health Thermometer (HTM) format. */
	  uint8_t HTM_flags = 0x00;   /* HTM flags set as 0 for Celsius, no time stamp and no temperature type. */
	  uint32_t temperature;   /* Stores the temperature data read from the sensor in the correct format */
	  uint8_t *p = HTM_BUFF; /* Pointer to HTM temperature buffer needed for converting values to bitstream. */

	    /* Convert flags to bitstream and append them in the HTM temperature data buffer (htmTempBuffer) */
	  UINT8_TO_BITSTREAM(p, HTM_flags);

	      /* Convert sensor data to correct temperature format */
	  temperature = FLT_TO_UINT32((temp), -3);

	  /* Convert temperature to bitstream andisplayLOG_INFO(DISPLAY_ROW_TEMPVALUE,"%f",(temp/1000));d place it in the HTM temperature data buffer (htmTempBuffer) */
	  UINT32_TO_BITSTREAM(p, temperature);

	  /* Send indication of the temperature in htmTempBuffer to all "listening" clients.
	       * This enables the Health Thermometer in the Blue Gecko app to display the temperature.
	       *  0xFF as connection ID will send indications to all connections. */
	  gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_Temperature, 5, HTM_BUFF);
	  displayPrintf(DISPLAY_ROW_TEMPVALUE,"%f",(temp/1000));
	          displayPrintf(DISPLAY_ROW_BTADDR2,"%02x:%02x:%02x:%02x:%02x:%02x",Server_Addr.addr[5],
        		Server_Addr.addr[4],
				Server_Addr.addr[3],
				Server_Addr.addr[2],
				Server_Addr.addr[1],
				Server_Addr.addr[0]);
	  //gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_tx_power_level, 5,rssi_value);
}
/*********************************************************************************
 * @func :	Disables the I2C pins
 * @brief:	GPIO_PinOutClear() is used to disable the SDA-SCL and Enable pins.
 * @param:	None
 * @return:	None
 *********************************************************************************/

void I2C_ShutDown(void)
{

		 // GPIO_PinOutClear(gpioPortD,15);
		  GPIO_PinOutClear(gpioPortC,10);
		  GPIO_PinOutClear(gpioPortC,11);


}

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
		//displayInit();
		I2C_Startup();
		//timerWaitUs(80000);
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
	  		//event=Sleep1;//EMU_EnterEM1();//EMU_EnterEM1();
	  	}
	  	/*****************************************************************************
	  	 * Post-Write - This is condition entered if the read transfer is completed
	  	 * 			   successfully. Once entered, the current state is changed to
	  	 * 			   Pre-Write.
	  	 *****************************************************************************/
	  	else if(current==PostWrite)
	  	{
	  		LOG_INFO("Entered PostWrite\n");
	  		CORE_DECLARE_IRQ_STATE;
			CORE_ENTER_CRITICAL();
	  		data |= (i2c_read_data[0]<<8)|(i2c_read_data[1]) ;
	  		CORE_EXIT_CRITICAL();
	  		temp=((175.72*data)/65535)-46.85;
	  		LOG_INFO("Temp:%f\n",temp);
	  		temp=temp*1000;
	  		current=PreWrite;
	  		//NVIC_DisableIRQ(I2C0_IRQn); //Must stay commented
	  		I2C_ShutDown();
	  		event_flag=1;//event=Sleep;
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
	  	//SLEEP_SleepBlockEnd(sleepEM2);
	  	//EMU_EnterEM3(false);
	  	break;

	  	/**************************************************************************
	  	* @event: Error - This event is called if I2C Transfer was complete but with
	  	* 		  errors. The system diables the I2C0 interrupts and enters EM3 mode.
	  	* @previous: i2cisDone or StartWrite.
	  	* @next:  The next state would be TurnOnPower.
	  	***************************************************************************/
	  	case Error:
	  	LOG_INFO("Entered Error\n");
	  	//NVIC_DisableIRQ(I2C0_IRQn);
	  	//event_flag=1;//event=Sleep;
	  	break;

	  	case Sleep1:
	  	LOG_INFO("Entered Sleep1\n");//EMU_EnterEM1();
	  	break;

	  	default : break;
	}
}

void I2C0_IRQHandler(void)
{
	ret = I2C_Transfer(I2C0);
	if(ret!=i2cTransferInProgress)
	{
		if(ret==i2cTransferDone)
			{
				LOG_INFO("ISR1");
				event = i2cisDone;
				Event_Mask |= WRITE_COMPLETE;
				gecko_external_signal(Event_Mask);
			}
		else
			{
				LOG_INFO("ISR2");
				event=Error;
			}
	}
	else
	{
		LOG_INFO("ISR3");
		event = Error;
		Event_Mask |= WRITE_COMPLETE;
		gecko_external_signal(Event_Mask);
	}

  NVIC_ClearPendingIRQ(I2C0_IRQn);

}
