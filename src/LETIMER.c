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
	  LETIMER_CompareSet(LETIMER0,0,Comp0_Cal());	//Comp0 1.00s
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
	case sleepEM1: EMU_EnterEM1();
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
		LOG_INFO("ENTER INTERRUPT\n");
		LETIMER_IntClear(LETIMER0,LETIMER_IF_COMP0);
//		LETIMER_CompareSet(LETIMER0,0,Comp0_Cal(1.0));
		event=TurnOnPower;
		Event_Mask |= DISP_UPDATE;
		gecko_external_signal(Event_Mask);
		++RollOver;
		if(RollOver%3 == 0){
			Event_Mask |= LETIMER_Triggered;
			gecko_external_signal(Event_Mask);
		}

	}

	else
	{
		LETIMER_IntClear(LETIMER0,LETIMER_IF_COMP1);
		//LETIMER_IntDisable(LETIMER0,_LETIMER_IF_COMP1_MASK);
		event=StartWrite;
		gecko_external_signal(LETIMER_Triggered);
		//SLEEP_SleepBlockBegin(sleepEM2);
	}


}


/************************************************************************
 * @func :	Initializes the Sensor, initiates 12C communication and takes
 * 			the reading.
 * @param:	None.
 ***********************************************************************/

void I2C_TempInit(void)
{
	  LETIMER_IntDisable(LETIMER0,LETIMER_IF_COMP0); //Disable Interrupts

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
	  GPIO_PinModeSet(gpioPortD, 15, gpioModePushPull, 1);
	  GPIO_PinOutSet(gpioPortD, 15);	  	  	  	  	  	  	  	  	  //Enable the I2C module
	  timerWaitUs(80000);//timerwaitus(80000);// event=StartWrite;
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
	  temp=(((175.72*data)/65535)-46.85)*1000;
	  LOG_INFO("%f",temp);



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
	  temperature = FLT_TO_UINT32(temp, -3);

	  /* Convert temperature to bitstream andisplayPrintf(DISPLAY_ROW_TEMPVALUE,"%f",(temp/1000));d place it in the HTM temperature data buffer (htmTempBuffer) */
	  UINT32_TO_BITSTREAM(p, temperature);

	  /* Send indication of the temperature in htmTempBuffer to all "listening" clients.
	       * This enables the Health Thermometer in the Blue Gecko app to display the temperature.
	       *  0xFF as connection ID will send indications to all connections. */
	  gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_Temperature, 5, HTM_BUFF);
	  displayPrintf(DISPLAY_ROW_TEMPVALUE,"%f",(temp/1000));
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
		if(ticks>counter)
		{
			LETIMER_CompareSet(LETIMER0,1,counter);				//COMP1 set to calculated counter value

		}
		else
		{
			LETIMER_CompareSet(LETIMER0,1,(Comp0_Cal()-counter-ticks));
		}
		NVIC_EnableIRQ(LETIMER0_IRQn);						//Enabling the LETIMER interrupts
		LETIMER_IntSet(LETIMER0,LETIMER_IFC_COMP0|LETIMER_IFC_COMP1);	//Setting the interupt for COMP1
		LETIMER_IntEnable(LETIMER0,LETIMER_IFC_COMP0|LETIMER_IFC_COMP1);//Enabling interrupt for COMP1.
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
	  		data |= (i2c_read_data[0]<<8)|(i2c_read_data[1]) ;
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
				//gecko_external_signal(ERROR_I2C);
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

void Init_Globals(void)
{
	event = Idle;
	current = PreWrite;
	addr = 0x80;
	command = 0xE3;
	SleepMode = sleepEM0;
	RollOver = 0;
	TimeoutVal=300;
	MinConnTime=60;
	MaxConnTime=60;
	SlaveLatency=3;
	Notifications_Status = 0;
	Event_Read = 0;
	LETIMER_Triggered = 1;
	BG13_Min_Power = -260;
	BG13_Max_Power = 80;
	Active_Connection=0;
	event_flag=0;
	WRITE_COMPLETE = 2;
	//ERROR_I2C = 4;
	I2C_INCOMPLETE = 4;
	count_write=0;
	//already_initiated=0;

}



void timerEnable1HzSchedulerEvent(uint32_t Scheduler_DisplayUpdate)
{
	DISP_UPDATE = Scheduler_DisplayUpdate;
}

void gecko_custom_update(struct gecko_cmd_packet* evt)
{
	gecko_update(evt);
	switch(BGLIB_MSG_ID(evt->header))
	{
/******************************************************************************
 @brief : This event is triggered once the BLE Stack is initiated.
 @func  : Power is set to 0. Advertising intervals are setup. The BLE address is
 		  obtained.
******************************************************************************/

		case gecko_evt_system_boot_id:
			gecko_cmd_system_set_tx_power(0);

			gecko_cmd_le_gap_set_advertise_timing(0, 400, 400, 0, 0);

			gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);
			displayPrintf(DISPLAY_ROW_CONNECTION,"Advertising");
			displayPrintf(DISPLAY_ROW_NAME,"Server");
			AddressBLE = gecko_cmd_system_get_bt_address();
			displayPrintf(DISPLAY_ROW_BTADDR,"%02x:%02x:%02x:%02x:%02x:%02x",AddressBLE->address.addr[5],
						AddressBLE->address.addr[4],
						AddressBLE->address.addr[3],
						AddressBLE->address.addr[2],
						AddressBLE->address.addr[1],
						AddressBLE->address.addr[0]);

			LOG_INFO("Gecko BLE Setup complete\n");
			break;
/******************************************************************************
 @brief : This event is triggered when the client
 @func  : The LETIMER is diabled so that no interrupt triggers at 1s intervals.
******************************************************************************/

		case gecko_evt_le_connection_opened_id:

			ConnectionHandle = evt->data.evt_le_connection_opened.connection;

			gecko_cmd_le_connection_set_parameters(ConnectionHandle, MinConnTime, MaxConnTime,SlaveLatency,TimeoutVal);
			displayPrintf(DISPLAY_ROW_CONNECTION,"Connected");
			displayPrintf(DISPLAY_ROW_NAME,"Server");
			LOG_INFO("Gecko BLE Connection start\n");
			break;

/******************************************************************************
 @brief : This event is triggered when the client ends the connection.
 @func  : The LETIMER is diabled so that no interrupt triggers at 1s intervals.
******************************************************************************/

		case gecko_evt_le_connection_closed_id:

			Active_Connection = 0;

			gecko_cmd_system_set_tx_power(0);

			LETIMER_Enable(LETIMER0, false);

			gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);
			LOG_INFO("Gecko BLE Connection end\n");
			displayPrintf(DISPLAY_ROW_CONNECTION,"Advertising");
			displayPrintf(DISPLAY_ROW_NAME,"Server");
			displayPrintf(DISPLAY_ROW_TEMPVALUE,"%s"," ");
			break;

/******************************************************************************
 @brief : This event is triggered when the client asks for the server character-
 			istic ID.
 @func : If a successful response from client is received Active_Connection = 1.
******************************************************************************/

		case gecko_evt_gatt_server_characteristic_status_id:
			LOG_INFO("status_flags=%d",evt->data.evt_gatt_server_characteristic_status.status_flags);
			displayPrintf(DISPLAY_ROW_CONNECTION,"Handling Indications");

			if((evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_Temperature)
			    && (evt->data.evt_gatt_server_characteristic_status.status_flags == 0x01))
			    {

				if(evt->data.evt_gatt_server_characteristic_status.client_config_flags == 0x02)
			    	{
						LOG_INFO("Interrupt not Stopped\n");
						Active_Connection = 1;
			    		LETIMER_Enable(LETIMER0, true);
			    		LETIMER_IntEnable(LETIMER0,LETIMER_IF_COMP0);
			    	}

			    }

			    break;
/******************************************************************************
 @brief : This event is triggered when an interrupt occurs.
		  An Event Mask is set to select the source of interrupt.
 @func : Handles interrupts from LETIMER and I2C interrupts.
******************************************************************************/

		case gecko_evt_system_external_signal_id:
			    LOG_INFO("External event occured\n");
			    if(Active_Connection == 1)
			    {

			    	Event_Read = evt->data.evt_system_external_signal.extsignals;
			    	if(Event_Read & LETIMER_Triggered )
			    	{
			    			/* Begin Critical Section */
			    			CORE_DECLARE_IRQ_STATE;
			    			CORE_ENTER_CRITICAL();
			    				Event_Mask &= ~LETIMER_Triggered ; //Clear the Event Mask
			    			CORE_EXIT_CRITICAL();
			    			/* End Critical Section */

			    			Event_Handler();
			    			LOG_INFO("3s call\n");
			    	}
			    	if(Event_Read & WRITE_COMPLETE)
			    	{
			    		/* Begin Critical Section */
			    		CORE_DECLARE_IRQ_STATE;
			    		CORE_ENTER_CRITICAL();
			    			Event_Mask &= ~WRITE_COMPLETE;			//Clear the Event Mask
			    		CORE_EXIT_CRITICAL();
			    		/* End Critical Section */

			    		LOG_INFO("Write\n");

			    		Event_Handler();
			    		if(event_flag==1)
			    		{
			    			I2C_TempConvertBLE();
			    			LETIMER_Enable(LETIMER0, true);

			    		}
			    		gecko_cmd_le_connection_get_rssi(ConnectionHandle);

			    	}

			    	if(Event_Read & DISP_UPDATE)	//8
			    	{
			    		/* Begin Critical Section */
			    		CORE_DECLARE_IRQ_STATE;
			    		CORE_ENTER_CRITICAL();
			    			Event_Mask &= ~DISP_UPDATE;				//Clear the Event Mask
			    		CORE_EXIT_CRITICAL();
			    		/* End Critical Section */
			    		displayUpdate();


			    		LOG_INFO("Disp update 1s\n");
			    	}

			    	else Event_Handler();

			    }


			    break;
/******************************************************************************
 @brief : This event is triggered when
 		  gecko_cmd_le_connection_get_rssi(ConnectionHandle); is called.
 @func  : provides the rssi strength of the connected client
******************************************************************************/

		case gecko_evt_le_connection_rssi_id:
			    LOG_INFO("RSSI \n");
	    		// Get RSSI Value
	    		rssi_value = evt->data.evt_le_connection_rssi.rssi;
	    		LOG_INFO("%d",rssi_value);

	    		// Halt the BT module to change the TX power
	    		gecko_cmd_system_halt(1);

	    		if(rssi_value > -35)
	    		{

	    			gecko_cmd_system_set_tx_power(BG13_Min_Power);
	    			LOG_INFO(" rssi_value > -35\n");

	    		}

	    		else if(-35 >= rssi_value && rssi_value > -45)
	    		{
	    			gecko_cmd_system_set_tx_power(-200);
	    			LOG_INFO(" rssi_value > -45\n");
	    		}


	    		else if(-45 >= rssi_value && rssi_value > -55)
	    		{
	    			gecko_cmd_system_set_tx_power(-150);
	    			LOG_INFO(" rssi_value > -55\n");
	    		}

	    		else if(-55 >= rssi_value && rssi_value > -65)
	    		{
	    			gecko_cmd_system_set_tx_power(-50);
	    			LOG_INFO(" rssi_value > -65\n");
	    		}

	    		else if(-65 >= rssi_value && rssi_value > -75)
	    		{
	    			gecko_cmd_system_set_tx_power(0);
	    			LOG_INFO(" rssi_value > -75\n");
	    		}


	    		else if(-75 >= rssi_value && rssi_value > -85)
	    		{
	    			gecko_cmd_system_set_tx_power(50);
	    			LOG_INFO(" rssi_value > -85\n");
	    		}


	    		else
	    		{

	    			gecko_cmd_system_set_tx_power(BG13_Max_Power);
	    			LOG_INFO(" rssi_value largest\n");

	    		}


	    		gecko_cmd_system_halt(0);

	    		break;

	}
}


