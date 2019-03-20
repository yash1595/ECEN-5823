#include "LETIMER.h"
#include "BLE.h"
#include "I2C.h"

 uint8_t thermoService[2];
 uint8_t thermoChar[2];
 uint8_t ButtonService[16];
 uint8_t ButtonChar[16];
 uint8_t ConnectionHandle;

//char* ButtonPressString[]={"Button Released","Button Pressed"};

//typedef struct {
//  uint8_t  connectionHandle;
//  int8_t   rssi;
//  uint16_t serverAddress;
//  uint32_t thermometerServiceHandle;
//  uint16_t thermometerCharacteristicHandle;
//  uint32_t temperature;
//  uint32_t ButtonServiceHandle;
//  uint16_t ButtonCharacteristicHandle;
//  uint32_t Button;
//}ConnProperties;
//
//ConnProperties connProperties;
const LETIMER_Init_TypeDef letimer=	//Structure
 {
 		.enable=true,
 		.debugRun=false,
 		.comp0Top=true,
 		.bufTop=false,
 		.out0Pol=false,
 		.out1Pol=false,
		.ufoa0= false,//false,         /* PWM output on output 0 */
		.ufoa1= false,//false,       /* PWM output on output 1*/
 		.repMode= letimerRepeatFree
  };

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
	  LETIMER_CompareSet(LETIMER0,0,Comp0_Cal());	//Comp0 1.00s
	  LETIMER_IntSet(LETIMER0,LETIMER_IFC_COMP0);	//0x1->COMP0
	  NVIC_EnableIRQ(LETIMER0_IRQn);
	  LETIMER_IntEnable(LETIMER0,LETIMER_IFC_COMP0);//LETIMER_IFC_COMP0

	  LETIMER_Enable(LETIMER0,true);	//Enable the timer

 }

/*********************************************************************************
 * @func :	LETIMER0 IRQ Handler
 * @brief:	Sets an Event for scheduler to process in main loop
 *********************************************************************************/
void LETIMER0_IRQHandler(void)
{
	if(LETIMER_IntGet(LETIMER0)&LETIMER_IFC_COMP0)
	{
		LETIMER_IntClear(LETIMER0,LETIMER_IF_COMP0);
		event=TurnOnPower;
		Event_Mask |= EXT_SIGNAL_PRINT_RESULTS;
		Event_Mask |= DISP_UPDATE;
		++RollOver;

#if (DEVICE_IS_BLE_SERVER==SERVER)
		if(RollOver%3 == 0){
			Event_Mask |= LETIMER_Triggered;

		}
#endif
			gecko_external_signal(Event_Mask);

	}

	else
	{
		LETIMER_IntClear(LETIMER0,LETIMER_IF_COMP1);
		event=StartWrite;
		Event_Mask |= LETIMER_Triggered;
		gecko_external_signal(Event_Mask);
	}


}

//void GPIO_EVEN_IRQHandler(void)
//{
//	static int Interrupt_Read;
//	CORE_AtomicDisableIrq();
//	Interrupt_Read = GPIO_IntGet();
//	GPIO_IntClear(Interrupt_Read);
//	value=GPIO_PinInGet(gpioPortF,6);
//	value^=1;
//	LOG_INFO("Entered:%d\n",value);
//	Event_Mask |= ButtonPress;
//	gecko_external_signal(Event_Mask);
//	CORE_AtomicEnableIrq();
//}

/************************************************************************
 * @func :	Sets up the ports for SCl-SDA and Enable pin.
 * 			Selects appropriate pins and assigns them to the structure
 * 			I2CSPM_Init_TypeDef.
 * @param:	None
 * @return:	None
 ***********************************************************************/
//void I2C_Startup(void)
//{
//
//	  I2CSPM_Init_TypeDef i2cInit=
//	  {
//			  .sclPin=10,
//			  .sdaPin=11,
//			  .portLocationScl=14,
//			  .portLocationSda=16
//	  };
//	  I2CSPM_Init(&i2cInit);
//	  GPIO_PinModeSet(gpioPortD, 15, gpioModePushPull, 1);
//	  GPIO_PinOutSet(gpioPortD, 15);	  	  	  	  	  	  	  	  	  //Enable the I2C module
//	  timerWaitUs(80000);//timerwaitus(80000);// event=StartWrite;
//}
//
//void I2C_TempConvertBLE(void)
//{
//	  uint8_t HTM_BUFF[5]; /* Stores the temperature data in the Health Thermometer (HTM) format. */
//	  uint8_t HTM_flags = 0x00;   /* HTM flags set as 0 for Celsius, no time stamp and no temperature type. */
//	  uint32_t temperature;   /* Stores the temperature data read from the sensor in the correct format */
//	  uint8_t *p = HTM_BUFF; /* Pointer to HTM temperature buffer needed for converting values to bitstream. */
//
//	    /* Convert flags to bitstream and append them in the HTM temperature data buffer (htmTempBuffer) */
//	  UINT8_TO_BITSTREAM(p, HTM_flags);
//
//	      /* Convert sensor data to correct temperature format */
//	  temperature = FLT_TO_UINT32((temp), -3);
//
//	  /* Convert temperature to bitstream andisplayLOG_INFO(DISPLAY_ROW_TEMPVALUE,"%f",(temp/1000));d place it in the HTM temperature data buffer (htmTempBuffer) */
//	  UINT32_TO_BITSTREAM(p, temperature);
//
//	  /* Send indication of the temperature in htmTempBuffer to all "listening" clients.
//	       * This enables the Health Thermometer in the Blue Gecko app to display the temperature.
//	       *  0xFF as connection ID will send indications to all connections. */
//	  gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_Temperature, 5, HTM_BUFF);
//	  displayPrintf(DISPLAY_ROW_TEMPVALUE,"%f",(temp/1000));
//	          displayPrintf(DISPLAY_ROW_BTADDR2,"%02x:%02x:%02x:%02x:%02x:%02x",Server_Addr.addr[5],
//        		Server_Addr.addr[4],
//				Server_Addr.addr[3],
//				Server_Addr.addr[2],
//				Server_Addr.addr[1],
//				Server_Addr.addr[0]);
//	  //gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_tx_power_level, 5,rssi_value);
//}
///*********************************************************************************
// * @func :	Disables the I2C pins
// * @brief:	GPIO_PinOutClear() is used to disable the SDA-SCL and Enable pins.
// * @param:	None
// * @return:	None
// *********************************************************************************/
//
//void I2C_ShutDown(void)
//{
//
//		 // GPIO_PinOutClear(gpioPortD,15);
//		  GPIO_PinOutClear(gpioPortC,10);
//		  GPIO_PinOutClear(gpioPortC,11);
//
//
//}

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

//void Event_Handler(void)
//{
//	switch(event)
//	{
//		/**************************************************************************
//		 * @event: TurnOnPower-This event initializes the I2C using I2CSPM_Init().
//		 *         It then triggers the TimerWaitUs() function
//		 * @previous: The previous state is Idle.
//		 * @next:  The state transitioned from this state is into Sleep till the COMP1
//		 * 		   interrupt triggers
//		 **************************************************************************/
//		case TurnOnPower:
//		LOG_INFO("Time Stamp:%d\t",loggerGetTimestamp(RollOver));	//Provides the current time
//		LOG_INFO("Entered TurnOnPower state\n");
//		//displayInit();
//		I2C_Startup();
//		//timerWaitUs(80000);
//		break;
//
//		/**************************************************************************
//		 * @event: StartWrite-This event initiates the write command to the I2C temperature
//		 * 		   sensor.
//		 * @previous: It transitions here from Sleep state called by the TurnOnPower event.
//		 * 			  After an ISR for COMP1 triggers, the event is set to StartWrite.
//		 * @next:  The next state is determined by the 12C ISR return value.
//		 **************************************************************************/
//		case StartWrite:
//		LOG_INFO("Time Stamp:%d\t",loggerGetTimestamp(RollOver));
//		LOG_INFO("Entered StartWrite state\n");
//		current = PreWrite;
//	  	seq.addr  = addr;
//	  	seq.flags = I2C_FLAG_WRITE;
//	  	i2c_write_data[0] = command;
//	  	seq.buf[0].data   = i2c_write_data;
//	  	seq.buf[0].len    = 1;
//	  	NVIC_EnableIRQ(I2C0_IRQn);
//	  	ret = I2C_TransferInit(I2C0, &seq);
//	  	break;
//
//	  	/**************************************************************************
//	  	* @event: i2cisDone-This event is called by the I2C ISR if the I2C Transfer was
//	  	* 		  completed. There are 2 cases for I2CTransfer complete, Pre-Write and
//	  	* 		  Post-Write.
//	  	* @previous: StartWrite was the previous event if the I2C returned i2cisDone if
//	  	* 			 not the previous event would be Incomplete
//	  	* @next:  The next state is determined by the 12C ISR return value.
//	  	***************************************************************************/
//	  	case i2cisDone:
//	  	LOG_INFO("Time Stamp:%d\t",loggerGetTimestamp(RollOver));
//	  	LOG_INFO("Entered i2cisDone state\n");
//	  	/*****************************************************************************
//	  	 * Pre-Write - This is condition entered if the write transfer is completed
//	  	 * 			   successfully. Once entered, the current state is changed to
//	  	 * 			   Post-Write.
//	  	 *****************************************************************************/
//	  	if(current==PreWrite)
//	  	{
//	  		LOG_INFO("Entered PreWrite\n");
//	  		current=PostWrite;
//	  		seq.flags = I2C_FLAG_READ;
//	  		seq.buf[0].data   = i2c_read_data;
//	  		seq.buf[0].len    = 2;
//	  		ret = I2C_TransferInit(I2C0,&seq);	//Initiate Read
//	  		//event=Sleep1;//EMU_EnterEM1();//EMU_EnterEM1();
//	  	}
//	  	/*****************************************************************************
//	  	 * Post-Write - This is condition entered if the read transfer is completed
//	  	 * 			   successfully. Once entered, the current state is changed to
//	  	 * 			   Pre-Write.
//	  	 *****************************************************************************/
//	  	else if(current==PostWrite)
//	  	{
//	  		LOG_INFO("Entered PostWrite\n");
//	  		CORE_DECLARE_IRQ_STATE;
//			CORE_ENTER_CRITICAL();
//	  		data |= (i2c_read_data[0]<<8)|(i2c_read_data[1]) ;
//	  		CORE_EXIT_CRITICAL();
//	  		temp=((175.72*data)/65535)-46.85;
//	  		LOG_INFO("Temp:%f\n",temp);
//	  		temp=temp*1000;
//	  		current=PreWrite;
//	  		//NVIC_DisableIRQ(I2C0_IRQn); //Must stay commented
//	  		I2C_ShutDown();
//	  		event_flag=1;//event=Sleep;
//	  	}
//	  	break;
//
//	  	/**************************************************************************
//	  	* @event: Idle-This event is the first at the time of initialization.
//	  	* @previous: There is no previous state for this
//	  	* @next:  The next event is TurnOnPower. Called by the COMP0 ISR
//	  	***************************************************************************/
//	  	case Idle:
//	  	LOG_INFO("Time Stamp:%d\t",loggerGetTimestamp(RollOver));
//	  	LOG_INFO("Entered Idle State\n");
//	  	break;
//
//	  	/**************************************************************************
//	  	* @event: Incomplete - This event is called if an I2C transfer is incomplete.
//	  	* @previous: StartWrite would be the previous state(ideally) or Incomplete
//	  	* 			 could be the previous state if I2C ISR had returned Incomplete
//	  	* 			 previously.
//	  	* @next:  The next event is determined by I2C ISR Handler. If the transfer is
//	  	* 		  successful the next event will be i2cisDone. If not, it would be
//	  	* 		  incomplete or Error
//	  	***************************************************************************/
//	  	case Incomplete:
//	  	LOG_INFO("Time Stamp:%d\t",loggerGetTimestamp(RollOver));
//	  	LOG_INFO("Entered Incomplete\n");
//	  	//EMU_EnterEM1();
//	  	break;
//	  	/**************************************************************************
//	  	* @event: Sleep-This event enters the EM3 sleep mode.
//	  	* @previous: Can be invoked from either from TurnOnPower or from i2cisDone.
//	  	* @next:  The next state if invoked from TurnOnPower(COMP1 ISR), will be StartWrite.
//	  	*         If invoked from i2cisDone the next state would be TurnOnPower.
//	  	***************************************************************************/
//	  	case Sleep:
//	  	LOG_INFO("Time Stamp:%d\t",loggerGetTimestamp(RollOver));
//	  	//SLEEP_SleepBlockEnd(sleepEM2);
//	  	//EMU_EnterEM3(false);
//	  	break;
//
//	  	/**************************************************************************
//	  	* @event: Error - This event is called if I2C Transfer was complete but with
//	  	* 		  errors. The system diables the I2C0 interrupts and enters EM3 mode.
//	  	* @previous: i2cisDone or StartWrite.
//	  	* @next:  The next state would be TurnOnPower.
//	  	***************************************************************************/
//	  	case Error:
//	  	LOG_INFO("Entered Error\n");
//	  	//NVIC_DisableIRQ(I2C0_IRQn);
//	  	//event_flag=1;//event=Sleep;
//	  	break;
//
//	  	case Sleep1:
//	  	LOG_INFO("Entered Sleep1\n");//EMU_EnterEM1();
//	  	break;
//
//	  	default : break;
//	}
//}

/**************************************************************************************
 * @irq: Checks if transfer is complete. If not, sets event= incomplete.
 * 		 This causes the system to enter EM1. If the transfer is complete it sets event,
 * 		 I2cisDone or Error depending on whether the transfer returned 0 or not.
 ************************************************************8**************************/
//void I2C0_IRQHandler(void)
//{
//	ret = I2C_Transfer(I2C0);
//	if(ret!=i2cTransferInProgress)
//	{
//		if(ret==i2cTransferDone)
//			{
//				LOG_INFO("ISR1");
//				event = i2cisDone;
//				Event_Mask |= WRITE_COMPLETE;
//				gecko_external_signal(Event_Mask);
//			}
//		else
//			{
//				LOG_INFO("ISR2");
//				event=Error;
//			}
//	}
//	else
//	{
//		LOG_INFO("ISR3");
//		event = Error;
//		Event_Mask |= WRITE_COMPLETE;
//		gecko_external_signal(Event_Mask);
//	}
//
//  NVIC_ClearPendingIRQ(I2C0_IRQn);
//
//}

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
	// I2C_INCOMPLETE = 4;
	count_write=0;
	printHeader=true;
	bootToDfu=0;
	thermoService[0] =  0x09;
	thermoService[1] = 0x18;


		// Temperature Measurement characteristic UUID defined by Bluetooth SIG

	thermoChar[0] =  0x1c;
	thermoChar[1] = 0x2a;
	Server_Addr.addr[0]=0xd5; //= { .addr =  {0xc0, 0x29, 0xef, 0x57, 0x0b, 0x00}};
	Server_Addr.addr[1]=0x2f;
	Server_Addr.addr[2]=0xef;
	Server_Addr.addr[3]=0x57;
	Server_Addr.addr[4]=0x0b;
	Server_Addr.addr[5]=0x00;
	NoBond = 0XFF;
	BondState = NoBond;
	bool PassKeyEvent = false;
	bool BondFailFlag = false;
	ButtonService[0]= 0x89;
	ButtonService[1]=0x62;
	ButtonService[2]=0x13;
	ButtonService[3]=0x2d;
	ButtonService[4]=0x2a;
	ButtonService[5]=0x65;
	ButtonService[6]=0xec;
	ButtonService[7]=0x87;
	ButtonService[8]=0x3e;
	ButtonService[9]=0x43;
	ButtonService[10]=0xc8;
	ButtonService[11]=0x38;
	ButtonService[12]=0x01;
	ButtonService[13]=0x00;
	ButtonService[14]=0x00;
	ButtonService[15]=0x00;

	ButtonChar[0]= 0x89;
	ButtonChar[1]=0x62;
	ButtonChar[2]=0x13;
	ButtonChar[3]=0x2d;
	ButtonChar[4]=0x2a;
	ButtonChar[5]=0x65;
	ButtonChar[6]=0xec;
	ButtonChar[7]=0x87;
	ButtonChar[8]=0x3e;
	ButtonChar[9]=0x43;
	ButtonChar[10]=0xc8;
	ButtonChar[11]=0x38;
	ButtonChar[12]=0x02;
	ButtonChar[13]=0x00;
	ButtonChar[14]=0x00;
	ButtonChar[15]=0x00;

	ButtonInitiationFlag=0;
	//already_initiated=0;

}



void timerEnable1HzSchedulerEvent(uint32_t Scheduler_DisplayUpdate)
{
	DISP_UPDATE = Scheduler_DisplayUpdate;
}



void initProperties(void)
{
	uint8_t i;
	connProperties.connectionHandle = CONNECTION_HANDLE_INVALID;
	connProperties.thermometerServiceHandle = SERVICE_HANDLE_INVALID;
	connProperties.thermometerCharacteristicHandle = CHARACTERISTIC_HANDLE_INVALID;
	connProperties.temperature = TEMP_INVALID;
	connProperties.rssi = RSSI_INVALID;

	connProperties.ButtonServiceHandle = SERVICE_HANDLE_INVALID;
	connProperties.ButtonCharacteristicHandle = CHARACTERISTIC_HANDLE_INVALID;


}

//uint8_t findIndexByConnectionHandle(uint8_t connection)
//{
//    if (connProperties.connectionHandle != connection)
//    	return TABLE_INDEX_INVALID;
//    return 1;
//}
//
//void addConnection(uint8_t connection, uint16_t address)
//{
//  connProperties.connectionHandle = connection;
//  connProperties.serverAddress    = address;
//  //activeConnectionsNum++;
//}
//
//void removeConnection(uint8_t connection)
//{
//  uint8_t i;
//  uint8_t table_index = findIndexByConnectionHandle(connection);
//
//connProperties.connectionHandle = CONNECTION_HANDLE_INVALID;
//connProperties.thermometerServiceHandle = SERVICE_HANDLE_INVALID;
//connProperties.thermometerCharacteristicHandle = CHARACTERISTIC_HANDLE_INVALID;
//connProperties.temperature = TEMP_INVALID;
//connProperties.rssi = RSSI_INVALID;
//connProperties.ButtonServiceHandle = SERVICE_HANDLE_INVALID;
//connProperties.ButtonCharacteristicHandle = CHARACTERISTIC_HANDLE_INVALID;
//
//}
//
//void gecko_custom_update(struct gecko_cmd_packet* evt)
//{
//  gecko_update(evt);
//  switch(BGLIB_MSG_ID(evt->header))
//  {
///******************************************************************************
// @brief : This event is triggered once the BLE Stack is initiated.
// @func  : Power is set to 0. Advertising intervals are setup. The BLE address is
//      obtained.
//******************************************************************************/
//
//    case gecko_evt_system_boot_id:
//    #if (DEVICE_IS_BLE_SERVER==CLIENT)			//Client
//      LOG_INFO("System Initiated\n");
//        // Set passive scanning on 1Mb PHY
//        gecko_cmd_le_gap_set_discovery_type(le_gap_phy_1m, SCAN_PASSIVE);
//        // Set scan interval and scan window
//        gecko_cmd_le_gap_set_discovery_timing(le_gap_phy_1m, SCAN_INTERVAL, SCAN_WINDOW);
//
//        gecko_cmd_le_gap_start_discovery(le_gap_phy_1m, le_gap_discover_generic);
//        displayPrintf(DISPLAY_ROW_CONNECTION,"Scanning");
//        displayPrintf(DISPLAY_ROW_NAME,"Client");
//        displayPrintf(DISPLAY_ROW_BTADDR2,"%02x:%02x:%02x:%02x:%02x:%02x",Server_Addr.addr[5],
//        		Server_Addr.addr[4],
//				Server_Addr.addr[3],
//				Server_Addr.addr[2],
//				Server_Addr.addr[1],
//				Server_Addr.addr[0]);
//         gecko_cmd_sm_delete_bondings();
//         gecko_cmd_sm_set_bondable_mode(1);
//		 gecko_cmd_sm_configure(SMConfig, sm_io_capability_displayyesno);
//         connState = scanning;
//        #else
//
//        gecko_cmd_system_set_tx_power(0);
//
//		gecko_cmd_le_gap_set_advertise_timing(0, 400, 400, 0, 0);
//
//		gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);
//		displayPrintf(DISPLAY_ROW_CONNECTION,"Advertising");
//		displayPrintf(DISPLAY_ROW_NAME,"Server");
//
//		gecko_cmd_sm_delete_bondings();		//Custom_BLE_Server_Delete_Bondings();
//		gecko_cmd_sm_set_passkey(-1);		// -1 for random passkey
//		gecko_cmd_sm_set_bondable_mode(1);	// Bondable mode enable
//		gecko_cmd_sm_configure(SMConfig, sm_io_capability_displayyesno);
//
//        #endif
//
//		AddressBLE = gecko_cmd_system_get_bt_address();
//        displayPrintf(DISPLAY_ROW_BTADDR,"%02x:%02x:%02x:%02x:%02x:%02x",AddressBLE->address.addr[5],
//        						AddressBLE->address.addr[4],
//        						AddressBLE->address.addr[3],
//        						AddressBLE->address.addr[2],
//        						AddressBLE->address.addr[1],
//        						AddressBLE->address.addr[0]);
//
//        break;
///******************************************************************************
// @brief : This event is triggered when the client
// @func  : The LETIMER is diabled so that no interrupt triggers at 1s intervals.
//******************************************************************************/
//
//case gecko_evt_sm_confirm_passkey_id:
//
//	PassKeyEvent = true;
//	uint32_t key;
//	key = evt->data.evt_sm_confirm_passkey.passkey; // reading passkey
//	displayPrintf(DISPLAY_ROW_PASSKEY,"Passkey-%6d",key);
//	LOG_INFO("Passkey-%6d",key);
//	displayPrintf(DISPLAY_ROW_ACTION,"Confirm with PB0");
//	break;
//
//case gecko_evt_sm_bonded_id:
//	displayPrintf(DISPLAY_ROW_PASSKEY,"Bonded Successfully");
//	displayPrintf(DISPLAY_ROW_ACTION,"Connected");
//#if(DEVICE_IS_BLE_SERVER==CLIENT)
//	DisplayButtonState=1;
//    LOG_INFO("Bonded\n");
//#endif
//	break;
//
//
//case gecko_evt_sm_bonding_failed_id:
//
//	displayPrintf(DISPLAY_ROW_PASSKEY,"Bonded UnSuccessfully");
//	BondFailFlag = true;
//	gecko_cmd_le_connection_close(ConnectionHandle);
//	break;
//
//
//case gecko_evt_le_gap_scan_response_id:
//			LOG_INFO("Received Scan Response ID\n");
//			if (evt->data.evt_le_gap_scan_response.packet_type == 0) {
//				if(memcmp(&evt->data.evt_le_gap_scan_response.address.addr[0],
//				&Server_Addr.addr[0], 6) == 0){
//				gecko_cmd_le_gap_end_procedure();
//				gecko_cmd_le_gap_connect(evt->data.evt_le_gap_scan_response.address,
//														evt->data.evt_le_gap_scan_response.address_type,
//														le_gap_phy_1m);
//				connState = opening;
//
//				}
//			}
//			break;
//
//
//case gecko_evt_gatt_server_characteristic_status_id: //Server
//		LOG_INFO("status_flags=%d",evt->data.evt_gatt_server_characteristic_status.status_flags);
//			displayPrintf(DISPLAY_ROW_CONNECTION,"Handling Indications");
//			               displayPrintf(DISPLAY_ROW_BTADDR2,"%02x:%02x:%02x:%02x:%02x:%02x",Server_Addr.addr[5],
//        		Server_Addr.addr[4],
//				Server_Addr.addr[3],
//				Server_Addr.addr[2],
//				Server_Addr.addr[1],
//				Server_Addr.addr[0]);
//	   AddressBLE = gecko_cmd_system_get_bt_address();
//	   displayPrintf(DISPLAY_ROW_BTADDR,"%02x:%02x:%02x:%02x:%02x:%02x",AddressBLE->address.addr[5],
//				AddressBLE->address.addr[4],
//				AddressBLE->address.addr[3],
//				AddressBLE->address.addr[2],
//				AddressBLE->address.addr[1],
//				AddressBLE->address.addr[0]);
//
//			if((evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_Temperature)
//			    && (evt->data.evt_gatt_server_characteristic_status.status_flags == 0x01))
//			    {
//
//				if(evt->data.evt_gatt_server_characteristic_status.client_config_flags == 0x02)
//			    	{
//						LOG_INFO("Interrupt not Stopped\n");
//						Active_Connection = 1;
//			    		LETIMER_Enable(LETIMER0, true);
//			    		LETIMER_IntEnable(LETIMER0,LETIMER_IF_COMP0);
//			    	}
//
//			    }
//	      break;
//
//
//case gecko_evt_le_connection_opened_id:
//    	#if(DEVICE_IS_BLE_SERVER==CLIENT)
//	    	LOG_INFO("Connection opened ID\n");
//	        addrValue = (uint16_t)(evt->data.evt_le_connection_opened.address.addr[1] << 8) \
//	                    + evt->data.evt_le_connection_opened.address.addr[0];
//	        ConnectionHandle = evt->data.evt_le_connection_opened.connection;
//	        addConnection(evt->data.evt_le_connection_opened.connection, addrValue);
//	        // Discover Health Thermometer service on the slave device
//	        gecko_cmd_gatt_discover_primary_services_by_uuid(evt->data.evt_le_connection_opened.connection,
//	                                                         2,
//	                                                         (const uint8_t*)thermoService);
//	        connState = discoverServices;
//        #else
//        	ConnectionHandle = evt->data.evt_le_connection_opened.connection;
//
//			gecko_cmd_le_connection_set_parameters(ConnectionHandle, MinConnTime, MaxConnTime,SlaveLatency,TimeoutVal);
//			LETIMER_Enable(LETIMER0,true);
//			displayPrintf(DISPLAY_ROW_CONNECTION,"Connected");
//			displayPrintf(DISPLAY_ROW_NAME,"Server");
//			LOG_INFO("Gecko BLE Connection start\n");
//
//			BondState = evt->data.evt_le_connection_opened.bonding;	//Checks bonding status
//
//			if(BondState != NoBond)								    // Successful Bonding condition
//			{
//				displayPrintf(DISPLAY_ROW_PASSKEY,"Already Bonded");
//				displayPrintf(DISPLAY_ROW_ACTION,"Connected");
//			}
//
//
//		#endif
//        break;
//
//case gecko_evt_gatt_service_id:
//			LOG_INFO("Service ID\n");
//			tableIndex = findIndexByConnectionHandle(evt->data.evt_gatt_service.connection);
//			if (tableIndex != TABLE_INDEX_INVALID) {
//			// Save service handle for future reference
//			if(ButtonInitiationFlag==0)
//				connProperties.thermometerServiceHandle = evt->data.evt_gatt_service.service;
//			else
//				connProperties.ButtonServiceHandle = evt->data.evt_gatt_service.service;
//			}
//			break;
//
//case gecko_evt_gatt_characteristic_id:
//			LOG_INFO("Characteristic ID\n");
//			tableIndex = findIndexByConnectionHandle(evt->data.evt_gatt_characteristic.connection);
//			if (tableIndex != TABLE_INDEX_INVALID) {
//			// Save characteristic handle for future reference
//			if(ButtonInitiationFlag==0)
//				connProperties.thermometerCharacteristicHandle = evt->data.evt_gatt_characteristic.characteristic;
//			else
//				connProperties.ButtonCharacteristicHandle = evt->data.evt_gatt_characteristic.characteristic;
//			}
//			break;
//
//case gecko_evt_gatt_procedure_completed_id:				//Client Event
//			LOG_INFO("Procedure Completed ID\n");
//			tableIndex = findIndexByConnectionHandle(evt->data.evt_gatt_procedure_completed.connection);
//			if (tableIndex == TABLE_INDEX_INVALID) {
//				LOG_INFO("Failed Discovery\n");
//			break;
//			}
//
//			if (connState == discoverServices ) {
//			LOG_INFO("service discovery finished thermometer\n");
//
//			// Discover thermometer characteristic on the slave device
//			if(ButtonInitiationFlag==0 && connProperties.thermometerServiceHandle != SERVICE_HANDLE_INVALID)
//			{
//				ButtonInitiationFlag=1;
//				gecko_cmd_gatt_discover_primary_services_by_uuid(evt->data.evt_gatt_procedure_completed.connection,
//					                                                         16,
//					                                                         (const uint8_t*)ButtonService);
//
//			}
//			else if(ButtonInitiationFlag==1 && connProperties.ButtonServiceHandle != SERVICE_HANDLE_INVALID){
//			LOG_INFO("service discovery finished button\n");
//			ButtonInitiationFlag=0;
//			gecko_cmd_gatt_discover_characteristics_by_uuid(evt->data.evt_gatt_procedure_completed.connection,
//											  connProperties.thermometerServiceHandle,
//											  2,
//											  (const uint8_t*)thermoChar);
//			connState = discoverCharacteristics;			//Update the state only after both the conditions have been met.
//			}
//
//			break;
//			}
//
//			if (connState == discoverCharacteristics ){
//
//			//connState = discoverCharacteristics;
//			if(ButtonInitiationFlag == 0 && connProperties.thermometerCharacteristicHandle != CHARACTERISTIC_HANDLE_INVALID){
//				ButtonInitiationFlag=1;
//			// Discover thermometer characteristic on the slave device
//				gecko_cmd_gatt_discover_characteristics_by_uuid(evt->data.evt_gatt_procedure_completed.connection,
//											  connProperties.ButtonServiceHandle,
//											  16,
//											  (const uint8_t*)ButtonChar);
//			}
//			else if(ButtonInitiationFlag == 1 && connProperties.ButtonCharacteristicHandle != CHARACTERISTIC_HANDLE_INVALID){
//				ButtonInitiationFlag=0;
//				gecko_cmd_gatt_set_characteristic_notification(evt->data.evt_gatt_procedure_completed.connection,
//												 connProperties.thermometerCharacteristicHandle,
//												 gatt_indication);
//				connState = enableIndication;
//			}
//			break;
//			}
//
//			if (connState == enableIndication) {
//				if(ButtonInitiationFlag==0)
//				{
//					ButtonInitiationFlag=1;
//					gecko_cmd_gatt_set_characteristic_notification(evt->data.evt_gatt_procedure_completed.connection,
//													 connProperties.ButtonCharacteristicHandle,
//													 gatt_indication);
//				}
//				else
//				{
//					ButtonInitiationFlag=0;
//					gecko_cmd_sm_increase_security(evt->data.evt_gatt_procedure_completed.connection);
//					connState = running;
//					LOG_INFO("Indication set\n");
//				}
//
//			}
//			break;
//
//
//case gecko_evt_le_connection_closed_id:
//        #if(DEVICE_IS_BLE_SERVER==CLIENT)
//        LOG_INFO("connection closed ID\n");
// 	   AddressBLE = gecko_cmd_system_get_bt_address();
// 	   displayPrintf(DISPLAY_ROW_BTADDR,"%02x:%02x:%02x:%02x:%02x:%02x",AddressBLE->address.addr[5],
// 				AddressBLE->address.addr[4],
// 				AddressBLE->address.addr[3],
// 				AddressBLE->address.addr[2],
// 				AddressBLE->address.addr[1],
// 				AddressBLE->address.addr[0]);
//		displayPrintf(DISPLAY_ROW_CONNECTION,"Discovering");
//		               displayPrintf(DISPLAY_ROW_BTADDR2,"%02x:%02x:%02x:%02x:%02x:%02x",Server_Addr.addr[5],
//        		Server_Addr.addr[4],
//				Server_Addr.addr[3],
//				Server_Addr.addr[2],
//				Server_Addr.addr[1],
//				Server_Addr.addr[0]);
//       	DisplayButtonState=0;
//		displayPrintf(DISPLAY_ROW_NAME,"Client");
//		displayPrintf(DISPLAY_ROW_TEMPVALUE,"%s"," ");
//		displayPrintf(DISPLAY_ROW_PASSKEY,"%s"," ");
//		displayPrintf(DISPLAY_ROW_ACTION,"%s"," ");
//        removeConnection(evt->data.evt_le_connection_closed.connection);
//        gecko_cmd_le_gap_start_discovery(le_gap_phy_1m, le_gap_discover_generic);
//        connState = scanning;
//
//        #else
//			Active_Connection = 0;
//
//			gecko_cmd_system_set_tx_power(0);
//
//			LETIMER_Enable(LETIMER0, false);
//
//			gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);
//			LOG_INFO("Gecko BLE Connection end\n");
//			displayPrintf(DISPLAY_ROW_CONNECTION,"Advertising");
//			displayPrintf(DISPLAY_ROW_NAME,"Server");
//			displayPrintf(DISPLAY_ROW_TEMPVALUE,"%s"," ");
//			displayPrintf(DISPLAY_ROW_PASSKEY," ");
//			displayPrintf(DISPLAY_ROW_ACTION," ");
//
//        #endif
//        break;
//
// case gecko_evt_gatt_characteristic_value_id:
//			LOG_INFO("characteristic values received\n");
//			   AddressBLE = gecko_cmd_system_get_bt_address();
//			   displayPrintf(DISPLAY_ROW_BTADDR,"%02x:%02x:%02x:%02x:%02x:%02x",AddressBLE->address.addr[5],
//						AddressBLE->address.addr[4],
//						AddressBLE->address.addr[3],
//						AddressBLE->address.addr[2],
//						AddressBLE->address.addr[1],
//						AddressBLE->address.addr[0]);
//			displayPrintf(DISPLAY_ROW_CONNECTION,"Handling Indications");
//			               displayPrintf(DISPLAY_ROW_BTADDR2,"%02x:%02x:%02x:%02x:%02x:%02x",Server_Addr.addr[5],
//        		Server_Addr.addr[4],
//				Server_Addr.addr[3],
//				Server_Addr.addr[2],
//				Server_Addr.addr[1],
//				Server_Addr.addr[0]);
//			charValue = &(evt->data.evt_gatt_characteristic_value.value.data[0]);
//			tableIndex = findIndexByConnectionHandle(evt->data.evt_gatt_characteristic_value.connection);
//			if (tableIndex != TABLE_INDEX_INVALID) {
//
//				if(connProperties.thermometerCharacteristicHandle==evt->data.evt_gatt_characteristic_value.characteristic)
//				{
//					LOG_INFO("%d%d%d%d",charValue[0],charValue[1],charValue[2],charValue[3],charValue[4]);
//					connProperties.temperature = (charValue[1] << 0) + (charValue[2] << 8) + (charValue[3] << 16);
//				}
//				else if(connProperties.ButtonCharacteristicHandle==evt->data.evt_gatt_characteristic_value.characteristic)
//				{
//					LOG_INFO("%d%d%d%d",charValue[0],charValue[1],charValue[2],charValue[3],charValue[4]);
//					connProperties.Button = (charValue[1] << 0) + (charValue[2] << 8) + (charValue[3] << 16);
//				}
//
//			}
//			// Send confirmation for the indication
//			gecko_cmd_gatt_send_characteristic_confirmation(evt->data.evt_gatt_characteristic_value.connection);
//			// Trigger RSSI measurement on the connection
//			//
//			gecko_cmd_le_connection_get_rssi(evt->data.evt_gatt_characteristic_value.connection);
//			break;
//
//
//case gecko_evt_le_connection_rssi_id:
//      #if (DEVICE_IS_BLE_SERVER==CLIENT) //Client
//       LOG_INFO("RSSI measured\n");
//        tableIndex = findIndexByConnectionHandle(evt->data.evt_le_connection_rssi.connection);
//        if (tableIndex != TABLE_INDEX_INVALID) {
//          connProperties.rssi = evt->data.evt_le_connection_rssi.rssi;
//        }
//        // Trigger printing
//         gecko_external_signal(EXT_SIGNAL_PRINT_RESULTS);
//
//
//        #else
//        LOG_INFO("RSSI \n");
//	    		// Get RSSI Value
//	    		rssi_value = evt->data.evt_le_connection_rssi.rssi;
//	    		LOG_INFO("%d",rssi_value);
//
//	    		// Halt the BT module to change the TX power
//	    		gecko_cmd_system_halt(1);
//
//	    		if(rssi_value > -35)
//	    		{
//
//	    			gecko_cmd_system_set_tx_power(BG13_Min_Power);
//	    			LOG_INFO(" rssi_value > -35\n");
//
//	    		}
//
//	    		else if(-35 >= rssi_value && rssi_value > -45)
//	    		{
//	    			gecko_cmd_system_set_tx_power(-200);
//	    			LOG_INFO(" rssi_value > -45\n");
//	    		}
//
//
//	    		else if(-45 >= rssi_value && rssi_value > -55)
//	    		{
//	    			gecko_cmd_system_set_tx_power(-150);
//	    			LOG_INFO(" rssi_value > -55\n");
//	    		}
//
//	    		else if(-55 >= rssi_value && rssi_value > -65)
//	    		{
//	    			gecko_cmd_system_set_tx_power(-50);
//	    			LOG_INFO(" rssi_value > -65\n");
//	    		}
//
//	    		else if(-65 >= rssi_value && rssi_value > -75)
//	    		{
//	    			gecko_cmd_system_set_tx_power(0);
//	    			LOG_INFO(" rssi_value > -75\n");
//	    		}
//
//
//	    		else if(-75 >= rssi_value && rssi_value > -85)
//	    		{
//	    			gecko_cmd_system_set_tx_power(50);
//	    			LOG_INFO(" rssi_value > -85\n");
//	    		}
//
//
//	    		else
//	    		{
//
//	    			gecko_cmd_system_set_tx_power(BG13_Max_Power);
//	    			LOG_INFO(" rssi_value largest\n");
//
//	    		}
//
//
//	    		gecko_cmd_system_halt(0);
//
//        #endif
//        break;
//
//		/******************************************************************************
//		@brief : This event is triggered when an interrupt occurs.
//		An Event Mask is set to select the source of interrupt.
//		@func : Handles interrupts from LETIMER and I2C interrupts.
//		******************************************************************************/
//
//		case gecko_evt_system_external_signal_id:
//
//			    	Event_Read = evt->data.evt_system_external_signal.extsignals;
//			    	#if (DEVICE_IS_BLE_SERVER==CLIENT) //Client
//			    	if(Event_Read & ButtonPress)	//8
//					{
//						/* Begin Critical Section */
//						CORE_DECLARE_IRQ_STATE;
//						CORE_ENTER_CRITICAL();
//							Event_Mask &= ~ButtonPress;				//Clear the Event Mask
//						CORE_EXIT_CRITICAL();
//						/* End Critical Section */
//						displayUpdate();
//						gecko_cmd_sm_passkey_confirm(ConnectionHandle, true);
//
//					}
//
//					if (Event_Read & EXT_SIGNAL_PRINT_RESULTS) {
//
//						CORE_DECLARE_IRQ_STATE;
//			    		CORE_ENTER_CRITICAL();
//			    			Event_Mask &= ~EXT_SIGNAL_PRINT_RESULTS;				//Clear the Event Mask
//			    		CORE_EXIT_CRITICAL();
//
//						if(connProperties.temperature != TEMP_INVALID)
//							displayPrintf(DISPLAY_ROW_TEMPVALUE,"%f",(float)connProperties.temperature*0.001);
//						if(DisplayButtonState==1)
//							displayPrintf(DISPLAY_ROW_ACTION,"%s",ButtonPressString[(uint8_t)connProperties.Button]);
//
//						}
//
//
//					 if(Event_Read & DISP_UPDATE)	//8
//			    	{
//			    		/* Begin Critical Section */
//			    		CORE_DECLARE_IRQ_STATE;
//			    		CORE_ENTER_CRITICAL();
//			    			Event_Mask &= ~DISP_UPDATE;				//Clear the Event Mask
//			    		CORE_EXIT_CRITICAL();
//			    		/* End Critical Section */
//			    		displayUpdate();
//
//
//			    		LOG_INFO("Disp update 1s\n");
//			    	}
//			    	break;
//			    	#else //Server
//			    	if(Event_Read & ButtonPress)	//8
//					{
//						/* Begin Critical Section */
//						CORE_DECLARE_IRQ_STATE;
//						CORE_ENTER_CRITICAL();
//							Event_Mask &= ~ButtonPress;				//Clear the Event Mask
//						CORE_EXIT_CRITICAL();
//						/* End Critical Section */
//						displayUpdate();
//						//edit here
//
//						if(PassKeyEvent == true)
//						{
//							PassKeyEvent = false;
//							gecko_cmd_sm_passkey_confirm(ConnectionHandle, true);
//							displayPrintf(DISPLAY_ROW_ACTION,"PassKey Accepted");
//						}
//						else
//						{
//							uint8_t *ptr = &value;
//							uint8_t BTN_BUFF[2]; 		// Buffer to store the data  - Button status and Flags.
//							uint8_t BTN_flags = 0x00;   // Flags for Button State.
//							uint8_t *p = BTN_BUFF; 		// Pointer to store the BTN_BUFF buffer.
//
//							UINT8_TO_BITSTREAM(p, BTN_flags);
//							UINT8_TO_BITSTREAM(p, value);
//
//							gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_button_state,2,BTN_BUFF);
//							displayPrintf(DISPLAY_ROW_ACTION,"Button:%d",value);
//
//						}
//
//						LOG_INFO("Disp update 1s\n");
//					}
//			    	if(Active_Connection==1)
//			    	{
//			    	if(Event_Read & LETIMER_Triggered )
//			    	{
//			    			/* Begin Critical Section */
//			    			CORE_DECLARE_IRQ_STATE;
//			    			CORE_ENTER_CRITICAL();
//			    				Event_Mask &= ~LETIMER_Triggered ; //Clear the Event Mask
//			    			CORE_EXIT_CRITICAL();
//			    			/* End Critical Section */
//
//			    			Event_Handler();
//			    			LOG_INFO("3s call\n");
//			    	}
//
//
//			    	if(Event_Read & WRITE_COMPLETE)
//			    	{
//			    		/* Begin Critical Section */
//			    		CORE_DECLARE_IRQ_STATE;
//			    		CORE_ENTER_CRITICAL();
//			    			Event_Mask &= ~WRITE_COMPLETE;			//Clear the Event Mask
//			    		CORE_EXIT_CRITICAL();
//			    		/* End Critical Section */
//
//			    		LOG_INFO("Write\n");
//
//			    		Event_Handler();
//			    		if(event_flag==1)
//			    		{
//			    			I2C_TempConvertBLE();
//			    			LETIMER_Enable(LETIMER0, true);
//
//			    		}
//			    		gecko_cmd_le_connection_get_rssi(ConnectionHandle);
//
//			    	}
//
//
//			    	if(Event_Read & DISP_UPDATE)	//8
//			    	{
//			    		/* Begin Critical Section */
//			    		CORE_DECLARE_IRQ_STATE;
//			    		CORE_ENTER_CRITICAL();
//			    			Event_Mask &= ~DISP_UPDATE;				//Clear the Event Mask
//			    		CORE_EXIT_CRITICAL();
//			    		/* End Critical Section */
//			    		displayUpdate();
//
//			    		LOG_INFO("Disp update 1s\n");
//			    	}
//
//			    }
//			    	#endif
//
//			break;
//
//    }
//
//  }
