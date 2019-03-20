#include "LETIMER.h"
#include "BLE.h"
#include "I2C.h"

 uint8_t thermoService[2];
 uint8_t thermoChar[2];
 uint8_t ButtonService[16];
 uint8_t ButtonChar[16];
 uint8_t ConnectionHandle;

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

