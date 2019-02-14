#include "LETIMER.h"

#include "init_mcu.h"
#include "init_board.h"
#include "init_app.h"
#include "ble-configuration.h"
#include "board_features.h"


#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"


#include "em_emu.h"
#include "em_cmu.h"


#include "hal-config.h"

#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif

#include "em_device.h"
#include "em_chip.h"
#include "gpio.h"

#include "sleep.h"
#include "em_letimer.h"


#include "em_assert.h"
#include "em_core.h"
#include "em_rmu.h"

#include "log.h"
#include "i2cspm.h"
//#include "i2c_tempsens.h"
//#include "em_int.h"
#ifndef MAX_CONNECTIONS
#define MAX_CONNECTIONS 4
#endif


uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS)];

/***************************************************************/


int main(void)
{

  initMcu();

  initBoard();

  initApp();

  logInit();

  gpioInit();

  Init_Globals();

  TimerInit();

  gecko_init(&config);

  SLEEP_InitEx(NULL);

  	while(1)
  	{
  		Event_Handler();
  		//SleepModeSel();//if(flag!=S0)SleepModeSel();
  	}


}
