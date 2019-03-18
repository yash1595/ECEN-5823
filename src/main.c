#include "LETIMER.h"
#include "ble_device_type.h"

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
//#include "ble_device_type.h"
//#define DEVICE_IS_BLE_SERVER 0
//#include "i2c_tempsens.h"
//#include "em_int.h"
#ifndef MAX_CONNECTIONS
#define MAX_CONNECTIONS 4
#endif


uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS)];

/***************************************************************/
//bd_addr Server_Addr = { .addr =  {0xc0, 0x29, 0xef, 0x57, 0x0b, 0x00}};

int main(void)
{

  initMcu();

  initBoard();

  initApp();

  logInit();

  gpioInit();

  Button0_Init();

  Init_Globals();

  TimerInit();

  gecko_init(&config);

  RETARGET_SerialInit();

  displayInit();

  initProperties();

//uint8array tmp;

  	while(1)
  	{

  		RETARGET_SerialFlush();
  		evt=gecko_wait_event();
  		gecko_custom_update(evt);

  	}

}


