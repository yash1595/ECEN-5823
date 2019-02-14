
/*****************************************************************/
/* Board headers */
#include "init_mcu.h"
#include "init_board.h"
#include "init_app.h"
#include "ble-configuration.h"
#include "board_features.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"

/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"

/* Device initialization header */
#include "hal-config.h"

#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif

#include "em_device.h"
#include "em_chip.h"
#include "src/gpio.h"

#include "sleep.h"
#include "em_letimer.h"
#include "em_assert.h"
#include "em_core.h"
#include "em_rmu.h"

#include "i2cspm.h"
#include "em_i2c.h"
#define INCLUDE_LOG_DEBUG 1
#include "log.h"


//#include "src/i2c_tempsens.h"

/* End */
//#include "em_int.h"
#ifndef MAX_CONNECTIONS
#define MAX_CONNECTIONS 4
#endif

uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS)];

// Gecko configuration parameters (see gecko_configuration.h)
static const gecko_configuration_t config = {
  .config_flags = 0,
  .sleep.flags = SLEEP_FLAGS_DEEP_SLEEP_ENABLE,
  .bluetooth.max_connections = MAX_CONNECTIONS,
  .bluetooth.heap = bluetooth_stack_heap,
  .bluetooth.heap_size = sizeof(bluetooth_stack_heap),
  .bluetooth.sleep_clock_accuracy = 100, // ppm
  .gattdb = &bg_gattdb_data,
  .ota.flags = 0,
  .ota.device_name_len = 3,
  .ota.device_name_ptr = "OTA",
#if (HAL_PA_ENABLE) && defined(FEATURE_PA_HIGH_POWER)
  .pa.config_enable = 1, // Enable high power PA
  .pa.input = GECKO_RADIO_PA_INPUT_VBAT, // Configure PA input to VBAT
#endif // (HAL_PA_ENABLE) && defined(FEATURE_PA_HIGH_POWER)
};
 /************************* Macros *****************************/
 //#define CLOCK_DIV  4//cmuClkDiv_1//cmuClkDiv_4

#define TIME_PERIOD (3.00)//700//225//225   //700
//#define ON_TIME     (0.01)   //((uint32_t)(175/10))
#define COMP0   ((uint32_t)(0X01))
#define COMP1   ((uint32_t)(0X02))
#define TimerInt  ((uint32_t)(0X03))
#define SLEEP_MODE  sleepEM3
typedef enum {i2cisDone=0,TurnPowerOff=2,TurnOnPower=3,StartWrite=4,Idle=5,Incomplete=6,Sleep=7,Wait=8,Sleep1=9}Event;
typedef enum {PreWrite=0,PostWrite=1,Error=2}State;
State current;
Event event;
uint16_t addr;
uint8_t command;
uint8_t i2c_read_data[2];
uint8_t i2c_write_data[1];
uint16_t data;
float temp;
I2C_TransferSeq_TypeDef    seq;
I2C_TransferReturn_TypeDef ret;
uint8_t SleepMode;
uint8_t RollOver;
/*#define CLOCK_DIV
#define CLOCK_SEL
#define CLOCK_OSC
*/

#if  (SLEEP_MODE)==sleepEM3
#define CLOCK_DIV 1
#define CLOCK_SEL  cmuSelect_ULFRCO
#define CLOCK_OSC  cmuOsc_ULFRCO

#else
#define CLOCK_DIV 4
#define CLOCK_SEL  cmuSelect_LFXO
#define CLOCK_OSC  cmuOsc_LFXO
#endif

 /************************* Functions ***************************/
 void TimerInit(void);
 uint32_t Comp0_Cal(void);
 uint32_t Comp1_Cal(void);
 void SleepModeSel(void);
 void timerWaitUs(uint32_t);
 void I2C_TempInit(void);
 void I2C_Startup(void);
 void I2C_ShutDown(void);
 //void InitTransfersAndBuffers(void);
 void I2CPM_TempMeasure(void);
 void Event_Handler(void);
 void Init_Globals(void);
 void LoggerTimeStamp(void);
 //int16_t I2CPM_TempRead(void);


 I2C_TransferReturn_TypeDef I2C_Status;


 //char* ErrorStates[]={"i2cTransferDone","i2cTransferInProgress","i2cTransferNack ","i2cTransferBusErr","i2cTransferArbLost","i2cTransferUsageFault","i2cTransferSwFault"};
/*****************************************************************/
#define Comp0_Cal() (((CMU_ClockFreqGet(cmuClock_LFA)/(CLOCK_DIV*1))*(TIME_PERIOD)))
#define CounterGet(us_wait)   ((uint32_t)(((us_wait*0.000001*CMU_ClockFreqGet(cmuClock_LFA))/CLOCK_DIV)))
