
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

#include "infrastructure.h"
#include "display.h"
////////////////////////////////////////////////////
#include "ble_device_type.h"


//#define SERVER_BT_ADDRESS {{3}}
///////////////////////////////////////////////////
#define SCHEDULER_SUPPORTS_DISPLAY_UPDATE_EVENT 1
#define TIMER_SUPPORTS_1HZ_TIMER_EVENT	1
//#define CORE_DECLARE_IRQ_STATE        CORE_irqState_t irqState

#define CONN_INTERVAL_MIN             80   //100ms
#define CONN_INTERVAL_MAX             80   //100ms
#define CONN_SLAVE_LATENCY            0    //no latency
#define CONN_TIMEOUT                  100  //1000ms

#define SCAN_INTERVAL                 16   //10ms
#define SCAN_WINDOW                   16   //10ms
#define SCAN_PASSIVE                  0

#define TEMP_INVALID                  (uint32_t)0xFFFFFFFFu
#define RSSI_INVALID                  (int8_t)127
#define CONNECTION_HANDLE_INVALID     (uint8_t)0xFFu
#define SERVICE_HANDLE_INVALID        (uint32_t)0xFFFFFFFFu
#define CHARACTERISTIC_HANDLE_INVALID (uint16_t)0xFFFFu
#define TABLE_INDEX_INVALID           (uint8_t)0xFFu

#define EXT_SIGNAL_PRINT_RESULTS      (uint32_t)(16)
#define SMConfig	(0x0B)

// Gecko configuration parameters (see gecko_configuration.h)
#ifndef MAX_CONNECTIONS
#define MAX_CONNECTIONS               4
#endif
uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS)];

typedef enum {
  scanning,
  opening,
  discoverServices,
  discoverCharacteristics,
  enableIndication,
  running
} ConnState;

typedef struct {
  uint8_t  connectionHandle;
  int8_t   rssi;
  uint16_t serverAddress;
  uint32_t thermometerServiceHandle;
  uint16_t thermometerCharacteristicHandle;
  uint32_t temperature;
} ConnProperties;

// Flag for indicating DFU Reset must be performed
uint8_t bootToDfu;
// Array for holding properties of multiple (parallel) connections
ConnProperties connProperties;
// Counter of active connections
uint8_t activeConnectionsNum;
// State of the connection under establishment
ConnState connState;
// Health Thermometer service UUID defined by Bluetooth SIG
uint8_t thermoService[2]; //
// Temperature Measurement characteristic UUID defined by Bluetooth SIG
uint8_t thermoChar[2]; //= { 0x1c, 0x2a };

////////////////////////////
//#define DISP_UPDATE 	((uint32_t)(3))

//#include "src/i2c_tempsens.h"
uint32_t DISP_UPDATE;
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

#define TIME_PERIOD (1.00)//700//225//225   //700
//#define ON_TIME     (0.01)   //((uint32_t)(175/10))
#define COMP0   ((uint32_t)(0X01))
#define COMP1   ((uint32_t)(0X02))
#define TimerInt  ((uint32_t)(0X03))
#define SLEEP_MODE  sleepEM2
#define ButtonPress	(1<<5)
#define ButtonMask	0x40
//bool button0_status;
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
uint8_t MinConnTime,MaxConnTime,TimeoutVal;
uint8_t SlaveLatency;
uint8_t ConnectionHandle;
uint8_t Notifications_Status;
uint8_t Event_Read;
uint32_t LETIMER_Triggered;
uint32_t WRITE_COMPLETE;
uint32_t ERROR_I2C;
uint32_t I2C_INCOMPLETE;
uint32_t rssi_value;
int16_t BG13_Min_Power;
int16_t BG13_Max_Power;
uint8_t Active_Connection;
float temp;
uint8_t event_flag;
uint8_t count_write;
uint32_t Event_Mask;
bool printHeader;
uint8_t* charValue;
uint16_t addrValue;
uint8_t tableIndex;
uint32_t NoBond;
uint8_t value;
uint8_t value1;
uint8_t BondState;// = NoBond;
bool PassKeyEvent;// = false;
bool BondFailFlag;// = false;
uint8_t temp_Val;

bd_addr Server_Addr;// = { .addr =  {0xc0, 0x29, 0xef, 0x57, 0x0b, 0x00}};
//uint8_t already_initiated;
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
 uint32_t Comp0_Cal(float);
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
 void timerEnable1HzSchedulerEvent(uint32_t Scheduler_DisplayUpdate);
 //int16_t I2CPM_TempRead(void);
 struct gecko_cmd_packet* evt;
 struct gecko_msg_system_get_bt_address_rsp_t * AddressBLE;

 I2C_TransferReturn_TypeDef I2C_Status;


 //char* ErrorStates[]={"i2cTransferDone","i2cTransferInProgress","i2cTransferNack ","i2cTransferBusErr","i2cTransferArbLost","i2cTransferUsageFault","i2cTransferSwFault"};
/*****************************************************************/
#define Comp0_Cal() 	((uint32_t)(CMU_ClockFreqGet(cmuClock_LFA)/(CLOCK_DIV*TIME_PERIOD))) //
#define CounterGet(us_wait)   ((uint32_t)(((us_wait*0.000001*CMU_ClockFreqGet(cmuClock_LFA))/CLOCK_DIV)))
