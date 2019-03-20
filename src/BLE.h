/*
 * BLE.h
 *
 *  Created on: Mar 19, 2019
 *      Author: yashm
 */
#include "LETIMER.h"

uint8_t findIndexByConnectionHandle(uint8_t connection);
void addConnection(uint8_t connection, uint16_t address);
void removeConnection(uint8_t connection);
void gecko_custom_update(struct gecko_cmd_packet* evt);

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

typedef struct {
  uint8_t  connectionHandle;
  int8_t   rssi;
  uint16_t serverAddress;
  uint32_t thermometerServiceHandle;
  uint16_t thermometerCharacteristicHandle;
  uint32_t temperature;
  uint32_t ButtonServiceHandle;
  uint16_t ButtonCharacteristicHandle;
  uint32_t Button;
}ConnProperties;

ConnProperties connProperties;
typedef enum {
  scanning,
  opening,
  discoverServices,
  discoverCharacteristics,
  enableIndication,
  running
} ConnState;

ConnState connState;

uint16_t error;

#ifndef SRC_BLE_H_
#define SRC_BLE_H_



#endif /* SRC_BLE_H_ */
