/*
 * BLE.c
 *
 *  Created on: Mar 19, 2019
 *      Author: yashm
 */
#include "LETIMER.h"
#include "BLE.h"
#include "I2C.h"

char* ButtonPressString[]={"Button Released","Button Pressed"};

uint8_t findIndexByConnectionHandle(uint8_t connection)
{
    if (connProperties.connectionHandle != connection)
    	return TABLE_INDEX_INVALID;
    return 1;
}

void addConnection(uint8_t connection, uint16_t address)
{
  connProperties.connectionHandle = connection;
  connProperties.serverAddress    = address;
  //activeConnectionsNum++;
}

void removeConnection(uint8_t connection)
{
  uint8_t i;
  uint8_t table_index = findIndexByConnectionHandle(connection);

connProperties.connectionHandle = CONNECTION_HANDLE_INVALID;
connProperties.thermometerServiceHandle = SERVICE_HANDLE_INVALID;
connProperties.thermometerCharacteristicHandle = CHARACTERISTIC_HANDLE_INVALID;
connProperties.temperature = TEMP_INVALID;
connProperties.rssi = RSSI_INVALID;
connProperties.ButtonServiceHandle = SERVICE_HANDLE_INVALID;
connProperties.ButtonCharacteristicHandle = CHARACTERISTIC_HANDLE_INVALID;

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
    #if (DEVICE_IS_BLE_SERVER==CLIENT)			//Client
      LOG_INFO("System Initiated\n");
        // Set passive scanning on 1Mb PHY
        gecko_cmd_le_gap_set_discovery_type(le_gap_phy_1m, SCAN_PASSIVE);
        // Set scan interval and scan window
        gecko_cmd_le_gap_set_discovery_timing(le_gap_phy_1m, SCAN_INTERVAL, SCAN_WINDOW);

        gecko_cmd_le_gap_start_discovery(le_gap_phy_1m, le_gap_discover_generic);
        displayPrintf(DISPLAY_ROW_CONNECTION,"Scanning");
        displayPrintf(DISPLAY_ROW_NAME,"Client");
        displayPrintf(DISPLAY_ROW_BTADDR2,"%02x:%02x:%02x:%02x:%02x:%02x",Server_Addr.addr[5],
        		Server_Addr.addr[4],
				Server_Addr.addr[3],
				Server_Addr.addr[2],
				Server_Addr.addr[1],
				Server_Addr.addr[0]);
         gecko_cmd_sm_delete_bondings();
         gecko_cmd_sm_set_bondable_mode(1);
		 gecko_cmd_sm_configure(SMConfig, sm_io_capability_displayyesno);
         connState = scanning;
        #else

        gecko_cmd_system_set_tx_power(0);

		gecko_cmd_le_gap_set_advertise_timing(0, 400, 400, 0, 0);

		gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);
		displayPrintf(DISPLAY_ROW_CONNECTION,"Advertising");
		displayPrintf(DISPLAY_ROW_NAME,"Server");

		gecko_cmd_sm_delete_bondings();		//Custom_BLE_Server_Delete_Bondings();
		gecko_cmd_sm_set_passkey(-1);		// -1 for random passkey
		gecko_cmd_sm_set_bondable_mode(1);	// Bondable mode enable
		gecko_cmd_sm_configure(SMConfig, sm_io_capability_displayyesno);

        #endif

		AddressBLE = gecko_cmd_system_get_bt_address();
        displayPrintf(DISPLAY_ROW_BTADDR,"%02x:%02x:%02x:%02x:%02x:%02x",AddressBLE->address.addr[5],
        						AddressBLE->address.addr[4],
        						AddressBLE->address.addr[3],
        						AddressBLE->address.addr[2],
        						AddressBLE->address.addr[1],
        						AddressBLE->address.addr[0]);

        break;
/******************************************************************************
 @brief : This event is triggered when the client
 @func  : The LETIMER is diabled so that no interrupt triggers at 1s intervals.
******************************************************************************/

case gecko_evt_sm_confirm_passkey_id:

	PassKeyEvent = true;
	uint32_t key;
	key = evt->data.evt_sm_confirm_passkey.passkey; // reading passkey
	displayPrintf(DISPLAY_ROW_PASSKEY,"Passkey-%6d",key);
	LOG_INFO("Passkey-%6d",key);
	displayPrintf(DISPLAY_ROW_ACTION,"Confirm with PB0");
	break;

case gecko_evt_sm_bonded_id:
	displayPrintf(DISPLAY_ROW_PASSKEY,"Bonded Successfully");
	displayPrintf(DISPLAY_ROW_ACTION,"Connected");
#if(DEVICE_IS_BLE_SERVER==CLIENT)
	DisplayButtonState=1;
    LOG_INFO("Bonded\n");
#endif
	break;


case gecko_evt_sm_bonding_failed_id:

	displayPrintf(DISPLAY_ROW_PASSKEY,"Bonded UnSuccessfully");
	BondFailFlag = true;
	gecko_cmd_le_connection_close(ConnectionHandle);
	break;


case gecko_evt_le_gap_scan_response_id:

			LOG_INFO("Received Scan Response ID\n");
			if (evt->data.evt_le_gap_scan_response.packet_type == 0) {
				if(memcmp(&evt->data.evt_le_gap_scan_response.address.addr[0],
				&Server_Addr.addr[0], 6) == 0){
				gecko_cmd_le_gap_end_procedure();
				error=(gecko_cmd_le_gap_connect(evt->data.evt_le_gap_scan_response.address,
														evt->data.evt_le_gap_scan_response.address_type,
														le_gap_phy_1m))->result;
				if(error!=0)
				{
					LOG_INFO("Scan Response failed:%x",error);

				}
				connState = opening;

				}
			}
			break;


case gecko_evt_gatt_server_characteristic_status_id: //Server

		LOG_INFO("status_flags=%d",evt->data.evt_gatt_server_characteristic_status.status_flags);
			displayPrintf(DISPLAY_ROW_CONNECTION,"Handling Indications");
			               displayPrintf(DISPLAY_ROW_BTADDR2,"%02x:%02x:%02x:%02x:%02x:%02x",Server_Addr.addr[5],
        		Server_Addr.addr[4],
				Server_Addr.addr[3],
				Server_Addr.addr[2],
				Server_Addr.addr[1],
				Server_Addr.addr[0]);
	   AddressBLE = gecko_cmd_system_get_bt_address();
	   displayPrintf(DISPLAY_ROW_BTADDR,"%02x:%02x:%02x:%02x:%02x:%02x",AddressBLE->address.addr[5],
				AddressBLE->address.addr[4],
				AddressBLE->address.addr[3],
				AddressBLE->address.addr[2],
				AddressBLE->address.addr[1],
				AddressBLE->address.addr[0]);

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


case gecko_evt_le_connection_opened_id:

    	#if(DEVICE_IS_BLE_SERVER==CLIENT)
	    	LOG_INFO("Connection opened ID\n");
	        addrValue = (uint16_t)(evt->data.evt_le_connection_opened.address.addr[1] << 8) \
	                    + evt->data.evt_le_connection_opened.address.addr[0];
	        ConnectionHandle = evt->data.evt_le_connection_opened.connection;
	        addConnection(evt->data.evt_le_connection_opened.connection, addrValue);
	        // Discover Health Thermometer service on the slave device
	        error=(gecko_cmd_gatt_discover_primary_services_by_uuid(evt->data.evt_le_connection_opened.connection,
	                                                         2,
	                                                         (const uint8_t*)thermoService))->result;
	        if(error!=0)
			{
				LOG_INFO("Connection open failed at client:%x",error);

			}
	        connState = discoverServices;
        #else
        	ConnectionHandle = evt->data.evt_le_connection_opened.connection;

			gecko_cmd_le_connection_set_parameters(ConnectionHandle, MinConnTime, MaxConnTime,SlaveLatency,TimeoutVal);
			LETIMER_Enable(LETIMER0,true);
			displayPrintf(DISPLAY_ROW_CONNECTION,"Connected");
			displayPrintf(DISPLAY_ROW_NAME,"Server");
			LOG_INFO("Gecko BLE Connection start\n");

			BondState = evt->data.evt_le_connection_opened.bonding;	//Checks bonding status

			if(BondState != NoBond)								    // Successful Bonding condition
			{
				displayPrintf(DISPLAY_ROW_PASSKEY,"Already Bonded");
				displayPrintf(DISPLAY_ROW_ACTION,"Connected");
			}


		#endif
        break;

case gecko_evt_gatt_service_id:
			LOG_INFO("Service ID\n");
			tableIndex = findIndexByConnectionHandle(evt->data.evt_gatt_service.connection);
			if (tableIndex != TABLE_INDEX_INVALID) {
			// Save service handle for future reference
			if(ButtonInitiationFlag==0)
				connProperties.thermometerServiceHandle = evt->data.evt_gatt_service.service;
			else
				connProperties.ButtonServiceHandle = evt->data.evt_gatt_service.service;
			}
			break;

case gecko_evt_gatt_characteristic_id:
			LOG_INFO("Characteristic ID\n");
			tableIndex = findIndexByConnectionHandle(evt->data.evt_gatt_characteristic.connection);
			if (tableIndex != TABLE_INDEX_INVALID) {
			// Save characteristic handle for future reference
			if(ButtonInitiationFlag==0)
				connProperties.thermometerCharacteristicHandle = evt->data.evt_gatt_characteristic.characteristic;
			else
				connProperties.ButtonCharacteristicHandle = evt->data.evt_gatt_characteristic.characteristic;
			}
			break;

case gecko_evt_gatt_procedure_completed_id:				//Client Event

		LOG_INFO("Procedure Completed ID\n");
			tableIndex = findIndexByConnectionHandle(evt->data.evt_gatt_procedure_completed.connection);
			if (tableIndex == TABLE_INDEX_INVALID) {
				LOG_INFO("Failed Discovery\n");
			break;
			}

			if (connState == discoverServices ) {
			LOG_INFO("service discovery finished thermometer\n");

			// Discover thermometer characteristic on the slave device
			if(ButtonInitiationFlag==0 && connProperties.thermometerServiceHandle != SERVICE_HANDLE_INVALID)
			{
				ButtonInitiationFlag=1;
				error=(gecko_cmd_gatt_discover_primary_services_by_uuid(evt->data.evt_gatt_procedure_completed.connection,
					                                                         16,
					                                                         (const uint8_t*)ButtonService))->result;
				if(error!=0)
				{
					LOG_INFO("procedure completed failed:%x",error);

				}


			}
			else if(ButtonInitiationFlag==1 && connProperties.ButtonServiceHandle != SERVICE_HANDLE_INVALID){
			LOG_INFO("service discovery finished button\n");
			ButtonInitiationFlag=0;
			error=(gecko_cmd_gatt_discover_characteristics_by_uuid(evt->data.evt_gatt_procedure_completed.connection,
											  connProperties.thermometerServiceHandle,
											  2,
											  (const uint8_t*)thermoChar))->result;
			if(error!=0)
			{
				LOG_INFO("procedure completed failed:%x",error);

			}

			connState = discoverCharacteristics;			//Update the state only after both the conditions have been met.
			}

			break;
			}

			if (connState == discoverCharacteristics ){

			//connState = discoverCharacteristics;
			if(ButtonInitiationFlag == 0 && connProperties.thermometerCharacteristicHandle != CHARACTERISTIC_HANDLE_INVALID){
				ButtonInitiationFlag=1;
			// Discover thermometer characteristic on the slave device
				error=(gecko_cmd_gatt_discover_characteristics_by_uuid(evt->data.evt_gatt_procedure_completed.connection,
											  connProperties.ButtonServiceHandle,
											  16,
											  (const uint8_t*)ButtonChar))->result;
				if(error!=0)
							{
								LOG_INFO("procedure completed failed:%x",error);

							}
			}
			else if(ButtonInitiationFlag == 1 && connProperties.ButtonCharacteristicHandle != CHARACTERISTIC_HANDLE_INVALID){
				ButtonInitiationFlag=0;
				error=(gecko_cmd_gatt_set_characteristic_notification(evt->data.evt_gatt_procedure_completed.connection,
												 connProperties.thermometerCharacteristicHandle,
												 gatt_indication))->result;
				if(error!=0)
				{
					LOG_INFO("procedure completed failed:%x",error);

				}
				connState = enableIndication;
			}
			break;
			}

			if (connState == enableIndication) {
				if(ButtonInitiationFlag==0)
				{
					ButtonInitiationFlag=1;
					error=(gecko_cmd_gatt_set_characteristic_notification(evt->data.evt_gatt_procedure_completed.connection,
													 connProperties.ButtonCharacteristicHandle,
													 gatt_indication))->result;
					if(error!=0)
								{
									LOG_INFO("procedure completed failed:%x",error);

								}
				}
				else
				{
					ButtonInitiationFlag=0;
					error=(gecko_cmd_sm_increase_security(evt->data.evt_gatt_procedure_completed.connection))->result;
					if(error!=0)
								{
									LOG_INFO("procedure completed failed:%x",error);

								}
					connState = running;
					LOG_INFO("Indication set\n");
				}

			}
			break;


case gecko_evt_le_connection_closed_id:
        #if(DEVICE_IS_BLE_SERVER==CLIENT)
        LOG_INFO("connection closed ID\n");
 	   AddressBLE = gecko_cmd_system_get_bt_address();
 	   displayPrintf(DISPLAY_ROW_BTADDR,"%02x:%02x:%02x:%02x:%02x:%02x",AddressBLE->address.addr[5],
 				AddressBLE->address.addr[4],
 				AddressBLE->address.addr[3],
 				AddressBLE->address.addr[2],
 				AddressBLE->address.addr[1],
 				AddressBLE->address.addr[0]);
		displayPrintf(DISPLAY_ROW_CONNECTION,"Discovering");
		               displayPrintf(DISPLAY_ROW_BTADDR2,"%02x:%02x:%02x:%02x:%02x:%02x",Server_Addr.addr[5],
        		Server_Addr.addr[4],
				Server_Addr.addr[3],
				Server_Addr.addr[2],
				Server_Addr.addr[1],
				Server_Addr.addr[0]);
       	DisplayButtonState=0;
		displayPrintf(DISPLAY_ROW_NAME,"Client");
		displayPrintf(DISPLAY_ROW_TEMPVALUE,"%s"," ");
		displayPrintf(DISPLAY_ROW_PASSKEY,"%s"," ");
		displayPrintf(DISPLAY_ROW_ACTION,"%s"," ");
        removeConnection(evt->data.evt_le_connection_closed.connection);
        gecko_cmd_le_gap_start_discovery(le_gap_phy_1m, le_gap_discover_generic);
        connState = scanning;

        #else
			Active_Connection = 0;

			gecko_cmd_system_set_tx_power(0);

			LETIMER_Enable(LETIMER0, false);

			gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);
			LOG_INFO("Gecko BLE Connection end\n");
			displayPrintf(DISPLAY_ROW_CONNECTION,"Advertising");
			displayPrintf(DISPLAY_ROW_NAME,"Server");
			displayPrintf(DISPLAY_ROW_TEMPVALUE,"%s"," ");
			displayPrintf(DISPLAY_ROW_PASSKEY," ");
			displayPrintf(DISPLAY_ROW_ACTION," ");

        #endif
        break;

 case gecko_evt_gatt_characteristic_value_id:

			LOG_INFO("characteristic values received\n");
			   AddressBLE = gecko_cmd_system_get_bt_address();
			   displayPrintf(DISPLAY_ROW_BTADDR,"%02x:%02x:%02x:%02x:%02x:%02x",AddressBLE->address.addr[5],
						AddressBLE->address.addr[4],
						AddressBLE->address.addr[3],
						AddressBLE->address.addr[2],
						AddressBLE->address.addr[1],
						AddressBLE->address.addr[0]);
			displayPrintf(DISPLAY_ROW_CONNECTION,"Handling Indications");
			               displayPrintf(DISPLAY_ROW_BTADDR2,"%02x:%02x:%02x:%02x:%02x:%02x",Server_Addr.addr[5],
        		Server_Addr.addr[4],
				Server_Addr.addr[3],
				Server_Addr.addr[2],
				Server_Addr.addr[1],
				Server_Addr.addr[0]);
			charValue = &(evt->data.evt_gatt_characteristic_value.value.data[0]);
			tableIndex = findIndexByConnectionHandle(evt->data.evt_gatt_characteristic_value.connection);
			if (tableIndex != TABLE_INDEX_INVALID) {

				if(connProperties.thermometerCharacteristicHandle==evt->data.evt_gatt_characteristic_value.characteristic)
				{
					LOG_INFO("%d%d%d%d",charValue[0],charValue[1],charValue[2],charValue[3],charValue[4]);
					connProperties.temperature = (charValue[1] << 0) + (charValue[2] << 8) + (charValue[3] << 16);
				}
				else if(connProperties.ButtonCharacteristicHandle==evt->data.evt_gatt_characteristic_value.characteristic)
				{
					LOG_INFO("%d%d%d%d",charValue[0],charValue[1],charValue[2],charValue[3],charValue[4]);
					connProperties.Button = charValue[1];
				}

			}
			// Send confirmation for the indication
			error=(gecko_cmd_gatt_send_characteristic_confirmation(evt->data.evt_gatt_characteristic_value.connection))->result;
			if(error!=0)
			{
				LOG_INFO("characteristic value id failed:%x",error);

			}
			// Trigger RSSI measurement on the connection
			//
			error=(gecko_cmd_le_connection_get_rssi(evt->data.evt_gatt_characteristic_value.connection))->result;
			if(error!=0)
			{
				LOG_INFO("characteristic value id failed:%x",error);

			}
			break;


case gecko_evt_le_connection_rssi_id:
      #if (DEVICE_IS_BLE_SERVER==CLIENT) //Client
       LOG_INFO("RSSI measured\n");
        tableIndex = findIndexByConnectionHandle(evt->data.evt_le_connection_rssi.connection);
        if (tableIndex != TABLE_INDEX_INVALID) {
          connProperties.rssi = evt->data.evt_le_connection_rssi.rssi;
        }
        // Trigger printing
         gecko_external_signal(EXT_SIGNAL_PRINT_RESULTS);


        #else
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

        #endif
        break;

		/******************************************************************************
		@brief : This event is triggered when an interrupt occurs.
		An Event Mask is set to select the source of interrupt.
		@func : Handles interrupts from LETIMER and I2C interrupts.
		******************************************************************************/

		case gecko_evt_system_external_signal_id:

			    	Event_Read = evt->data.evt_system_external_signal.extsignals;
			    	#if (DEVICE_IS_BLE_SERVER==CLIENT) //Client
			    	if(Event_Read & ButtonPress)	//8
					{
						/* Begin Critical Section */
						CORE_DECLARE_IRQ_STATE;
						CORE_ENTER_CRITICAL();
							Event_Mask &= ~ButtonPress;				//Clear the Event Mask
						CORE_EXIT_CRITICAL();
						/* End Critical Section */
						displayUpdate();
						error=(gecko_cmd_sm_passkey_confirm(ConnectionHandle, true))->result;
						if(error!=0)
						{
							LOG_INFO("passkey confirm failed:%x",error);

						}

					}

					if (Event_Read & EXT_SIGNAL_PRINT_RESULTS) {

						CORE_DECLARE_IRQ_STATE;
			    		CORE_ENTER_CRITICAL();
			    			Event_Mask &= ~EXT_SIGNAL_PRINT_RESULTS;				//Clear the Event Mask
			    		CORE_EXIT_CRITICAL();

						if(connProperties.temperature != TEMP_INVALID)
							displayPrintf(DISPLAY_ROW_TEMPVALUE,"%f",(float)connProperties.temperature*0.001);
						if(DisplayButtonState==1)
							displayPrintf(DISPLAY_ROW_ACTION,"%s",ButtonPressString[(uint8_t)connProperties.Button]);

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
			    	break;
			    	#else //Server
			    	if(Event_Read & ButtonPress)	//8
					{
						/* Begin Critical Section */
						CORE_DECLARE_IRQ_STATE;
						CORE_ENTER_CRITICAL();
							Event_Mask &= ~ButtonPress;				//Clear the Event Mask
						CORE_EXIT_CRITICAL();
						/* End Critical Section */
						displayUpdate();
						//edit here

						if(PassKeyEvent == true)
						{
							PassKeyEvent = false;
							error=gecko_cmd_sm_passkey_confirm(ConnectionHandle, true);
							if(error!=0)
							{
								LOG_INFO("passkey failed:%x",error);

							}
							displayPrintf(DISPLAY_ROW_ACTION,"PassKey Accepted");
						}
						else
						{
							uint8_t *ptr = &value;
							uint8_t BTN_BUFF[2]; 		// Buffer to store the data  - Button status and Flags.
							uint8_t BTN_flags = 0x00;   // Flags for Button State.
							uint8_t *p = BTN_BUFF; 		// Pointer to store the BTN_BUFF buffer.

							UINT8_TO_BITSTREAM(p, BTN_flags);
							UINT8_TO_BITSTREAM(p, value);

							error=gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_button_state,2,BTN_BUFF);
							if(error!=0)
							{
								LOG_INFO("charac notification id failed:%x",error);

							}
							displayPrintf(DISPLAY_ROW_ACTION,"Button:%d",value);

						}

						LOG_INFO("Disp update 1s\n");
					}
			    	if(Active_Connection==1)
			    	{
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

			    }
			    	#endif

			break;

    }

  }
