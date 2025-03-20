/***************************************************************************//**
 * @file app.c
 * @brief Core application logic for the vendor server node.
 *******************************************************************************
 * # License
 * <b>Copyright 2022 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************
 * # Experimental Quality
 * This code has not been formally tested and is provided as-is. It is not
 * suitable for production environments. In addition, this code will not be
 * maintained and there may be no bug maintenance planned for these resources.
 * Silicon Labs may update projects from time to time.
 ******************************************************************************/


// DOS: For ECEN 5823 this is the code for a BT Mesh Server/Friend.
//      Note: The number of Friends this node supports is defined in:
//      Software Component: "Bluetooth Mesh Stack". Adjusting the number of
//      Friends (or other values) re-generates:
//        config/sl_btmesh_config.h -> #define SL_BTMESH_CONFIG_MAX_FRIENDSHIPS       (4)


#include <stdio.h> // DOS: for snprintf()

#include "em_common.h"
#include "app_assert.h"
#include "app_log.h"
#include "sl_status.h"
#include "app.h"

#include "sl_btmesh_api.h"
#include "sl_bt_api.h"
#include "sl_simple_timer.h"
#include "sl_sensor_rht.h"

#include "em_cmu.h"
#include "em_gpio.h"

#include "my_model_def.h"

#include "app_button_press.h"
#include "sl_simple_button.h"
#include "sl_simple_button_instances.h"

#include "sl_btmesh_wstk_lcd.h"

#include "em_gpio.h" // DOS: for gpio


#ifdef PROV_LOCALLY
// Group Addresses
// Choose any 16-bit address starting at 0xC000
#define CUSTOM_STATUS_GRP_ADDR                      0xC001  // Server PUB address
#define CUSTOM_CTRL_GRP_ADDR                        0xC002  // Server SUB address

// The default settings of the network and the node
#define NET_KEY_IDX                                 0
#define APP_KEY_IDX                                 0
#define IVI                                         0
#define DEFAULT_TTL                                 5
// #define ELEMENT_ID
#endif // #ifdef PROV_LOCALLY

#define EX_B0_PRESS                                 ((1) << 5)
#define EX_B1_PRESS                                 ((1) << 6)

// Timing
// Check section 4.2.2.2 of Mesh Profile Specification 1.0 for format
#define STEP_RES_100_MILLI                          0
#define STEP_RES_1_SEC                              ((1) << 6)
#define STEP_RES_10_SEC                             ((2) << 6)
#define STEP_RES_10_MIN                             ((3) << 6)

#define STEP_RES_BIT_MASK                           0xC0

/// Advertising Provisioning Bearer
#define PB_ADV                                      0x1
/// GATT Provisioning Bearer
#define PB_GATT                                     0x2

// Used button indexes
#define BUTTON_PRESS_BUTTON_0                       0
#define BUTTON_PRESS_BUTTON_1                       1

static uint8_t temperature[TEMP_DATA_LENGTH] = {0, 0, 0, 0}; // DOS: temperature measurement that the server maintains
static unit_t unit[UNIT_DATA_LENGTH] = {
  celsius
};
// Check section 4.2.2.2 of Mesh Profile Specification 1.0 for format
static uint8_t update_interval[UPDATE_INTERVAL_LENGTH] = {
  0
};

static uint32_t periodic_timer_ms = 0;

static my_model_t my_model = { // DOS: declare storage for saving important model values
  .elem_index = PRIMARY_ELEMENT,
  .vendor_id = MY_VENDOR_ID,
  .model_id = MY_MODEL_SERVER_ID,
  .publish = 1,
  .opcodes_len = NUMBER_OF_OPCODES,
  .opcodes_data[0] = temperature_get,
  .opcodes_data[1] = temperature_status,
  .opcodes_data[2] = unit_get,
  .opcodes_data[3] = unit_set,
  .opcodes_data[4] = unit_set_unack,
  .opcodes_data[5] = unit_status,
  .opcodes_data[6] = update_interval_get,
  .opcodes_data[7] = update_interval_set,
  .opcodes_data[8] = update_interval_set_unack,
  .opcodes_data[9] = update_interval_status
};


/// Local provisioning
#ifdef PROV_LOCALLY
static uint16_t uni_addr = 0;

static aes_key_128 enc_key = {
// DOS: encryption key to use for provisioning. This is terribly insecure
//      >>> NEVER DO THIS FOR A PRODUCTION PRODUCT!!! <<<
  .data = "\x03\x03\x03\x03\x03\x03\x03\x03\x03\x03\x03\x03\x03\x03\x03\x03"
};
#endif /* #ifdef PROV_LOCALLY */


// Globals
bd_addr          myAddress;
uint8_t          myAddressType;


static void factory_reset(void);
static void read_temperature(void);
static void setup_periodcal_update(uint8_t interval);
static void delay_reset_ms(uint32_t ms);



// DOS:
// global
uint32_t logging_timestamp = 0; // in ms

/**************************************************************************//**
 * Logging timer callback
 *****************************************************************************/
static void logging_timer_cb(sl_simple_timer_t *handle, void *data)
{
  (void)handle;
  (void)data;

  logging_timestamp += 500; // increment the timestamp
}

// DOS:
/**************************************************************************//**
 * Public function to retrieve the timestamp
 *****************************************************************************/
uint32_t get_logger_timestamp() {
  return logging_timestamp;
}


/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{
  app_log("=================\r\n");
  app_log("Server/Friend\r\n");
  app_button_press_enable();

  // DOS: For LCD
  GPIO_PinModeSet(Si7021SENSOR_EN_port, Si7021SENSOR_EN_pin, gpioModePushPull, false);
  GPIO_PinOutSet(Si7021SENSOR_EN_port, Si7021SENSOR_EN_pin);

  char   device_type[LCD_ROW_LEN];
  memset(device_type, 0, LCD_ROW_LEN);
  // handle % format, no printf() capability in sl_btmesh_LCD_write(), by
  // using snprintf() prior to calling sl_btmesh_LCD_write() as snprintf() can
  // handle all of the % format conversion characters
  snprintf(device_type, LCD_ROW_LEN, "Server/Friend");
  sl_btmesh_LCD_write(device_type, LCD_ROW_1);

  // DOS:
  // Run a periodic timer for logging timestamp
  static sl_simple_timer_t logging_timer;
  sl_simple_timer_start(&logging_timer,
                        500,   // every 500 ms
                        logging_timer_cb, // a callback that increments a timestamp for logging
                        NULL,  // pointer to callback data
                        true); // is periodic

}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application code here!                              //
  // This is called infinitely.                                              //
  // Do not call blocking functions from here!                               //
  /////////////////////////////////////////////////////////////////////////////
}



/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(struct sl_bt_msg *evt)
{
  sl_status_t sc;

  switch (SL_BT_MSG_ID(evt->header)) {

    case sl_bt_evt_system_boot_id:
//      // Factory reset the device if Button 0 or 1 is being pressed during reset
//      if((sl_simple_button_get_state(&sl_button_btn0) == SL_SIMPLE_BUTTON_PRESSED) || (sl_simple_button_get_state(&sl_button_btn1) == SL_SIMPLE_BUTTON_PRESSED)) {
//          factory_reset();
//          break;
//      }
      // DOS ----------------------------------------
      // Hold down PB0, then press and release the reset button on the main PCB,
      // lastly release PB0.
      if (GPIO_PinInGet (PB0_port, PB0_pin) == 0) {

          while (GPIO_PinInGet (PB0_port, PB0_pin) == 0) {
              ; // stay here while PB0 is pressed
          }

          // We get here on the release of PB0
          factory_reset();

      }
      // DOS ----------------------------------------

      // Initialize Mesh stack in Node operation mode,
      // wait for initialized event
      app_log("Node init\r\n");
      sc = sl_btmesh_node_init();
      app_assert_status_f(sc, "Failed to init node\r\n");

      // get our BT Device Address and type
      // address type: 0: public
      //               1: static random
      sc = sl_bt_system_get_identity_address(&myAddress, &myAddressType);
      if (sc != SL_STATUS_OK) {
          app_log("sl_bt_system_get_identity_address() returned != 0 status=0x%04x", (unsigned int) sc);
      } else {
          char   myAddressStr[LCD_ROW_LEN];
          memset(myAddressStr, 0, LCD_ROW_LEN);
          // handle % format, no printf() capability in sl_btmesh_LCD_write(), by
          // using snprintf() prior to calling sl_btmesh_LCD_write() as snprintf() can
          // handle all of the % format conversion characters
          snprintf(myAddressStr, LCD_ROW_LEN, "%02x:%02x:%02x:%02x:%02x:%02x",
                   myAddress.addr[5],
                   myAddress.addr[4],
                   myAddress.addr[3],
                   myAddress.addr[2],
                   myAddress.addr[1],
                   myAddress.addr[0]
                   );
          sl_btmesh_LCD_write(myAddressStr, LCD_ROW_4);
      }

      break;

    // -------------------------------
    // Handle Button Presses
    // DOS: See app_button_press_cb() below.
    case sl_bt_evt_system_external_signal_id: {

      uint8_t opcode = 0, length = 0, *data = NULL;

      // check if external signal triggered by button 0 press
      if(evt->data.evt_system_external_signal.extsignals & EX_B0_PRESS) {
          read_temperature(); // DOS: This reads the temp & publishes to the Client, see publish below
          opcode = temperature_status;
          length = TEMP_DATA_LENGTH;
          data = temperature;
          app_log("B0 Pressed.\r\n");
      }

      // check if external signal triggered by button 1 press
      if(evt->data.evt_system_external_signal.extsignals & EX_B1_PRESS) {
          opcode = unit_status;
          length = UNIT_DATA_LENGTH;
          data = unit;
          app_log("B1 Pressed.\r\n");
      }

      // DOS: Set the message data to publish, i.e. queue the data for publishing.
      //      Publish temperature or publish units to the Client.
      sc = sl_btmesh_vendor_model_set_publication(my_model.elem_index,
                                                  my_model.vendor_id,
                                                  my_model.model_id,
                                                  opcode,
                                                  1, // DOS: the final payload "chunk"
                                                  length,
                                                  data);
      if(sc != SL_STATUS_OK) {
          app_log("Set publication error: 0x%04X\r\n", sc);
      } else {
          app_log("Set publication done. Publishing...\r\n");
          // DOS: Publish the queued message to the group address.
          sc = sl_btmesh_vendor_model_publish(my_model.elem_index,
                                              my_model.vendor_id,
                                              my_model.model_id);
          if(sc != SL_STATUS_OK) {
              app_log("Publish error: 0x%04X\r\n", sc);
          } else {
              app_log("Publish done push.\r\n");
          }
      }
      break;
    }

    // -------------------------------
    // Default event handler.
    default:
      break;

  } // switch

} // sl_bt_on_event()

/**************************************************************************//**
 * Bluetooth Mesh stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth Mesh stack.
 *****************************************************************************/
void sl_btmesh_on_event(sl_btmesh_msg_t *evt)
{
  sl_status_t      sc;
  char             friend_status[LCD_ROW_LEN];
  static int       number_of_friendships = 0; // count of number of current friendships


  switch (SL_BT_MSG_ID(evt->header)) {

    case sl_btmesh_evt_node_initialized_id:
      app_log("Node initialized ...\r\n");

      // DOS: Init the vendor model
      sc = sl_btmesh_vendor_model_init(my_model.elem_index,
                                       my_model.vendor_id,
                                       my_model.model_id,
                                       my_model.publish,
                                       my_model.opcodes_len,
                                       my_model.opcodes_data);
      app_assert_status_f(sc, "Failed to initialize vendor model\r\n");

      if(evt->data.evt_node_initialized.provisioned) {
          app_log("Node already provisioned.\r\n");
      } else {
          // Start unprovisioned Beaconing using PB-ADV and PB-GATT Bearers
          app_log("Node unprovisioned\r\n");


#ifdef PROV_LOCALLY
          // Derive the unicast address from the LSB 2 bytes from the BD_ADDR
          bd_addr address;
          sc = sl_bt_system_get_identity_address(&address, 0);
          uni_addr = ((address.addr[1] << 8) | address.addr[0]) & 0x7FFF;
          app_log("Unicast Address = 0x%04X\r\n", uni_addr);
          app_log("Provisioning itself.\r\n");
          // DOS: The device must be reset after this command has been issued,
          //      see delay_reset_ms() below.
          sc = sl_btmesh_node_set_provisioning_data(enc_key, // DOS: device key
                                                    enc_key, // DOS: network key
                                                    NET_KEY_IDX,
                                                    IVI,
                                                    uni_addr,
                                                    0);
          app_assert_status_f(sc, "Failed to provision itself\r\n");
          delay_reset_ms(100);
          break;

#else
        app_log("Send unprovisioned beacons.\r\n");
        sc = sl_btmesh_node_start_unprov_beaconing(PB_ADV | PB_GATT);
        app_assert_status_f(sc, "Failed to start unprovisioned beaconing\r\n");
#endif // #ifdef PROV_LOCALLY
      }

#ifdef PROV_LOCALLY
      // Set the publication and subscription
      uint16_t appkey_index;
      uint16_t pub_address;
      uint8_t ttl;
      uint8_t period;
      uint8_t retrans;
      uint8_t credentials;
      sc = sl_btmesh_test_get_local_model_pub(my_model.elem_index,
                                              my_model.vendor_id,
                                              my_model.model_id,
                                              &appkey_index,
                                              &pub_address,
                                              &ttl,
                                              &period,
                                              &retrans,
                                              &credentials);
      if (!sc && pub_address == CUSTOM_STATUS_GRP_ADDR) {
        app_log("Configuration done already.\r\n");
      } else {
        app_log("Pub setting result = 0x%04X, pub setting address = 0x%04X\r\n", sc, pub_address);
        app_log("Add local app key ...\r\n");
        sc = sl_btmesh_test_add_local_key(1,
                                          enc_key, // DOS: app key
                                          APP_KEY_IDX,
                                          NET_KEY_IDX);
        app_assert_status_f(sc, "Failed to add local app key\r\n");

        app_log("Bind local app key ...\r\n");
        sc = sl_btmesh_test_bind_local_model_app(my_model.elem_index,
                                                 APP_KEY_IDX,
                                                 my_model.vendor_id,
                                                 my_model.model_id);
        app_assert_status_f(sc, "Failed to bind local app key\r\n");

        app_log("Set local model pub ...\r\n");
        sc = sl_btmesh_test_set_local_model_pub(my_model.elem_index,
                                                APP_KEY_IDX,
                                                my_model.vendor_id,
                                                my_model.model_id,
                                                CUSTOM_STATUS_GRP_ADDR,
                                                DEFAULT_TTL,
                                                0, 0, 0);
        app_assert_status_f(sc, "Failed to set local model pub\r\n");

        app_log("Add local model sub ...\r\n");
        sc = sl_btmesh_test_add_local_model_sub(my_model.elem_index,
                                                my_model.vendor_id,
                                                my_model.model_id,
                                                CUSTOM_CTRL_GRP_ADDR);
        app_assert_status_f(sc, "Failed to add local model sub\r\n");

        app_log("Set relay ...\r\n");
        sc = sl_btmesh_test_set_relay(1, 0, 0);
        app_assert_status_f(sc, "Failed to set relay\r\n");

        app_log("Set Network tx state.\r\n");
        sc = sl_btmesh_test_set_nettx(2, 4);
        app_assert_status_f(sc, "Failed to set network tx state\r\n");
      }
#endif // #ifdef PROV_LOCALLY



      break;

    // -------------------------------
    // Provisioning Events
    case sl_btmesh_evt_node_provisioned_id:
      app_log("Provisioning done.\r\n");
      break;

    case sl_btmesh_evt_node_provisioning_failed_id:
      app_log("Provisioning failed. Result = 0x%04x\r\n",
              evt->data.evt_node_provisioning_failed.result);
      break;

    case sl_btmesh_evt_node_provisioning_started_id:
      app_log("Provisioning started.\r\n");
      break;

    case sl_btmesh_evt_node_key_added_id:
      app_log("got new %s key with index %x\r\n",
              evt->data.evt_node_key_added.type == 0 ? "network" : "application",
              evt->data.evt_node_key_added.index);
      break;

    case sl_btmesh_evt_node_config_set_id:
      app_log("evt_node_config_set_id\r\n\t");
      break;

    case sl_btmesh_evt_node_model_config_changed_id:
      app_log("model config changed, type: %d, elem_addr: %x, model_id: %x, vendor_id: %x\r\n",
              evt->data.evt_node_model_config_changed.node_config_state,
              evt->data.evt_node_model_config_changed.element_address,
              evt->data.evt_node_model_config_changed.model_id,
              evt->data.evt_node_model_config_changed.vendor_id);
      break;

    // -------------------------------
    // Handle vendor model messages
    case sl_btmesh_evt_vendor_model_receive_id: {

      // DOS: get the pointer to the vendor message
      sl_btmesh_evt_vendor_model_receive_t *rx_evt = (sl_btmesh_evt_vendor_model_receive_t *)&evt->data;

      uint8_t action_req = 0, opcode = 0, payload_len = 0, *payload_data = NULL;

      // DOS: log the received data
      app_log("Server: Vendor model data received.\r\n"
              "  Element index = %d\r\n"
              "  Vendor id = 0x%04X\r\n"
              "  Model id = 0x%04X\r\n"
              "  Source address = 0x%04X\r\n"
              "  Destination address = 0x%04X\r\n"
              "  Destination label UUID index = 0x%02X\r\n"
              "  App key index = 0x%04X\r\n"
              "  Non-relayed = 0x%02X\r\n"
              "  Opcode = 0x%02X\r\n"
              "  Final = 0x%04X\r\n"
              "  Payload: ",
              rx_evt->elem_index,
              rx_evt->vendor_id,
              rx_evt->model_id,
              rx_evt->source_address,
              rx_evt->destination_address,
              rx_evt->va_index,
              rx_evt->appkey_index,
              rx_evt->nonrelayed,
              rx_evt->opcode,
              rx_evt->final);
      for(int i = 0; i < evt->data.evt_vendor_model_receive.payload.len; i++) {
          app_log("%x ", evt->data.evt_vendor_model_receive.payload.data[i]);
      }
      app_log("\r\n");

      // DOS: Handle the opcode from the Client.
      //      This is the code that handles messages/requests/commands from the Client.
      switch (rx_evt->opcode) {

        // Server
        case temperature_get:
          app_log("Sending/publishing temperature status as response to "
                  "temperature get from client...\r\n");
          // DOS: This reads the temp & publishes to the Client in response to the "temperature_get" message.
          read_temperature();
          action_req = ACK_REQ;
          opcode = temperature_status;
          payload_len = TEMP_DATA_LENGTH;
          payload_data = temperature;
          break;

        // Server
        case unit_get:
          app_log("Sending/publishing unit status as response to unit get from "
                  "client...\r\n");
          action_req = ACK_REQ;
          opcode = unit_status;
          payload_len = UNIT_DATA_LENGTH;
          payload_data = (uint8_t *) unit;
          break;

        // Server
        case unit_set:
          app_log("Sending/publishing unit status as response to unit set from "
                  "client...\r\n");
          memcpy(unit, rx_evt->payload.data, rx_evt->payload.len);
          action_req = ACK_REQ | STATUS_UPDATE_REQ;
          opcode = unit_status;
          payload_len = UNIT_DATA_LENGTH;
          payload_data = (uint8_t *) unit;
          break;

        // Server
        case unit_set_unack:
          app_log("Publishing unit status as response to unit set unacknowledged "
                  "from client...\r\n");
          memcpy(unit, rx_evt->payload.data, rx_evt->payload.len);
          action_req = STATUS_UPDATE_REQ;
          opcode = unit_status;
          payload_len = UNIT_DATA_LENGTH;
          payload_data = (uint8_t *) unit;
          break;

          // DOS: Not used in the Client
//        case update_interval_get:
//          app_log("Publishing Update Interval status as response to Update "
//                  "interval get from client...\r\n");
//          action_req = ACK_REQ;
//          opcode = update_interval_status;
//          payload_len = UPDATE_INTERVAL_LENGTH;
//          payload_data = update_interval;
//          break;

          // DOS: Not used in the Client
//        case update_interval_set:
//          app_log("Publishing Update Interval (%d) status as response to "
//                  "update_interval_set from client...\r\n", update_interval[0]); // DOS
//          memcpy(update_interval,
//                 rx_evt->payload.data,
//                 rx_evt->payload.len);
//          action_req = ACK_REQ | STATUS_UPDATE_REQ;
//          opcode = update_interval_status;
//          payload_len = UPDATE_INTERVAL_LENGTH;
//          payload_data = update_interval;
//          setup_periodcal_update(update_interval[0]);
//          break;

        case update_interval_set_unack:
          memcpy(update_interval,       // destination
                 rx_evt->payload.data,  // source
                 rx_evt->payload.len);
          app_log("Publishing Update Interval (0x%x) status as response to "
                            "update_interval_set_unack from client...\r\n", update_interval[0]); // DOS
          action_req = STATUS_UPDATE_REQ;
          opcode = update_interval_status;
          payload_len = UPDATE_INTERVAL_LENGTH;
          payload_data = update_interval;
          setup_periodcal_update(update_interval[0]);
          break;

        // Add more cases here if more opcodes are defined
        default:
          break;

      } // switch (rx_evt->opcode)

      if(action_req & ACK_REQ) {
          // DOS: Send an Acknowledge to the Client
          //      *** This function sends a message to a specific destination/node.
          
          // Student edit: Un-comment this code to send the Acknowledge message back to the specific Client node.
          
//           sc = sl_btmesh_vendor_model_send(rx_evt->source_address, // destination address, use evt source address to send to
//                                            rx_evt->va_index,
//                                            rx_evt->appkey_index,
//                                            my_model.elem_index,
//                                            my_model.vendor_id,
//                                            my_model.model_id,
//                                            rx_evt->nonrelayed,
//                                            opcode,
//                                            1,
//                                            payload_len,
//                                            payload_data);
//           // Errors that are returned from this function are usually due to low
//           // memory. Low memory is non-critical and we can try sending again later.
//           if(sc != SL_STATUS_OK) {
//               app_log("Vendor model send error: 0x%04X\r\n", sc);
//           } else {
//               app_log("Acknowledge sent.\r\n");
//           }

       } // if

      if(action_req & STATUS_UPDATE_REQ) {
          app_log("Publishing status update.\r\n");
          // DOS: Set the message data to publish, i.e. queue the data for publishing.
          sc = sl_btmesh_vendor_model_set_publication(my_model.elem_index,
                                                      my_model.vendor_id,
                                                      my_model.model_id,
                                                      opcode,
                                                      1,  // DOS: the final payload "chunk"
                                                      payload_len,
                                                      payload_data);
          if(sc != SL_STATUS_OK) {
              app_log("Set publication error: 0x%04X\r\n", sc);
          } else {
              app_log("Set publication done. Publishing ...\r\n");
              // DOS: Publish the queued message.
              //      *** This function sends (broadcasts) to the group address.
              //          All nodes subscribing to messages in the group, receive this message.
              sc = sl_btmesh_vendor_model_publish(my_model.elem_index,
                                                  my_model.vendor_id,
                                                  my_model.model_id);
              if(sc != SL_STATUS_OK) {
                  app_log("Publish error: 0x%04X\r\n", sc);
              } else {
                  app_log("Status Update Publish done.\r\n");
              }
          }
      }

      app_log("\r\n"); // DOS, added an extra blank line after message reception

      break;

    } // case sl_btmesh_evt_vendor_model_receive_id:


    // ----------------------------------------
    // Handle vendor Friend - Friendship events

    case sl_btmesh_evt_friend_friendship_established_id:
      number_of_friendships++;

      memset(friend_status, 0, LCD_ROW_LEN);
      snprintf(friend_status, LCD_ROW_LEN, "Friend Est."); // friendship established
      sl_btmesh_LCD_write(friend_status, LCD_ROW_2);

      memset(friend_status, 0, LCD_ROW_LEN);
      snprintf(friend_status, LCD_ROW_LEN, "NumFriends=%d", number_of_friendships); // number of friendships
      sl_btmesh_LCD_write(friend_status, LCD_ROW_3);

      app_log("  ***Friendship Established\r\n"); // DOS

      break;

    case sl_btmesh_evt_friend_friendship_terminated_id:
      number_of_friendships--;

      memset(friend_status, 0, LCD_ROW_LEN);
      snprintf(friend_status, LCD_ROW_LEN, "Friend Term."); // friendship terminated
      sl_btmesh_LCD_write(friend_status, LCD_ROW_2);

      memset(friend_status, 0, LCD_ROW_LEN);
      snprintf(friend_status, LCD_ROW_LEN, "NumFriends=%d", number_of_friendships); // number of friendships
      sl_btmesh_LCD_write(friend_status, LCD_ROW_3);

      app_log("  ***Friendship terminated\r\n"); // DOS

      break;


    // -------------------------------
    // Default event handler.
    default:
      break;

  } // switch

} // sl_btmesh_on_event()


// DOS: The App / Utility / Button Press (app_button_press) software component
//      implements a callback scheme for handling button press interrupts. The
//      duration of the presses are measured starting from the "press" until
//      the "release". So we're really getting "button release" interrupts.
//      This callback is called on the release, and the code here switch/cases on
//      the duration of the press and calls sl_bt_external_signal() which is handled
//      in sl_bt_on_event() and the sl_bt_evt_system_external_signal_id event
//      handler.

void app_button_press_cb(uint8_t button, uint8_t duration)
{
  // Selecting action by duration
  switch (duration) {
    case APP_BUTTON_PRESS_DURATION_SHORT:
      // Handling of button press less than 0.25s
    case APP_BUTTON_PRESS_DURATION_MEDIUM:
      // Handling of button press greater than 0.25s and less than 1s
    case APP_BUTTON_PRESS_DURATION_LONG:
      // Handling of button press greater than 1s and less than 5s
    case APP_BUTTON_PRESS_DURATION_VERYLONG:
      if (button == BUTTON_PRESS_BUTTON_0) {
        sl_bt_external_signal(EX_B0_PRESS);
      } else {
        sl_bt_external_signal(EX_B1_PRESS);
      }
      break;
    default:
      break;
  }
}

/// Read the Temperature
static void read_temperature(void)
{
  uint32_t rel_hum;
  float temp;
  // DOS: The function sl_sensor_rht_get() resides in sl_sensor_rht_mock.c
  //      which is a "stub" that simulates reading a temperature sensor.
  //      As currently configured it returns values that sequence on each
  //      "get" from -20C to +20C, incrementing by 1, and then back to -20C etc.
  //      Note: Values are returned in:
  //            millicelsius, and
  //            millipercentage
  //
  //      temperature is a global variable
  //      rel_hum is a local variable
  if(sl_sensor_rht_get(&rel_hum, (int32_t *)temperature) != SL_STATUS_OK) {
    app_log("Error while reading temperature sensor. Clear the buffer.\r\n");
    memset(temperature, 0, sizeof(temperature));
    return; // DOS: added exit if error detected
  }

//  app_log(" ***TEMP milliC = %d\r\n", * ((int32_t *) &temperature[0])); // DOS

  if (unit[0] == fahrenheit) {
    temp = (float) (*(int32_t *) temperature / 1000); // DOS: convert millicelsius to celsius, and to float data type
    temp = (temp * 1.8) + 32; // convert to F
    *(int32_t *) temperature = (int32_t) (temp * 1000); // DOS: convert to millifahrenheit

//    app_log(" ***TEMP milliF = %d\r\n", * ((int32_t *) &temperature[0])); // DOS
  }
}


/// Reset
static void factory_reset(void)
{
  // DOS moved log statement here + update the LCD here
  char   reset_status[LCD_ROW_LEN];

  app_log("factory reset\r\n");

  memset(reset_status, 0, LCD_ROW_LEN);
  snprintf(reset_status, LCD_ROW_LEN, "Factory Reset");
  sl_btmesh_LCD_write(reset_status, LCD_ROW_2);

  sl_btmesh_node_reset();

  delay_reset_ms(100);
}

// DOS: Simple timer callback
static void app_reset_timer_cb(sl_simple_timer_t *handle, void *data)
{
  (void)handle;
  (void)data;
  sl_bt_system_reset(0);
}

static sl_simple_timer_t app_reset_timer;
static void delay_reset_ms(uint32_t ms)
{
  if(ms < 10) {
      ms = 10;
  }
  sl_simple_timer_start(&app_reset_timer,
                         ms,
                         app_reset_timer_cb, // DOS: function to call when time is expired
                         NULL,               // callback data
                         false);             // not periodic, i.e. a one-shot

}

/// Update Interval
static void periodic_update_timer_cb(sl_simple_timer_t *handle, void *data)
{
  (void)handle;
  (void)data;
  sl_status_t sc;

  app_log("New temperature update cb\r\n");
  read_temperature();
  // DOS: Set the message data to publish, i.e. queue the data for publishing.
  sc = sl_btmesh_vendor_model_set_publication(my_model.elem_index,
                                              my_model.vendor_id,
                                              my_model.model_id,
                                              temperature_status,
                                              1, // DOS: the final payload "chunk"
                                              TEMP_DATA_LENGTH,
                                              temperature);
  if(sc != SL_STATUS_OK) {
    app_log("Set publication error: 0x%04X\r\n", sc);
  } else {
    app_log("Set publication done. Publishing...\r\n");
    // DOS: Publish the queued message to the group address.
    sc = sl_btmesh_vendor_model_publish(my_model.elem_index,
                                        my_model.vendor_id,
                                        my_model.model_id);
    if (sc != SL_STATUS_OK) {
      app_log("Publish error: 0x%04X\r\n", sc);
    } else {
      app_log("Periodic Publish cb done.\r\n");
    }
  }
}

static void parse_period(uint8_t interval)
{

  app_log ("  parse_period() interval = 0x%x\r\n", interval); // DOS

  switch (interval & STEP_RES_BIT_MASK) {
    case STEP_RES_100_MILLI:
      periodic_timer_ms = 100 * (interval & (~STEP_RES_BIT_MASK));
      break;
    case STEP_RES_1_SEC:
      periodic_timer_ms = 1000 * (interval & (~STEP_RES_BIT_MASK));
      break;
    case STEP_RES_10_SEC:
      periodic_timer_ms = 10000 * (interval & (~STEP_RES_BIT_MASK));
      break;
    case STEP_RES_10_MIN:
      // 10 min = 600000ms
      periodic_timer_ms = 600000 * (interval & (~STEP_RES_BIT_MASK));
      break;
    default:
      break;
  }
  if (periodic_timer_ms) { // DOS: Added from Client implementation
        app_log("Update period [hh:mm:ss:ms]= %02d:%02d:%02d:%04d\r\n",
                (periodic_timer_ms / (1000 * 60 * 60)),
                (periodic_timer_ms % (1000 * 60 * 60)) / (1000 * 60),
                (periodic_timer_ms % (1000 * 60)) / 1000,
                ((periodic_timer_ms % (1000)) / 1000) * 100);
    } else {
        app_log("  *** Periodic update off.\r\n");
    }
}

static app_timer_t periodic_update_timer;
static void setup_periodcal_update(uint8_t interval)
{
  app_log ("  setup_periodcal_update() interval = 0x%x\r\n", interval); // DOS
  parse_period(interval);
  // DOS: There was a bug in the code whereby intervals == 0 were not stopping
  //      the timer and periodic_update_timer_cb() was continued to be called.
  if (interval == 0) {                              // DOS
      app_timer_stop(&periodic_update_timer); // DOS: bug fix
  } else {                                          // DOS
      app_timer_start(&periodic_update_timer,
                             periodic_timer_ms,
                             periodic_update_timer_cb,
                             NULL,  // pointer to callback data
                             true); // is periodic
  }
}


