01/11/2024 - Dave Sluiter

Server/Friend README

The code for this project originated from:
  https://github.com/SiliconLabs/bluetooth_mesh_stack_features/tree/master
    or
  https://github.com/SiliconLabs/bluetooth_mesh_stack_features.git 
    branch = Master

Follow the instructions in the vendor model sub-folder to craft the initial
code. Then see below.

********************************************************************************
Design:
  Client is the LPN
  Server is the Friend, and has a synthesized (a stub) temp measurement.
  The Server's Software Component "Bluetooth Mesh Stack" contains a number
  of Configurable parameters. This component is configured to support 4
  LPNs via the "Maximum number of Friendships allowed" parameter.


-----------------------------------------------------------------------------------------
Client/LPN                                                         Server/Friend Response
-----------------------------------------------------------------------------------------
PB0 Press      -> Get Temp Measurement from the Server             Return temperature + ACK

PB0 Long Press -> Set/Update the update interval in the Server     Return interval, start periodic 
                                                                   transmitting of temperature updates
                                                                   for non-zero values. 0's in the
                                                                   table turn off periodic updates
                                                                   from the Server + STATUS_UPDATE
                                                                   
                  Periodic updates initially start "off". The first PB0 Long Press starts us off.
                  Subsequent PB0 Long Presses cycle through a table of "period durations" until
                  finally cycling to "off". It then cycles through again for subsequent 
                  PB0 Long Presses. The Client will log the period durations and does
                  display "Periodic update off." for values of 0.
                    
PB1 Press      -> Get the units from the Server                    Return units + ACK

PB1 Long Press -> Toggle the units (units set) in the Server       Return units + STATUS_UPDATE

-----------------------------------------------------------------------------------------
Server/Friend                                                      Client/LPN Response
-----------------------------------------------------------------------------------------
PB0 Press -> Publish the temperature measurement to the Client     Apparently nothing

PB1 Press -> Publish the units to the Client                       Apparently nothing

********************************************************************************

See the subfolder models/vendor/client and models/vendor/server. It starts with
an example Silicon Labs project: Bluetooth Mesh - SoC Empty
and then modified according to the documentation in this github repo.

The Server has "the temperature" and the Client can request the temperature
from the Server. Note that both the Client and the Server can publish messages,
and that both the Client and the Server subscribe to those messages. So we have
a "two-way" communication scheme implemented here. The Server acts as the Friend
and the Client acts as the Low-Power Node (LPN).

I had to add software components:
Application / Utility / Button Press
   that installed 2 instances of Driver / Button / Simple Button : btn0 and btn1.
Bluetooth Mesh / Features / Friend
   See the Configuration where you can set:
      Enable Logging
   
Also had to fix the factory reset code in sl_bt_on_event() event = 
sl_bt_evt_system_boot_id as it wasn't functioning. I added my own code to check 
for PB0 pressed at boot/reset time to perform a factory reset. 


07/13/2023 - Dave Sluiter

Added software component Bluetooth Mesh / Utility / WSTK LCD Driver for the function:
sl_btmesh_LCD_write(). Also had to set GPIO SENSOR_ENABLE - unclear why the WSTK
driver doesn't perform this function. Configured the "initializing" string to a
null string so that we don't have clear that LCD row in our code.


07/20/2023 - Dave Sluiter
Added software component Bluetooth Mesh / Utility / Event Logging so we can see
all important events for both BLE and Mesh, logged to a terminal.


01/11/2024 - Dave Sluiter
     Note: The number of Friends this node supports is defined in:
     Software Component: "Bluetooth Mesh Stack". Adjusting the number of
     Friends (or other values) re-generates:
        config/sl_btmesh_config.h -> #define SL_BTMESH_CONFIG_MAX_FRIENDSHIPS       (4)






