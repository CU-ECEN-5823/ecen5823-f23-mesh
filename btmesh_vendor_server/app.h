/***************************************************************************//**
 * @file
 * @brief Application interface provided to main().
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
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
 ******************************************************************************/

#ifndef APP_H
#define APP_H

// -------------------------------------------------------------------------
// DOS: These are here so any .c file that needs them can include this file.
// -------------------------------------------------------------------------
// push button 0
#define PB0_port    gpioPortF
#define PB0_pin     6

// SENSOR_ENABLE (Si7021) = DISP_ENABLE (LCD) on the main PCB
#define Si7021SENSOR_EN_port    (gpioPortD) // Sensor Enable for I2C Si7021
#define Si7021SENSOR_EN_pin     (15)        // Sensor Enable for I2C Si7021 at PD15

// LCD row definitions. Note: The WSTK Software Component : sl_btmesh_wstk_lcd
// uses physical rows 1, 2 and 3 on the LCD. So row 1 here is physically
// row 4 on the LCD. I can tell you from experiments I've run in the past, that
// it is possible to reclaim physical rows 1, 2 and 3. You'll have to do some
// reading/research.
enum LCD_ROW {
  LCD_ROW_1 = 1,
  LCD_ROW_2,
  LCD_ROW_3,
  LCD_ROW_4,
  LCD_ROW_5,
  LCD_ROW_6,
  LCD_ROW_7,
  LCD_ROW_8,
  LCD_ROW_9
};



/***************************************************************************//**
 * Application Init.
 ******************************************************************************/
void app_init(void);

/***************************************************************************//**
 * Application Process Action.
 ******************************************************************************/
void app_process_action(void);

#endif // APP_H
