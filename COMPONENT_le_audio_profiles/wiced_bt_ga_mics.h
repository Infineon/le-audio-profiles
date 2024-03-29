/*
 * Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

#ifndef __WICED_BT_GA_MICS_H__
#define __WICED_BT_GA_MICS_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "gatt_interface.h"
#include "wiced_bt_ga_common.h"
#include "wiced_bt_ga_aics.h"

#define MICS_TRACE(...)
#define MICS_TRACE_CRIT(...)

typedef struct {
    uint8_t index;                           /**< index of the included service viz, vocs or aics */
    union {
        wiced_bt_ga_aics_data_t* p_aics;        /**< aics data */
    };
} mics_included_t;

typedef union {
    wiced_bt_ga_mute_val_t           mute_val;        /**< mute state */
    mics_included_t                  mics_included;   /**< volume included service data */
} wiced_bt_ga_mics_data_t;

/**
* Initialize the MICS service/profile
*/

wiced_result_t wiced_bt_ga_mics_init(ga_cfg_t *p_cfg);

wiced_bt_gatt_status_t mics_notify(uint16_t conn_id, gatt_intf_service_object_t* p_service,
    gatt_intf_attribute_t *p_char, void *p_data);

wiced_bt_gatt_status_t mics_read_remote_attribute(uint16_t conn_id, gatt_intf_service_object_t* p_service,
    gatt_intf_attribute_t *p_char);

wiced_bt_gatt_status_t mics_enable_notifications(uint16_t conn_id, gatt_intf_service_object_t* p_service,
    gatt_intf_attribute_t *p_char, uint16_t value);

wiced_bt_gatt_status_t mics_write_remote_attribute(uint16_t conn_id, gatt_intf_service_object_t* p_service,
                                                 gatt_intf_attribute_t *p_char,
                                                 void *p_data);

#ifdef __cplusplus
}
#endif

#endif /* __WICED_BT_GA_MICS_H__ */
