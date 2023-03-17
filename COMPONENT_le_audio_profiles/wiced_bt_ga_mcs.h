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

#ifndef __WICED_BT_GA_MCS_H__
#define __WICED_BT_GA_MCS_H__

#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "gatt_interface.h"
#include "wiced_bt_ga_common.h"
#include "wiced_bt_ga_mcs_common.h"

#define MCS_TRACE(...)
#define MCS_TRACE_CRIT(...)

 /* Media Control Opcode Data */
typedef union
{
    int32_t   move_relative_offset;             /**< Move Relative Offset in seconds */
    int32_t   track_number;                     /**< Track number */
} wiced_bt_ga_mcs_operation_data_t;

typedef struct
{
    wiced_bt_ga_mcp_media_control_operation_t       opcode;     /**< Media Opcode */
    wiced_bt_ga_mcs_operation_data_t                data;       /**< Media Operation data */
    wiced_bt_ga_mcp_result_t                        result;     /**< Operation result */
} wiced_bt_ga_mcs_operation_t;

/* Audio Input Control Service event data */
typedef union
{
    wiced_bt_ga_string_t    media_player_name;                      /**< Media Player Name */
    wiced_bt_ga_string_t    track_title;                            /**< Track Title */
    int32_t                 track_duration;                         /**< Track Duration */
    int32_t                 track_position;                         /**< Track Position */
    int8_t                  playback_speed;                         /**< Playback Speed */
    int8_t                  seeking_speed;                          /**< Seeking Speed */
    wiced_bt_ga_media_control_playing_order_t    playing_order;     /**< Playing Order */
    uint16_t                playing_order_supported;                /**< Playing Order Supported bit field */
    wiced_bt_ga_media_control_state_t  media_state;                 /**< Media State */
    uint32_t                media_control_supported_opcodes;        /**< Media Control Supported Opcodes */
    uint8_t                 content_control_id;                     /**< Content Control ID */
    wiced_bt_ga_mcs_operation_t control_point_operation;                /**< Control point operation result */
} wiced_bt_ga_mcs_data_t;

/**
* Initialize the GMCS service_type/profile
*/

wiced_result_t wiced_bt_ga_gmcs_init(ga_cfg_t *p_cfg);

/**
* Initialize the MCS service_type/profile
*/
wiced_result_t wiced_bt_ga_mcs_init(ga_cfg_t *p_cfg);


void wiced_bt_ga_mcs_media_track_selected(gatt_intf_service_object_t* p_service, wiced_bool_t is_track_selected);

#endif // __WICED_BT_GA_MCS_H__

