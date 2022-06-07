/*
 * Copyright 2022, Cypress Semiconductor Corporation (an Infineon company) or
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

/** @file
 *
 * Volume Control Service (VCS) Application Programming Interface
 */
#pragma once

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
#include "wiced_bt_ga_vocs.h"

#define VCS_TRACE(...)
#define VCS_TRACE_CRIT(...)

/**
 * @addtogroup Volume_Control_APIs
 * @{
 */

/**
 * @addtogroup wiced_bt_ga_vcs
 * @{
 */

/** VCS minimum volume */
#define WICED_BT_GA_VCS_MINIMUM_VOLUME     0x0

/** VCS maximum volume */
#define WICED_BT_GA_VCS_MAXIMUM_VOLUME     0xFF

/** VCS muted state */
#define WICED_BT_GA_VCS_MUTED              0x1

/** VCS unmuted state */
#define WICED_BT_GA_VCS_NOT_MUTED          0x0

/** Volume profile opcode codes */
typedef enum
{
    VOLUME_CONTROL_OPCODE_RELATIVE_VOLUME_DOWN = 0x00,          /**< Relative volume down */
    VOLUME_CONTROL_OPCODE_RELATIVE_VOLUME_UP = 0x01,            /**< Relative volume up */
    VOLUME_CONTROL_OPCODE_UNMUTE_RELATIVE_VOLUME_DOWN = 0x02,   /**< Unmute and relative volume down */
    VOLUME_CONTROL_OPCODE_UNMUTE_RELATIVE_VOLUME_UP = 0x03,     /**< Unmute and relative volume up */
    VOLUME_CONTROL_OPCODE_SET_ABSOLUTE_VOLUME = 0x04,           /**< Set Absolute volume */
    VOLUME_CONTROL_OPCODE_UNMUTE = 0x05,                        /**< Unmute */
    VOLUME_CONTROL_OPCODE_MUTE = 0x06                           /**< Mute */
} volume_control_opcodes_t;

/** Volume state data */
typedef struct
{
    uint8_t volume_setting;                        /**< current volume setting value */
    wiced_bt_ga_mute_val_t mute_state;             /**< current mute state value */
} wiced_bt_ga_vcs_volume_state_t;

/** Volume state control point data */
typedef struct
{
    volume_control_opcodes_t opcode;               /**< opcode of volume control point*/
    wiced_bt_ga_vcs_volume_state_t volume_state;   /**< volume state updated by the control point operation */
} wiced_bt_ga_vcs_control_point_t;

/** Volume service included service data */
typedef struct {
    uint8_t index;                           /**< index of the included service viz, vocs or aics */
    union {
        wiced_bt_ga_vocs_data_t* p_vocs;        /**< vocs data */
        wiced_bt_ga_aics_data_t* p_aics;        /**< aics data */
    };
} vcs_included_t;

/** Volume service data */
typedef union {
    wiced_bt_ga_vcs_control_point_t control_point_data;      /**< volume information */
    wiced_bt_ga_volume_flag_val_t   volume_flag;      /**< volume persistence flag */
    vcs_included_t                  volume_included;  /**< volume included service data */
} wiced_bt_ga_vcs_data_t;

/**
 * @brief Initialize the VCS service/profile
 *
 * @param[in] p_cfg : Generic Audio configuration
 */
wiced_result_t wiced_bt_ga_vcs_init(ga_cfg_t *p_cfg);

/**@} wiced_bt_ga_vcs */
#ifdef __cplusplus
}
#endif
/**@} Volume_Control_APIs */
