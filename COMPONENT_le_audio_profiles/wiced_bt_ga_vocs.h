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

/** @file
 *
 * Volume Offset Control Service (VOCS) Application Programming Interface
 */

#pragma once 

#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "gatt_interface.h"
#include "wiced_bt_ga_common.h"

#define VOCS_TRACE(...)
#define VOCS_TRACE_CRIT(...)
/**
 * @addtogroup Volume_Control_APIs
 * @{
 */

/**
 * @addtogroup wiced_bt_ga_vocs
 * @{
 */
 
/** Volume Offset Control Service opcodes */
typedef enum {
    WICED_BT_GA_VOCS_OPCODE_SET_VOLUME_OFFSET = 1 /**< Set Volume Offset*/
} wiced_bt_ga_vocs_opcode_t;

/** Volume offset control point data */
typedef struct {
    wiced_bt_ga_vocs_opcode_t opcode; /**< VOCS opcode */
    int16_t volume_offset;            /**< VOCS offset value */
} vocs_opcode_t;

/** Volume offset data which is passed between application and profile */
typedef union {
    int16_t volume_offset;             /**< VOCS offset value */
    uint32_t audio_location;           /**< Audio location value */
    wiced_bt_ga_string_t description;  /**< VOCS offset value */
    vocs_opcode_t control_point;       /**< VOCS control point data */
}wiced_bt_ga_vocs_data_t;

/** Volume offset related methods defined in the init file*/
extern const gatt_intf_service_methods_t vocs_methods;

/**
 * @brief VOCS API to notify characteristic data
 *
 * @param[in] conn_id : GATT Connection ID
 * @param[in] p_service : VOCS service object
 * @param[in] p_char : characteristic to be notified
 * @param[in] p_n : characteristic data to be notified 
 */
wiced_bt_gatt_status_t vocs_notify(uint16_t conn_id, gatt_intf_service_object_t* p_service, 
    gatt_intf_attribute_t *p_char, void *p_n);

/**
 * @brief VOCS API to write characteristic data
 *
 * @param[in] conn_id : GATT Connection ID
 * @param[in] p_service : VOCS service object
 * @param[in] p_char : characteristic to be written
 * @param[in] p_n : characteristic data to be written 
 */
wiced_bt_gatt_status_t vocs_write_remote_attribute(uint16_t conn_id, gatt_intf_service_object_t* p_service,
    gatt_intf_attribute_t *p_char, void* p_n);
	
	
/**@} wiced_bt_ga_vocs */
/**@} Volume_Control_APIs */
