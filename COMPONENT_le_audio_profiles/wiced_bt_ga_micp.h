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

#ifndef __WICED_BT_GA_MICP_H__
#define __WICED_BT_GA_MICP_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_ga_common.h"
#include "wiced_bt_ga_aics.h"

#define MICP_TRACE(...)
#define MICP_TRACE_CRIT(...)

    /* Maximum Instance of AICS service supported by client */
#define WICED_BT_MAX_AICS_INSTANCE          2

/* microphone State data */
typedef struct
{
    wiced_bt_ga_mute_val_t  mute_state;                        /**< current mute state value of the peer*/
} wiced_bt_ga_microphone_state_data_t;

/* microphone Control client status Data */
typedef union
{
    wiced_bt_ga_mute_val_t mute_state;                        /**< changed microphone setting value */
    uint8_t error_status;                                  /**< error opcode in case of error event */
    wiced_bt_ga_aics_data_t aics_data;          /**< AICS Status data */
} wiced_bt_ga_microphone_control_client_status_data_t;

    
extern const gatt_intf_service_methods_t mics_methods;

#ifdef __cplusplus
}
#endif

#endif /* __WICED_BT_GA_MICP_H__ */
