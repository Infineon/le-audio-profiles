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

#pragma once

#include "wiced_data_types.h"

#include "hci_control_api.h"
#include "wiced_bt_ble.h"

typedef enum
{
    TEST_MSG = 0, // not used
    PA_SYNC_ESTABLISHED = 1,
    PA_SYNC_LOST = 2,
    BIG_SYNC_ESTABLISHED = 3,
    BIG_SYNC_LOST = 4,
    STATUS_ANY_EVT = 20
} le_audio_rpc_sts_t;

typedef enum
{
    MUTE_STATUS = 0,
    VOLUME_STATUS = 1,
    MUTE_AND_VOLUME_STATUS = 2,
} le_audio_rpc_vcs_sts_t;

typedef wiced_bool_t (*le_audio_rpc_cback_t)(uint16_t opcode, uint8_t *p_data, uint32_t data_len);
void le_audio_rpc_init(int host_instance, le_audio_rpc_cback_t le_audio_rpc_cback, wiced_bool_t b_route_traces_to_CC);

typedef void (*route_data_to_client_control_t)(uint8_t type, uint8_t *buffer, uint16_t length, uint8_t spy_instance);

void le_audio_rpc_send_data(int type, uint8_t *p_data, uint16_t data_size);

void le_audio_rpc_send_dev_role(uint8_t dev_role);

void le_audio_rpc_send_status_update(le_audio_rpc_sts_t msg);

void le_audio_rpc_send_vcs_state_update(uint16_t conn_id,
                                        uint8_t volume_setting,
                                        uint8_t mute_state,
                                        le_audio_rpc_vcs_sts_t which_vcs_data);

void le_audio_rpc_send_mcs_state_update(uint16_t conn_id, uint8_t operation_status, uint8_t state);
void le_audio_rpc_send_connect_event(uint8_t addr_type, uint8_t *addr, uint16_t con_handle, wiced_bool_t role);
void le_audio_rpc_send_disconnect_evt(uint16_t reason, uint16_t conn_idx);
void le_audio_rpc_send_scan_res_event(uint8_t *addr, uint8_t addr_type, wiced_bt_dev_ble_evt_type_t evt_type);
