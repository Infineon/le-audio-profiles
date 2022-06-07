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

#ifndef __WICED_BT_GA_PACS_H__
#define __WICED_BT_GA_PACS_H__
/*
    PACS : Published Audio Capability Service
  */

#define PACS_TRACE(...)
#define PACS_TRACE_CRIT(...)

#include "wiced_bt_ga_common.h"
#include "wiced_bt_ga_bap.h"

#define MAX_CODEC_SPECIFIC_CAPABILITIES_LENGTH 128
#define MAX_METADATA_LENGTH 128

typedef struct
{
    uint8_t coding_format;
    uint16_t company_id;
    uint16_t vendor_specific_codec_id;
} wiced_bt_ga_pacs_codec_id_t;

/**
 * @brief Defines PAC record
 *
 */
typedef struct
{
    wiced_bt_ga_pacs_codec_id_t codec_id;
    uint8_t codec_specific_capabilities[MAX_CODEC_SPECIFIC_CAPABILITIES_LENGTH];
    uint8_t codec_specific_capabilities_length;
    uint8_t metadata_length;
    uint8_t metadata[MAX_METADATA_LENGTH];
} wiced_bt_ga_pacs_record_t;

/**
 * @brief TODO:
 *
 */
typedef struct
{
    uint8_t num_of_records;
    wiced_bt_ga_pacs_record_t *record_list;
} wiced_bt_ga_pacs_char_data_t;

/**
 * @brief TODO:
 *
 */
typedef struct
{
    uint16_t source_contexts;
    uint16_t sink_contexts;
} wiced_bt_ga_pacs_audio_contexts_t;

/**
 * @brief Device-wide bitmap of supported Audio Location values for all PAC records
 *
 */
typedef uint32_t wiced_bt_ga_pacs_audio_location_t;

/**
 * @brief Defines all the fields required for Published Audio Capability Service
 *
 */
typedef struct
{
    uint8_t char_instance;
    wiced_bt_ga_pacs_char_data_t source_pac_list;
    wiced_bt_ga_pacs_char_data_t sink_pac_list;
    wiced_bt_ga_pacs_audio_location_t source_audio_location;
    wiced_bt_ga_pacs_audio_location_t sink_audio_location;
    wiced_bt_ga_pacs_audio_contexts_t supported;
    wiced_bt_ga_pacs_audio_contexts_t available;
} wiced_bt_ga_pacs_data_t;

wiced_result_t wiced_bt_ga_pacs_init(ga_cfg_t *p_cfg);

void wiced_bt_ga_pacs_notify_all_characteristics(uint16_t conn_id, gatt_intf_service_object_t *p_service,
                                                 wiced_bt_ga_pacs_data_t *p_notification);

#endif // __WICED_BT_GA_PACS_H__
