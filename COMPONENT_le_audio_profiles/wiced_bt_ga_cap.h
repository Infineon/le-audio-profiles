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

 #pragma once

#include "wiced_bt_isoc.h"
#include "wiced_bt_types.h"

#include "wiced_bt_ga_ascs.h"
#include "wiced_bt_ga_bap.h"
#include "wiced_bt_ga_bap_broadcast.h"
#include "wiced_bt_ga_mics.h"
#include "wiced_bt_ga_pacs.h"
#include "wiced_bt_ga_vcs.h"

#define CAP_METADATA_CCID_TYPE 3 // TODO: Hardik Update the value based on spec.

#define WICED_BT_CAP_DIRECTION_SOURCE     1
#define WICED_BT_CAP_DIRECTION_SINK       2

#define     MAX_NUM_ASE_ID          5

#define CAP_METADATA_CCID_TYPE 3

#define CAP_TRACE(...)
#define CAP_TRACE_CRIT(...)

/** CAP Events */
enum wiced_bt_ga_cap_event_t
{
    WICED_BT_GA_CAP_STATE_CHANGED_EVENT,        /**< CAP ASE State  */
    WICED_BT_GA_CAP_ERROR_EVENT,                /**< CAP Error Event */
};
typedef uint8_t wiced_bt_ga_cap_event_t; /**< CAP Events (see #wiced_bt_ga_cap_event_t) */

/** CAP Device data */
typedef struct {
    uint16_t                                conn_id;
    wiced_bt_ga_pacs_data_t                 pacs_data;                    /**< PACS Data */
    uint8_t                                 num_ase;                       /**< Number of ASE ID */
    wiced_bt_ga_ascs_ase_t                  *ascs_data[MAX_NUM_ASE_ID];    /**< ASCS Data */
    wiced_bt_ga_vcs_volume_state_t          vcs_data;                     /**< VCS Data */
    wiced_bt_ga_mute_val_t                  mics_mute_val;                /**< MICS Mute State */
}wiced_bt_ga_cap_device_data_t;

/** CAP APP data */
typedef struct {
    wiced_bool_t                    is_bonded;          /**< Is device bonded */
    uint8_t                         num_devices;        /**< Number of Devices */
    wiced_bt_ga_cap_device_data_t   *device_info_list;  /**< Device Info List */
}wiced_bt_ga_cap_app_data_t;

/**< ISOC Data */
typedef struct
{
    wiced_bt_ga_cap_app_data_t      *data;          /**< App data */
    wiced_bt_isoc_event_t       isoc_event;      /**< ISOC Event ID */
    wiced_bt_isoc_event_data_t  *event_data;      /**< ISOC Event Data */
}wiced_bt_ga_cap_isoc_data_t;

typedef struct
{
    uint16_t                     conn_id;   /**<Connection Id */
    gatt_intf_attribute_t*  p_char;    /**<Char pointer */
}wiced_bt_ga_cap_req_profile_data_t;

/** CAP Event Data */
typedef union
{
    wiced_bt_ga_cap_app_data_t      *data;          /**< App data */
    wiced_bt_ga_cap_isoc_data_t     isoc_data;      /**< ISOC Data */
    wiced_bt_ga_cap_req_profile_data_t profile_data; /**< Request Profile data */
    wiced_bt_ga_ascs_ase_t         *group_state;   /**< CAP Group state */
}wiced_bt_ga_cap_event_data_t;

/** CAP Start Unicast Streaming Data */
typedef struct
{
    uint16_t   context_type;                                          /**< Targeted Context Type */
    uint8_t    ccid_count;                                            /**< Number of CCID */
    uint8_t    *ccid_list;                                            /**< Associate CCID List */
	uint8_t    dir;
    wiced_bt_ga_ascs_config_codec_args_t *codec_configuration;   /**< Targeted Codec Configuration */
    wiced_bt_ga_ascs_config_qos_args_t *qos_configuration; /**< Targeted QOS Configuration */
}wiced_bt_ga_cap_start_unicast_param_t;

/** Codec configuration */
typedef struct
{
    uint16_t min_data_per_frame;
    uint16_t max_data_per_frame;
    uint16_t sf;
    uint8_t frame_duration;
    uint8_t audio_ch_count;
    uint8_t frame_per_sdu;
    uint16_t octet_per_frame;
    uint16_t blocks_per_sdu;
}wiced_ga_cap_codec_param_t;

/** CAP APP Broadcast data */
typedef struct {
    uint8_t adv_handle; /**< Adv Handle */
    uint8_t big_handle; /**< BIG Handle */
    uint16_t bis_conn_id_count;
    uint16_t bis_conn_id_list[BROADCAST_MAX_BIS_PER_SUB_GROUP * BROADCAST_MAX_SUB_GROUP];
    wiced_bt_ga_bap_broadcast_base_t *p_base; /**< BASE Data */
}wiced_bt_ga_cap_broadcast_app_data_t;

/** CAP Start Broadcast Streaming Data */
typedef struct
{
    wiced_bt_isoc_phy_t         phy;                /**< ISOC LE PHY */
    wiced_bt_isoc_packing_t     packing;            /**< ISOC packing methods  */
    wiced_bt_isoc_framing_t     framing;            /**< ISOC Framing types */
    wiced_bt_isoc_encryption_t  encrypt;            /**< ISOC Encryption */
    uint8_t                     *broadcast_code;    /**< Broadcast Code */
}wiced_bt_ga_cap_start_broadcast_param_t;

/**
 * CAP event callback
 *
 * Callback for CAP event notification
 * Registered using #wiced_bt_ga_cap_register_cb
 *
 * @param event             : Event ID
 * @param p_event_data      : Event data
 *
 * @return none
 */
typedef void wiced_ble_ga_cap_cback_t(wiced_bt_ga_cap_event_t event, wiced_bt_ga_cap_event_data_t *p_event_data);

/**
 *
 * Function         wiced_bt_ga_cap_register_cb
 *
 *                  CAP Register event callback handler
 *
 * @param[in]       cb_ptr  : CAP event callback
 *
 * @return      None
 *
 */
void wiced_bt_ga_cap_register_cb(wiced_ble_ga_cap_cback_t *cb_ptr);

/**
 *
 * Function         wiced_bt_ga_cap_start_unicast_streaming
 *
 *                  CAP Start Unicast Streaming procedure
 *
 * @param[in]       app_data : App device list info data pointer
 * @param[in]       params   : start unicast param data pointer
 *
 * @return      wiced_result_t
 *
 */
wiced_result_t wiced_bt_ga_cap_start_unicast_streaming(wiced_bt_ga_cap_app_data_t *app_data, wiced_bt_ga_cap_start_unicast_param_t *params);

/**
 *
 * Function         wiced_bt_ga_cap_update_streaming
 *
 *                  CAP Update Unicast Streaming procedure
 *
 * @param[in]       app_data : App device list info data pointer
 * @param[in]       context_type : Context type
 * @param[in]       ccid : CCID Value
 *
 * @return      wiced_result_t
 *
 */
wiced_result_t wiced_bt_ga_cap_update_streaming(wiced_bt_ga_cap_app_data_t *app_data, uint16_t context_type, uint8_t ccid_count, uint8_t *ccid_list);

/**
 *
 * Function         wiced_bt_ga_cap_stop_streaming
 *
 *                  CAP Stop Unicast Streaming procedure
 *
 * @param[in]       app_data : App device list info data pointer
 *
 * @return      wiced_result_t
 *
 */
wiced_result_t wiced_bt_ga_cap_stop_streaming(wiced_bt_ga_cap_app_data_t *app_data);

/**
 *
 * Function         wiced_bt_ga_cap_set_absolute_volume
 *
 *                  CAP Set Absolute Volume
 *
 * @param[in]       app_data         : App device list info data pointer
 * @param[in]       volume_setting   : Volume (Range 0-255)
 *
 * @return      wiced_result_t
 *
 */
wiced_result_t wiced_bt_ga_cap_set_absolute_volume(wiced_bt_ga_cap_app_data_t *app_data, uint8_t volume);

/**
 *
 * Function         wiced_bt_ga_cap_set_volume_mute_state
 *
 *                  Set absolute volume on all the devices mentioned in device_info_list
 *
 * @param[in]       app_data   : App device list info data pointer
 * @param[in]       mute_state : Mute State (See #wiced_bt_ga_mute_val_t)
 *
 * @return      wiced_result_t
 *
 */
wiced_result_t wiced_bt_ga_cap_set_volume_mute_state(wiced_bt_ga_cap_app_data_t *app_data, wiced_bt_ga_mute_val_t mute_state);

/**
 *
 * Function         wiced_bt_ga_cap_set_volume_offset
 *
 *                  Set volume offset on all the devices mentioned in device_info_list
 *
 * @param[in]       app_data   : App device list info data pointer
 * @param[in]       offset : Volume offset
 *
 * @return      wiced_result_t
 *
 */
wiced_result_t wiced_bt_ga_cap_set_volume_offset(wiced_bt_ga_cap_app_data_t *app_data, int16_t volume_offset);

/**
 *
 * Function         wiced_bt_ga_cap_set_mics_mute_state
 *
 *                  Set Mute State on all the devices mentioned in device_info_list
 *
 * @param[in]       app_data   : App device list info data pointer
 * @param[in]       mute_state : Mute State
 *
 * @return      wiced_result_t
 *
 */
wiced_result_t wiced_bt_ga_cap_set_mics_mute_state(wiced_bt_ga_cap_app_data_t *app_data, wiced_bt_ga_mute_val_t mute_state);

/**
 *
 * Function         wiced_bt_ga_cap_set_mics_gain
 *
 *                  Set MICS Input Gain on all the devices mentioned in device_info_list
 *
 * @param[in]       app_data   : App device list info data pointer
 * @param[in]       gain       : Gain in dB
 *
 * @return      wiced_result_t
 *
 */
wiced_result_t wiced_bt_ga_cap_set_mics_gain(wiced_bt_ga_cap_app_data_t *app_data, int8_t gain);

/**
 *
 * Function         wiced_bt_ga_cap_parse_lc3_codec_param
 *
 *                  Parse lc3 codec params
 *
 * @param[in]       codec_arg    : Input codec capabilities
 * @param[out]     output : Prased output data
 *
 * @return      wiced_result_t
 *
 */
void wiced_bt_ga_cap_parse_lc3_codec_param(wiced_bt_ga_ascs_config_codec_args_t *codec_arg, wiced_ga_cap_codec_param_t *output);

/**
 *
 * Function         wiced_ga_cap_vcs_update_event
 *
 *                  This API should be invoked from VCS registered callback on receiving notification.
 *
 * @param app_data       : App device list info data pointer
 * @param conn_id        : GATT connection id
 * @param p_app_ctx      : Application context registered using \ref gatt_intf_service_methods_t.set_callback
 * @param p_service      : GATT Service instance
 * @param status         : GATT operation status
 * @param event_type     : Event type, \ref gatt_interface_events_t
 * @param p_char         : Characteristic for which the event has occurred
 * @param p_data         : Event Data
 * @param len            : Event Data len
 * @return      wiced_result_t
 *
 */
wiced_result_t wiced_ga_cap_vcs_update_event(wiced_bt_ga_cap_app_data_t* app_data, uint16_t conn_id, wiced_bt_gatt_status_t status, uint32_t evt_type,
    gatt_intf_attribute_t* p_char, void* p_data, int len);

/**
 *
 * Function         wiced_bt_ga_cap_mics_update_event
 *
 *                  This API should be invoked from MICS registered callback on receiving notification.
 *
 * @param conn_id        : GATT connection id
 * @param status         : GATT operation status
 * @param event_type     : Event type, \ref gatt_interface_events_t
 * @param p_char         : Characteristic for which the event has occurred
 * @param p_data         : Event Data
 * @param len            : Event Data len
 *
 * @return      wiced_result_t
 *
 */
wiced_result_t wiced_bt_ga_cap_mics_update_event(wiced_bt_ga_cap_app_data_t* app_data, uint16_t conn_id, wiced_bt_gatt_status_t status, uint32_t evt_type,
    gatt_intf_attribute_t* p_char, void* p_data, int len);

/**
 *
 * Function         wiced_ga_cap_ascs_update_event
 *
 *                  This API acts on ASCS notification with respect to CAP procedure and should be invoked from ASCS registered callback on receiving notification.
 *
 * @param conn_id        : GATT connection id
 * @param status         : GATT operation status
 * @param event_type     : Event type, \ref gatt_interface_events_t
 * @param p_char         : Characteristic for which the event has occurred
 * @param p_data         : Event Data
 * @param len            : Event Data len
 * @return      wiced_resut_t
 *
 */
wiced_result_t wiced_ga_cap_ascs_update_event(uint16_t conn_id, wiced_bt_gatt_status_t status, uint32_t evt_type,
    gatt_intf_attribute_t* p_char, void* p_data, int len);

void wiced_bt_ga_cap_reset_state();

/**
 *
 * Function         wiced_bt_ga_cap_broadcast_configure_stream
 *
 *                  Configure Broadcast Stream and starts Broadcast & Basic audio announcements
 *
 * @param app_data     : Broadcast App Data
 *
 * @return      wiced_result_t
 *
 */
wiced_result_t wiced_bt_ga_cap_broadcast_configure_stream(wiced_bt_ga_cap_broadcast_app_data_t *app_data);

/**
 *
 * Function         wiced_bt_ga_cap_broadcast_start_stream
 *
 *                  Start Broadcast Stream
 *
 * @param app_data     : Broadcast App Data
 * @param start_param     : Broadcast Start param
 * @return      wiced_result_t
 *
 */
wiced_result_t wiced_bt_ga_cap_broadcast_start_stream(wiced_bt_ga_cap_broadcast_app_data_t *app_data, wiced_bt_ga_cap_start_broadcast_param_t *start_param);

/**
 *
 * Function         wiced_bt_ga_cap_broadcast_disable_stream
 *
 *                  Disable Broadcast Stream
 *
 * @param app_data     : Broadcast App Data
 * @param reason       : Disable reason code. 
 * @return      wiced_result_t
 *
 */
wiced_result_t wiced_bt_ga_cap_broadcast_disable_stream(wiced_bt_ga_cap_broadcast_app_data_t *app_data, uint8_t reason);

/**
 *
 * Function         wiced_bt_ga_cap_broadcast_release_stream
 *
 *                  Release Broadcast Stream
 *
 * @param app_data     : Broadcast App Data
 * @return      wiced_result_t
 *
 */
wiced_result_t wiced_bt_ga_cap_broadcast_release_stream(wiced_bt_ga_cap_broadcast_app_data_t *app_data);

/**
 *
 * Function         wiced_bt_ga_cap_broadcast_update_stream_metadata
 *
 *                  CAP Update Broadcast Streaming procedure
 *
 * @param[in]       app_data        : Broadcast App Data
 *
 * @return      wiced_result_t
 *
 */
wiced_result_t wiced_bt_ga_cap_broadcast_update_stream_metadata(wiced_bt_ga_cap_broadcast_app_data_t *app_data);
