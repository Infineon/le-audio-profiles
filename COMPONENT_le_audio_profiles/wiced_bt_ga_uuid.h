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

#ifndef __WICED_BT_GA_UUID__
#define __WICED_BT_GA_UUID__

#define LC3_CODEC_ID 0x6

enum uuid_attribute
{
    WICED_BT_UUID_AUDIO_INPUT_CONTROL_SERVICE = 0x1843,           // 43 18
    WICED_BT_UUID_VOLUME_CONTROL_SERVICE = 0x1844,                // 44 18
    WICED_BT_UUID_VOLUME_OFFSET_CONTROL_SERVICE = 0x1845,         // 45 18
    WICED_BT_UUID_COORDINATE_SET_IDENTIFICATION_SERVICE = 0x1846, // 46 18
    WICED_BT_UUID_MEDIA_CONTROL_SERVICE = 0x1848,                 // 48 18
    WICED_BT_UUID_GENERIC_MEDIA_CONTROL_SERVICE = 0X1849,         // 49 18
    WICED_BT_UUID_TELEPHONE_BEARER_SERVICE = 0x184B,              // 4B 18
    WICED_BT_UUID_GENERIC_TELEPHONE_BEARER_SERVICE = 0X184C,      // 4C 18
    WICED_BT_UUID_MICROPHONE_CONTROL_SERVICE = 0x184D,            // 4D 18
    WICED_BT_UUID_AUDIO_STREAM_CONTROL_SERVICE = 0x184E,          // 4e 18
    WICED_BT_UUID_BROADCAST_AUDIO_SCAN_SERVICE = 0X184F,          // 4f 18
    WICED_BT_UUID_PUBLISHED_AUDIO_CAPABILITY_SERVICE = 0x1850,    // 50 18
    WICED_BT_UUID_BASIC_AUDIO_ANNOUNCEMENT_SERVICE = 0X1851,      // 51 18
    WICED_BT_UUID_BROADCAST_AUDIO_ANNOUNCEMENT_SERVICE = 0x1852,  // 52 18
    WICED_BT_UUID_ROUTING_ACTIVE_AUDIO_SERVICE = 0X8FD6,          // d6 8f
    WICED_BT_UUID_COMMON_AUDIO_SERVICE = 0X8FE0,                  // e0 8f

    WICED_BT_UUID_INPUT_STATE = 0x2B77,
    WICED_BT_UUID_GAIN_SETTING_ATTRIBUTE = 0x2B78,
    WICED_BT_UUID_INPUT_TYPE = 0x2B79,
    WICED_BT_UUID_INPUT_STATUS = 0x2B7A,
    WICED_BT_UUID_AUDIO_INPUT_CONTROL_POINT = 0x2B7B,
    WICED_BT_UUID_AUDIO_INPUT_DESCRIPTION = 0x2B7C,

    WICED_BT_UUID_VOLUME_STATE = 0x2B7D,
    WICED_BT_UUID_CONTROL_POINT = 0x2B7E,
    WICED_BT_UUID_VOLUME_FLAG = 0x2B7F,

    WICED_BT_UUID_VOLUME_OFFSET_STATE = 0x2B80,
    WICED_BT_UUID_AUDIO_LOCATION = 0x2B81,
    WICED_BT_UUID_VOLUME_OFFSET_CONTROL_POINT = 0x2B82,
    WICED_BT_UUID_AUDIO_OUTPUT_DESCRIPTION = 0x2B83,

    WICED_BT_UUID_CSIS_SIRK                          = 0x2B84,
    WICED_BT_UUID_CSIS_SIZE                          = 0x2B85,
    WICED_BT_UUID_CSIS_LOCK                          = 0x2B86,
    WICED_BT_UUID_CSIS_RANK                          = 0x2B87,

    WICED_BT_UUID_MEDIA_PLAYER_NAME = 0x2B93,
    WICED_BT_UUID_MEDIA_ICON_OBJECT = 0x2B94,
    WICED_BT_UUID_MEDIA_ICON_URI = 0x2B95,
    WICED_BT_UUID_MEDIA_TRACK_CHANGED = 0x2B96,
    WICED_BT_UUID_MEDIA_TRACK_TITLE = 0x2B97,
    WICED_BT_UUID_MEDIA_TRACK_DURATION = 0x2B98,
    WICED_BT_UUID_MEDIA_TRACK_POSITION = 0x2B99,
    WICED_BT_UUID_MEDIA_PLAYBACK_SPEED = 0x2B9A,
    WICED_BT_UUID_MEDIA_SEEKING_SPEED = 0x2B9B,
    WICED_BT_UUID_MEDIA_TRACK_SEGMENT_OBJECT = 0x2B9C,
    WICED_BT_UUID_MEDIA_CURRENT_TRACK_OBJECT = 0x2B9D,
    WICED_BT_UUID_MEDIA_NEXT_TRACK_OBJECT = 0x2B9E,
    WICED_BT_UUID_MEDIA_PARENT_GROUP_OBJECT = 0x2B9F,
    WICED_BT_UUID_MEDIA_CURRENT_GROUP_OBJECT = 0x2BA0,
    WICED_BT_UUID_MEDIA_PLAYING_ORDER = 0x2BA1,
    WICED_BT_UUID_MEDIA_PLAYING_ORDER_SUPPORTED = 0x2BA2,
    WICED_BT_UUID_MEDIA_STATE = 0x2BA3,
    WICED_BT_UUID_MEDIA_CONTROL_POINT = 0x2BA4,
    WICED_BT_UUID_MEDIA_CONTROL_OPCODE_SUPPORTED = 0x2BA5,
    WICED_BT_UUID_MEDIA_SEARCH_RESULTS_OBJECT = 0x2BA6,
    WICED_BT_UUID_MEDIA_SEARCH_CONTROL_POINT = 0x2BA7,
    WICED_BT_UUID_MEDIA_CONTENT_CONTROL_ID = 0x2BBA,

    WICED_BT_UUID_TBS_BEARER_PROVIDER_NAME = 0x2BB3,
    WICED_BT_UUID_TBS_BEARER_UCI = 0x2BB4,
    WICED_BT_UUID_TBS_BEARER_TECHNOLOGY = 0x2BB5,
    WICED_BT_UUID_TBS_BEARER_URI_SCHEMES = 0x2BB6,
    WICED_BT_UUID_TBS_BEARER_SIGNAL_STRENGTH = 0x2BB7,
    WICED_BT_UUID_TBS_SIG_STR_REPORTING_INTERVAL = 0x2BB8,
    WICED_BT_UUID_TBS_LIST_CURRENT_CALL = 0x2BB9,
    WICED_BT_UUID_TBS_CONTENT_CONTROL_ID = 0x2BBA,
    WICED_BT_UUID_TBS_STATUS_FLAGS = 0x2BBB,
    WICED_BT_UUID_TBS_INCOMING_TG_URI = 0x2BBC,
    WICED_BT_UUID_TBS_CALL_STATE = 0x2BBD,
    WICED_BT_UUID_TBS_CALL_CONTROL_POINT = 0x2BBE,
    WICED_BT_UUID_TBS_CALL_CONTROL_POINT_OPTIONAL_OPCODE = 0x2BBF,
    WICED_BT_UUID_TBS_TERMINATION_REASON = 0x2BC0,
    WICED_BT_UUID_TBS_INCOMING_CALL = 0x2BC1,
    WICED_BT_UUID_TBS_CALL_FRIENDLY_NAME = 0x2BC2,

    WICED_BT_UUID_MICS_MUTE_STATE = 0x2BC3,

    WICED_BT_UUID_ASCS_SINK_ASE = 0x2BC4,
    WICED_BT_UUID_ASCS_SOURCE_ASE = 0X2BC5,
    WICED_BT_UUID_ASCS_ASE_CONTROL_POINT = 0X2BC6,

    WICED_BT_UUID_BASS_CONTROL_POINT = 0X2BC7,
    WICED_BT_UUID_BASS_BROADCAST_RECEIVE_STATE = 0X2BC8,

    WICED_BT_UUID_PACS_SINK_PAC = 0X2BC9,
    WICED_BT_UUID_PACS_SINK_AUDIO_LOCATIONS = 0X2BCA,
    WICED_BT_UUID_PACS_SOURCE_PAC = 0X2BCB,
    WICED_BT_UUID_PACS_SOURCE_AUDIO_LOCATIONS = 0X2BCC,
    WICED_BT_UUID_PACS_AUDIO_CONTEXT_AVAILABILITY = 0X2BCD,
    WICED_BT_UUID_PACS_SUPPORTED_AUDIO_CONTEXT = 0X2BCE,

    WICED_BT_UUID_RAAS_SELECTABLE_AREP                       = 0X8FC0,
    WICED_BT_UUID_RAAS_CFG_AUDIO_ROUTE_LIST                  = 0X8FC1,
    WICED_BT_UUID_RAAS_CFG_AUDIO_ROUTE_CONTENT_TYPE          = 0X8FC2,
    WICED_BT_UUID_RAAS_MODIFY_AUDIO_ROUTE_CONTROL_POINT      = 0X8FC3,
};
#endif // __WICED_BT_GA_UUID__
