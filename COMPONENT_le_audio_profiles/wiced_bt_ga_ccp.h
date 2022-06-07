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
 * Call Control Profile (CCP) Application Programming Interface
 */

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_ga_common.h"
#include "wiced_bt_ga_tbs_common.h"

#define CCP_TRACE(...)
#define CCP_TRACE_CRIT(...)

/**
 * @addtogroup Telephone_Bearer_Service_APIs
 * @{
 */

/**
 * @addtogroup wiced_bt_ga_tbs
 * @{
 */
 /** list of bearer properties that is read from the app */
typedef enum
{
    TBS_BEARER_PROVIDER_NAME,                         /**< telephone bearer provider name value */
    TBS_BEARER_UCI,                                   /**< telephone bearer supported uci */
    TBS_BEARER_TECHNOLOGY,                            /**< telephone bearer supported technology */
    BEARER_URI_PREFIX_SUPPORTED_LIST,                 /**< telephone bearer supported uri*/
    TBS_BEARER_SIGNAL_STRENGTH,                       /**< telephone bearer signal strength*/
    TBS_BEARER_SIGNAL_STRENGTH_REPORTING_INTERVAL,    /**< telephone bearer signal strength reporting interval*/
} wiced_bt_ga_ccp_bearer_property_t;
/**@} wiced_bt_ga_tbs */
/**@} Telephone_Bearer_Service_APIs */
#ifdef __cplusplus
}
#endif

