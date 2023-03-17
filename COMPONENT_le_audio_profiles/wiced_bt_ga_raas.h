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

#ifndef __WICED_BT_GA_RAAS_H__
#define __WICED_BT_GA_RAAS_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "gatt_interface.h"
#include "wiced_bt_ga_raas_common.h"

#define RAAS_TRACE(...)
#define RAAS_TRACE_CRIT(...)

typedef union
{
    wiced_bt_ga_raas_selectable_arep_data_t     arep_list;           /**< List of available AREP */
    wiced_bt_ga_raas_cfg_audio_route_data_t     audio_route_list;    /**< Configured audio route list for a given route ID */
    wiced_bt_ga_raas_cfg_content_type_data_t    content_type_list;   /**< Configured audio route content type list for a given route ID */
} wiced_bt_ga_raas_data_t;


/**
* \brief Notify selectable audio route end point list change event to all the subscribed clients
* \details On any change in the selectable audio route enf point, Application is expected to send notify selectable arep list change api to the clients.
* The API checks for the subscription from the client, if subscribed notifies the client with change.
*
* @param   p_raas       instance of the Routing Active Audio service
*
* @return  WICED_TRUE is operation is successful otherwise WICED_FALSE.
*/
wiced_bool_t wiced_bt_ga_raas_selectable_arep_list_change(wiced_bt_ga_raas_t* p_raas, uint8_t count , wiced_bt_ga_raas_selectable_arep_t* arep_list);

/**
* \brief Notify configured audio route list change event to all the subscribed clients
* \details On any change in the configured audio route, Application is expected to send notify configured audio route list change api to the clients.
* The API checks for the subscription from the client, if subscribed notifies the client with change.
*
* @param   p_raas       instance of the Routing Active Audio service
*
* @return  WICED_TRUE is operation is successful otherwise WICED_FALSE.
*/
wiced_bool_t wiced_bt_ga_raas_cgf_audio_route_list_change(wiced_bt_ga_raas_t* p_raas, uint8_t count, wiced_bt_ga_raas_cfg_audio_route_t* audio_route_list);

/**
* \brief Notify configured audio route content type list change event to all the subscribed clients
* \details On any change in the configured audio route content type, Application is expected to send notify configured audio route content type list change api to the clients.
* The API checks for the subscription from the client, if subscribed notifies the client with change.
*
* @param   p_raas       instance of the Routing Active Audio service
*
* @return  WICED_TRUE is operation is successful otherwise WICED_FALSE.
*/
wiced_bool_t wiced_bt_ga_raas_cgf_audio_route_content_type_list_change(wiced_bt_ga_raas_t* p_raas,uint8_t count, wiced_bt_ga_raas_config_content_type_t* audio_route_content_type_list);

/**
* \brief Notify audio route change event to all the subscribed clients
* \details On any change in the configured audio route , Application is expected to send notify configured audio route list change api to the clients.
* The API checks for the subscription from the client, if subscribed notifies the client with change.
*
* @param   p_raas       instance of the Routing Active Audio service
*
* @return  WICED_TRUE is operation is successful otherwise WICED_FALSE.
*/
wiced_bool_t wiced_bt_ga_raas_audio_route_control_point_change(wiced_bt_ga_raas_t* p_raas, wiced_bt_ga_raas_cfg_audio_route_t* modified_audio_route);

#ifdef __cplusplus
}
#endif

#endif /* __WICED_BT_GA_RAAS_H__ */

