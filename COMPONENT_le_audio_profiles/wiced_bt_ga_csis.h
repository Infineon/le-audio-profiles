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
 * Co-ordinate Set Identification Service (CSIS) Application Programming Interface
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
#include "wiced_bt_ga_csis_common.h"

#define CSIS_TRACE(...)
#define CSIS_TRACE_CRIT(...)
#define CSIS_TRACE_ARRAY(...)

/**
 * @addtogroup Coordinate_Set_APIs
 * @{
 */

/**
 * @addtogroup wiced_bt_ga_csis
 * @{
 */

/**
 * \brief Generate PSRI valiue using the SIRK of the coordinated set.
 * \details PSRI value will be used during advertisement procedure to inform the clients regarding the specific set.
 *
 * @param[in]   sirk           SIRK value to be used for the specific coordinated set
 * @return  PSRI           PSRI value generated
 */
PSRI* wiced_bt_ga_csis_generate_psri(SIRK* sirk);

/**
 * \brief Set PSRI adv data.
 * \details Application use this api to advertise the set members coordinate set.
 *
 * @param[in]   psri           psri value to be set in the advertisement
 * @param[in]   adv_elem       memory in which the advertisement data has to be filled
 */
void wiced_bt_ga_csis_get_adv_data(PSRI* psri, wiced_bt_ble_advert_elem_t* adv_elem);

/**
 * \brief Set Lock timeout in seconds
 * \details Higher layer profile can set timeout value in seconds, it is by default set to 60 seconds.
 *
 * @param[in]   p_service    instance of the coordinated set identification service
 * @param[in]   timeout_in_sec timeout  value in seconds
 * @return  WICED_TRUE is operation is successful otherwise WICED_FALSE.
 */
void wiced_bt_ga_csis_set_lock_timeout_value(gatt_intf_service_object_t* p_service, uint8_t timeout_in_sec);

/**
 * \brief wiced_bt_ga_csis_is_operation_allowed
 * \details Other profile should invoke this API to check whether operation is allowed or not for given connection id.
 *
  * @param[in]   conn_id Connection id
 * @return  WICED_TRUE is operation is allowed otherwise WICED_FALSE.
 */
wiced_bool_t wiced_bt_ga_csis_is_operation_allowed(uint16_t conn_id);

/**
* Initialize the CSIS service_type/profile
* @param[in] p_cfg : Generic Audio configuration
*/
wiced_result_t wiced_bt_ga_csis_init(ga_cfg_t* p_cfg);

/**@} wiced_bt_ga_csis */
/**@} Coordinate_Set_APIs */
#ifdef __cplusplus
}
#endif

