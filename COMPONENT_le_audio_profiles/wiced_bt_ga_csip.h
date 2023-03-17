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
 * Co-ordinate Set Identification Profile (CSIP) Application Programming Interface
 */

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_ga_csis_common.h"

#define CSIP_TRACE(...)
#define CSIP_TRACE_CRIT(...)
#define CSIP_TRACE_ARRAY(...)

/**
 * @addtogroup Coordinate_Set_APIs
 * @{
 */

/**
 * @addtogroup wiced_bt_ga_csip
 * @{
 */

/** CSIP timeout to discover the set members */
#define WICED_BT_GA_CSIP_SET_MEMBER_DISCOVERY_TIMEOUT_IN_SEC      10 //10 seconds

/** Ordered Access complete callback to be set by the application, this will be invoked once the ordered access procedure is complete */
typedef wiced_bt_gatt_status_t(*wiced_bt_ordered_access_procedure_cmpl_cb)(gatt_intf_service_object_t* p_csip_inst, void *data, wiced_bool_t status);
/** Lock procedure complete callback to be set by the application, this will be invoked once the lock procedure is complete */
typedef wiced_bt_gatt_status_t(*wiced_bt_lock_procedure_cmpl_cb)(gatt_intf_service_object_t* p_csip_inst, void *data, wiced_bool_t status);

/** CSIP set entry per set memeber */
typedef struct {
    uint16_t conn_id; /**< GATT Connection ID */
    gatt_intf_service_object_t* p_service; /**< instance of the coordinated set identification client */
} wiced_bt_ga_csip_set_entry;

/**
* \brief Checks if the device belong to the co-ordinated set
* \details Checks if PSRI belongs to the co-ordinated set
*
* @param[in]   p_adv_report:  adv_report of the scan
* @param[in]   p_adv_data  :  adv data of the scan
* @param[in]   p_sirk      :  sirk value of the co-ordinate set to be found
*
* @return  wiced_bool_t  WICED_TRUE if psri belongs to coordinated set of sirk
*/
wiced_bool_t wiced_bt_ga_csip_check_if_belongs_to_coordinated_set(wiced_bt_ble_scan_results_t* p_adv_report, uint8_t* p_adv_data, SIRK* p_sirk);

/**
* \brief Read set identification resolving key
* \details Reads the set identification resolving key characteristic of the csis
*
* @param[in] conn_id : GATT Connection ID
* @param[in] p_service :  instance of the coordinated set identification client
*
* @return  wiced_bt_gatt_status_t result of the read operation
*/
wiced_bt_gatt_status_t wiced_bt_ga_csip_read_set_identity_resolving_key(uint16_t conn_id, gatt_intf_service_object_t * p_service);

/**
* \brief Read lock state
* \details Reads the lock state of the csis instance
*
* @param[in] conn_id : GATT Connection ID
* @param[in] p_service :  instance of the coordinated set identification client
*
* @return  wiced_bt_gatt_status_t result of the read operation
*/
wiced_bt_gatt_status_t wiced_bt_ga_csip_read_lock_value(uint16_t conn_id, gatt_intf_service_object_t * p_service);

/**
* \brief Set lock state
* \details Sets the lock state of the csis instance
*
* @param[in]   conn_id : GATT Connection ID
* @param[in]   p_service :  instance of the coordinated set identification client
* @param[in]   lock_val  :    lock value to be set
*
* @return  wiced_bt_gatt_status_t result of the read operation
*/
wiced_bt_gatt_status_t wiced_bt_ga_csip_request_lock(uint16_t conn_id, gatt_intf_service_object_t * p_service, wiced_bt_ga_csis_lock_val_t lock_val);

/**
* \brief Lock Procedure
* \details Tries to aquire lock from the least rank device to highest rank. If any device responds with lock denied unlocks already locked devices.
*
*
* @param[in]   p_set : csip set memeber entries to perform lock procedure
* @param[in]   num_instances : number of client instances sent
* @param[in]   lock_val : lock value to be set
* @param[in]   cb : callback returned when the complete lock procedure is completed
* @param[in]   data pointer : this will be return in given cb
*/
wiced_bt_gatt_status_t wiced_bt_ga_csip_start_lock_procedure(wiced_bt_ga_csip_set_entry *p_set,
                                                             int num_instances,
                                                             wiced_bt_ga_csis_lock_val_t lock_val,
                                                             wiced_bt_lock_procedure_cmpl_cb cb,
                                                             void *data);

/**
* \brief Ordered Access Procedure
* \details Reads lock value from the least rank device to highest rank. If any device responds with locked,
* returns with the event WICED_BT_GA_CSIP_ORDERED_ACCESS_PROCEDURE_COMPLETE with status set to the procedure status
*
* @param[in]   p_set : csip set memeber entries to be locked
* @param[in]   num_instances : number of client instances sent
* @param[in]   cb : callback returned when the complete ordered access procedure is completed
* @param[in]   data pointer : this swill be return in given cb
*/
void wiced_bt_ga_csip_start_ordered_access_procedure(wiced_bt_ga_csip_set_entry* p_set, int num_instances, wiced_bt_ordered_access_procedure_cmpl_cb cb, void *data);

/**@} wiced_bt_ga_csip */
/**@} Coordinate_Set_APIs */
#ifdef __cplusplus
}
#endif
