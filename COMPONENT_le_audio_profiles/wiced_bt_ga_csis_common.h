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
 * Co-ordinate Set Identification Service (CSIS) Application Programming Interface
 */
 
#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#define WICED_BT_GA_CSIS_SET_IDENTITY_RESOLVING_KEY_LEN 16
#define WICED_BT_GA_CSIS_PRIVATE_SET_RANDOM_IDENTIFIER_LEN 6

typedef uint8_t wiced_bt_ga_csis_sirk_t[WICED_BT_GA_CSIS_SET_IDENTITY_RESOLVING_KEY_LEN]; /**< Device address length */
typedef uint8_t wiced_bt_ga_csis_psri_t[WICED_BT_GA_CSIS_PRIVATE_SET_RANDOM_IDENTIFIER_LEN]; /**< Device address length */

#define SIRK wiced_bt_ga_csis_sirk_t
#define PSRI wiced_bt_ga_csis_psri_t

/**
 * @anchor LOCK_VALUE
 * @name Definition for lock values which can be set by application
 * @{ */
typedef enum
{
    WICED_BT_GA_CSIS_UNLOCKED = 1, /**< Lock value unlocked*/
    WICED_BT_GA_CSIS_LOCKED = 2,   /**< Lock value locked */
} wiced_bt_ga_csis_lock_val_t;
/** @} LOCK_VALUE */

/**
 * @anchor SIRK_TYPE
 * @name Definition for SIRK types which can be set by application
 * @{ */
typedef enum
{
    WICED_BT_GA_CSIS_SIRK_ENCR = 0,  /**< SIRK in encypted format */
    WICED_BT_GA_CSIS_SIRK_PLAIN = 1, /**< SIRK in plain text */
} wiced_bt_ga_csis_sirk_type_t;
/** @} SIRK_TYPE */

/** SIRK Data */
typedef struct
{
    wiced_bool_t                 is_oob;    /**< is SIRK to be obtained via OOB methods */
    wiced_bt_ga_csis_sirk_type_t sirk_type; /**< SIRK type */
    SIRK                         sirk;      /**< SIRK key */
} wiced_bt_ga_csis_sirk_data_t;


/** CSIS Data */
typedef union
{
    wiced_bt_ga_csis_sirk_data_t sirk_data;     /**< set identity resolving key */
    uint8_t                      size;          /**< number of devices in the coordinated set values from 0x02 to 0xFF. Values 0x00 and 0x01 are Prohibited.*/
    uint8_t                      rank;          /**< rank of the device in the coordinated set */
    wiced_bt_ga_csis_lock_val_t  lock_val;      /**< lock value */
    uint8_t                      status;        /**< procedure status */
} wiced_bt_ga_csis_data_t;

/**@} wiced_bt_ga_csis */
/**@} Coordinate_Set_APIs */

#ifdef __cplusplus
}
#endif
