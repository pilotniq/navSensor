/*
 * erl_ble.h
 */
#ifndef ERL_BLE_H
#define ERL_BLE_H

#include "ble_bas.h"
#include "ble_lns.h"

#define APP_TIMER_PRESCALER                  0                                          /**< Value of the RTC1 PRESCALER register. */

typedef void (*SleepModeEnterFunc)(void);

extern uint16_t                              m_conn_handle;                             /**< Handle of the current connection. */

extern ble_bas_t                             m_bas;                                     /**< Structure used to identify the battery service. */
extern ble_lns_t                             m_lns;                                     /**< Structure used to identify the location and navigation service. */

extern ble_lns_loc_speed_t                   m_sim_location_speed;                      /**< Location and speed simulation. */
extern ble_lns_pos_quality_t                 m_sim_position_quality;                    /**< Position measurement quality simulation. */
extern ble_lns_navigation_t                  m_sim_navigation;                          /**< Navigation data structure simulation. */

void erl_ble_init( bool erase_bonds, ble_srv_error_handler_t lnsErrorHandler, SleepModeEnterFunc sleepFunc );

#endif
