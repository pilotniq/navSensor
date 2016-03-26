/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_app_lns_main main.c
 * @{
 * @ingroup ble_sdk_app_lns
 * @brief Location and Navigation Service Sample Application main file.
 *
 * This file contains the source code for a sample application using the Location and Navigation service
 * (and also Battery and Device Information services). This application uses the
 * @ref srvlib_conn_params module.
 */

#define USE_BSP 0

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_assert.h"
#include "nrf_error.h"
// #include "app_util_platform.h" // for interrupts, 
// #include "ble_date_time.h"
#include "nrf_delay.h"

#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
// #include "ble_advdata.h"
#include "ble_advertising.h" // main calls ble_advertising_start
#include "ble_dis.h"

#include "sensorsim.h"
#include "erl_ble.h"
#include "ble_hci.h"
#if USE_BSP
#include "bsp.h"  // Board support package
#include "bsp_btn_ble.h"
#endif
#include "app_timer.h"


/*
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_lns.h"
#include "ble_dis.h"
#include "softdevice_handler.h"
#include "ble_conn_params.h"
#include "bsp.h"
#include "device_manager.h"
#include "pstorage.h"
*/
#include "sensorsim.h"
// #define ENABLE_DEBUG_LOG_SUPPORT
#include "app_trace.h"

// #include "SEGGER_RTT.h"

#include "app_util_platform.h" // for interrupts
#include "i2c_nrf51822_app.h"
#include "i2c.h"
#include "lsm303dlhc.h"

#define APPL_LOG                             app_trace_log

#define DEAD_BEEF                            0xDEADBEEF                                 /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

// #define APP_TIMER_PRESCALER                  0                                          /**< Value of the RTC1 PRESCALER register. */
#if USE_BSP
#define APP_TIMER_MAX_TIMERS                 (6+BSP_APP_TIMERS_NUMBER)                  /**< Maximum number of simultaneously created timers. */
#else
#define APP_TIMER_MAX_TIMERS                 (6)                  /**< Maximum number of simultaneously created timers. */
#endif
#define APP_TIMER_OP_QUEUE_SIZE              4                                          /**< Size of timer operation queues. */

#define BATTERY_LEVEL_MEAS_INTERVAL          APP_TIMER_TICKS(2400, APP_TIMER_PRESCALER) /**< Battery level measurement interval (ticks). */
#define MIN_BATTERY_LEVEL                    81                                         /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL                    100                                        /**< Maximum simulated battery level. */
#define BATTERY_LEVEL_INCREMENT              1                                          /**< Increment between each simulated battery level measurement. */

#define LOC_AND_NAV_DATA_INTERVAL            APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER) /**< Location and Navigation data interval (ticks). */

APP_TIMER_DEF(m_battery_timer_id);                                                      /**< Battery timer. */
APP_TIMER_DEF(m_loc_and_nav_timer_id);                                                  /**< Location and navigation measurement timer. */

static sensorsim_cfg_t                       m_battery_sim_cfg;                         /**< Battery Level sensor simulator configuration. */
static sensorsim_state_t                     m_battery_sim_state;                       /**< Battery Level sensor simulator state. */

/*
 * static function prototypes
 */
// Simulation stuff
static void battery_level_meas_timeout_handler(void * p_context);
static void loc_and_nav_timeout_handler(void * p_context);
static void sim_init(void);

static void increment_time(ble_date_time_t * p_time);
static void loc_speed_simulation_update(void);
static void position_quality_simulation_update(void);
static void navigation_simulation_update(void);
                                                            
static void buttons_leds_init(bool * p_erase_bonds);
#if USE_BSP
static void bsp_event_handler(bsp_event_t event);
#endif

/*
 * Start of code
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name) 
{
  char buf[ 1024 ];

  printf( buf, "app_error_handler( %lu, %s:%lu )\n", error_code, p_file_name, line_num );

  //  SEGGER_RTT_WriteString(0, buf);
} 

/**@brief Start application timers.
 */
static void application_timers_start(void)
{
    uint32_t err_code;
    
    // Start application timers
    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_loc_and_nav_timer_id, LOC_AND_NAV_DATA_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Initialize the simulators.
 */
static void sim_init(void)
{
    // battery simulation
    m_battery_sim_cfg.min          = MIN_BATTERY_LEVEL;
    m_battery_sim_cfg.max          = MAX_BATTERY_LEVEL;
    m_battery_sim_cfg.incr         = BATTERY_LEVEL_INCREMENT;
    m_battery_sim_cfg.start_at_max = true;
    
    sensorsim_init(&m_battery_sim_state, &m_battery_sim_cfg);
}

/**@brief Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    uint32_t err_code;
    
    // Initialize timer module
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_loc_and_nav_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                loc_and_nav_timeout_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for performing battery measurement and updating the Battery Level characteristic
 *        in Battery Service.
 */
static void battery_level_update(void)
{
    uint32_t err_code;
    uint8_t  battery_level;

    battery_level = (uint8_t)sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);

    err_code = ble_bas_battery_level_update(&m_bas, battery_level);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_BUFFERS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
        )
    {
        APP_ERROR_HANDLER(err_code);
    }
}

/**@brief Function for handling the Battery measurement timer time-out.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the time-out handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_level_update();
}

/**@brief Location and navigation time-out handler.
 *
 * @details This function will be called each time the location and navigation measurement timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the time-out handler.
 */
static void loc_and_nav_timeout_handler(void * p_context)
{
    uint32_t err_code;
    
    UNUSED_PARAMETER(p_context);

    loc_speed_simulation_update();
    position_quality_simulation_update();
    navigation_simulation_update();

    err_code = ble_lns_loc_speed_send(&m_lns);
    if (
        (err_code != NRF_ERROR_INVALID_STATE)
        &&
        (err_code != BLE_ERROR_NO_TX_BUFFERS)
        &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
    )
    {
        APP_ERROR_CHECK(err_code);
    }
    
    err_code = ble_lns_navigation_send(&m_lns);
    if (
        (err_code != NRF_ERROR_INVALID_STATE)
        &&
        (err_code != BLE_ERROR_NO_TX_BUFFERS)
        &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
    )
    {
        APP_ERROR_CHECK(err_code);
    }
}
                                                            
/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Callback function for errors in the Location Navigation Service.
 *
 * @details This function will be called in case of an error in the Location Navigation Service.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 */
static void lns_error_handler(uint32_t err_code)
{
    app_error_handler(DEAD_BEEF, 0, 0);
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
  uint32_t err_code;
#if USE_BSP
    bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);
#endif
    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}



/**@brief Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/**@brief Application main function.
 */
int main(void)
{
  uint32_t err_code;
    bool erase_bonds = false;
    sI2Cchannel i2cChannel;
    int16_t acc[3];
    int16_t mag[3];
    // bool erase_bonds;
      uint16_t heading;

    // SEGGER_RTT_WriteString(0, "Hello World 3!\n");

    // Initialize
    app_trace_init();
    
    app_trace_log( "test trace log\r\n" );

    printf( "Hej, printf!\n" );

    timers_init();
    // buttons_leds_init(&erase_bonds);
    erl_ble_init( erase_bonds, lns_error_handler, sleep_mode_enter );
    // sim_init();

    // Start out slow, at 100 kbps. Keep it simple - blocking - to start
    nrf51822_app_i2c_init( &i2cChannel, NRF_TWI_FREQ_400K, APP_IRQ_PRIORITY_LOW, 1, 0 );

    // SEGGER_RTT_WriteString(0, "i2c initialized\n");

    lsm303dlhc_init( &i2cChannel );

    // Test tilt compensation
    acc[0] = 20000;
    acc[1] = 2;
    acc[2] = 999;

    mag[0] = -408;
    mag[1] = -297;
    mag[2] = 237;
    
    heading = lsm303dlhc_tilt_compensate( acc, mag );
    heading = ((uint32_t) heading) * 360 / UINT16_MAX;

    printf( "Test heading: %d\n", heading );

    lsm303dlhc_acc_setDataRate( ACC_RATE_1_HZ );
    lsm303dlhc_mag_setMode( LSM303DLHC_MAG_MODE_CONTINUOUS );
    
    lsm303dlhc_acc_read( acc );
    lsm303dlhc_mag_read( mag );

    printf( "Acc: %d,%d,%d\n", acc[0], acc[1], acc[2] );
    printf( "Mag: %d,%d,%d\n", mag[0], mag[1], mag[2] );

    // Start execution 
    // application_timers_start();
    
    // Move this function call to erl_bleng 
    // why is fast/slow specified here, when it is also enabled/disabled in advertising options?
    err_code = ble_advertising_start(BLE_ADV_MODE_SLOW); // changed from fast to slow
    APP_ERROR_CHECK(err_code);
#if USE_BSP
    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    APP_ERROR_CHECK(err_code);
#endif

    APPL_LOG( "main(): before loop\r\n" );
    // Enter main loop
    for (;;)
    {
      lsm303dlhc_acc_read( acc );
      lsm303dlhc_mag_read( mag );
      heading = lsm303dlhc_tilt_compensate( acc, mag );
      heading = ((uint32_t) heading) * 36000 / UINT16_MAX;
      
      printf( "Acc: %d,%d,%d\n", acc[0], acc[1], acc[2] );
      printf( "Mag: %d,%d,%d\n", mag[0], mag[1], mag[2] );
      printf( "Heading=%d\n", heading );

      m_sim_location_speed.heading = heading; // ++;

      err_code = ble_lns_loc_speed_send(&m_lns);
      if (
	  (err_code != NRF_ERROR_INVALID_STATE)
	  &&
	  (err_code != BLE_ERROR_NO_TX_BUFFERS)
	  &&
	  (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
	  )
      {
	APP_ERROR_CHECK(err_code);
      }

      nrf_delay_ms( 500 );

      // fflush( stdout );
      //  power_manage();
    }
}

static void navigation_simulation_update(void)
{
    // ugly updating of status 
    m_sim_navigation.position_status           = (ble_lns_pos_status_type_t)
                                                 (
                                                    ( (uint32_t) m_sim_navigation.position_status + 1) % 
                                                    ( (uint32_t) BLE_LNS_LAST_KNOWN_POSITION + 1)
                                                 ); 
#if 0
    m_sim_navigation.heading_source            = (ble_lns_heading_source_t)
                                                 (
                                                    ( (uint32_t) m_sim_navigation.heading_source  + 1) % 
                                                    ( (uint32_t) BLE_LNS_HEADING_SOURCE_COMPASS + 1)
                                                 );
#endif
    m_sim_navigation.navigation_indicator_type = (ble_lns_nav_indicator_type_t)
                                                 (
                                                    ( (uint32_t) m_sim_navigation.navigation_indicator_type + 1) % 
                                                    ( (uint32_t) BLE_LNS_NAV_TO_DESTINATION + 1)
                                                 ); 
    
    m_sim_navigation.waypoint_reached    = !m_sim_navigation.waypoint_reached;
    m_sim_navigation.destination_reached = !m_sim_navigation.destination_reached;
    m_sim_navigation.bearing++;
    // m_sim_navigation.heading++;
    m_sim_navigation.remaining_distance++;
    m_sim_navigation.remaining_vert_distance++;

    increment_time(&m_sim_navigation.eta);
}

static void position_quality_simulation_update(void)
{
    // ugly updating of status 
    m_sim_position_quality.position_status = (ble_lns_pos_status_type_t)
                                             (
                                                ( (uint32_t) m_sim_position_quality.position_status + 1) %
                                                ( (uint32_t) BLE_LNS_LAST_KNOWN_POSITION + 1) 
                                             );
    m_sim_position_quality.number_of_satellites_in_solution++;
    m_sim_position_quality.number_of_satellites_in_view++;
    m_sim_position_quality.time_to_first_fix++;
    m_sim_position_quality.ehpe++;
    m_sim_position_quality.evpe++;
    m_sim_position_quality.hdop++;
    m_sim_position_quality.vdop++;
}


/**@brief Provide simulated location and speed.
 */
static void loc_speed_simulation_update(void)
{
    // ugly updating of status 
    m_sim_location_speed.position_status  = (ble_lns_pos_status_type_t)
                                            ( 
                                                ( (uint32_t) m_sim_location_speed.position_status + 1) %
                                                ( (uint32_t) BLE_LNS_LAST_KNOWN_POSITION + 1)
                                            ); 
    m_sim_location_speed.data_format      = (ble_lns_speed_distance_format_t)
                                            (
                                                ( (uint32_t) m_sim_location_speed.data_format + 1) % 
                                                ( (uint32_t) BLE_LNS_SPEED_DISTANCE_FORMAT_3D + 1)
                                            ); 
    m_sim_location_speed.elevation_source = (ble_lns_elevation_source_t)
                                            (
                                                ( (uint32_t) m_sim_location_speed.elevation_source + 1) %
                                                ( (uint32_t) BLE_LNS_ELEV_SOURCE_OTHER + 1)
                                            ); 
#if 0
    m_sim_location_speed.heading_source   = (ble_lns_heading_source_t)
                                            (
                                                ( (uint32_t) m_sim_location_speed.heading_source + 1) %
                                                ( (uint32_t) BLE_LNS_HEADING_SOURCE_COMPASS + 1)
                                            ); 
#endif
    m_sim_location_speed.total_distance++;
    m_sim_location_speed.latitude++;
    m_sim_location_speed.longitude++;
    m_sim_location_speed.elevation++;
    // m_sim_location_speed.heading++;
    m_sim_location_speed.rolling_time++;
    
    increment_time(&m_sim_location_speed.utc_time);    
}

static void increment_time(ble_date_time_t * p_time)
{
    p_time->seconds++;
    if (p_time->seconds > 59)
    {
        p_time->seconds = 0;
        p_time->minutes++;
        if (p_time->minutes > 59)
        {
            p_time->minutes = 0;
            p_time->hours++;
            if (p_time->hours > 24)
            {
                p_time->hours = 0;
                p_time->day++;
                if (p_time->day > 31)
                {
                    p_time->day = 0;
                    p_time->month++;
                    if (p_time->month > 12)
                    {
                        p_time->year++;
                    }
                }
            }
        }
    }
}

/**@brief Function for initializing buttons and LEDs.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
#if USE_BSP
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), 
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
#endif
}
#if USE_BSP
/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
static void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            err_code = ble_advertising_restart_without_whitelist();
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            break;
    }
}
#endif
/** 
 * @}
 */
