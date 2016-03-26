/*
 * Move all BLE stuff from main to here 
 */

#define USE_BSP 0

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_assert.h"
#include "nrf_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_lns.h"
#include "ble_dis.h"
// #include "sensorsim.h"
#include "app_timer.h"
#include "softdevice_handler.h"
#include "ble_conn_params.h"
#if USE_BSP
#include "bsp.h"
#include "bsp_btn_ble.h"
#endif
#include "device_manager.h"
#include "pstorage.h"

// #define ENABLE_DEBUG_LOG_SUPPORT
#include "app_trace.h"

// #include "SEGGER_RTT.h"

#include "app_util_platform.h" // for interrupts

#include "erl_ble.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT  1                              /**< Include or not the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#define DEVICE_NAME                          "Wireless_LNS"                             /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                    "ErlandLewin"                              /**< Manufacturer. Will be passed to Device Information Service. */

/* Only slow advertising, 1 per second. Optimize later for power savings? */
#define APP_ADV_FAST_ENABLED                 false
#define APP_ADV_FAST_INTERVAL                40                                         /**< The advertising interval (in units of 0.625 ms; 40 corresponds to 25 ms, 1600 to 1s). */
#define APP_ADV_FAST_TIMEOUT_IN_SECONDS      180                                        /**< The advertising time-out in units of seconds. */

#define APP_ADV_SLOW_ENABLED                 true
#define APP_ADV_SLOW_INTERVAL                1600 /* 40 */                              /**< The advertising interval (in units of 0.625 ms; 40 corresponds to 25 ms, 1600 to 1s). */
#define APP_ADV_SLOW_TIMEOUT_IN_SECONDS      0                                          /**< The advertising time-out in units of seconds. */

#define APPL_LOG                             app_trace_log

#define SECOND_1_25_MS_UNITS                 800                                        /**< Definition of 1 second, when 1 unit is 1.25 ms. */
#define SECOND_10_MS_UNITS                   100                                        /**< Definition of 1 second, when 1 unit is 10 ms. */
#define MIN_CONN_INTERVAL                    MSEC_TO_UNITS(250, UNIT_1_25_MS)           /**< Minimum connection interval (250 ms). */
#define MAX_CONN_INTERVAL                    MSEC_TO_UNITS(500, UNIT_1_25_MS)           /**< Maximum connection interval (500 ms). */
#define SLAVE_LATENCY                        0                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                     (4 * SECOND_10_MS_UNITS)                   /**< Connection supervisory time-out (4 seconds). Supervision time-out uses 10 ms units. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY       APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY        APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT         3                                          /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                       1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                       0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES            BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define SEC_PARAM_OOB                        0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE               7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE               16                                         /**< Maximum encryption key size. */

uint16_t                              m_conn_handle = BLE_CONN_HANDLE_INVALID;   /**< Handle of the current connection. */
ble_bas_t                             m_bas;                                     /**< Structure used to identify the battery service. */
ble_lns_t                             m_lns;                                     /**< Structure used to identify the location and navigation service. */

static dm_application_instance_t             m_app_handle;                              /**< Application identifier allocated by Device Manager. */

ble_lns_loc_speed_t                   m_sim_location_speed;                      /**< Location and speed simulation. */
ble_lns_pos_quality_t                 m_sim_position_quality;                    /**< Position measurement quality simulation. */
ble_lns_navigation_t                  m_sim_navigation;                          /**< Navigation data structure simulation. */

static ble_uuid_t                            m_adv_uuids[] = {
                                                               {BLE_UUID_LOCATION_AND_NAVIGATION_SERVICE,  BLE_UUID_TYPE_BLE}, 
                                                               {BLE_UUID_BATTERY_SERVICE,                  BLE_UUID_TYPE_BLE}, 
                                                               {BLE_UUID_DEVICE_INFORMATION_SERVICE,       BLE_UUID_TYPE_BLE}
                                                             };

static SleepModeEnterFunc sleepFunc;

static const ble_lns_loc_speed_t initial_lns_location_speed = {
  .instant_speed_present   = false, // true,
  .total_distance_present  = false, // true,,
    .location_present        = true,
    .elevation_present       = true,
    .heading_present         = true,
    .rolling_time_present    = true,
    .utc_time_time_present   = true,
    .position_status         = BLE_LNS_POSITION_OK,
    .data_format             = BLE_LNS_SPEED_DISTANCE_FORMAT_2D,
    .elevation_source        = BLE_LNS_ELEV_SOURCE_POSITIONING_SYSTEM,
    .heading_source          = BLE_LNS_HEADING_SOURCE_COMPASS,
    .instant_speed           = 12,         // = 1.2 meter/second
  .total_distance          = 0,       // = 2356 meters/second
    .latitude                = -103123567, // = -10.3123567 degrees
    .longitude               = 601234567,  // = 60.1234567 degrees
    .elevation               = 1350,       // = 13.5 meter
    .heading                 = 2123,       // = 21.23 degrees
    .rolling_time            = 1,          // = 1 second
    .utc_time                = {
                                 .year    = 2015,
                                 .month   = 7,
                                 .day     = 8,
                                 .hours   = 12,
                                 .minutes = 43,
                                 .seconds = 33            
                               }
};


static const ble_lns_pos_quality_t initial_lns_pos_quality = {
    .number_of_satellites_in_solution_present = true, 
    .number_of_satellites_in_view_present     = true,
    .time_to_first_fix_present                = true,
    .ehpe_present                             = true,
    .evpe_present                             = true,
    .hdop_present                             = true,
    .vdop_present                             = true,
    .position_status                          = BLE_LNS_POSITION_OK,
    .number_of_satellites_in_solution         = 12,
    .number_of_satellites_in_view             = 6,
    .time_to_first_fix                        = 63,  // = 6.3 seconds
    .ehpe                                     = 100, // = 1 meter
    .evpe                                     = 123, // = 1.23 meter
    .hdop                                     = 123,
    .vdop                                     = 143
};

static const ble_lns_navigation_t initial_lns_navigation = {
    .remaining_dist_present       = true,
    .remaining_vert_dist_present  = true,
    .eta_present                  = true,
    .position_status              = BLE_LNS_POSITION_OK,
    .heading_source               = BLE_LNS_HEADING_SOURCE_COMPASS,
    .navigation_indicator_type    = BLE_LNS_NAV_TO_WAYPOINT,
    .waypoint_reached             = false,
    .destination_reached          = false,
    .bearing                      = 1234,   // = 12.34 degrees
    .heading                      = 2123,   // = 21.23 degrees
    .remaining_distance           = 532576, // = 53257.6 meters
    .remaining_vert_distance      = 123,    // = 12.3 meters
    .eta                          = {
                                      .year    = 2015,
                                      .month   = 7,
                                      .day     = 8,
                                      .hours   = 16,
                                      .minutes = 43,
                                      .seconds = 33            
                                   }
};

/*
 * static function prototypes
 */
static void ble_stack_init(void);
static void device_manager_init(bool erase_bonds);
static void gap_params_init(void);
static void advertising_init(void);
static void services_init( ble_srv_error_handler_t errorHandler );
static void conn_params_init(void);
static void on_lns_evt(ble_lns_t * p_lns, ble_lns_evt_t * p_evt);

/*
 * Start of code
 */
void erl_ble_init( bool erase_bonds, ble_srv_error_handler_t lnsErrorHandler, SleepModeEnterFunc sleepModeEnter )
{
  // timers_init();

  sleepFunc = sleepModeEnter;

  ble_stack_init();
  device_manager_init(erase_bonds);
  gap_params_init();
  advertising_init();
  services_init( lnsErrorHandler);
  conn_params_init();
}


/**@brief GAP initialization.
 *
 * @details This function shall be used to set up all the necessary GAP (Generic Access Profile)
 *          parameters of the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode, 
                                          (const uint8_t *)DEVICE_NAME, 
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    // Changed to Location Pod from location ANd navigation display device
    err_code = sd_ble_gap_appearance_set( BLE_APPEARANCE_OUTDOOR_SPORTS_ACT_LOC_POD );
    APP_ERROR_CHECK(err_code);
    
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Initialize services that will be used by the application.
 *
 * @details Initialize the Location and Navigation, Battery and Device Information services.
 */
static void services_init( ble_srv_error_handler_t errorHandler )
{
    uint32_t       err_code;
    ble_lns_init_t lns_init;
    ble_bas_init_t bas_init;
    ble_dis_init_t dis_init;
    
    memset(&lns_init, 0, sizeof(lns_init));
    
    lns_init.evt_handler = on_lns_evt;
    lns_init.error_handler = errorHandler /* lns_error_handler */;
       
    lns_init.is_position_quality_present = true;
    lns_init.is_control_point_present    = true;
    lns_init.is_navigation_present       = true;
    
    lns_init.available_features     = /* BLE_LNS_FEATURE_INSTANT_SPEED_SUPPORTED                 |
					 BLE_LNS_FEATURE_TOTAL_DISTANCE_SUPPORTED                | */
                                      BLE_LNS_FEATURE_LOCATION_SUPPORTED                      |
                                      BLE_LNS_FEATURE_ELEVATION_SUPPORTED                     |
                                      BLE_LNS_FEATURE_HEADING_SUPPORTED                       |
      /* BLE_LNS_FEATURE_ROLLING_TIME_SUPPORTED                  | */
                                      BLE_LNS_FEATURE_UTC_TIME_SUPPORTED                      |
      /* BLE_LNS_FEATURE_REMAINING_DISTANCE_SUPPORTED            |
                                      BLE_LNS_FEATURE_REMAINING_VERT_DISTANCE_SUPPORTED       |
                                      BLE_LNS_FEATURE_EST_TIME_OF_ARRIVAL_SUPPORTED           | */
                                      BLE_LNS_FEATURE_NUM_SATS_IN_SOLUTION_SUPPORTED          |
                                      BLE_LNS_FEATURE_NUM_SATS_IN_VIEW_SUPPORTED              |
                                      BLE_LNS_FEATURE_TIME_TO_FIRST_FIX_SUPPORTED             |
                                      BLE_LNS_FEATURE_EST_HORZ_POS_ERROR_SUPPORTED            |
                                      BLE_LNS_FEATURE_EST_VERT_POS_ERROR_SUPPORTED            |
                                      BLE_LNS_FEATURE_HORZ_DILUTION_OF_PRECISION_SUPPORTED    |
                                      BLE_LNS_FEATURE_VERT_DILUTION_OF_PRECISION_SUPPORTED    | 
                                      BLE_LNS_FEATURE_LOC_AND_SPEED_CONTENT_MASKING_SUPPORTED |
                                      BLE_LNS_FEATURE_FIX_RATE_SETTING_SUPPORTED              | 
                                      BLE_LNS_FEATURE_ELEVATION_SETTING_SUPPORTED             |
                                      BLE_LNS_FEATURE_POSITION_STATUS_SUPPORTED;


    m_sim_location_speed   = initial_lns_location_speed;
    m_sim_position_quality = initial_lns_pos_quality;
    m_sim_navigation       = initial_lns_navigation;

    lns_init.p_location_speed   = &m_sim_location_speed;
    lns_init.p_position_quality = &m_sim_position_quality;
    lns_init.p_navigation       = &m_sim_navigation;
              
    lns_init.loc_nav_feature_security_req_read_perm  = SEC_OPEN;
    lns_init.loc_speed_security_req_cccd_write_perm  = SEC_OPEN;
    lns_init.position_quality_security_req_read_perm = SEC_OPEN;
    lns_init.navigation_security_req_cccd_write_perm = SEC_OPEN;
    lns_init.ctrl_point_security_req_write_perm      = SEC_OPEN;
    lns_init.ctrl_point_security_req_cccd_write_perm = SEC_OPEN;     
    
    err_code = ble_lns_init(&m_lns, &lns_init);
    APP_ERROR_CHECK(err_code);
    
    ble_lns_route_t route = {.route_name = "My route"};
    err_code = ble_lns_add_route(&m_lns, &route);
    
    // Initialize Battery Service
    memset(&bas_init, 0, sizeof(bas_init));
    
    // Here the sec level for the Battery Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;
    
    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service
    memset(&dis_init, 0, sizeof(dis_init));

    // ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);
    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, "MyDev");
    ble_srv_ascii_to_utf8(&dis_init.fw_rev_str, "FW-17");

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);

    // SEGGER_RTT_WriteString(0, "Called ble_dis_init\n" );
}






/**@brief Connection Parameters Module handler.
 *
 * @details This function will be called for all events in the Connection Parameters Module that
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;
    
    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Connection Parameters module error handler.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Initialize the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;
    
    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = m_lns.loc_speed_handles.cccd_handle;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;
    
    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events that are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
#if USE_BSP
    uint32_t err_code;
#endif
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
#if USE_BSP
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
#endif
            break;
        case BLE_ADV_EVT_IDLE:
	  sleepFunc();
          //  sleep_mode_enter();
            break;
        default:
            break;
    }
}

/**@brief Application's BLE Stack event handler.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
#if USE_BSP
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
#endif
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
#if USE_BSP
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
#endif
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;
        
        default:
            break;
    }

    APP_ERROR_CHECK(err_code);
}

/**@brief Dispatches a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    dm_ble_evt_handler(p_ble_evt);
    ble_lns_on_ble_evt(&m_lns, p_ble_evt);
    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
#if USE_BSP
    bsp_btn_ble_on_ble_evt(p_ble_evt);
#endif
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
}

/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    ble_advertising_on_sys_evt(sys_evt);
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);

#if defined(S110) || defined(S130) || defined(S310)
    // Enable BLE stack.
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
#if defined(S130) || defined(S310)
    ble_enable_params.gatts_enable_params.attr_tab_size   = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;
#endif
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
#endif

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Device Manager events.
 *
 * @param[in] p_evt  Data associated to the Device Manager event.
 */
static uint32_t device_manager_evt_handler(dm_handle_t const * p_handle,
                                           dm_event_t const  * p_event,
                                           ret_code_t        event_result)
{
    APP_ERROR_CHECK(event_result);
    return NRF_SUCCESS;
}


/**@brief Function for the Device Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Device Manager.
 */
static void device_manager_init(bool erase_bonds)
{
    uint32_t               err_code;
    dm_init_param_t        init_param = {.clear_persistent_data = erase_bonds};
    dm_application_param_t register_param;

    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_param);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));

    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&m_app_handle, &register_param);
    APP_ERROR_CHECK(err_code);

    app_trace_log( "device_manager_init: err_code=%d, m_app_handle=%d\r\n", err_code, m_app_handle );
}

/**@brief Advertising functionality initialization.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = m_adv_uuids;

    /* See http://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.sdk51.v9.0.0%2Flib_ble_advertising.html&cp=4_1_0_3_1_1 */
    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = APP_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_FAST_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_FAST_TIMEOUT_IN_SECONDS;

    options.ble_adv_slow_enabled  = APP_ADV_SLOW_ENABLED;
    options.ble_adv_slow_interval = APP_ADV_SLOW_INTERVAL;
    options.ble_adv_slow_timeout  = APP_ADV_SLOW_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Location Navigation event handler.
 *
 * @details This function will be called for all events of the Location Navigation Module that
 *          are passed to the application.
 *
 * @param[in]   p_evt   Event received from the Location Navigation Module.
 */
static void on_lns_evt(ble_lns_t * p_lns, ble_lns_evt_t * p_evt)
{   
  // SEGGER_RTT_WriteString(0, "on_lns_evt()\n" );

    switch(p_evt->evt_type)
    {
                    
        case BLE_LNS_LOC_SPEED_EVT_NOTIFICATION_ENABLED:
            APPL_LOG("LOC_SPEED_EVT: Notification enabled\r\n");
            break;
        
        case BLE_LNS_LOC_SPEED_EVT_NOTIFICATION_DISABLED:
            APPL_LOG("LOC_SPEED_EVT: Notification disabled\r\n");
            break;
    
        case BLE_LNS_LOC_SPEED_EVT_MASK_SET:
            APPL_LOG("LOC_SPEED_EVT: Feature mask set\r\n");
            break;
        
        case BLE_LNS_LOC_SPEED_EVT_TOTAL_DISTANCE_SET:
            APPL_LOG("LOC_SPEED_EVT: Set total distance: %ld\r\n", p_evt->params.total_distance);
            break;
    
        case BLE_LNS_LOC_SPEED_EVT_ELEVATION_SET:
            APPL_LOG("LOC_SPEED_EVT: Set elevation: %ld\r\n", p_evt->params.elevation);
            break;

        case BLE_LNS_POS_QUAL_EVT_FIX_RATE_SET:
            APPL_LOG("POS_QUAL_EVT: Fix rate set to %d\r\n", p_evt->params.fix_rate);
            break;
        
        case BLE_LNS_NAVIGATION_EVT_COMMAND:
            APPL_LOG("NAV_EVT: Navigation state changed to %d\r\n", p_evt->params.navigation_command);
            break;
        
        case BLE_LNS_NAVIGATION_EVT_ROUTE_SELECTED:
            APPL_LOG("NAV_EVT: Route selected %d\r\n", p_evt->params.selected_route);
            break;
        
        case BLE_LNS_NAVIGATION_EVT_NOTIFICATION_ENABLED:
            APPL_LOG("NAV_EVT: Notification enabled\r\n");
            break;
    
        case BLE_LNS_NAVIGATION_EVT_NOTIFICATION_DISABLED:
            APPL_LOG("NAV_EVT: Notification disabled\r\n");
            break;

        case BLE_LNS_CONTROL_POINT_EVT_INDICATION_ENABLED:
            APPL_LOG("CTRL_PNT_EVT: Indication enabled\r\n");
            break;
        
        case BLE_LNS_CONTROL_POINT_EVT_INDICATION_DISABLED:
            APPL_LOG("CTRL_PNT_EVT: Indication disabled\r\n");
            break;

        default:
            break;
    }
}





