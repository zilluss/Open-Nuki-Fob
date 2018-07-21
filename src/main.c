#include <stdbool.h>
#include <stdint.h>
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "ble_db_discovery.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_ble_gatt.h"
#include "nrf_drv_rng.h"
#include "nrf_gpio.h"
#include "crc16.h"
#include "nuki.h"
#include "nrf_power.h"
#include "ble_flash.h"

#include "app_timer.h"
#include "nrf_drv_clock.h"
#include "nrf_delay.h"

#include "bt_comm_nrf.h"
#include "bt_comm.h"
#include "utils.h"

#define UUID_LEN 6
#define CENTRAL_LINK_COUNT    1                                 
#define PERIPHERAL_LINK_COUNT    0                                 

#define APP_CFG_NON_CONN_ADV_TIMEOUT    0                                 /**< Time for which the device must be advertising in non-connectable mode (in seconds). 0 disables timeout. */
#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(100, UNIT_0_625_MS) /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define DEAD_BEEF                       0xDEADBEEF                        /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

//ticks every ms
#define APP_TIMER_PRESCALER             31                                 /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         6                                 /**< Size of timer operation queues. */

#define BLE_SCAN_INTERVAL               MSEC_TO_UNITS(50, UNIT_0_625_MS)             
#define BLE_SCAN_WINDOW                 MSEC_TO_UNITS(25, UNIT_0_625_MS)             
#define BLE_SCAN_TIMEOUT_SECONDS        90 


#define MIN_CONNECTION_INTERVAL         MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONNECTION_INTERVAL         MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define SUPERVISION_TIMEOUT             MSEC_TO_UNITS(32000, UNIT_10_MS) 

#define BUTTON_PIN 28
#define LED_PIN 29
#define NRF_CLOCK_LFCLKSRC {.source = NRF_CLOCK_LF_SRC_XTAL, .rc_ctiv = 0, .rc_temp_ctiv  = 0, .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}

#define BUTTON_PULL NRF_GPIO_PIN_PULLUP
#define BUTTON_PRESSED 0
#define BUTTON_RELEASED 1
#define BUTTON_SENSE NRF_GPIO_PIN_SENSE_LOW
#define DELAY_100_MS 100 * 1000
#define DEBOUNCE_TIME_MS 50

APP_TIMER_DEF(button_counter_timer_id);
APP_TIMER_DEF(shutdown_timer_id);
APP_TIMER_DEF(led_timer_id);
APP_TIMER_DEF(reconnect_timer_id);
#define SHUTDOWN_TICKS APP_TIMER_TICKS(90*1000, APP_TIMER_PRESCALER)
#define BUTTON_TIMER_TICKS APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)
#define LED_TIMER_TICKS APP_TIMER_TICKS(100, APP_TIMER_PRESCALER)
#define RECONNECT_TIMER_TICKS APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)

const ble_uuid128_t NUKI_KEYTURNER_SERVICE_UUID = {.uuid128={ NUKI_KEYTURNER_SERVICE_BASE_UUID }};
const ble_uuid128_t NUKI_PAIRING_SERVICE_UUID = {.uuid128={ NUKI_PAIRING_SERVICE_BASE_UUID }};

static nrf_ble_gatt_t m_gatt;
static ble_db_discovery_t m_ble_db_discovery;

static uint16_t io_characteristic = 0;

enum fob_action {
    ACTION_NONE = 0, ACTION_FOB_1 = 1, ACTION_FOB_2 = 2, ACTION_FOB_3 = 3, ACTION_PAIRING = 4
};
static uint16_t action_to_execute = ACTION_NONE;
static uint8_t led_counter = 0;

static uint16_t connection_handle = 0;

/**@brief Parameters used when scanning.
 */
static ble_gap_scan_params_t const m_scan_params =
{
    .active            = 0,
    .selective         = 0,
    .p_whitelist       = 0,
    .interval          = BLE_SCAN_INTERVAL,
    .window            = BLE_SCAN_WINDOW,
    .timeout           = BLE_SCAN_TIMEOUT_SECONDS 
};

/**@brief Connection parameters requested for connection.
 */
static ble_gap_conn_params_t const m_connection_param =
{
    MIN_CONNECTION_INTERVAL,
    MAX_CONNECTION_INTERVAL,
    SLAVE_LATENCY,
    SUPERVISION_TIMEOUT
};

static void shutdown() 
{
    NRF_LOG_INFO("Enter sleep mode");
    nrf_gpio_pin_clear(LED_PIN);
    NRF_POWER->SYSTEMOFF = 1;
}

static void shutdown_on_error(uint32_t error_code)
{
    if(error_code != NRF_SUCCESS) 
    {
        shutdown();
    }
}

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

__WEAK void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    shutdown();
}

void HardFault_Handler(void)
{
    shutdown();
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    nrf_ble_gatt_init(&m_gatt, NULL);
}

static bool is_paired_uuid(const uint8_t* uuid2) 
{
    pairing_context* pairing_ctx = get_pairing_context();
    for(int i = 0; i < UUID_LEN; i++) 
    {
        if(pairing_ctx->paired_lock_uuid[i] != uuid2[i]) return false;
    }
    return true;
}

static bool advertises_pairing(const uint8_t* advertising_data) 
{
    for(int i = 0; i < 16; i++) 
    {
        if(advertising_data[5+i] != NUKI_PAIRING_SERVICE_UUID.uuid128[i]) 
        {
            return false;
        }
    }
    return true;
}

static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    if(io_characteristic != 0) return;
    if (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE) {
        for (int i = 0; i < p_evt->params.discovered_db.char_count; i++)
        {
            uint16_t characteristic_uuid = p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid;
            if(action_to_execute == ACTION_PAIRING) {
                if (characteristic_uuid == GDIO_CHAR_UUID)
                {
                    io_characteristic = p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
                    start_pairing();
                    break;
                }
            } 
            else 
            {
                if (characteristic_uuid == USDIO_CHAR_UUID)
                {
                    io_characteristic = p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
                    perform_lock_action(action_to_execute + 0x80); //convert enum value to actual nuki command value
                    break;
                }
            }
            
        }
    }
}

static void db_discovery_init(void)
{
    ble_db_discovery_init(db_disc_handler);

    uint8_t uuid_type;
    ble_uuid_t uuid;

    sd_ble_uuid_vs_add(&NUKI_KEYTURNER_SERVICE_UUID, &uuid_type);
    uuid.uuid = 0xe200;
    uuid.type = uuid_type;
    ble_db_discovery_evt_register(&uuid);

    sd_ble_uuid_vs_add(&NUKI_PAIRING_SERVICE_UUID, &uuid_type);
    uuid.uuid = 0xe100;
    uuid.type = uuid_type;
    ble_db_discovery_evt_register(&uuid);
}

static void scan_start()
{
    if(action_to_execute!=ACTION_PAIRING) 
    {
        app_timer_start(reconnect_timer_id, RECONNECT_TIMER_TICKS, NULL);
    }
    sd_ble_gap_disconnect(connection_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    sd_ble_gap_scan_stop();
    sd_ble_gap_scan_start(&m_scan_params);
}

static void find_lock_to_connect(ble_gap_evt_adv_report_t * p_adv_report) 
{
    bool lock_found = false;
    //search for a device that advertises the pairing service
    if(action_to_execute == ACTION_PAIRING && advertises_pairing(p_adv_report->data))
    {
        lock_found = true;
        pairing_context* pairing_ctx = get_pairing_context();
        memcpy(&pairing_ctx->paired_lock_uuid, p_adv_report->peer_addr.addr, 6);
    }
    //find the already paired lock
    else if(is_paired_uuid(&p_adv_report->peer_addr.addr[0])) 
    {
        lock_found = true;
    }

    if(lock_found) 
    {
        uint32_t err_code = sd_ble_gap_scan_stop();
        err_code = sd_ble_gap_connect(&p_adv_report->peer_addr, &m_scan_params, &m_connection_param);
        shutdown_on_error(err_code);
    }
}

static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code = 0;
    ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;
    ble_db_discovery_on_ble_evt(&m_ble_db_discovery, p_ble_evt);
    bt_comm_on_ble_evt(p_ble_evt);
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
        {
            ble_gap_evt_adv_report_t * p_adv_report = &p_gap_evt->params.adv_report;
            find_lock_to_connect(p_adv_report);
        }
        break; 

        case BLE_GAP_EVT_CONNECTED:
            connection_handle = p_ble_evt->evt.gap_evt.conn_handle;
            memset(&m_ble_db_discovery, 0x00, sizeof(m_ble_db_discovery));
            err_code  = ble_db_discovery_start(&m_ble_db_discovery, connection_handle);
            shutdown_on_error(err_code);
            break;

        case BLE_GAP_EVT_TIMEOUT:

            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
            {
                shutdown();
            }
            else if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                shutdown();
            }
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            sd_ble_gap_conn_param_update(p_gap_evt->conn_handle, &p_gap_evt->params.conn_param_update_request.conn_params);
            break; 
        default:
            break;
    }
}

static void ble_stack_init(void)
{
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);

    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
    softdevice_enable(&ble_enable_params);
    softdevice_ble_evt_handler_set(on_ble_evt);
}

static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    shutdown_on_error(err_code);
}

static bool button_timer_running = false;
static void button_timer_handler(void * p_context)
{
    button_timer_running = false;
}

static void start_button_timer() 
{
    button_timer_running = true;
    app_timer_start(button_counter_timer_id, BUTTON_TIMER_TICKS, NULL);
}

static void shutdown_timer() 
{
    shutdown();
}

static void led_timer() 
{
    if(action_to_execute == ACTION_PAIRING) 
    {
        nrf_gpio_pin_set(LED_PIN);
        return;
    }

    bool is_on = nrf_gpio_pin_out_read(LED_PIN);
    if((is_on && led_counter > 2) || (!is_on && led_counter > 7))
    {
        nrf_gpio_pin_toggle(LED_PIN);
        led_counter = 0;
    }
    led_counter++;
}

static void initialize_timer(void)
{
    nrf_drv_clock_init();
    nrf_drv_clock_lfclk_request(NULL);

	APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    app_timer_create(&button_counter_timer_id, APP_TIMER_MODE_SINGLE_SHOT, button_timer_handler);

    app_timer_create(&shutdown_timer_id, APP_TIMER_MODE_SINGLE_SHOT, shutdown_timer);
    app_timer_start(shutdown_timer_id, SHUTDOWN_TICKS, NULL);

    app_timer_create(&led_timer_id, APP_TIMER_MODE_REPEATED, led_timer);
    app_timer_start(led_timer_id, LED_TIMER_TICKS, NULL);

    app_timer_create(&reconnect_timer_id, APP_TIMER_MODE_SINGLE_SHOT, scan_start);
}

static void init_gpio() {
    nrf_gpio_cfg_sense_input(BUTTON_PIN, BUTTON_PULL, BUTTON_SENSE);
    nrf_gpio_cfg_output(LED_PIN);
    nrf_gpio_pin_clear(LED_PIN);
}

static bool click_registered() {
    uint32_t last_button_state = nrf_gpio_pin_read(BUTTON_PIN);

    start_button_timer();
    while(button_timer_running) {
        uint32_t current_button_state = nrf_gpio_pin_read(BUTTON_PIN);
        if(last_button_state == BUTTON_PRESSED && current_button_state == BUTTON_RELEASED) {
            return true;
        }
        last_button_state = current_button_state;
    }
    return false;
}

static bool hold_registered() {
    //9 seconds because click 1 already consumed 1 second
    for(int i = 0; i < 90; i++) {
        if(nrf_gpio_pin_read(BUTTON_PIN) == BUTTON_RELEASED) {
            return false;
        }
        nrf_delay_us(DELAY_100_MS);
    }
    return true;
}

static uint16_t read_action_to_execute() {
    
    bool click_1 = click_registered();
    nrf_delay_ms(DEBOUNCE_TIME_MS);
    if(click_1) {
        bool click_2 = click_registered();
        if(!click_2) {return ACTION_FOB_1;}
        nrf_delay_us(DEBOUNCE_TIME_MS);
        bool click_3 = click_registered();
        if(!click_3) {return ACTION_FOB_2;}
        return ACTION_FOB_3;
    } 
    if(hold_registered()) {
        return ACTION_PAIRING;
    }
    return ACTION_NONE;
}

void load_settings_from_flash() {
    pairing_context* pairing_ctx = get_pairing_context();
    uint8_t last_flash_page = NRF_FICR->CODESIZE - 1;
    uint8_t words_read;
    ble_flash_page_read (last_flash_page, (uint32_t*)pairing_ctx, &words_read);
}

void unlock_finished() {
    shutdown();
}

void pairing_finished() {
    sd_softdevice_disable();
    pairing_context* pairing_ctx = get_pairing_context();
    uint8_t last_flash_page = NRF_FICR->CODESIZE - 1;
    ble_flash_page_write(last_flash_page, (uint32_t*)(pairing_ctx), sizeof(pairing_context)/4);
    shutdown();
}

int main(void)
{
    initialize_timer();
    init_gpio();
    NRF_LOG_INIT(NULL);

    action_to_execute = read_action_to_execute();

    if(action_to_execute == ACTION_NONE) {
        shutdown();
    }
    load_settings_from_flash();

    nrf_drv_rng_init(NULL);
	ble_stack_init();
    gatt_init();
    db_discovery_init();
    scan_start();

    for (;; )
    {
        if (NRF_LOG_PROCESS() == false)
        {
            power_manage();
            process_messages(connection_handle, io_characteristic);

            uint32_t button_state = nrf_gpio_pin_read(BUTTON_PIN);
            if(button_state == BUTTON_PRESSED && action_to_execute != ACTION_PAIRING) {
                NVIC_SystemReset();
            }
        }
    }
}
