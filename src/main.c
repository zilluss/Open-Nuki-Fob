#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "nordic_common.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_ble_scan.h"
#include "nrf_ble_gatt.h"
#include "nrf_drv_rng.h"
#include "nrf_gpio.h"
#include "ble_db_discovery.h"
#include "nuki.h"
#include "nrf_power.h"

#include "app_timer.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf_sdm.h"
#include "nrf_sdh.h"
#include "nrf_fstorage.h"
#include "nrf_fstorage_sd.h"
#include "nrfx_saadc.h"

#include "bt_comm_nrf.h"
#include "bt_comm.h"
#include "utils.h"

#define UUID_LEN 6
#define APP_BLE_CONN_CFG_TAG 1                                 
#define APP_BLE_OBSERVER_PRIO 3

//#define APP_CFG_NON_CONN_ADV_TIMEOUT    0                                 /**< Time for which the device must be advertising in non-connectable mode (in seconds). 0 disables timeout. */
//#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(100, UNIT_0_625_MS) /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define DEAD_BEEF 0xDEADBEEF

#define BLE_SCAN_INTERVAL 160
#define BLE_SCAN_WINDOW 80
#define BLE_SCAN_TIMEOUT_SECONDS MSEC_TO_UNITS(90000, UNIT_10_MS) 


#define MIN_CONNECTION_INTERVAL MSEC_TO_UNITS(20, UNIT_1_25_MS)
#define MAX_CONNECTION_INTERVAL MSEC_TO_UNITS(200, UNIT_1_25_MS)
#define SLAVE_LATENCY 0
#define SUPERVISION_TIMEOUT MSEC_TO_UNITS(4000, UNIT_10_MS) 

#define BUTTON_PIN 25
#define LED_PIN 26

#define BUTTON_PULL NRF_GPIO_PIN_PULLUP
#define BUTTON_PRESSED 0
#define BUTTON_RELEASED 1
#define BUTTON_SENSE NRF_GPIO_PIN_SENSE_LOW
#define DELAY_100_MS 100 * 1000
#define DEBOUNCE_TIME_MS 50

#define BATTERY_LOW_VOLTAGE 2.7
#define ADC_GAIN (1.f/6.f)
#define ADC_REFERENCE_VOLTAGE (0.6f)
#define ADC_RESOLUTION_BITS (8 + (NRFX_SAADC_CONFIG_RESOLUTION * 2))

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt);
#define LAST_FLASH_PAGE ((NRF_FICR->CODESIZE - 1) * NRF_FICR->CODEPAGESIZE)

APP_TIMER_DEF(button_counter_timer_id);
APP_TIMER_DEF(shutdown_timer_id);
APP_TIMER_DEF(led_timer_id);
APP_TIMER_DEF(reconnect_timer_id);
#define SHUTDOWN_TICKS APP_TIMER_TICKS(30*1000)
#define BUTTON_TIMER_TICKS APP_TIMER_TICKS(1000)
#define LED_TIMER_TICKS APP_TIMER_TICKS(100)
#define RECONNECT_TIMER_TICKS APP_TIMER_TICKS(5000)

#define BLINK_PATTERN_BITS 12
#define BLINK_PATTERN_UNLOCK                0b110000000000
#define BLINK_PATTERN_UNLOCK_LOW_BATTERY    0b101010000000
#define BLINK_PATTERN_PAIRING               0b111111111111

const ble_uuid128_t NUKI_VENDOR_UUID = {.uuid128={ NUKI_KEYTURNER_SERVICE_BASE_UUID }};
const ble_uuid128_t NUKI_KEYTURNER_SERVICE_UUID = {.uuid128={ NUKI_KEYTURNER_SERVICE_BASE_UUID }};
const ble_uuid128_t NUKI_PAIRING_SERVICE_UUID = {.uuid128={ NUKI_PAIRING_SERVICE_BASE_UUID }};

NRF_BLE_SCAN_DEF(m_scan);
NRF_BLE_GATT_DEF(m_gatt);
BLE_DB_DISCOVERY_DEF(m_ble_db_discovery);
NRF_BLE_GQ_DEF(m_ble_gatt_queue,
               NRF_SDH_BLE_CENTRAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);
NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) =
{
    .evt_handler = fstorage_evt_handler,
    .start_addr = 0x3e000,
    .end_addr   = 0x80000,
};

static uint16_t connection_handle = 0;
static uint16_t io_characteristic = 0;

static volatile bool button_timer_running = false;
static bool wakeup_from_gpio = false;
static volatile bool flash_ready_to_write = false;
static bool battery_is_low = false;
static int blink_bit = 0;
static int blink_pattern = BLINK_PATTERN_UNLOCK;

enum fob_action {
    ACTION_NONE = 0, ACTION_FOB_1 = 1, ACTION_FOB_2 = 2, ACTION_FOB_3 = 3, ACTION_PAIRING = 4
};
static uint16_t action_to_execute = ACTION_NONE;

static ble_gap_scan_params_t const m_scan_params =
{
    .active            = 0,
    .filter_policy     = BLE_GAP_SCAN_FP_ACCEPT_ALL,
    .interval          = BLE_SCAN_INTERVAL,
    .window            = BLE_SCAN_WINDOW,
    .timeout           = BLE_SCAN_TIMEOUT_SECONDS 
};

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
    nrf_gpio_cfg_sense_input(BUTTON_PIN, BUTTON_PULL, BUTTON_SENSE);
    sd_power_system_off();
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

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    error_info_t* error_info = (error_info_t*)info;
    UNUSED_VARIABLE(error_info);
    shutdown();
}

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

static bool advertises_pairing(const ble_data_t* advertising_data) 
{
    if(advertising_data->len <21) {return false;}
    for(int i = 0; i < 16; i++) 
    {
        if(advertising_data->p_data[5+i] != NUKI_PAIRING_SERVICE_UUID.uuid128[i]) 
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
                    ble_db_discovery_close(&m_ble_db_discovery);
                    io_characteristic = p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
                    start_pairing();
                    break;
                }
            } 
            else 
            {
                if (characteristic_uuid == USDIO_CHAR_UUID)
                {
                    ble_db_discovery_close(&m_ble_db_discovery);
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
    ble_db_discovery_init_t db_init;
    db_init.evt_handler  = db_disc_handler;
    db_init.p_gatt_queue = &m_ble_gatt_queue;

    ret_code_t err_code = ble_db_discovery_init(&db_init);
    shutdown_on_error(err_code);

    uint8_t uuid_type;
    ble_uuid_t uuid;

    if(action_to_execute == ACTION_PAIRING){
        sd_ble_uuid_vs_add(&NUKI_PAIRING_SERVICE_UUID, &uuid_type);
        uuid.uuid = 0xe100;
        uuid.type = uuid_type;
        err_code = ble_db_discovery_evt_register(&uuid);
    }
    else {
        sd_ble_uuid_vs_add(&NUKI_KEYTURNER_SERVICE_UUID, &uuid_type);
        uuid.uuid = 0xe200;
        uuid.type = uuid_type;
        err_code = ble_db_discovery_evt_register(&uuid);
    }
}

static void scan_init(void)
{
    ret_code_t err_code;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.connect_if_match = true;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;
    init_scan.p_scan_param     = &m_scan_params;
    
    err_code = nrf_ble_scan_init(&m_scan, &init_scan, NULL);
    shutdown_on_error(err_code);
}

static void scan_start()
{
    if(action_to_execute!=ACTION_PAIRING) 
    {
        app_timer_start(reconnect_timer_id, RECONNECT_TIMER_TICKS, NULL);
    }
    sd_ble_gap_disconnect(connection_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    nrf_ble_scan_stop();

    ret_code_t err_code = nrf_ble_scan_start(&m_scan);
    shutdown_on_error(err_code);
}

static void find_lock_to_connect(const ble_gap_evt_adv_report_t * p_adv_report) 
{
    bool lock_found = false;
    //search for a device that advertises the pairing service
    if(action_to_execute == ACTION_PAIRING && advertises_pairing(&p_adv_report->data))
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
        err_code = sd_ble_gap_connect(&p_adv_report->peer_addr, &m_scan_params, &m_connection_param, APP_BLE_CONN_CFG_TAG);
        shutdown_on_error(err_code);
    }
}

static void on_ble_evt(ble_evt_t const * p_ble_evt, void* p_context)
{
    uint32_t err_code = 0;
    const ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;
    ble_db_discovery_on_ble_evt(p_ble_evt, &m_ble_db_discovery);
    bt_comm_on_ble_evt(p_ble_evt);
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
        {
            const ble_gap_evt_adv_report_t * p_adv_report = &p_gap_evt->params.adv_report;
            find_lock_to_connect(p_adv_report);
        }
        break; 

        case BLE_GAP_EVT_CONNECTED:
            nrf_ble_scan_stop();
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
        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            shutdown_on_error(err_code);
        } break;

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

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt) 
{
    switch(p_evt->id){
        case NRF_FSTORAGE_EVT_ERASE_RESULT: {
            flash_ready_to_write = true;    
        }
            break;
        case NRF_FSTORAGE_EVT_WRITE_RESULT: {
            shutdown();
        }
            break;
        default:
            break;
    }
}

static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    shutdown_on_error(err_code);

    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    shutdown_on_error(err_code);

    err_code = nrf_sdh_ble_enable(&ram_start);
    shutdown_on_error(err_code);

    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, on_ble_evt, NULL);
}

static void button_timer_handler(void * p_context)
{
    button_timer_running = false;
}

static void start_button_timer() 
{
    button_timer_running = true;
    ret_code_t error = app_timer_start(button_counter_timer_id, BUTTON_TIMER_TICKS, NULL);
    shutdown_on_error(error);
}

static void led_timer() 
{
    nrf_gpio_pin_write(LED_PIN, (blink_pattern >> blink_bit) & 0x01);
    blink_bit++;
    if(blink_bit >= BLINK_PATTERN_BITS) blink_bit = 0;
}

static void initialize_timer(void)
{
	app_timer_init();

    app_timer_create(&button_counter_timer_id, APP_TIMER_MODE_SINGLE_SHOT, button_timer_handler);
    app_timer_create(&shutdown_timer_id, APP_TIMER_MODE_SINGLE_SHOT, shutdown);
    app_timer_create(&reconnect_timer_id, APP_TIMER_MODE_SINGLE_SHOT, scan_start);
    app_timer_create(&led_timer_id, APP_TIMER_MODE_REPEATED, led_timer);

    app_timer_start(shutdown_timer_id, SHUTDOWN_TICKS, NULL);
}

static void init_gpio() {
    nrf_gpio_cfg_sense_input(BUTTON_PIN, BUTTON_PULL, BUTTON_SENSE);
    nrf_gpio_cfg_output(LED_PIN);
    nrf_gpio_pin_clear(LED_PIN);
    wakeup_from_gpio = (nrf_gpio_pin_read(BUTTON_PIN) == BUTTON_PRESSED);
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
    //4 seconds because click 1 already consumed 1 second
    for(int i = 0; i < 40; i++) {
        if(nrf_gpio_pin_read(BUTTON_PIN) == BUTTON_RELEASED) {
            return false;
        }
        nrf_delay_us(DELAY_100_MS);
    }
    return true;
}

static uint16_t read_action_to_execute() {
    
    //fix to detect a click that occured before the initialization of the Softdevice
    bool click_before_init = wakeup_from_gpio && nrf_gpio_pin_read(BUTTON_PIN) == BUTTON_RELEASED;
    bool click_1 = click_registered() || click_before_init;
    nrf_delay_us(DEBOUNCE_TIME_MS);
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

void init_flash_and_load_settings_from_flash() {
    ret_code_t error = nrf_fstorage_init(
        &fstorage,
        &nrf_fstorage_sd,
        NULL
    );
    shutdown_on_error(error);

    pairing_context* pairing_ctx = get_pairing_context();
    error = nrf_fstorage_read (&fstorage, LAST_FLASH_PAGE, (void*)pairing_ctx, sizeof(pairing_context));
    if(action_to_execute != ACTION_PAIRING && pairing_ctx->magic_number != MAGIC_NUMBER_PAIRING_CONTEXT) {
        shutdown();
    }
    shutdown_on_error(error);
}

void unlock_finished() {
    shutdown();
}

void pairing_finished() {
    app_timer_stop(shutdown_timer_id);
    //prior erasing is necessary because erasing sets all flash to 1 and writing only writes 0s
    ret_code_t error = nrf_fstorage_erase(&fstorage, LAST_FLASH_PAGE, 1, NULL);
    shutdown_on_error(error);
    //shutdown will happen once writing is finished
}

void write_pairing_context_to_flash() {
    flash_ready_to_write = false;
    pairing_context* pairing_ctx = get_pairing_context();
    pairing_ctx->magic_number = MAGIC_NUMBER_PAIRING_CONTEXT;
    nrf_fstorage_write(&fstorage, LAST_FLASH_PAGE, (pairing_ctx), sizeof(pairing_context), NULL);
}

static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    shutdown_on_error(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

void saadc_event_handler(nrfx_saadc_evt_t const * p_event) {
    //not needed since we only need a one time measurement
}

static float voltage_from_adc_measurement(uint32_t adc_val)
{
    float voltage = adc_val / ((ADC_GAIN / ADC_REFERENCE_VOLTAGE) * pow(2, ADC_RESOLUTION_BITS));
    return voltage;
}

void saadc_init() {
  nrfx_saadc_config_t saadc_config = NRFX_SAADC_DEFAULT_CONFIG;
  ret_code_t err_code = nrfx_saadc_init(&saadc_config, saadc_event_handler);
  shutdown_on_error(err_code);
  nrf_saadc_channel_config_t channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
  err_code = nrfx_saadc_channel_init(0, &channel_config);
  shutdown_on_error(err_code);
}

void measure_battery_voltage() {
  nrf_saadc_value_t value;
  nrfx_saadc_sample_convert(0, &value);
  float voltage = voltage_from_adc_measurement(value);
  battery_is_low = voltage < BATTERY_LOW_VOLTAGE;
  nrfx_saadc_channel_uninit(NRF_SAADC_INPUT_AIN0);
}

int main(void)
{
    init_gpio();
    log_init();
	ble_stack_init();
    saadc_init();
    measure_battery_voltage();
    initialize_timer();
    action_to_execute = read_action_to_execute();
    if(action_to_execute == ACTION_NONE) {
        shutdown();
    } 

    if(battery_is_low) {
        blink_pattern = BLINK_PATTERN_UNLOCK_LOW_BATTERY;
    }

    if(action_to_execute == ACTION_PAIRING) {
        blink_pattern = BLINK_PATTERN_PAIRING;
    }
    app_timer_start(led_timer_id, LED_TIMER_TICKS, NULL);

    init_flash_and_load_settings_from_flash();
    gatt_init();
    scan_init();
    db_discovery_init();
    scan_start();

    while (true)
    {
        if (NRF_LOG_PROCESS() == false)
        {
            if(flash_ready_to_write) {
                write_pairing_context_to_flash();
            }
            process_messages(connection_handle, io_characteristic);

            uint32_t button_state = nrf_gpio_pin_read(BUTTON_PIN);
            if(button_state == BUTTON_PRESSED && action_to_execute != ACTION_PAIRING) {
                shutdown();
            }
        }
    }
}
