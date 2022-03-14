#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "nordic_common.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_scan.h"
#include "nrf_delay.h"
#include "nrf_drv_rng.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_power.h"
#include "nrf_sdm.h"
#include "nrf_sdh.h"
#include "nrfx_saadc.h"


#include "ble_srv_common.h"
#include "app_timer.h"
#include "app_error.h"
#include "app_button.h"

#include "bt_comm_nrf.h"
#include "bt_comm.h"
#include "fob_data.h"
#include "nuki.h"
#include "utils.h"

#define UUID_LEN 6
#define APP_BLE_CONN_CFG_TAG 1                                 
#define APP_BLE_OBSERVER_PRIO 3

#define DEAD_BEEF 0xDEADBEEF

#define BLE_SCAN_INTERVAL 15
#define BLE_SCAN_WINDOW 15
#define BLE_SCAN_TIMEOUT_UNLOCK_SECONDS MSEC_TO_UNITS(1000, UNIT_10_MS) 
#define BLE_SCAN_TIMEOUT_PAIRING_SECONDS MSEC_TO_UNITS(9000, UNIT_10_MS) 


#define MIN_CONNECTION_INTERVAL MSEC_TO_UNITS(20, UNIT_1_25_MS)
#define MAX_CONNECTION_INTERVAL MSEC_TO_UNITS(200, UNIT_1_25_MS)
#define SLAVE_LATENCY 0
#define SUPERVISION_TIMEOUT MSEC_TO_UNITS(4000, UNIT_10_MS) 

#define BUTTON_PIN 11
#define LED_PIN 30

#define LED_ON 1
#define LED_OFF !LED_ON

#define BUTTON_PULL NRF_GPIO_PIN_PULLUP
#define BUTTON_SENSE NRF_GPIO_PIN_SENSE_LOW
#define BUTTON_DETECTION_DELAY APP_TIMER_TICKS(50)

APP_TIMER_DEF(input_timer_id);
APP_TIMER_DEF(shutdown_timer_id);
APP_TIMER_DEF(led_timer_id);

#define INPUT_TICKS APP_TIMER_TICKS(500)
#define SHUTDOWN_TICKS APP_TIMER_TICKS(30*1000)
#define LED_TIMER_TICKS APP_TIMER_TICKS(100)

#define BLINK_PATTERN_BITS 12
#define BLINK_PATTERN_IDLE                  0b000000000000
#define BLINK_PATTERN_UNLOCK                0b110000000000
#define BLINK_PATTERN_PAIRING               0b111111111111

static uint8_t scratch_buffer[4096];
const ble_uuid128_t NUKI_VENDOR_UUID = { .uuid128 = { NUKI_KEYTURNER_SERVICE_BASE_UUID } };
const ble_uuid128_t NUKI_KEYTURNER_SERVICE_UUID = { .uuid128 = { NUKI_KEYTURNER_SERVICE_BASE_UUID } };
const ble_uuid128_t NUKI_PAIRING_SERVICE_UUID = { .uuid128 = { NUKI_PAIRING_SERVICE_BASE_UUID } };

#define RESET_TO_DFU_BOOTLOADER 0xB1
#define FIRST_STARTUP_CHECK 0x3A

NRF_BLE_SCAN_DEF(m_scan);
NRF_BLE_GATT_DEF(m_gatt);
NRF_BLE_GQ_DEF(m_ble_gatt_queue,
    NRF_SDH_BLE_CENTRAL_LINK_COUNT,
    NRF_BLE_GQ_QUEUE_SIZE);

static uint16_t connection_handle = BLE_CONN_HANDLE_INVALID;
static uint16_t io_characteristic_handle = 0;
static uint16_t cccd_handle = 0;

static volatile uint8_t times_button_released = 0;
static volatile float seconds_since_startup = 0;

static int blink_bit = 0;
static int blink_pattern = BLINK_PATTERN_IDLE;

enum fob_action {
    ACTION_NONE = 0, ACTION_FOB_1 = 1, ACTION_FOB_2 = 2, ACTION_FOB_3 = 3, ACTION_PAIRING = 4
};
static uint16_t action_to_execute = ACTION_NONE;
static fob_data_writing_context fob_data_writing_ctx;
typedef union {
    nuki_lock_action_context lock_action;
    nuki_pairing_context pairing;
} nuki_context;
static nuki_context nuki_ctx;


static ble_gap_scan_params_t m_scan_params =
{
    .active = 0,
    .filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL,
    .interval = BLE_SCAN_INTERVAL,
    .window = BLE_SCAN_WINDOW,
    .timeout = BLE_SCAN_TIMEOUT_UNLOCK_SECONDS
};

enum application_states {
    AS_WAIT_FOR_INPUT = 0, AS_INPUT_FINISHED, AS_CONNECT_TO_LOCK, AS_ENABLE_INDICATIONS, AS_WAIT_FOR_INDICATIONS_ENABLED,
    AS_LOCK_COMMUNICATION, AS_WRITE_FOB_DATA
};
static uint16_t application_state = AS_WAIT_FOR_INPUT;

static ble_gap_conn_params_t const m_connection_param =
{
    MIN_CONNECTION_INTERVAL,
    MAX_CONNECTION_INTERVAL,
    SLAVE_LATENCY,
    SUPERVISION_TIMEOUT
};

typedef struct {
    int8_t rssi;
    ble_gap_addr_t address;
} advertised_lock;

typedef struct {
    int32_t n_locks;
    advertised_lock found_locks[100];
} lock_scan;

static void reset_into_bootloader() {
    NRF_LOG_INFO("Resetting into bootloader");
    sd_power_gpregret_set(0, RESET_TO_DFU_BOOTLOADER);
    sd_nvic_SystemReset();
}

static void shutdown() {
    NRF_LOG_INFO("Enter sleep mode");
    nrf_gpio_pin_write(LED_PIN, LED_OFF);
    nrf_gpio_cfg_sense_input(BUTTON_PIN, BUTTON_PULL, BUTTON_SENSE);
    sd_power_system_off();
}

static void shutdown_on_error(ret_code_t error_code) {
    if(error_code != NRF_SUCCESS) {
        NRF_LOG_ERROR("Error code: %i. Shutting down", error_code);
        shutdown();
    }
}

void assert_nrf_callback(uint16_t line_num, const uint8_t* p_file_name) {
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info) {
    NRF_LOG_ERROR("App fault id: %i, info %i. Shutting down", id, info);
    error_info_t* error_info = (error_info_t*)info;
    UNUSED_VARIABLE(error_info);
    shutdown();
}

static void unlock_finished() {
    shutdown();
}

static void pairing_finished() {
    app_timer_stop(shutdown_timer_id);
    fob_data_writing_ctx.data.pairing = nuki_ctx.pairing.key;
    fob_data_writing_ctx.page_buffer = scratch_buffer;

    fob_data_writing_ctx.write_state = WS_START;
    application_state = AS_WRITE_FOB_DATA;
}

static void gatt_init(void) {
    nrf_ble_gatt_init(&m_gatt, NULL);
}

static bool is_paired_uuid(lock_pairing const* lock_pairing, const uint8_t* uuid2) {
    for(int i = 0; i < NUKI_UUID_LEN; i++) {
        if(lock_pairing->pairing.lock_uuid[i] != uuid2[i]) return false;
    }
    return true;
}

static bool advertises_pairing(const ble_data_t* advertising_data) {
    if(advertising_data->len < 21) { return false; }
    for(int i = 0; i < 16; i++) {
        if(advertising_data->p_data[5 + i] != NUKI_PAIRING_SERVICE_UUID.uuid128[i]) {
            return false;
        }
    }
    return true;
}

static bool advertises_keyturner(const ble_data_t* advertising_data) {
    if(advertising_data->len < 25) { return false; }
    for(int i = 0; i < 16; i++) {
        if(advertising_data->p_data[24 - i] != NUKI_KEYTURNER_SERVICE_UUID.uuid128[i]) {
            return false;
        }
    }
    return true;
}

static uint32_t enable_indications() {
    uint8_t buf[BLE_CCCD_VALUE_LEN];

    buf[0] = BLE_GATT_HVX_INDICATION;
    buf[1] = 0;

    const ble_gattc_write_params_t write_params = {
        .write_op = BLE_GATT_OP_WRITE_REQ,
        .flags = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE,
        .handle = cccd_handle,
        .offset = 0,
        .len = sizeof(buf),
        .p_value = buf
    };

    application_state = AS_WAIT_FOR_INDICATIONS_ENABLED;
    ret_code_t err_code = sd_ble_gattc_write(connection_handle, &write_params);
    return err_code;
}

static void start_service_discovery() {
    uint8_t uuid_type;
    ble_uuid_t uuid;

    if(action_to_execute == ACTION_PAIRING) {
        sd_ble_uuid_vs_add(&NUKI_PAIRING_SERVICE_UUID, &uuid_type);
        uuid.uuid = NUKI_PAIRING_SERVICE_VENDOR_UUID;
        uuid.type = uuid_type;
        sd_ble_gattc_primary_services_discover(connection_handle, 0x0001, &uuid);
    }
    else {
        sd_ble_uuid_vs_add(&NUKI_KEYTURNER_SERVICE_UUID, &uuid_type);
        uuid.uuid = NUKI_KEYTURNER_SERVICE_VENDOR_UUID;
        uuid.type = uuid_type;
        sd_ble_gattc_primary_services_discover(connection_handle, 0x0002, &uuid);
    }
}

static void scan_start() {
    ret_code_t err_code;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));
    lock_scan* lock_buffer = (lock_scan*)&scratch_buffer[0];
    lock_buffer->n_locks = 0;

    init_scan.connect_if_match = true;
    init_scan.conn_cfg_tag = APP_BLE_CONN_CFG_TAG;
    m_scan_params.timeout = action_to_execute == ACTION_PAIRING ? BLE_SCAN_TIMEOUT_PAIRING_SECONDS : BLE_SCAN_TIMEOUT_UNLOCK_SECONDS;
    init_scan.p_scan_param = &m_scan_params;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, NULL);
    shutdown_on_error(err_code);

    sd_ble_gap_disconnect(connection_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    nrf_ble_scan_stop();

    err_code = nrf_ble_scan_start(&m_scan);
    shutdown_on_error(err_code);
}

static void find_lock_to_connect(const ble_gap_evt_adv_report_t* p_adv_report) {
    if(action_to_execute == ACTION_NONE) { shutdown(); }
    //search for a device that advertises the pairing service
    if(action_to_execute == ACTION_PAIRING && advertises_pairing(&p_adv_report->data)) {
        memcpy(&nuki_ctx.pairing.key.lock_uuid, p_adv_report->peer_addr.addr, BLE_GAP_ADDR_LEN);

        ret_code_t err_code = sd_ble_gap_scan_stop();
        err_code = sd_ble_gap_connect(&p_adv_report->peer_addr, &m_scan_params, &m_connection_param, APP_BLE_CONN_CFG_TAG);
        shutdown_on_error(err_code);
    }
    //find the already paired lock
    else if(advertises_keyturner(&p_adv_report->data)) {
        lock_scan* lock_buffer = (lock_scan*)&scratch_buffer[0];
        lock_buffer->found_locks[lock_buffer->n_locks].rssi = p_adv_report->rssi;
        memcpy(&lock_buffer->found_locks[lock_buffer->n_locks].address.addr, p_adv_report->peer_addr.addr, BLE_GAP_ADDR_LEN);
        lock_buffer->n_locks++;
    }
}

static void connect_to_nearest_lock() {
    lock_scan* lock_buffer = (lock_scan*)&scratch_buffer[0];
    fob_data const* fob_data = get_fob_data();

    advertised_lock* nearest_advertised_lock = NULL;
    lock_pairing const* nearest_paired_lock = NULL;

    int8_t lowest_rssi = INT8_MAX;
    for(int found_lock_index = 0; found_lock_index < lock_buffer->n_locks; found_lock_index++) {
        for(int paired_lock_index = 0; paired_lock_index < MAX_PAIRINGS; paired_lock_index++) {
            advertised_lock* advertised_lock = &lock_buffer->found_locks[found_lock_index];
            lock_pairing const* paired_lock = &fob_data->paired_locks[paired_lock_index];
            if(is_paired_uuid(paired_lock, advertised_lock->address.addr) && advertised_lock->rssi < lowest_rssi) {
                lowest_rssi = advertised_lock->rssi;
                nearest_advertised_lock = advertised_lock;
                nearest_paired_lock = &fob_data->paired_locks[paired_lock_index];
            }
        }
    }

    if(nearest_paired_lock == NULL) {
        shutdown();
        return;
    }

    nuki_ctx.lock_action.lock_action_done_callback = unlock_finished;
    nuki_ctx.lock_action.lock_action = action_to_execute + 0x80; //convert enum value to actual nuki command value
    nuki_ctx.lock_action.key = nearest_paired_lock->pairing;

    sd_ble_gap_connect(
        &nearest_advertised_lock->address,
        &m_scan_params,
        &m_connection_param,
        APP_BLE_CONN_CFG_TAG);
}

static void on_ble_evt(ble_evt_t const* p_ble_evt, void* p_context) {
    const ble_gap_evt_t* p_gap_evt = &p_ble_evt->evt.gap_evt;
    if(application_state == AS_LOCK_COMMUNICATION) {
        bt_comm_on_ble_evt(p_ble_evt);
    }

    switch(p_ble_evt->header.evt_id) {
        case BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP: {
            const ble_gattc_evt_prim_srvc_disc_rsp_t* p_prim_srvc_disc_rsp = &p_ble_evt->evt.gattc_evt.params.prim_srvc_disc_rsp;
            const uint16_t looked_for_service_uuid = action_to_execute == ACTION_PAIRING ? NUKI_PAIRING_SERVICE_VENDOR_UUID : NUKI_KEYTURNER_SERVICE_VENDOR_UUID;
            for(int i = 0; i < p_prim_srvc_disc_rsp->count; i++) {
                if(p_prim_srvc_disc_rsp->services[i].uuid.uuid == looked_for_service_uuid) {
                    sd_ble_gattc_characteristics_discover(connection_handle, &p_prim_srvc_disc_rsp->services[i].handle_range);
                    break;
                }
            }
            break;
        }

        case BLE_GATTC_EVT_CHAR_DISC_RSP: {
            const ble_gattc_evt_char_disc_rsp_t* p_char_disc_rsp = &p_ble_evt->evt.gattc_evt.params.char_disc_rsp;
            const uint16_t looked_for_characteristic_uuid = action_to_execute == ACTION_PAIRING ? GDIO_CHAR_UUID : USDIO_CHAR_UUID;
            uint16_t last_handle = 0;
            for(int i = 0; i < p_char_disc_rsp->count; i++) {
                if(p_char_disc_rsp->chars[i].uuid.uuid == looked_for_characteristic_uuid) {
                    ble_gattc_handle_range_t handle_range = {
                        .start_handle = p_char_disc_rsp->chars[i].handle_value,
                        .end_handle = p_char_disc_rsp->chars[i].handle_value,
                    };
                    io_characteristic_handle = p_char_disc_rsp->chars[i].handle_value;
                    sd_ble_gattc_descriptors_discover(connection_handle, &handle_range);
                    break;
                }
                last_handle = p_char_disc_rsp->chars[i].handle_value;
            }
            if(p_ble_evt->evt.gattc_evt.gatt_status != BLE_GATT_STATUS_ATTERR_ATTRIBUTE_NOT_FOUND) {
                ble_gattc_handle_range_t handle_range = {
                        .start_handle = last_handle + 1,
                        .end_handle = last_handle + 2
                };
                sd_ble_gattc_characteristics_discover(connection_handle, &handle_range);
            }
            break;
        }

        case BLE_GATTC_EVT_DESC_DISC_RSP: {
            const ble_gattc_evt_desc_disc_rsp_t* p_desc_disc_rsp = &p_ble_evt->evt.gattc_evt.params.desc_disc_rsp;
            uint16_t last_handle = 0;
            for(int i = 0; i < p_desc_disc_rsp->count; i++) {
                if(p_desc_disc_rsp->descs[i].uuid.uuid == BLE_UUID_DESCRIPTOR_CLIENT_CHAR_CONFIG) {
                    cccd_handle = p_desc_disc_rsp->descs[i].handle;
                    enable_indications();
                    break;
                }
                last_handle = p_desc_disc_rsp->descs[i].handle;
            }

            if(p_ble_evt->evt.gattc_evt.gatt_status != BLE_GATT_STATUS_ATTERR_ATTRIBUTE_NOT_FOUND) {
                ble_gattc_handle_range_t handle_range = {
                        .start_handle = last_handle + 1,
                        .end_handle = last_handle + 2
                };
                sd_ble_gattc_descriptors_discover(connection_handle, &handle_range);
            }
            break;
        }
        case BLE_GAP_EVT_ADV_REPORT:
        {
            if(connection_handle != BLE_CONN_HANDLE_INVALID) { break; }
            const ble_gap_evt_adv_report_t* p_adv_report = &p_gap_evt->params.adv_report;
            find_lock_to_connect(p_adv_report);
        }
        break;

        case BLE_GAP_EVT_CONNECTED:
            nrf_ble_scan_stop();
            connection_handle = p_ble_evt->evt.gap_evt.conn_handle;
            //Pairing mode doesn't support mtu exchange
            if(action_to_execute == ACTION_PAIRING) {
                start_service_discovery();
            }
            else {
                sd_ble_gattc_exchange_mtu_request(connection_handle, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
            }
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if(p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN) {
                if(action_to_execute == ACTION_PAIRING) {
                    shutdown();
                }
                else {
                    connect_to_nearest_lock();
                }
            }
            else if(p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN) {
                shutdown();
            }
            break;

        case BLE_GATTC_EVT_WRITE_RSP:
            if(application_state == AS_WAIT_FOR_INDICATIONS_ENABLED) {
                application_state = AS_LOCK_COMMUNICATION;
                if(action_to_execute == ACTION_PAIRING) {
                    fob_settings fob_settings;
                    get_fob_settings(&fob_settings);
                    memcpy(nuki_ctx.pairing.fob_name, fob_settings.fob_name, sizeof(fob_settings.fob_name));
                    nuki_ctx.pairing.app_id = fob_settings.app_id;
                    nuki_ctx.pairing.pairing_done_callback = pairing_finished;
                    start_pairing(&nuki_ctx.pairing);
                }
                else {
                    perform_lock_action(&nuki_ctx.lock_action);
                }
            }
            break;

        case BLE_GATTC_EVT_EXCHANGE_MTU_RSP:
            set_bt_comm_mtu_size(p_ble_evt->evt.gattc_evt.params.exchange_mtu_rsp.server_rx_mtu);
            start_service_discovery();
            break;

        default:
            break;
    }
}

static void ble_stack_init(void) {
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    shutdown_on_error(err_code);

    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);

    shutdown_on_error(err_code);

    err_code = nrf_sdh_ble_enable(&ram_start);
    shutdown_on_error(err_code);

    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, on_ble_evt, NULL);

    uint32_t gpregret;
    sd_power_gpregret_get(1, &gpregret);
    sd_power_gpregret_set(1, FIRST_STARTUP_CHECK);

    if(gpregret != FIRST_STARTUP_CHECK) {
        //Shutdown if the fob was started from a power cycle or DFU reboot
        shutdown();
    }
}

static void finish_input_handling(uint16_t action) {
    action_to_execute = action;
    app_timer_stop(input_timer_id);

    if(action_to_execute != ACTION_PAIRING && get_fob_data() == NULL) {
        //no valid data, needs to be paired first
        shutdown();
    }

    application_state = AS_INPUT_FINISHED;
    NRF_LOG_INFO("Action to execute: %i", action);
}

static void input_timer_handler(void* p_context) {
    if(application_state != AS_WAIT_FOR_INPUT) { return; }
    seconds_since_startup += 0.5;

    if(seconds_since_startup >= 5) {
        blink_pattern = BLINK_PATTERN_PAIRING;
        if(times_button_released == 1) {
            finish_input_handling(ACTION_PAIRING);
            return;
        }
    }

    if(seconds_since_startup >= 10) {
        reset_into_bootloader();
        return;
    }

    if(seconds_since_startup >= 2.0) {
        switch(times_button_released) {
            case 0:
                return;
            case 1:
                finish_input_handling(ACTION_FOB_1);
                return;
            case 2:
                finish_input_handling(ACTION_FOB_2);
                return;
            case 3:
                finish_input_handling(ACTION_FOB_3);
                return;
            default:
                NRF_LOG_INFO("Invalid input, shutting down");
                shutdown();
        }
    }
}

static void led_timer() {
    nrf_gpio_pin_write(LED_PIN, ((blink_pattern >> blink_bit) & 0x01) ^ LED_OFF);
    blink_bit++;
    if(blink_bit >= BLINK_PATTERN_BITS) blink_bit = 0;
}

static void initialize_timer(void) {
    app_timer_init();

    app_timer_create(&input_timer_id, APP_TIMER_MODE_REPEATED, input_timer_handler);
    app_timer_start(input_timer_id, INPUT_TICKS, NULL);

    app_timer_create(&shutdown_timer_id, APP_TIMER_MODE_SINGLE_SHOT, shutdown);
    app_timer_start(shutdown_timer_id, SHUTDOWN_TICKS, NULL);

    app_timer_create(&led_timer_id, APP_TIMER_MODE_REPEATED, led_timer);
    app_timer_start(led_timer_id, LED_TIMER_TICKS, NULL);
}

static void button_handler_callback(uint8_t pin, uint8_t action) {
    if(application_state != AS_WAIT_FOR_INPUT && times_button_released > 0) {
        sd_nvic_SystemReset();
    }

    if(action == APP_BUTTON_RELEASE) {
        times_button_released++;
    }
}

static void init_gpio() {
    NRF_UICR->NFCPINS = 0xFFFFFFFE; //enable NFC pins as gpio for logging
    static app_button_cfg_t buttons[] =
    {
        {BUTTON_PIN, APP_BUTTON_ACTIVE_LOW, BUTTON_PULL, button_handler_callback}
    };

    ret_code_t err_code = app_button_init(buttons, ARRAY_SIZE(buttons), BUTTON_DETECTION_DELAY);
    nrf_gpio_cfg_sense_input(BUTTON_PIN, BUTTON_PULL, BUTTON_SENSE);
    shutdown_on_error(err_code);

    nrf_gpio_cfg_output(LED_PIN);
    nrf_gpio_pin_write(LED_PIN, LED_OFF);

    //handle the first input that woke the fob
    bool button_pressed = nrf_gpio_pin_read(BUTTON_PIN) == APP_BUTTON_ACTIVE_LOW;
    button_handler_callback(BUTTON_PIN, button_pressed ? APP_BUTTON_PUSH : APP_BUTTON_RELEASE);
}

static void log_init(void) {
    ret_code_t err_code = NRF_LOG_INIT(app_timer_cnt_get);
    shutdown_on_error(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

static void signal_action() {
    switch(action_to_execute) {
        case ACTION_PAIRING:
            blink_pattern = BLINK_PATTERN_PAIRING;
            break;
        case ACTION_FOB_1:
        case ACTION_FOB_2:
        case ACTION_FOB_3:
            blink_pattern = BLINK_PATTERN_UNLOCK;
            break;
    }
}

static void handle_application(void) {
    switch(application_state) {
        case AS_INPUT_FINISHED:
            signal_action();
            scan_start();
            application_state = AS_WAIT_FOR_INDICATIONS_ENABLED;
            break;
        case AS_LOCK_COMMUNICATION:
            process_messages(&nuki_ctx, connection_handle, io_characteristic_handle);
            break;
        case AS_WRITE_FOB_DATA:
            write_fob_data(&fob_data_writing_ctx);
            if(fob_data_writing_ctx.write_state == WS_WRITING_DONE) {
                shutdown();
            }
            break;
    }
}

int main(void) {
    log_init();
    ble_stack_init();
    initialize_timer();
    init_gpio();
    gatt_init();

    while(true) {
        if(NRF_LOG_PROCESS() == false) {
            handle_application();
        }
    }
}
