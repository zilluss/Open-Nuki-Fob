#include "nordic_common.h"
#include "nrf_ble_gatt.h"
#include "ble_hci.h"
#include "app_error.h"
#include "nrf_ble_gq.h"

#define DEFAULT_MTU_SIZE  20

void bt_comm_on_ble_evt(const ble_evt_t* p_ble_evt);
void process_messages(void* param, uint16_t connection_handle, uint16_t attribute_handle);
//NRF specific bits of the bt communication
void init_bt_comm_nrf(nrf_ble_gq_t* gatt_queue);
void set_bt_comm_mtu_size(uint16_t mtu_size);