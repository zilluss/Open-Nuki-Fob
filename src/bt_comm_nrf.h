#include "nordic_common.h"
#include "nrf_ble_gatt.h"
#include "ble_hci.h"
#include "app_error.h"
#include "nrf_ble_gq.h"

#define MTU_SIZE  18

void bt_comm_on_ble_evt(const ble_evt_t* p_ble_evt);
void process_messages(uint16_t connection_handle, uint16_t attribute_handle);
//NRF specific bits of the bt communication
void init_bt_comm_nrf(nrf_ble_gq_t* gatt_queue);