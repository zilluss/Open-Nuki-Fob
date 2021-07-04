#include "bt_comm.h"
#include "bt_comm_nrf.h"
#include "nrf_log.h"

static void (*m_response_callback)(uint8_t*, uint16_t) = NULL;
static uint16_t m_out_message_length = 0;
static uint16_t m_out_message_progress = 0;
static uint8_t m_out_message_buffer[200];

static uint16_t m_expected_response_length = 0;
static uint8_t m_response_message_progress = 0;
static uint8_t m_response_message_buffer[200];

static uint16_t m_mtu_size = DEFAULT_MTU_SIZE;

//bt_comm.h
void send_with_response(uint8_t* message_out, uint16_t message_out_length, 
    uint16_t expected_response_length, void (*callback)(uint8_t*, uint16_t)) 
{
    memcpy(m_out_message_buffer, message_out, message_out_length);
    m_response_callback = callback;
    m_out_message_progress = 0;
    m_response_message_progress = 0;
    m_expected_response_length = expected_response_length;
    m_out_message_length = message_out_length;
}

//bt_comm_nrf.h

//process the gattc long write queued in m_out_message
static void gattc_long_write(uint16_t connection_handle, uint16_t attribute_handle) 
{
    if(m_out_message_progress < m_out_message_length) {
        int32_t remaining_message_length = m_out_message_length - m_out_message_progress;
        uint16_t mtu_length = m_mtu_size;
        if(remaining_message_length < m_mtu_size) {
            mtu_length = remaining_message_length;
        }

        ble_gattc_write_params_t write_params = {
            .write_op = BLE_GATT_OP_PREP_WRITE_REQ,
            .flags    = 0,
            .handle   = attribute_handle,
            .offset   = m_out_message_progress,
            .len      = mtu_length,
            .p_value  = &m_out_message_buffer[m_out_message_progress]
        };


        uint32_t err_code = sd_ble_gattc_write(connection_handle, &write_params);
        if(err_code == NRF_SUCCESS) {
            m_out_message_progress += mtu_length;
        }
   } 
   else 
   {
        ble_gattc_write_params_t write_params = {
            .write_op = BLE_GATT_OP_EXEC_WRITE_REQ,
            .flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE,
            .handle   = attribute_handle,
            .len      = 0,
            .offset   = 0
        };
        m_out_message_length = 0;
        m_out_message_progress = 0;
        sd_ble_gattc_write(connection_handle, &write_params);
    }
}

void bt_comm_on_ble_evt(const ble_evt_t * p_ble_evt)
{
    uint32_t err_code = 0;
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTC_EVT_TIMEOUT:
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            break; 

        case BLE_GATTC_EVT_WRITE_RSP:
            if(m_out_message_length <= 0) {break;}
            const ble_gattc_evt_write_rsp_t* write_evt = &p_ble_evt->evt.gattc_evt.params.write_rsp;
            gattc_long_write(p_ble_evt->evt.gattc_evt.conn_handle, write_evt->handle);
            break;
        case BLE_GATTC_EVT_HVX:
            if(m_expected_response_length <= 0) {break;}
            const ble_gattc_evt_hvx_t* hvx_evt = &p_ble_evt->evt.gattc_evt.params.hvx;
            if(m_response_message_progress < m_expected_response_length) {
                memcpy((void*)(&m_response_message_buffer[m_response_message_progress]), hvx_evt->data, hvx_evt->len);
                m_response_message_progress += hvx_evt->len;
            }
            if(hvx_evt->type == BLE_GATT_HVX_INDICATION) {
                err_code = sd_ble_gattc_hv_confirm(p_ble_evt->evt.gattc_evt.conn_handle, hvx_evt->handle);
            }
            break;

        default:
            break;
    }
    UNUSED_VARIABLE(err_code);
}

void process_messages(uint16_t connection_handle, uint16_t attribute_handle) 
{
    if(m_out_message_length > 0 && m_out_message_progress == 0) 
    {
        gattc_long_write(connection_handle, attribute_handle);
    }

    if(m_expected_response_length > 0 && m_response_message_progress == m_expected_response_length) 
    {
        void (*response_callback)(uint8_t*, uint16_t) = m_response_callback;
        m_response_callback = NULL;
        m_response_message_progress = 0;
        m_expected_response_length = 0;
        response_callback(m_response_message_buffer, m_expected_response_length);
    }

}

void set_bt_comm_mtu_size(uint16_t mtu_size) {
    m_mtu_size = mtu_size;
}