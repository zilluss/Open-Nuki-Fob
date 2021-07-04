#include "nuki.h"
#include "bt_comm.h"
#include "nrf_log.h"

extern void pairing_finished();

static void complete_confirmation_received(uint8_t* response, uint16_t length) 
{
    NRF_LOG_INFO("Pairing complete");
    pairing_finished();
}

static void authorization_id_received(uint8_t* response, uint16_t length) {
    NRF_LOG_INFO("Received authorization ID");
    uint16_t payload_length = create_authorization_id_confirmation_payload(send_buffer, response);
    send_with_response(send_buffer, payload_length, 5, complete_confirmation_received);
}

static void challenge_for_authorization_data_received(uint8_t* response, uint16_t length) {
    NRF_LOG_INFO("Received challenge for authorization data");
    uint16_t payload_length = create_authorization_data_payload(send_buffer, response);
    send_with_response(send_buffer, payload_length, 88, authorization_id_received);
}

static void challenge_for_authorization_authenticator_received(uint8_t* response, uint16_t length) {
    NRF_LOG_INFO("Received challenge for authorization authenticator");
    uint16_t payload_length = create_authorization_authenticator_payload(send_buffer, response);
    send_with_response(send_buffer, payload_length, 36, challenge_for_authorization_data_received);
}

static void public_key_received(uint8_t* response, uint16_t length) {
    NRF_LOG_INFO("Received public key from smartlock");
    uint16_t payload_length = create_write_public_key_payload(send_buffer, response);
    send_with_response(send_buffer, payload_length, 36, challenge_for_authorization_authenticator_received);
}

void start_pairing() {
    NRF_LOG_INFO("Start pairing");
    uint16_t payload_length = create_request_public_key_payload(send_buffer);
    send_with_response(send_buffer, payload_length , 36, public_key_received);
}
