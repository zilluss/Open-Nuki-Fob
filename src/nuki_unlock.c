#include "nuki.h"
#include "bt_comm.h"
#include "nrf_log.h"

static uint8_t lock_action_to_execute = 0;
extern void unlock_finished();

static void command_finished(uint8_t* received_message, uint16_t message_length) {
    NRF_LOG_INFO("Unlock complete");
    unlock_finished();
}

static void send_command(uint8_t* received_message, uint16_t message_length) {
    NRF_LOG_INFO("Send command to lock");
    uint16_t message_out_length = create_lock_action_payload(lock_action_to_execute, send_buffer, received_message);
    send_with_response(send_buffer, message_out_length, 55u, command_finished);
}

void perform_lock_action(uint8_t lock_action) {
    NRF_LOG_INFO("Send challenge for command");
    lock_action_to_execute = lock_action;
    uint16_t message_out_length = create_challenge_payload(send_buffer);
    send_with_response(send_buffer, message_out_length, 86u, send_command);
}
