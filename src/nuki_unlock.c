#include "nuki.h"
#include "bt_comm.h"
#include "nrf_log.h"

static void command_finished(nuki_lock_action_context* lock_action_ctx, uint8_t* received_message, uint16_t message_length) {
    NRF_LOG_INFO("Lock action complete");
    lock_action_ctx->lock_action_done_callback();
}

static void send_command(nuki_lock_action_context* lock_action_ctx, uint8_t* received_message, uint16_t message_length) {
    NRF_LOG_INFO("Send command to lock");
    uint16_t message_out_length = create_lock_action_payload(lock_action_ctx, send_buffer, received_message);
    send_with_response(send_buffer, message_out_length, 55u, (bt_callback)command_finished);
}

void perform_lock_action(nuki_lock_action_context* lock_action_ctx) {
    NRF_LOG_INFO("Send challenge for command");
    uint16_t message_out_length = create_challenge_payload(&lock_action_ctx->key, send_buffer);
    send_with_response(send_buffer, message_out_length, 86u, (bt_callback)send_command);
}
