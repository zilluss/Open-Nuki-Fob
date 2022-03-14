#ifndef NUKI_H
#define NUKI_H

#include <stdint.h>

#define USDIO_CHAR_UUID 0xe202
#define NUKI_KEYTURNER_SERVICE_BASE_UUID 0x66,0x9a,0x0c,0x20,0x00,0x08,0x6c,0x91,0xe4,0x11,0x01,0x55,0x00,0xe2,0x2e,0xa9
#define NUKI_KEYTURNER_SERVICE_VENDOR_UUID 0xe200

#define GDIO_CHAR_UUID 0xe101
#define NUKI_PAIRING_SERVICE_BASE_UUID 0x66,0x9a,0x0c,0x20,0x00,0x08,0x6c,0x91,0xe4,0x11,0x01,0x55,0x00,0xe1,0x2e,0xa9
#define NUKI_PAIRING_SERVICE_VENDOR_UUID 0xe100

#define PAIRING_NONCEBYTES 32
#define NUKI_UUID_LEN 6

typedef void (*nuki_cmd_done_callback)();

typedef struct {
    uint8_t shared_secret[32];
    uint32_t authorization_id;
    uint8_t lock_uuid[NUKI_UUID_LEN];
} __attribute__((packed, aligned(4))) nuki_key;

typedef struct {
    char fob_name[32];
    uint8_t public_key_nuki[32];
    uint8_t public_key_fob[32];
    uint8_t private_key_fob[32];
    uint32_t app_id;
    nuki_key key;
    nuki_cmd_done_callback pairing_done_callback;
} __attribute__((packed, aligned(4))) nuki_pairing_context;

typedef struct {
    nuki_key key;
    uint32_t app_id;
    uint8_t lock_action;
    nuki_cmd_done_callback lock_action_done_callback;
} __attribute__((packed, aligned(4))) nuki_lock_action_context;

uint16_t create_challenge_payload(nuki_key* key, uint8_t* output_buffer);
uint16_t create_lock_action_payload(nuki_lock_action_context* lock_action_ctx, uint8_t* output_buffer, uint8_t* encrypted_challenge);

uint16_t create_request_public_key_payload(nuki_pairing_context* pairing_ctx, uint8_t* output_buffer);
uint16_t create_write_public_key_payload(nuki_pairing_context* pairing_ctx, uint8_t* output_buffer, uint8_t* received_data);
uint16_t create_authorization_authenticator_payload(nuki_pairing_context* pairing_ctx, uint8_t* output_buffer, uint8_t* received_data);
uint16_t create_authorization_data_payload(nuki_pairing_context* pairing_ctx, uint8_t* output_buffer, uint8_t* received_data);
uint16_t create_authorization_id_confirmation_payload(nuki_pairing_context* pairing_ctx, uint8_t* output_buffer, uint8_t* received_data);

void start_pairing(nuki_pairing_context* pairing_ctx);
void perform_lock_action(nuki_lock_action_context* lock_action_ctx);

#endif