#ifndef NUKI_H
#define NUKI_H

#include <stdint.h>

uint8_t public_key_nuki[32];
uint8_t public_key_fob[32];
uint8_t private_key_fob[32];

typedef struct 
{
    uint8_t shared_secret[32];
    uint32_t authorization_id;
    uint32_t app_id;
    uint8_t paired_lock_uuid[6];
} __attribute__((packed, aligned(4))) pairing_context;

#define USDIO_CHAR_UUID 0xe202
#define NUKI_KEYTURNER_SERVICE_BASE_UUID 0x66,0x9a,0x0c,0x20,0x00,0x08,0x6c,0x91,0xe4,0x11,0x01,0x55,0x00,0xe2,0x2e,0xa9

#define GDIO_CHAR_UUID 0xe101
#define NUKI_PAIRING_SERVICE_BASE_UUID 0x66,0x9a,0x0c,0x20,0x00,0x08,0x6c,0x91,0xe4,0x11,0x01,0x55,0x00,0xe1,0x2e,0xa9

#define PAIRING_NONCEBYTES 32


uint16_t create_challenge_payload(uint8_t* output_buffer);
uint16_t create_lock_action_payload(uint8_t lock_action, uint8_t* output_buffer, uint8_t* encrypted_challenge);

uint16_t create_request_public_key_payload(uint8_t* output_buffer);
uint16_t create_write_public_key_payload(uint8_t* output_buffer, uint8_t* received_data); 
uint16_t create_authorization_authenticator_payload(uint8_t* output_buffer, uint8_t* received_data);
uint16_t create_authorization_data_payload(uint8_t* output_buffer, uint8_t* received_data);
uint16_t create_authorization_id_confirmation_payload(uint8_t* output_buffer, uint8_t* received_data);

pairing_context* get_pairing_context();
void start_pairing();
void perform_lock_action(uint8_t lock_action);

#endif