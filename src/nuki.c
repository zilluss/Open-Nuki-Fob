#include "nuki.h"
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "tweetnacl.h"
#include "hmac_sha256.h"
#include "nrf_log.h"
#include "utils.h"

#define ADATA_LENGTH  30
#define MAX_PDATA_LENGTH  120
#define CHALLENGE_MESSAGE_LENGTH  86
#define CHALLENGE_CMD_PDATA_LENGTH  10

#define ENCRYPTED_CHALLENGE_PDATA_LENGTH  (CHALLENGE_MESSAGE_LENGTH - ADATA_LENGTH)
#define DECRYPTED_CHALLENGE_PDATA_LENGTH  (ENCRYPTED_CHALLENGE_PDATA_LENGTH - crypto_secretbox_MACBYTES)
#define LOCK_ACTION_PDATA_LENGTH 46
#define CLIENT_NONCE_LENGTH crypto_box_NONCEBYTES
#define SMARTLOCK_NONCE_LENGTH 32
#define crypto_secretbox_MACBYTES (crypto_secretbox_ZEROBYTES - crypto_secretbox_BOXZEROBYTES)

static const uint16_t request_data_cmd = 0x0001;
static const uint16_t public_key_cmd = 0x0003;
static const uint16_t challenge_cmd = 0x0004;
static const uint16_t authorization_authenticator_cmd = 0x0005;
static const uint16_t authorization_data_cmd = 0x0006;
static const uint16_t lock_action_cmd = 0x000D;
static const uint16_t authorization_id_confirmation_cmd = 0x001E;

static const uint8_t APP_TYPE_FOB = 0x02;

enum lock_action {
    unlock = 0x01,
    lock = 0x02,
    unlatch = 0x03,
    lock_n_go = 0x04,
    lock_n_go_with_unlatch = 0x05,
    fob_action_1 = 0x81,
    fob_action_2 = 0x82,
    fob_action_3 = 0x83
};

static pairing_context pairing_ctx;

static void crc_payload(uint8_t* output_buffer, uint16_t length) 
{
    uint16_t crc = crc_16(output_buffer, length-2, 0xFFFF);
    write_uint16LE(output_buffer, crc, length-2);
}

static uint16_t encrypt_payload(uint8_t* output_buffer, uint8_t* pdata_unencrypted, uint16_t pdata_length) {
    if(pdata_length > MAX_PDATA_LENGTH) {return 0;}
    uint8_t adata[ADATA_LENGTH];
    randombytes(adata, 24); //generate nonce
    write_uint32LE(adata, pairing_ctx.authorization_id, 24);
    write_uint16LE(adata, pdata_length + crypto_secretbox_MACBYTES, 28);
    
    uint16_t pdataCRC = crc_16(pdata_unencrypted, pdata_length-2, 0xFFFF);
    write_uint16LE(pdata_unencrypted, pdataCRC, pdata_length-2);

    uint8_t pdata_encrypted_with_padding[crypto_secretbox_ZEROBYTES + MAX_PDATA_LENGTH];
    memset(pdata_encrypted_with_padding, 0, crypto_secretbox_ZEROBYTES);
    memcpy(&pdata_encrypted_with_padding[32], pdata_unencrypted, pdata_length); //fill pdata_encrypted with pdata_unencrypted, encrypt in-place
    uint8_t* pdata_encrypted = &pdata_encrypted_with_padding[crypto_secretbox_BOXZEROBYTES];
    crypto_secretbox(
        pdata_encrypted_with_padding, 
        pdata_encrypted_with_padding, 
        crypto_secretbox_ZEROBYTES + pdata_length, 
        adata, //adata[0] to adata[23] contains the nonce
        pairing_ctx.shared_secret);

    memcpy(output_buffer, adata, ADATA_LENGTH);
    memcpy(output_buffer+ADATA_LENGTH, pdata_encrypted, pdata_length + crypto_secretbox_MACBYTES);
    return ADATA_LENGTH + pdata_length + crypto_secretbox_MACBYTES;
}

static bool decrypt_challenge(uint8_t* out_nonce, uint8_t* encrypted_challenge) {
    uint8_t decryption_buffer[ENCRYPTED_CHALLENGE_PDATA_LENGTH+crypto_box_ZEROBYTES];
    memset(decryption_buffer, 0, crypto_secretbox_BOXZEROBYTES);

    //check the size entry in adata
    if(read_uint16LE(encrypted_challenge, 28) != ENCRYPTED_CHALLENGE_PDATA_LENGTH) { return false; }
    //encrypted message contains
    //[0]: nonce (24 bytes)
    //[24]: auth_id (4 bytes)
    //[28]: length (2 bytes)
    //[30]: encrypted_pdata ((encrypted_message_length - 30) bytes)
    memcpy(&decryption_buffer[crypto_secretbox_BOXZEROBYTES], &encrypted_challenge[30], ENCRYPTED_CHALLENGE_PDATA_LENGTH); //decrypt in-place

    int32_t result = crypto_secretbox_open(
        decryption_buffer, 
        decryption_buffer, 
        crypto_secretbox_BOXZEROBYTES + ENCRYPTED_CHALLENGE_PDATA_LENGTH, 
        encrypted_challenge, //encrypted_message[0] to encrypted_message[23] contains the nonce
        pairing_ctx.shared_secret);
        
    if(result != 0) { return false; }

    uint8_t* decrypted_message = &decryption_buffer[crypto_box_ZEROBYTES];
    uint16_t crc_offset = DECRYPTED_CHALLENGE_PDATA_LENGTH - 2;
    uint16_t crc = crc_16(decrypted_message, crc_offset, 0xFFFF);
    if(read_uint16LE(decrypted_message, crc_offset) != crc || read_uint32LE(decrypted_message, 0) != pairing_ctx.authorization_id) {
        return false;
    }

    memcpy(out_nonce, &(decrypted_message[6]), SMARTLOCK_NONCE_LENGTH);
    return true;
}

static void calculate_authenticator(uint8_t* output_buffer, uint8_t* message, uint16_t message_length) {
    HMAC_SHA256_compute(message, message_length, pairing_ctx.shared_secret, 32, output_buffer);
}

uint16_t create_challenge_payload(uint8_t* output_buffer) {
    uint8_t pdata_unencrypted[CHALLENGE_CMD_PDATA_LENGTH];
    write_uint32LE(pdata_unencrypted, pairing_ctx.authorization_id, 0);
    write_uint16LE(pdata_unencrypted, request_data_cmd, 4);
    write_uint16LE(pdata_unencrypted, challenge_cmd, 6);
    return encrypt_payload(output_buffer, pdata_unencrypted, CHALLENGE_CMD_PDATA_LENGTH);
}

uint16_t create_lock_action_payload(uint8_t lock_action, uint8_t* output_buffer, uint8_t* encrypted_challenge) {
    uint8_t pdata_unencrypted[LOCK_ACTION_PDATA_LENGTH];
    bool decrypted = decrypt_challenge(&pdata_unencrypted[12], encrypted_challenge);
    if(!decrypted) { return 0; }
    uint8_t flags = 0;
    write_uint32LE(pdata_unencrypted, pairing_ctx.authorization_id, 0);
    write_uint16LE(pdata_unencrypted, lock_action_cmd, 4);
    write_uint8LE(pdata_unencrypted, lock_action, 6);
    write_uint32LE(pdata_unencrypted, pairing_ctx.app_id, 7);
    write_uint8LE(pdata_unencrypted, flags, 11);

    return encrypt_payload(output_buffer, pdata_unencrypted, LOCK_ACTION_PDATA_LENGTH);
}

uint16_t create_request_public_key_payload(uint8_t* output_buffer) {
    uint16_t command_length = 6;
    write_uint16LE(output_buffer, request_data_cmd, 0);
    write_uint16LE(output_buffer, public_key_cmd, 2);
    crc_payload(output_buffer, command_length);
    return command_length;
}

uint16_t create_write_public_key_payload(uint8_t* output_buffer, uint8_t* received_data) 
{
    uint8_t* received_public_key = &received_data[2];
    crypto_box_keypair(public_key_fob, private_key_fob);
    memcpy(public_key_nuki, received_public_key, crypto_box_PUBLICKEYBYTES);

    uint16_t command_length = 36;
    write_uint16LE(output_buffer, public_key_cmd, 0);
    memcpy(&output_buffer[2], public_key_fob, crypto_box_PUBLICKEYBYTES);
    crc_payload(output_buffer, command_length);
    return command_length;
}

uint16_t create_authorization_authenticator_payload(uint8_t* output_buffer, uint8_t* received_data) 
{
    uint8_t* nonce = &received_data[2];
    uint16_t command_length = 36;
    write_uint16LE(output_buffer, authorization_authenticator_cmd, 0);

    //Shared key calculation
    uint8_t dh_key[crypto_scalarmult_BYTES];
    crypto_scalarmult(dh_key, private_key_fob, public_key_nuki);
    unsigned char _0[16];
    memset(_0, 0, 16);
    const unsigned char sigma[17] = "expand 32-byte k";
    crypto_core_hsalsa20(pairing_ctx.shared_secret, _0, dh_key, sigma);

    //keep r in a seperate buffer to prevent 32-bit boundary issues
    const uint16_t r_length = crypto_box_PUBLICKEYBYTES+crypto_box_PUBLICKEYBYTES+PAIRING_NONCEBYTES;
    uint8_t r[crypto_box_PUBLICKEYBYTES+crypto_box_PUBLICKEYBYTES+PAIRING_NONCEBYTES];
    memcpy(r, public_key_fob, crypto_box_PUBLICKEYBYTES);
    memcpy(&r[crypto_box_PUBLICKEYBYTES], public_key_nuki, crypto_box_PUBLICKEYBYTES);
    memcpy(&r[crypto_box_PUBLICKEYBYTES+crypto_box_PUBLICKEYBYTES], nonce, PAIRING_NONCEBYTES);
    uint8_t authenticator[32];
    calculate_authenticator(authenticator, r, r_length);
    memcpy(&output_buffer[2], authenticator, crypto_auth_hmacsha512256_BYTES);
    crc_payload(output_buffer, command_length);
    return command_length;
}

uint16_t create_authorization_data_payload(uint8_t* output_buffer, uint8_t* received_data) {
    uint8_t* nonce = &received_data[2];
    uint16_t command_length = 105;
    write_uint16LE(output_buffer, authorization_data_cmd, 0);
    uint8_t app_id_buffer[4];
    randombytes(app_id_buffer, sizeof(pairing_ctx.app_id));
    pairing_ctx.app_id = read_uint32LE(app_id_buffer, 0);

    char name[33];
    snprintf(name, 32, "Open Nuki Fob %08lX               ", pairing_ctx.app_id);

    const uint16_t r_length = 101;
    uint8_t r[101];
    write_uint8LE(r, APP_TYPE_FOB, 0);
    write_uint32LE(r, pairing_ctx.app_id, 1);
    memcpy(&r[5], name, 32);
    randombytes(&r[37], PAIRING_NONCEBYTES);
    memcpy(&r[69], nonce, PAIRING_NONCEBYTES); 

    uint8_t authenticator[32];
    calculate_authenticator(authenticator, r, r_length);
    memcpy(&output_buffer[2], authenticator, crypto_auth_hmacsha512256_BYTES);
    memcpy(&output_buffer[34], r, 69);
    crc_payload(output_buffer, command_length);
    return command_length;
}

uint16_t create_authorization_id_confirmation_payload(uint8_t* output_buffer, uint8_t* received_data) {
    uint8_t* nonce = &received_data[54];
    uint16_t command_length = 40;
    write_uint16LE(output_buffer, authorization_id_confirmation_cmd, 0);
    pairing_ctx.authorization_id = read_uint32LE(received_data, 34);
    const uint16_t r_length = 36;
    uint8_t r[36];
    write_uint32LE(r, pairing_ctx.authorization_id, 0);
    memcpy(&r[4], nonce, PAIRING_NONCEBYTES); 

    uint8_t authenticator[32];
    calculate_authenticator(authenticator, r, r_length);
    memcpy(&output_buffer[2], authenticator, crypto_auth_hmacsha512256_BYTES);
    memcpy(&output_buffer[34], r, 4);
    crc_payload(output_buffer, command_length);
    return command_length;
}

pairing_context* get_pairing_context() 
{
    return &pairing_ctx;
}


