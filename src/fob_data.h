#ifndef FOB_DATA_H
#define FOB_DATA_H

#include <stdint.h>
#include "nuki.h"

#define MAGIC_NUMBER_FOB_DATA 3355
#define SETTINGS_VERSION 1
#define LOCK_UUID_LEN 6

typedef union {
    nuki_key pairing;
    uint8_t padding[64];
} lock_pairing;

_Static_assert(sizeof(lock_pairing) == 64, "Pairing data needs 64 bytes to be page aligned");

typedef struct {
    uint16_t settings_version;
    uint16_t most_recent_pairing_index;
    char fob_name[32];
    uint32_t app_id;
    uint16_t magic_number;
    uint32_t update_count;
    uint8_t unused[128]; //for use in future versions
} fob_settings;

#define MAX_PAIRINGS 50
typedef struct {
    lock_pairing paired_locks[MAX_PAIRINGS];
    fob_settings settings;
} fob_data;
_Static_assert(sizeof(fob_data) <= 4096, "Fob data mustn't exceed a single flash page");

typedef struct {
    uint16_t cccd_handle;
    uint16_t keyturner_handle;
} keyturner_characteristic_handles;

typedef struct {
    keyturner_characteristic_handles handles;
    uint8_t lock_uuid[NUKI_UUID_LEN];
} lock_handle_update;

typedef struct {
    keyturner_characteristic_handles handle_for_lock[MAX_PAIRINGS];
} handle_cache;

enum write_states {
    WS_INVALID, 

    WS_START_FOB_DATA, WS_ERASE_FOB_DATA, WS_ERASING_FOB_DATA, WS_WRITE_FOB_DATA, WS_WRITING_FOB_DATA,
    WS_ERASE_BACKUP_DATA, WS_ERASING_BACKUP_DATA, WS_WRITE_BACKUP_DATA, WS_WRITING_BACKUP_DATA,

    WS_START_HANDLE_CACHE, WS_ERASE_HANDLE_CACHE, WS_ERASING_HANDLE_CACHE, 
    WS_WRITE_HANDLE_CACHE, WS_WRITING_HANDLE_CACHE,

    WS_WRITING_DONE, WS_ERROR
};

typedef union {
    nuki_key pairing;
    lock_handle_update handles;
} write_request;

typedef struct {
    void* page_buffer;
    write_request data;
    enum write_states write_state;
} fob_data_writing_context;


void write_fob_data(fob_data_writing_context* ctx);
void get_fob_settings(fob_settings* out);
fob_data const* get_fob_data();

handle_cache const* get_handle_cache();

#endif