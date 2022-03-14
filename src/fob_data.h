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

enum write_states {
    WS_START, WS_ERASE_FOB_DATA, WS_ERASING_FOB_DATA, WS_WRITE_FOB_DATA, WS_WRITING_FOB_DATA,
    WS_ERASE_BACKUP_DATA, WS_ERASING_BACKUP_DATA, WS_WRITE_BACKUP_DATA, WS_WRITING_BACKUP_DATA,
    WS_WRITING_DONE, WS_ERROR
};

typedef struct {
    void* page_buffer;
    lock_pairing data;
    enum write_states write_state;
} fob_data_writing_context;


void write_fob_data(fob_data_writing_context* ctx);
void get_fob_settings(fob_settings* out);
fob_data const* get_fob_data();

#endif