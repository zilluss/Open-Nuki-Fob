#include <stddef.h>
#include <string.h>
#include "nrf_ficr.h"
#include "nrf_fstorage.h"
#include "nrf_fstorage_sd.h"
#include "nrf_log.h"
#include "nrf_sdm.h"
#include "fob_data.h"

#define BOOTLOADER_PAGES 12
#define FOB_DATA_FLASH_ADDRESS ((NRF_FICR->CODESIZE - BOOTLOADER_PAGES) * NRF_FICR->CODEPAGESIZE)
#define FOB_DATA_BACKUP_FLASH_ADDRESS ((NRF_FICR->CODESIZE - BOOTLOADER_PAGES-1) * NRF_FICR->CODEPAGESIZE)
#define HANDLE_CACHE_FLASH_ADDRESS ((NRF_FICR->CODESIZE - BOOTLOADER_PAGES-2) * NRF_FICR->CODEPAGESIZE)

static void fstorage_evt_handler(nrf_fstorage_evt_t* p_evt);

NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) =
{
    .evt_handler = fstorage_evt_handler,
    .start_addr = 0x3e000,
    .end_addr = 0x80000,
};

static void fstorage_evt_handler(nrf_fstorage_evt_t* p_evt) {
    fob_data_writing_context* ctx = (fob_data_writing_context*)p_evt->p_param;
    switch(p_evt->id) {
        case NRF_FSTORAGE_EVT_ERASE_RESULT: {
            if(ctx->write_state == WS_ERASING_FOB_DATA) {
                NRF_LOG_INFO("Erasing fob data done");
                ctx->write_state = WS_WRITE_FOB_DATA;
            }
            if(ctx->write_state == WS_ERASING_BACKUP_DATA) {
                NRF_LOG_INFO("Erasing fob data backup done");
                ctx->write_state = WS_WRITE_BACKUP_DATA;
            }
            if(ctx->write_state == WS_ERASING_HANDLE_CACHE) {
                NRF_LOG_INFO("Erasing handle cache done");
                ctx->write_state = WS_WRITE_HANDLE_CACHE;
            }
            break;

        }

        case NRF_FSTORAGE_EVT_WRITE_RESULT: {
            if(ctx->write_state == WS_WRITING_FOB_DATA) {
                NRF_LOG_INFO("Writing fob data done");
                ctx->write_state = WS_ERASE_BACKUP_DATA;
            }
            if(ctx->write_state == WS_WRITING_BACKUP_DATA) {
                NRF_LOG_INFO("Writing fob data backup done");
                ctx->write_state = WS_WRITING_DONE;
            }
            if(ctx->write_state == WS_WRITING_HANDLE_CACHE) {
                NRF_LOG_INFO("Writing handle cache done");
                ctx->write_state = WS_WRITING_DONE;
            }
            break;
        }

        default:
            break;
    }
}

static bool is_same_uuid(const lock_pairing* existing_pairing, uint8_t* new_pairing) {
    for(int i = 0; i < NUKI_UUID_LEN; i++) {
        if(existing_pairing->pairing.lock_uuid[i] != new_pairing[i]) {
            return false;
        }
    }
    return true;
}

static void prepare_writing_flash(fob_data_writing_context* ctx) {
    ret_code_t error = nrf_fstorage_init(
        &fstorage,
        &nrf_fstorage_sd,
        ctx);

    if(error != NRF_SUCCESS) {
        ctx->write_state = WS_ERROR;
        return;
    }
}

static void update_fob_settings(fob_data_writing_context* ctx) {
    fob_data const* fob_data_flash = get_fob_data();
    if(fob_data_flash != NULL) {
        memcpy(ctx->page_buffer, fob_data_flash, sizeof(fob_data));
    }
    fob_data* fob_data_ram = (fob_data*)ctx->page_buffer;
    get_fob_settings(&fob_data_ram->settings);
    fob_data_ram->settings.settings_version = SETTINGS_VERSION;
    fob_data_ram->settings.update_count++;
    fob_data_ram->settings.most_recent_pairing_index++;
    if(fob_data_ram->settings.most_recent_pairing_index >= MAX_PAIRINGS) {
        fob_data_ram->settings.most_recent_pairing_index = 0;
    }

    uint16_t pairing_slot = fob_data_ram->settings.most_recent_pairing_index;
    //if a pairing for this lock already exists, overwrite it
    for(uint16_t i = 0; i < MAX_PAIRINGS; i++) {
        if(is_same_uuid(&fob_data_ram->paired_locks[i], ctx->data.pairing.lock_uuid)) {
            pairing_slot = i;
        }
    }
    fob_data_ram->paired_locks[pairing_slot].pairing = ctx->data.pairing;
    NRF_LOG_INFO("Storing pairing at slot %i", pairing_slot);
}

static void update_handles(fob_data_writing_context* ctx) {
    uint16_t pairing_slot = -1;
    memcpy(ctx->page_buffer, get_handle_cache(), sizeof(handle_cache));

    const fob_data* fob_data_flash = get_fob_data();
    for(uint16_t i = 0; i < MAX_PAIRINGS; i++) {
        if(is_same_uuid(&fob_data_flash->paired_locks[i], ctx->data.handles.lock_uuid)) {
            pairing_slot = i;
        }
    }
    if(pairing_slot < 0) {
        ctx->write_state = WS_ERROR;
        return;
    }
    ((handle_cache*)ctx->page_buffer)->handle_for_lock[pairing_slot] = ctx->data.handles.handles;
}

static void erase_flash_page(uint32_t address, fob_data_writing_context* ctx) {
    //prior erasing is necessary because erasing sets all flash to 1 and writing only writes 0s
    ret_code_t error = nrf_fstorage_erase(&fstorage, address, 1, ctx);
    if(error != NRF_SUCCESS) {
        ctx->write_state = WS_ERROR;
    }
}

static void write_flash_page(uint32_t address, fob_data_writing_context* ctx) {
    ret_code_t error = nrf_fstorage_write(&fstorage, address, ctx->page_buffer, NRF_FICR->CODEPAGESIZE, ctx);
    if(error != NRF_SUCCESS) {
        ctx->write_state = WS_ERROR;
    }
}

void write_fob_data(fob_data_writing_context* ctx) {
    switch(ctx->write_state) {
        case WS_START_FOB_DATA:
            prepare_writing_flash(ctx);
            update_fob_settings(ctx);
            ctx->write_state = WS_ERASE_FOB_DATA;
            break;

        case WS_ERASE_FOB_DATA:
            erase_flash_page(FOB_DATA_FLASH_ADDRESS, ctx);
            ctx->write_state = WS_ERASING_FOB_DATA;
            NRF_LOG_INFO("Erasing fob data");
            break;

        case WS_WRITE_FOB_DATA:
            write_flash_page(FOB_DATA_FLASH_ADDRESS, ctx);
            ctx->write_state = WS_WRITING_FOB_DATA;
            NRF_LOG_INFO("Writing fob data");
            break;

        case WS_ERASE_BACKUP_DATA:
            erase_flash_page(FOB_DATA_BACKUP_FLASH_ADDRESS, ctx);
            ctx->write_state = WS_ERASING_BACKUP_DATA;
            NRF_LOG_INFO("Erasing fob data backup");
            break;

        case WS_WRITE_BACKUP_DATA:
            write_flash_page(FOB_DATA_BACKUP_FLASH_ADDRESS, ctx);
            ctx->write_state = WS_WRITING_BACKUP_DATA;
            NRF_LOG_INFO("Writing fob data backup");
            break;

        case WS_START_HANDLE_CACHE:
            prepare_writing_flash(ctx);
            update_handles(ctx);
            ctx->write_state = WS_ERASE_HANDLE_CACHE;
            break;

        case WS_ERASE_HANDLE_CACHE:
            erase_flash_page(HANDLE_CACHE_FLASH_ADDRESS, ctx);
            ctx->write_state = WS_ERASING_HANDLE_CACHE;
            NRF_LOG_INFO("Erasing handle cache");
            break;

        case WS_WRITE_HANDLE_CACHE:
            write_flash_page(HANDLE_CACHE_FLASH_ADDRESS, ctx);
            ctx->write_state = WS_WRITING_HANDLE_CACHE;
            NRF_LOG_INFO("Writing handle cache");
            break;

        default:
            break;
    }
}

fob_data const* get_fob_data() {
    fob_data* main_fob_data = (fob_data*)FOB_DATA_FLASH_ADDRESS;
    fob_data* backup_fob_data = (fob_data*)FOB_DATA_FLASH_ADDRESS;

    //if both main data and backup are valid, check if one is newer than the other
    if(main_fob_data->settings.magic_number == MAGIC_NUMBER_FOB_DATA &&
        backup_fob_data->settings.magic_number == MAGIC_NUMBER_FOB_DATA) {
        if(main_fob_data->settings.update_count < backup_fob_data->settings.update_count) {
            return backup_fob_data;
        }
        else {
            return main_fob_data;
        }
    }

    else if(main_fob_data->settings.magic_number == MAGIC_NUMBER_FOB_DATA) {
        return main_fob_data;
    }

    else if(backup_fob_data->settings.magic_number == MAGIC_NUMBER_FOB_DATA) {
        return backup_fob_data;
    }
    return NULL;
}

void get_fob_settings(fob_settings* out) {
    fob_data const* fob_data_flash = get_fob_data();

    if(fob_data_flash != NULL) {
        memcpy(out, &fob_data_flash->settings, sizeof(fob_settings));
    }
    //no valid fob data found, initialize the settings
    else {
        out->magic_number = MAGIC_NUMBER_FOB_DATA;
        out->update_count = 0;
        out->most_recent_pairing_index = MAX_PAIRINGS;
        uint32_t device_address = NRF_FICR->DEVICEADDR[0];
        snprintf(out->fob_name, 32, "Open Nuki Fob %08lX", device_address);
        out->app_id = device_address;
    }
}

const handle_cache* get_handle_cache() {
    return (handle_cache*)(HANDLE_CACHE_FLASH_ADDRESS);
}