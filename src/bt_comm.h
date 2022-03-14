#include <stdint.h>

uint8_t send_buffer[200];
typedef void (*bt_callback)(void*, uint8_t*, uint16_t);
void send_with_response(uint8_t* data, uint16_t data_length, uint16_t expected_response_length, bt_callback);