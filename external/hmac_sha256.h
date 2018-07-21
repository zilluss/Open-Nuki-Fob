#ifndef __hmac_sha256_h__
#define __hmac_sha256_h__

#include <stdbool.h>
#include <stdint.h>

#include "sha2.h"

/**@brief Compute HMAC-SHA256 digest as defined by RFC 2104 (http://tools.ietf.org/html/rfc2104.html)
 *
 * @param[in]  p_msg     Message to verify
 * @param[in]  p_msg_len Length of message
 * @param[in]  p_key     Secret key. Must be 64 bytes or less. 
 * @param[in]  p_key_len Length of secret key
 * @param[out] p_digest  Resulting digest
 *
 * @return true if successful
 */
bool HMAC_SHA256_compute(const uint8_t * p_msg, uint32_t p_msg_len, const uint8_t * p_key, uint8_t p_key_len, uint8_t p_digest[SHA256_DIGEST_LENGTH]);

#endif /* __hmac_sha256_h__ */
