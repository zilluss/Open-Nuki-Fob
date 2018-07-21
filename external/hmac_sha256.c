#include "hmac_sha256.h"

#include <string.h>

 /***************************************************************
  ** (From RFC 2104 (http://tools.ietf.org/html/rfc2104.html)) **
  ***************************************************************

   We define two fixed and different strings ipad and opad as follows
   (the 'i' and 'o' are mnemonics for inner and outer):

                   ipad = the byte 0x36 repeated B times
                  opad = the byte 0x5C repeated B times.

   To compute HMAC over the data `text' we perform

                    H(K XOR opad, H(K XOR ipad, text))

   Namely,

    (1) append zeros to the end of K to create a B byte string
        (e.g., if K is of length 20 bytes and B=64, then K will be
         appended with 44 zero bytes 0x00)
    (2) XOR (bitwise exclusive-OR) the B byte string computed in step
        (1) with ipad
    (3) append the stream of data 'text' to the B byte string resulting
        from step (2)
    (4) apply H to the stream generated in step (3)
    (5) XOR (bitwise exclusive-OR) the B byte string computed in
        step (1) with opad
    (6) append the H result from step (4) to the B byte string
        resulting from step (5)
    (7) apply H to the stream generated in step (6) and output
        the result        
*/

bool HMAC_SHA256_compute(const uint8_t * p_msg, uint32_t p_msg_len, const uint8_t * p_key, uint8_t p_key_len, uint8_t p_digest[SHA256_DIGEST_LENGTH])
{
    const uint8_t ipad = 0x36;
    const uint8_t opad = 0x5c;
    uint8_t       buf[SHA256_BLOCK_LENGTH + SHA256_DIGEST_LENGTH];
    uint8_t       digest_buf[SHA256_DIGEST_LENGTH];
    SHA256_CTX    ctx256;
        
    if (p_key_len > SHA256_BLOCK_LENGTH || p_key == 0 || p_key_len == 0)
    {
        return false;
    }
    if (p_msg_len == 0 || p_msg == 0)
    {
        return false;
    }
    if (p_digest == 0)
    {
        return 0;
    }
        
    // Step 1
    memset(buf, 0, sizeof(buf));
    memcpy(buf, p_key, p_key_len);
                                               
    // Step 2
    for (int i = 0; i < SHA256_BLOCK_LENGTH; ++i)
    {
        buf[i] ^= ipad;
    }
    
    // Steps 3 and 4
    SHA256_Init(&ctx256);
    SHA256_Update(&ctx256, buf, SHA256_BLOCK_LENGTH);
    SHA256_Update(&ctx256, p_msg, p_msg_len);
    SHA256_Final(digest_buf, &ctx256);
    
    // Step 5
    memset(buf, 0, sizeof(buf));
    memcpy(buf, p_key, p_key_len);
    for (int i = 0; i < SHA256_BLOCK_LENGTH; ++i)
    {
        buf[i] ^= opad;
    }
    
    // Step 6 
    memcpy(&buf[SHA256_BLOCK_LENGTH], digest_buf, sizeof(digest_buf));
    
    // Step 7
    SHA256_Init(&ctx256);
    SHA256_Update(&ctx256, buf, sizeof(buf));
    SHA256_Final(p_digest, &ctx256);

    return true;
}
