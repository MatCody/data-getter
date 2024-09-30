// encryption_lib.cpp

#include "encrypt.h"
#include <mbedtls/aes.h>
#include <string.h>
#include <esp_log.h>

// AES key and IV (initialization vector)
// These can either be passed to the functions or be made global, depending on your use case.
unsigned char aes_key[16] = { 
    0x12, 0x34, 0x56, 0x78, 0x90, 0xAB, 0xCD, 0xEF, 
    0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF 
};

unsigned char aes_iv[16] = { 
    0xFE, 0xDC, 0xBA, 0x98, 0x76, 0x54, 0x32, 0x10, 
    0xFF, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA, 0x99, 0x88 
};

// Encryption function (C-style)
uint8_t* my_aes_encrypt(const uint8_t* plaintext, size_t plaintext_len, size_t* encrypted_len) {
    mbedtls_aes_context aes;
    mbedtls_aes_init(&aes);

    // Calculate padding length (PKCS#7 padding)
    size_t padding_len = 16 - (plaintext_len % 16);
    size_t total_len = plaintext_len + padding_len;

    // Allocate memory for the padded plaintext and copy the original plaintext
    uint8_t* padded_plaintext = (uint8_t*)malloc(total_len);
    if (padded_plaintext == NULL) {
        ESP_LOGE("AES", "Memory allocation failed for padded plaintext.");
        return NULL;
    }
    memcpy(padded_plaintext, plaintext, plaintext_len);

    // Apply padding
    memset(padded_plaintext + plaintext_len, padding_len, padding_len);

    // Allocate memory for the ciphertext
    uint8_t* ciphertext = (uint8_t*)malloc(total_len);
    if (ciphertext == NULL) {
        ESP_LOGE("AES", "Memory allocation failed for ciphertext.");
        free(padded_plaintext);
        return NULL;
    }

    mbedtls_aes_setkey_enc(&aes, aes_key, 128);

    unsigned char iv[16];
    memcpy(iv, aes_iv, 16);  // Copy the IV

    int ret = mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_ENCRYPT, total_len, iv, padded_plaintext, ciphertext);

    mbedtls_aes_free(&aes);
    free(padded_plaintext);  // Free the padded plaintext after encryption

    if (ret != 0) {
        ESP_LOGE("AES", "Encryption failed with error code: %d", ret);
        free(ciphertext);
        return NULL;
    }

    *encrypted_len = total_len;  // Set the encrypted length
    return ciphertext;
}

// Decryption function (C-style)
uint8_t* my_aes_decrypt(const uint8_t* ciphertext, size_t ciphertext_len, size_t* decrypted_len) {
    mbedtls_aes_context aes;
    mbedtls_aes_init(&aes);

    // Allocate memory for the decrypted text
    uint8_t* decrypted_text = (uint8_t*)malloc(ciphertext_len);
    if (decrypted_text == NULL) {
        ESP_LOGE("AES", "Memory allocation failed for decrypted text.");
        return NULL;
    }

    mbedtls_aes_setkey_dec(&aes, aes_key, 128);

    unsigned char iv[16];
    memcpy(iv, aes_iv, 16);  // Copy the IV

    int ret = mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_DECRYPT, ciphertext_len, iv, ciphertext, decrypted_text);

    mbedtls_aes_free(&aes);

    if (ret != 0) {
        ESP_LOGE("AES", "Decryption failed with error code: %d", ret);
        free(decrypted_text);
        return NULL;
    }

    // Get the padding length (PKCS#7 padding)
    size_t padding_len = decrypted_text[ciphertext_len - 1];
    if (padding_len > 16 || padding_len == 0) {
        ESP_LOGE("AES", "Invalid padding detected.");
        free(decrypted_text);
        return NULL;
    }

    *decrypted_len = ciphertext_len - padding_len;  // Set the decrypted length
    return decrypted_text;
}