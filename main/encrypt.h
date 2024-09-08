#ifndef ENCRYPT_H
#define ENCRYPT_H

#include <stdint.h>
#include "esp_log.h"
#include "esp_now.h"

// Encryption key (TEA requires a 128-bit key)
const uint32_t encryptionKey[4] = {0x01234567, 0x89ABCDEF, 0xFEDCBA98, 0x76543210};

// TEA encryption function (encrypts two 32-bit values)
void TEAEncrypt(uint32_t* v, const uint32_t* k) {
    uint32_t v0 = v[0], v1 = v[1], sum = 0;
    uint32_t delta = 0x9e3779b9; // TEA magic constant
    uint32_t k0 = k[0], k1 = k[1], k2 = k[2], k3 = k[3];

    for (int i = 0; i < 32; i++) { // 32 rounds
        sum += delta;
        v0 += ((v1 << 4) + k0) ^ (v1 + sum) ^ ((v1 >> 5) + k1);
        v1 += ((v0 << 4) + k2) ^ (v0 + sum) ^ ((v0 >> 5) + k3);
    }

    v[0] = v0;
    v[1] = v1;
}

// Function to send encrypted data using ESP-NOW
void sendEncryptedData(uint8_t* data, size_t length, const uint32_t* key, const uint8_t* peer_mac) {
    if (length % 8 != 0) {
        ESP_LOGE("TEA", "Data length must be a multiple of 8 bytes.");
        return;
    }

    // Encrypt each 8-byte block using TEA
    for (size_t i = 0; i < length; i += 8) {
        uint32_t* block = reinterpret_cast<uint32_t*>(&data[i]);
        TEAEncrypt(block, key);
    }

    // Send encrypted data using ESP-NOW
    esp_err_t err = esp_now_send(peer_mac, data, length);
    if (err != ESP_OK) {
        ESP_LOGE("ESP-NOW", "Error sending encrypted data: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI("ESP-NOW", "Encrypted data sent successfully");
    }
}

#endif // ENCRYPT_H


// void TEADecrypt(uint32_t* v, const uint32_t* k) {
//     uint32_t v0 = v[0], v1 = v[1], sum = 0xC6EF3720; // sum is delta * 32
//     uint32_t delta = 0x9e3779b9;
//     uint32_t k0 = k[0], k1 = k[1], k2 = k[2], k3 = k[3];

//     for (int i = 0; i < 32; i++) { // 32 rounds
//         v1 -= ((v0 << 4) + k2) ^ (v0 + sum) ^ ((v0 >> 5) + k3);
//         v0 -= ((v1 << 4) + k0) ^ (v1 + sum) ^ ((v1 >> 5) + k1);
//         sum -= delta;
//     }

//     v[0] = v0;
//     v[1] = v1;
// }
