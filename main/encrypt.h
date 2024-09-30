// encryption_lib.h
#ifndef ENCRYPT_H
#define ENCRYPT_H

#include <stdint.h>
#include <stdlib.h>

// Declare the AES key and IV as extern variables so they can be accessed globally
extern unsigned char aes_key[16];
extern unsigned char aes_iv[16];

// Function declarations
uint8_t* my_aes_encrypt(const uint8_t* plaintext, size_t plaintext_len, size_t* encrypted_len);
uint8_t* my_aes_decrypt(const uint8_t* ciphertext, size_t ciphertext_len, size_t* decrypted_len);

#endif // ENCRYPT_H
