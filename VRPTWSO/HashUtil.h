#pragma once

#include <string>

#define HASH_PRIME   100711433U  // a large prime number

/** Hash function for integers. */
unsigned int intHash(unsigned int key);

/** Hash function for strings. */
unsigned int strHash(std::string key);
