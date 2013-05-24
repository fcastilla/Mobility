#pragma once

#include <string>

#ifdef DEBUG
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
#define DEBUG_NEW new(_NORMAL_BLOCK, __FILE__, __LINE__)
#define new DEBUG_NEW
#endif

#define HASH_PRIME   100711433U  // a large prime number

/** Hash function for integers. */
unsigned int intHash(unsigned int key);

/** Hash function for strings. */
unsigned int strHash(std::string key);
