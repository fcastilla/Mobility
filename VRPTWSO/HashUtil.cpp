#include "HashUtil.h"

unsigned int intHash(unsigned int key)
{
   key += (key << 12);
   key ^= (key >> 22);
   key += (key << 4);
   key ^= (key >> 9);
   key += (key << 10);
   key ^= (key >> 2);
   key += (key << 7);
   key ^= (key >> 12);
   return key;
}

unsigned int strHash(std::string key)
{
   int i;
   unsigned int value = 0;
   int n = (int)key.size();
   char* keyStr = (char*)key.c_str();
   for (i = 0; i < n; i++)
   {
      value *= HASH_PRIME;
      value += (unsigned int)keyStr[i];
   }
   return value;
}
