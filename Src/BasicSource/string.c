#include "string.h"

/*
* @brief Simple version of strlen
*/
uint16_t strlen(const char * s)
{
	const char *sc;

	for (sc = s; *sc != '\0'; ++sc)
		/* nothing */;
	return sc - s;
}