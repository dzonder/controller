/*
 * Tool to convert fonts which were downloaded from:
 * http://jared.geek.nz/custom-fonts-for-microcontrollers
 *
 * Basically inverts bits in each byte.
 */

#include <stdio.h>
#include "font.h"

int main(int argc, const char *argv[])
{
	printf("const unsigned char font[96][7] = {\n");
	for (int i = 0; i < 96; ++i) {
		printf("\t{ ");
		for (int j = 0; j < 7; ++j) {
			unsigned char b = font[i][j];
			b = (b * 0x0202020202ULL & 0x010884422010ULL) % 1023;
			printf("0x%02x, ", b);
		}
		printf("}, \n");
	}
	printf("};\n");
	return 0;
}
