/*
 * Hard-decision packet looter
 * Copyright (C) 2019-2020 szlldm, ha7wen
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <stdlib.h>
#include <complex.h>
#include <unistd.h>
#include <malloc.h>
#include <stdint.h>
#include <time.h>
#include <stdbool.h>
#include <string.h>

//#define SYNC 0x2dd4

// uniform hypotetical packet length: 650 bytes == 5200 bit
// odd or even bits -> 2*5200 == 10400 hypotetical overlapping packets

int main(int argc, char **argv)
{
	uint8_t * hypot_packets = malloc(10400*1024);	// 650 rounded up to 1024
	uint32_t * hypot_bitcnt = calloc(10400, sizeof(uint32_t));
	uint32_t hypot_tail[2];
	uint32_t hypot_head[2];
	uint32_t hypot_ring;

	uint32_t sync[2];
	uint8_t d = 0;
	int hypot_pointer = 0;
	int ix;
	int i, j, k, m;
	int bytepos;

	complex float in[16];

	char outbuf[2048];

	hypot_tail[0] = 0;
	hypot_tail[1] = 1;
	hypot_head[0] = 0;
	hypot_head[1] = 1;

	while(1){
		k = fread(in,sizeof(complex float),16,stdin);
		if (feof(stdin)) break;
		for (m=0; m<k ;m++) {
		//if (k > 0){
			hypot_pointer %= 10400;
			d = (creal(in[0+m])>cimag(in[0+m]));


			hypot_ring = hypot_tail[hypot_pointer % 2];
			while (hypot_head[hypot_pointer % 2] != hypot_ring) {
				bytepos = hypot_ring*1024 + (hypot_bitcnt[hypot_ring] / 8);
				hypot_packets[bytepos] = (hypot_packets[bytepos] << 1) | d;
				hypot_bitcnt[hypot_ring] += 1;
				if (hypot_bitcnt[hypot_ring] >= 5200) {	// all bit available
					hypot_bitcnt[hypot_ring] = 0;
					hypot_tail[hypot_pointer % 2] += 2;
					hypot_tail[hypot_pointer % 2] %= 10400;
					bytepos = hypot_ring*1024;
					for (i=0; i<650; i++) {
						sprintf(&outbuf[i*2], "%.2hhX", hypot_packets[bytepos + i]);
					}
					sprintf(&outbuf[650*2], " RSSI -132 dBm\n");
					printf("%s", outbuf);
					fflush(stdout);

				}


				hypot_ring += 2;
				hypot_ring %= 10400;
			}

			sync[hypot_pointer % 2] = (sync[hypot_pointer % 2] << 1) | d;
			sync[hypot_pointer % 2] &= 0x00FFFFFF;

			if (__builtin_popcount(sync[hypot_pointer % 2] ^ 0x00AA2DD4) <= 5) {	// max 5 bit error
				hypot_ring = hypot_head[hypot_pointer % 2];
				hypot_bitcnt[hypot_ring] = 0;
				hypot_head[hypot_pointer % 2] += 2;
				hypot_head[hypot_pointer % 2] %= 10400;
				if (hypot_head[hypot_pointer % 2] == hypot_tail[hypot_pointer % 2]) {
					hypot_ring = hypot_tail[hypot_pointer % 2];
					hypot_bitcnt[hypot_ring] = 0;
					hypot_tail[hypot_pointer % 2] += 2;
					hypot_tail[hypot_pointer % 2] %= 10400;
				}
			}

			hypot_pointer += 1;

		}
	}
	printf("\r\n");
	return 0;
}

