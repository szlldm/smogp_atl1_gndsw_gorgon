/*
 * Copyright 2015-2019 Miklos Maroti.
 * Copyright      2020 szlldm (modified to be thread safe; speed improvement in case of speculative decoding)
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include "ra_decoder.h"
#include "ra_decoder_gen.h"

void ra_decode(const uint8_t *inputBytes, uint8_t *outputBytes, size_t length) {

	ractx_t ctx;

	ra_length_init(&ctx, length/2);

	float * ra_bits = malloc(sizeof(float) * ctx.ra_code_length * RA_BITCOUNT);

	ra_word_t *ra_input = (ra_word_t*)(inputBytes);
	ra_word_t *ra_output = (ra_word_t*)(outputBytes);
	//memset(ra_bits, 0, ctx.ra_code_length * RA_BITCOUNT);
	ra_index_t i;
	int j;

	for (i = 0; i < (ctx.ra_code_length); i++) {
		for (j = 0; j < RA_BITCOUNT; j++){
			if ((ra_input[i] & (1 << j)) == 0) {
				ra_bits[RA_BITCOUNT * i + j] = 1.0;
			} else {
				ra_bits[RA_BITCOUNT * i + j] = -1.0;
			}
		}
	}

	ra_decoder_gen(&ctx, ra_bits, ra_output, 20);
	free(ra_bits);
}
