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

#ifndef RA_CONFIG_H
#define RA_CONFIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint16_t  ra_word_t;  /* use uint16_t for SSE4.1 soft decoder */
typedef uint16_t ra_index_t; /* use uint16_t for RA_PACKET_LENGTH >= 256 */



enum {
  /* number of words, must be at least 4 */
  RA_MAX_DATA_LENGTH = 2048,

  /* 1 for rate 1/4, 2 for 2/5, 3 for 1/2, 5 for 5/8 */
  RA_PUNCTURE_RATE = 3,

  /* use the test program to verify it */
  RA_MAX_CODE_LENGTH = RA_MAX_DATA_LENGTH * 2 + 3,
};




enum { RA_BITCOUNT = 8 * sizeof(ra_word_t), RA_BITSHIFT = RA_BITCOUNT - 1 };

typedef struct {
	// ra_config:
	ra_index_t ra_data_length;
	ra_index_t ra_code_length;
	ra_index_t ra_chck_length;
	uint16_t ra_lfsr_masks[4];
	uint8_t ra_lfsr_highbit;
	// ra_lfsr:
	ra_index_t ra_lfsr_mask;
	ra_index_t ra_lfsr_state;
	ra_index_t ra_lfsr_offset;
	// ra_decoder_gen
	float ra_dataword_gen[RA_MAX_DATA_LENGTH * RA_BITCOUNT];
	float ra_codeword_gen[RA_MAX_CODE_LENGTH * RA_BITCOUNT];
	float ra_forward_gen[RA_MAX_DATA_LENGTH * RA_BITCOUNT];
} ractx_t;

/* data length in words */
void ra_length_init(ractx_t * ctx, ra_index_t data_length);



#ifdef __cplusplus
}
#endif

#endif // RA_CONFIG_H
