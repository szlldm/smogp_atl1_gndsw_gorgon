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


#ifndef RA_LFSR_H
#define RA_LFSR_H

#include "ra_config.h"

#ifdef __cplusplus
extern "C" {
#endif

void ra_lfsr_init(ractx_t * ctx, uint8_t seqno);
inline ra_index_t ra_lfsr_next(ractx_t * ctx);
inline ra_index_t ra_lfsr_prev(ractx_t * ctx);

#ifdef __cplusplus
}
#endif

#endif // RA_LFSR_H
