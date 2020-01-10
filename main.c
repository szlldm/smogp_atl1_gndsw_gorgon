/*
 * Decoder and packet interpreter for Smog-P & ATL-1 pocketqube satellites
 * Copyright (C) 2019-2020 szlldm
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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <pthread.h>
#include <sched.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>

#include "main.h"
#include "lib/tsprint.h"
#include "lib/packet_check.h"

#include "ts_radecoder/ra_decoder.h"
#include "ao40/long/ao40_decode_message.h"
#include "ao40/short/ao40short_decode_message.h"

#if defined(BUILD_ATL1) + defined(BUILD_SMOGP) != 1
	#error BUILD TARGET NOT DEFINED!
#endif

#define DOWNLINK_PCKT_TYPE_TELEMETRY_1		(1)
#define DOWNLINK_PCKT_TYPE_TELEMETRY_2		(2)
#define DOWNLINK_PCKT_TYPE_TELEMETRY_3		(3)
#define DOWNLINK_PCKT_TYPE_BEACON		(4)
#define DOWNLINK_PCKT_TYPE_SPECTRUM_RESULT	(5)
#define DOWNLINK_PCKT_TYPE_FILE_INFO		(6)
#define DOWNLINK_PCKT_TYPE_FILE_FRAGMENT	(7)

#define DOWNLINK_PCKT_TYPE_SMOGP_TELEMETRY_1	(33)
#define DOWNLINK_PCKT_TYPE_SMOGP_TELEMETRY_2	(34)

// ATL-1 specific packet types - MSb == 1
#define DOWNLINK_PCKT_TYPE_ATL_TELEMETRY_1	(129)
#define DOWNLINK_PCKT_TYPE_ATL_TELEMETRY_2	(130)
#define DOWNLINK_PCKT_TYPE_ATL_TELEMETRY_3	(131)

#define DOWNLINK_PCKT_TYPE_RESERVED_SYNC	(151)	// SYNC_PCKT first byte


void us_sleep()
{
	struct timespec delay;
	delay.tv_sec = 0;
	delay.tv_nsec = 1000;
	nanosleep(&delay,NULL);
}


void store_ackd_serial(uint16_t u16, uint8_t u8)
{
}



#define SIGN_BUF_DEPTH	16
static unsigned int sign_buf_head = 0;
static unsigned int sign_buf_tail = 0;
static uint8_t sign_buf[SIGN_BUF_DEPTH][10];
int add_check_duplicate_signature(uint8_t * pckt, int len)
{
	sign_buf_head %= SIGN_BUF_DEPTH;
	sign_buf_tail %= SIGN_BUF_DEPTH;

	unsigned int h,t;
	int i;

	if (sign_buf_head != sign_buf_tail) {	// buffer not empty
		h = sign_buf_head;
		t = sign_buf_tail;
		for (i=0; i<SIGN_BUF_DEPTH; i++) {
			if (h == t) break;
			if (memcmp(&pckt[len - 10], sign_buf[t], 10) == 0) {
				return 1;	// already processed packet
				break;
			}
			t += 1;
			t %= SIGN_BUF_DEPTH;
		}
	}

	memcpy(sign_buf[sign_buf_head], &pckt[len - 10], 10);
	sign_buf_head += 1;
	sign_buf_head %= SIGN_BUF_DEPTH;
	if (sign_buf_head == sign_buf_tail) {
		sign_buf_tail += 1;
		sign_buf_tail %= SIGN_BUF_DEPTH;
	}

	return 0;
}



void save_pckt_to_json(char * satellite, uint8_t * pckt, int pckt_len)
{

	// {"satellite":"smog1","packet":"HEXA"}
	char filename[256];
	struct tm tmp_tm;
	time_t tmp_time;
	struct timeval tv;
	long int ms;
	char timestr[20];
	FILE * json_file = NULL;
	int i;

	gettimeofday(&tv, NULL);

	tmp_time = tv.tv_sec;
	strftime(timestr, 20, "%Y%m%d%H%M%S", gmtime_r(&tmp_time, &tmp_tm));
	timestr[19] = 0;

	ms = (tv.tv_usec / 1000L) % 1000L;
	sprintf(filename, "json/%s%03ld.json", timestr, ms);

	json_file = fopen(filename, "wb");

	if (json_file == NULL) return;

	fprintf(json_file, "{\"satellite\":\"%s\",\"packet\":\"", satellite);
	for (i=0; i<pckt_len; i++) {
		fprintf(json_file, "%02hhX", pckt[i]);
	}
	fprintf(json_file, "\"}");

	fflush(json_file);
	fclose(json_file);
}

static FILE * pcktprint_file = NULL;
static char pcktprint_buf[65536];

void pcktprint_open(char * extension)
{
	char filename[256];
	struct tm tmp_tm;
	time_t tmp_time;
	struct timeval tv;
	long int ms;
	char timestr[20];

	gettimeofday(&tv, NULL);

	tmp_time = tv.tv_sec;
	strftime(timestr, 20, "%Y%m%d%H%M%S", gmtime_r(&tmp_time, &tmp_tm));
	timestr[19] = 0;

	ms = (tv.tv_usec / 1000L) % 1000L;
	sprintf(filename, "packets/%s%03ld.txt", timestr, ms);
	if (extension != NULL) {
		if (extension[0] != 0) {
			sprintf(filename, "packets/%s%03ld.%s", timestr, ms, extension);
		}
	}

	if (pcktprint_file != NULL) {
		fclose(pcktprint_file);
		pcktprint_file = NULL;
	}

	pcktprint_file = fopen(filename, "wb");
}

void pcktprint_close()
{
	if (pcktprint_file == NULL) return;
	fclose(pcktprint_file);
	pcktprint_file = NULL;
	tsprintf("\n");
}

void pcktonlyfileprintf(const char *format, ...)
{
	if (pcktprint_file != NULL) {
		va_list args;
		va_start(args, format);
		vfprintf(pcktprint_file, format, args);
		fflush(pcktprint_file);
		va_end(args);
	}
}

void pcktprintf(const char *format, ...)
{
	va_list args;
	va_start(args, format);
	vsprintf(pcktprint_buf, format, args);
	va_end(args);

	if (pcktprint_file != NULL) {
		fprintf(pcktprint_file, "%s", pcktprint_buf);
		fflush(pcktprint_file);
	}
	tsprintf("%s", pcktprint_buf);

}





typedef struct {
	int rssi;
	int pckt_len;
	uint8_t pckt[1024];
} pckt_t;


typedef struct {
	pthread_mutex_t fifo_mutex;
	pthread_cond_t fifo_cond;
	unsigned int elem_size;
	unsigned int fifo_size;
	unsigned int fifo_sizem1;
	unsigned int head;
	unsigned int tail;
	unsigned int unread_cnt;
	uint8_t * fifo_storage;
} block_fifo_t;

#define ACCESS_LOCK	pthread_mutex_lock(&fifo->fifo_mutex)
#define ACCESS_TRYLOCK	pthread_mutex_trylock(&fifo->fifo_mutex)
#define ACCESS_UNLOCK	pthread_mutex_unlock(&fifo->fifo_mutex)

int block_fifo_init(block_fifo_t * fifo, unsigned int elem_size, unsigned int fifo_size)
{
	pthread_condattr_t attr;

	if (fifo == NULL) return -1;
	if (elem_size < 1) return -1;
	if (fifo_size < 2) return -1;

	fifo->elem_size = elem_size;
	fifo->fifo_size = fifo_size;
	fifo->fifo_sizem1 = fifo_size - 1;
	fifo->head = 0;
	fifo->tail = 0;
	fifo->unread_cnt = 0;
	if (pthread_mutex_init( &fifo->fifo_mutex, NULL ) != 0) return -1;
	pthread_condattr_init(&attr);
	pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
	if (pthread_cond_init( &fifo->fifo_cond, &attr ) != 0) return -1;
	if ((fifo->fifo_storage = calloc(fifo->fifo_size, fifo->elem_size)) == NULL) return -1;

	return 0;
}

bool block_fifo_empty(block_fifo_t * fifo)
{
	if (fifo == NULL) return true;
	bool tmp_bool = false;

	ACCESS_LOCK;
	tmp_bool = (fifo->unread_cnt == 0);
	ACCESS_UNLOCK;

	return tmp_bool;
}

int block_fifo_read(block_fifo_t * fifo, void * block)	// blocking read
{
	if (fifo == NULL) return -1;
	if (block == NULL) return -1;

	ACCESS_LOCK;

	if (fifo->unread_cnt == 0) {
		// wait indefinitely
		while ((fifo->unread_cnt == 0)) {
			pthread_cond_wait(&fifo->fifo_cond, &fifo->fifo_mutex);
		}
	}

	if (fifo->unread_cnt > 0) {

		memcpy(block, &fifo->fifo_storage[fifo->elem_size * fifo->tail], fifo->elem_size);
		//memset(&fifo->fifo_storage[fifo->elem_size * fifo->tail], 0, fifo->elem_size);

		fifo->tail += 1;
		if (fifo->tail >= fifo->fifo_size) {
			fifo->tail = 0;
		}

		fifo->unread_cnt -= 1;

		ACCESS_UNLOCK;

		return 0;
	}

	ACCESS_UNLOCK;

	return -1;
}

bool block_fifo_writeable(block_fifo_t * fifo)
{
	if (fifo == NULL) return false;
	bool tmp_bool = false;

	ACCESS_LOCK;
	tmp_bool = (fifo->unread_cnt < fifo->fifo_sizem1);	// fifo is notfull
	ACCESS_UNLOCK;

	return tmp_bool;
}

int block_fifo_write(block_fifo_t * fifo, void * block, bool overwrite)
{
	if (fifo == NULL) return -1;
	if (block == NULL) return -1;

	if (!overwrite) {
			while(!block_fifo_writeable(fifo)) {
				//us_sleep();
				sched_yield();
			}
	}

	ACCESS_LOCK;

	if ((fifo->unread_cnt >= fifo->fifo_sizem1)) {	// fifo is full
		pthread_cond_signal(&fifo->fifo_cond);	// signaling if the fifo is full

		if (!overwrite) {
			ACCESS_UNLOCK;
			sched_yield();
			return 0;
		}

		//ACCESS_UNLOCK;
		//sched_yield();		// try to hand over processing power to the reader side if the fifo is full
		//ACCESS_LOCK;
	} else {	// fifo is not full yet
		fifo->unread_cnt += 1;
	}

	memcpy(&fifo->fifo_storage[fifo->elem_size * fifo->head], block, fifo->elem_size);

	fifo->head += 1;
	if (fifo->head >= fifo->fifo_size) {
		fifo->head = 0;
	}

	if (fifo->head == fifo->tail) {		// overflow
		fifo->tail += 1;
		if (fifo->tail >= fifo->fifo_size) {
			fifo->tail = 0;
		}
	}

	if (fifo->unread_cnt == 1) pthread_cond_signal(&fifo->fifo_cond);	// signaling only if the fifo becomes non-empty

	ACCESS_UNLOCK;

	return 1;
}



bool hex_check(char c)
{
	if ((c >= '0') && (c <= '9')) return true;
	if ((c >= 'A') && (c <= 'F')) return true;

	return false;
}

uint8_t hex_to_num(char c)
{
	if ((c >= '0') && (c <= '9')) return (c-'0');
	if ((c >= 'A') && (c <= 'F')) return (c+10-'A');
	return 0;
}

char upper_case(char c)
{
	if (c < 'a') return c;
	if (c > 'z') return c;
	return c - ('a' - 'A');
}

void global_break(char * str)
{
	tsprintf(str);
	exit(-1);
}

bool timespec_expired(struct timespec deadline)
{
	struct timespec timeout_time;

	clock_gettime(CLOCK_MONOTONIC, &timeout_time);

	if (timeout_time.tv_sec < deadline.tv_sec) return false;
	if (timeout_time.tv_sec > deadline.tv_sec) return true;

	if (timeout_time.tv_nsec < deadline.tv_nsec) return false;
	return true;
}


static block_fifo_t received_raw_packet_fifo;
static block_fifo_t received_decoded_packet_fifo;





void * input_ptfun(void *arguments)
{
	char * input_buffer;
	size_t input_buffer_len;
	ssize_t read_len;

	int i,j,k;
	pckt_t * received_pckt = malloc(sizeof(pckt_t));

	bool rssi_valid;
	int rssi;
	int rssi_sign;


	input_buffer_len = 65536;
	input_buffer = malloc(input_buffer_len);

	do {
		read_len = getline(&input_buffer, &input_buffer_len, stdin);
		if (input_buffer == NULL) global_break("Getline failed\n");
		if (read_len < 0) {	// input is gone!
			tsprintf("STDIN reached EOF\nWait for processing...\n");
			while(!block_fifo_empty(&received_raw_packet_fifo)) us_sleep();
			while(!block_fifo_empty(&received_decoded_packet_fifo)) us_sleep();
			global_break("STDIN reached EOF, processing completed\n");
		}
		if (read_len == 0) continue;
		if (read_len < 8192) {
			received_pckt->pckt_len = 0;
			rssi_valid = false;
			rssi_sign = 1;
			for (i=0; i<read_len; i++) {
				if (received_pckt->pckt_len > 1024) break;
				if (i%2 == 0) {		// upper nibble
					if (!hex_check(upper_case(input_buffer[i]))) {
						if (input_buffer[i] != ' ') {
							received_pckt->pckt_len = -1;
						}
						break;
					}
					received_pckt->pckt[received_pckt->pckt_len] = (hex_to_num(upper_case(input_buffer[i])) << 4);
				} else {		// lower nibble
					// if lower nibble not HEX, binary is not valid
					if (!hex_check(upper_case(input_buffer[i]))) {
						received_pckt->pckt_len = -1;
						break;
					}
					received_pckt->pckt[received_pckt->pckt_len] += hex_to_num(upper_case(input_buffer[i]));
					received_pckt->pckt_len += 1;
				}
			}

			if (received_pckt->pckt_len > 0) {	// may be valid packet
				k=0;
				for (; i<read_len; i++) {	// 'i' continues
					switch(k) {
						case 0: k = ((input_buffer[i] == 'R') ? (k+1) : 0); break;
						case 1: k = ((input_buffer[i] == 'S') ? (k+1) : 0); break;
						case 2: k = ((input_buffer[i] == 'S') ? (k+1) : 0); break;
						case 3: k = ((input_buffer[i] == 'I') ? (k+1) : 0); break;
						case 4: k = ((input_buffer[i] == ' ') ? (k+1) : 0); break;
						default: k = 0; break;
					}
					if (k == 5) break;
				}
				i += 1;
				if (k == 5) {
					for (; i<read_len; i++) {	// 'i' continues
						if (k == 5) {	// sign or number
							if (input_buffer[i] == '-') {
								rssi_sign = -1;
								rssi = 0;
							} else
							if (input_buffer[i] == '+') {
								rssi_sign = 1;
								rssi = 0;
							} else
							if ((input_buffer[i] >= '0') && (input_buffer[i] <= '9')) {
								rssi = (input_buffer[i] - '0');
								rssi_valid = true;
							} else {	// invalid
								break;
							}
							k += 1;
							continue;
						} else {
							if ((input_buffer[i] >= '0') && (input_buffer[i] <= '9')) {
								rssi *= 10;
								rssi += (input_buffer[i] - '0');
								rssi_valid = true;
							} else {
								break;
							}
						}
					}
					if (rssi_valid) {
						rssi *= rssi_sign;
					}
				}

				if (rssi_valid && (received_pckt->pckt_len <= 1024)) {	// hand over the received packet
					received_pckt->rssi = rssi;
					block_fifo_write(&received_raw_packet_fifo, received_pckt, false);
				}
			} else {
				for (i=0; i<read_len; i++) {
					/*TODO*/
					// non-packet strings
				}
			}
		}
	} while (1);

	pthread_exit(NULL);
}

typedef struct {
	uint8_t * raw_buf;
	pckt_t * decoded_pckt;
	pthread_mutex_t mutex_A, mutex_B, mutex_C;
} dec_thread_arg_t;


void * rx_pckt_decode_lao40_ptfun(void *arguments)
{
	if (arguments == NULL) pthread_exit(NULL);
	dec_thread_arg_t * dtarg = (dec_thread_arg_t*)arguments;

	int8_t ao40_err[2];
	do {
		pthread_mutex_lock(&dtarg->mutex_C);
		pthread_mutex_lock(&dtarg->mutex_A);
		pthread_mutex_unlock(&dtarg->mutex_C);
		// long AO40
		memset(dtarg->decoded_pckt->pckt, 0, 256);
		dtarg->decoded_pckt->pckt_len = 256;
		ao40_decode_data(dtarg->raw_buf, dtarg->decoded_pckt->pckt, ao40_err);
		if ((ao40_err[0] < 0) || (ao40_err[1] < 0)) {	// decode error
			//nop
			dtarg->decoded_pckt->pckt_len = 0;
		} else {
			//pthread_exit(arguments);
			//block_fifo_write(&received_decoded_packet_fifo, dtarg->decoded_pckt, true);
		}
		pthread_mutex_unlock(&dtarg->mutex_A);
		pthread_mutex_lock(&dtarg->mutex_B);
		pthread_mutex_unlock(&dtarg->mutex_B);
	} while(1);

	pthread_exit(NULL);
}

void * rx_pckt_decode_sao40_ptfun(void *arguments)
{
	if (arguments == NULL) pthread_exit(NULL);
	dec_thread_arg_t * dtarg = (dec_thread_arg_t*)arguments;

	int8_t ao40_err[2];
	do {
		pthread_mutex_lock(&dtarg->mutex_C);
		pthread_mutex_lock(&dtarg->mutex_A);
		pthread_mutex_unlock(&dtarg->mutex_C);
		// short AO40
		memset(dtarg->decoded_pckt->pckt, 0, 128);
		dtarg->decoded_pckt->pckt_len = 128;
		ao40short_decode_data(dtarg->raw_buf, dtarg->decoded_pckt->pckt, ao40_err);
		if (ao40_err[0] < 0) {	// decode error
			//nop
			dtarg->decoded_pckt->pckt_len = 0;
		} else {
			//pthread_exit(arguments);
			//block_fifo_write(&received_decoded_packet_fifo, dtarg->decoded_pckt, true);
		}
		pthread_mutex_unlock(&dtarg->mutex_A);
		pthread_mutex_lock(&dtarg->mutex_B);
		pthread_mutex_unlock(&dtarg->mutex_B);
	} while(1);

	pthread_exit(NULL);
}

void * rx_pckt_decode_ra256_ptfun(void *arguments)
{
	if (arguments == NULL) pthread_exit(NULL);
	dec_thread_arg_t * dtarg = (dec_thread_arg_t*)arguments;

	do {
		pthread_mutex_lock(&dtarg->mutex_C);
		pthread_mutex_lock(&dtarg->mutex_A);
		pthread_mutex_unlock(&dtarg->mutex_C);
		memset(dtarg->decoded_pckt->pckt, 0, 256);
		dtarg->decoded_pckt->pckt_len = 256;
		ra_decode(dtarg->raw_buf, dtarg->decoded_pckt->pckt, 256);
		pthread_mutex_unlock(&dtarg->mutex_A);
		pthread_mutex_lock(&dtarg->mutex_B);
		pthread_mutex_unlock(&dtarg->mutex_B);
	} while(1);
	pthread_exit(arguments);
}

void * rx_pckt_decode_ra128_ptfun(void *arguments)
{
	if (arguments == NULL) pthread_exit(NULL);
	dec_thread_arg_t * dtarg = (dec_thread_arg_t*)arguments;

	do {
		pthread_mutex_lock(&dtarg->mutex_C);
		pthread_mutex_lock(&dtarg->mutex_A);
		pthread_mutex_unlock(&dtarg->mutex_C);
		memset(dtarg->decoded_pckt->pckt, 0, 128);
		dtarg->decoded_pckt->pckt_len = 128;
		ra_decode(dtarg->raw_buf, dtarg->decoded_pckt->pckt, 128);
		pthread_mutex_unlock(&dtarg->mutex_A);
		pthread_mutex_lock(&dtarg->mutex_B);
		pthread_mutex_unlock(&dtarg->mutex_B);
	} while(1);
	pthread_exit(arguments);
}


void * rx_pckt_proc_ptfun(void *arguments)
{
	int i,j,k, syncbe;
	uint8_t sync_x8, sync_mode, sync_speed;
	char * sync_mode_str;
	pckt_t * received_pckt = malloc(sizeof(pckt_t));
	uint8_t * ao40_raw = malloc(AO40_RAW_SIZE+256);
	int8_t ao40_err[2];

	pthread_t pt[4];
	dec_thread_arg_t dtarg[4];

	for (i=0; i<4; i++) {
		dtarg[i].decoded_pckt = malloc(sizeof(pckt_t));
		pthread_mutex_init( &dtarg[i].mutex_A, NULL );
		pthread_mutex_init( &dtarg[i].mutex_B, NULL );
		pthread_mutex_init( &dtarg[i].mutex_C, NULL );
		pthread_mutex_lock(&dtarg[i].mutex_A);
		//pthread_mutex_lock(&dtarg[i].mutex_B);
	}
	dtarg[0].raw_buf = received_pckt->pckt;
	dtarg[1].raw_buf = received_pckt->pckt;
	dtarg[2].raw_buf = ao40_raw;
	dtarg[3].raw_buf = ao40_raw;

	pthread_create(&pt[0], NULL, rx_pckt_decode_ra256_ptfun, (void*)(&dtarg[0]));
	pthread_create(&pt[1], NULL, rx_pckt_decode_ra128_ptfun, (void*)(&dtarg[1]));
	pthread_create(&pt[2], NULL, rx_pckt_decode_lao40_ptfun, (void*)(&dtarg[2]));
	pthread_create(&pt[3], NULL, rx_pckt_decode_sao40_ptfun, (void*)(&dtarg[3]));

	for (i=0; i<4; i++) {	// make sure mutex_C is locked in the working threads
		while (pthread_mutex_trylock(&dtarg[i].mutex_C) == 0) {
			pthread_mutex_unlock(&dtarg[i].mutex_C);
			sched_yield();
		}
	}

	void * ptretval = NULL;

	pckt_t * decoded_pckt = malloc(sizeof(pckt_t));

	do {
		if (block_fifo_read(&received_raw_packet_fifo, received_pckt) == 0) {	//blocking read
			if (received_pckt->pckt_len >= 650) {	// only DATA
				for (i=0; i<4; i++) dtarg[i].decoded_pckt->rssi = received_pckt->rssi;

				// make AO-40 raw; good for short & long decoder too
				k = 0;
				for (i=0; i<650; i++) {
					for (j = 0; j < 8; j++) {
						ao40_raw[k] = ((received_pckt->pckt[i] & (0x80 >> j)) ? 255 : 0);
						k += 1;
					}
				}

				for (i=0; i<4; i++) {	// start working threads
					pthread_mutex_lock(&dtarg[i].mutex_B);
					pthread_mutex_unlock(&dtarg[i].mutex_A);
				}

				for (i=3; i>=0; i--) {
				//for (i=0; i<4; i++) {
					pthread_mutex_lock(&dtarg[i].mutex_C);
					pthread_mutex_unlock(&dtarg[i].mutex_C);
					pthread_mutex_lock(&dtarg[i].mutex_A);
					pthread_mutex_unlock(&dtarg[i].mutex_B);
					if (dtarg[i].decoded_pckt->pckt_len > 0) {
						if (packet_check(dtarg[i].decoded_pckt->pckt, dtarg[i].decoded_pckt->pckt_len) == 0) block_fifo_write(&received_decoded_packet_fifo, dtarg[i].decoded_pckt, false);	// packet_check is low cost
					}
				}

			}
		}
	} while(1);

	pthread_exit(NULL);
}


void pckt_proc_DOWNLINK_PCKT_TYPE_TELEMETRY_1(pckt_t * pckt);
void pckt_proc_DOWNLINK_PCKT_TYPE_TELEMETRY_2(pckt_t * pckt);
void pckt_proc_DOWNLINK_PCKT_TYPE_TELEMETRY_3(pckt_t * pckt);
void pckt_proc_DOWNLINK_PCKT_TYPE_BEACON(pckt_t * pckt);
void pckt_proc_DOWNLINK_PCKT_TYPE_SPECTRUM_RESULT(pckt_t * pckt);
void pckt_proc_DOWNLINK_PCKT_TYPE_FILE_INFO(pckt_t * pckt);
void pckt_proc_DOWNLINK_PCKT_TYPE_FILE_FRAGMENT(pckt_t * pckt);
void pckt_proc_DOWNLINK_PCKT_TYPE_SMOGP_TELEMETRY_1(pckt_t * pckt);
void pckt_proc_DOWNLINK_PCKT_TYPE_SMOGP_TELEMETRY_2(pckt_t * pckt);
void pckt_proc_DOWNLINK_PCKT_TYPE_ATL_TELEMETRY_1(pckt_t * pckt);
void pckt_proc_DOWNLINK_PCKT_TYPE_ATL_TELEMETRY_2(pckt_t * pckt);
void pckt_proc_DOWNLINK_PCKT_TYPE_ATL_TELEMETRY_3(pckt_t * pckt);

void * rx_pckt_data_ptfun(void *arguments)
{
	int i,j,k, auth;
	char * auth_str;
	pckt_t * received_pckt = malloc(sizeof(pckt_t));

	do {
		if (block_fifo_read(&received_decoded_packet_fifo, received_pckt) == 0) {	//blocking read

			// check expected sizes
			switch (received_pckt->pckt[0]) {
				case DOWNLINK_PCKT_TYPE_TELEMETRY_1:
					if (received_pckt->pckt_len != 128) continue;
					break;
				case DOWNLINK_PCKT_TYPE_TELEMETRY_2:
					if (received_pckt->pckt_len != 128) continue;
					break;
				case DOWNLINK_PCKT_TYPE_TELEMETRY_3:
					if (received_pckt->pckt_len != 128) continue;
					break;
				case DOWNLINK_PCKT_TYPE_BEACON:
					if (received_pckt->pckt_len != 128) continue;
					break;
				case DOWNLINK_PCKT_TYPE_SPECTRUM_RESULT:
					if (received_pckt->pckt_len != 256) continue;
					break;
				case DOWNLINK_PCKT_TYPE_FILE_INFO:
					if (received_pckt->pckt_len != 128) continue;
					break;
				case DOWNLINK_PCKT_TYPE_FILE_FRAGMENT:
					if (received_pckt->pckt_len != 256) continue;
					break;
#ifdef BUILD_SMOGP
				case DOWNLINK_PCKT_TYPE_SMOGP_TELEMETRY_1:
					if (received_pckt->pckt_len != 128) continue;
					break;
				case DOWNLINK_PCKT_TYPE_SMOGP_TELEMETRY_2:
					if (received_pckt->pckt_len != 128) continue;
					break;
#endif
#ifdef BUILD_ATL1
				case DOWNLINK_PCKT_TYPE_ATL_TELEMETRY_1:
					if (received_pckt->pckt_len != 128) continue;
					break;
				case DOWNLINK_PCKT_TYPE_ATL_TELEMETRY_2:
					if (received_pckt->pckt_len != 128) continue;
					break;
				case DOWNLINK_PCKT_TYPE_ATL_TELEMETRY_3:
					if (received_pckt->pckt_len != 128) continue;
					break;
#endif
				default:	// invalid type
					continue;
					break;
			}

			// DATA PACKETS
			auth = packet_check(received_pckt->pckt, received_pckt->pckt_len);

			if (auth < 0) continue;	// 

			if (add_check_duplicate_signature(received_pckt->pckt, received_pckt->pckt_len)) continue;	// do not process packets with duplicate signature

			tsprintf("Received %db long packet with %d RSSI;\tAUTHENTICATION: not available\n", received_pckt->pckt_len, received_pckt->rssi);



			// save packet to json file
#ifdef BUILD_SMOGP
			save_pckt_to_json("smogp", received_pckt->pckt, received_pckt->pckt_len);
#endif
#ifdef BUILD_ATL1
			save_pckt_to_json("atl1",  received_pckt->pckt, received_pckt->pckt_len);
#endif
			switch (received_pckt->pckt[0]) {
				case DOWNLINK_PCKT_TYPE_TELEMETRY_1:
					pckt_proc_DOWNLINK_PCKT_TYPE_TELEMETRY_1(received_pckt);
					break;
				case DOWNLINK_PCKT_TYPE_TELEMETRY_2:
					pckt_proc_DOWNLINK_PCKT_TYPE_TELEMETRY_2(received_pckt);
					break;
				case DOWNLINK_PCKT_TYPE_TELEMETRY_3:
					pckt_proc_DOWNLINK_PCKT_TYPE_TELEMETRY_3(received_pckt);
					break;
				case DOWNLINK_PCKT_TYPE_BEACON:
					pckt_proc_DOWNLINK_PCKT_TYPE_BEACON(received_pckt);
					break;
				case DOWNLINK_PCKT_TYPE_SPECTRUM_RESULT:
					pckt_proc_DOWNLINK_PCKT_TYPE_SPECTRUM_RESULT(received_pckt);
					break;
				case DOWNLINK_PCKT_TYPE_FILE_INFO:
					pckt_proc_DOWNLINK_PCKT_TYPE_FILE_INFO(received_pckt);
					break;
				case DOWNLINK_PCKT_TYPE_FILE_FRAGMENT:
					pckt_proc_DOWNLINK_PCKT_TYPE_FILE_FRAGMENT(received_pckt);
					break;
#ifdef BUILD_SMOGP
				case DOWNLINK_PCKT_TYPE_SMOGP_TELEMETRY_1:
					pckt_proc_DOWNLINK_PCKT_TYPE_SMOGP_TELEMETRY_1(received_pckt);
					break;
				case DOWNLINK_PCKT_TYPE_SMOGP_TELEMETRY_2:
					pckt_proc_DOWNLINK_PCKT_TYPE_SMOGP_TELEMETRY_2(received_pckt);
					break;
#endif
#ifdef BUILD_ATL1
				case DOWNLINK_PCKT_TYPE_ATL_TELEMETRY_1:
					pckt_proc_DOWNLINK_PCKT_TYPE_ATL_TELEMETRY_1(received_pckt);
					break;
				case DOWNLINK_PCKT_TYPE_ATL_TELEMETRY_2:
					pckt_proc_DOWNLINK_PCKT_TYPE_ATL_TELEMETRY_2(received_pckt);
					break;
				case DOWNLINK_PCKT_TYPE_ATL_TELEMETRY_3:
					pckt_proc_DOWNLINK_PCKT_TYPE_ATL_TELEMETRY_3(received_pckt);
					break;
#endif
				//case DOWNLINK_PCKT_TYPE_RESERVED_SYNC:
				//	tsprintf("\tPacket type: Possible SYNC!!\n");
				//	break;
				default:
					tsprintf("\tInvalid packet type: %hhu!\n", received_pckt->pckt[0]);
					break;
			}
		}
	} while (1);

	pthread_exit(NULL);
}



#ifdef BUILD_SMOGP
static const char * const_mppt_panel_names[6] = {"Bottom", "Front", "Right", "Left", "Back", "Top"};
#endif
#ifdef BUILD_ATL1
static const char * const_mppt_panel_names[6] = {"Bottom", "Left", "Right", "Front", "Back", "Top"};
#endif

void pckt_proc_DOWNLINK_PCKT_TYPE_TELEMETRY_1(pckt_t * pckt)
{
	if (pckt == NULL) return;
	tsprintf("\tPacket type: \x1b[1m\x1b[32mTelemetry 1\n\x1b[0m");
	if (pckt->pckt_len < 128) {
		tsprintf("\tERROR: too short\n");
		return;
	}
	pcktprint_open("ct1");
	int pphex;
	for (pphex=0; pphex < pckt->pckt_len; pphex++) pcktonlyfileprintf("%02hhX", pckt->pckt[pphex]);
	pcktonlyfileprintf("\n");
	pcktonlyfileprintf("\tPacket type: Telemetry 1\n");

	int32_t i32;
	uint16_t u16;
	uint8_t u8;
	int i,p,n;
	struct tm tmp_tm;
	time_t tmp_time;
	char timestr[20];
	double tmpd;

	uint8_t * buf = pckt->pckt;


	p = 1;
	memcpy(&i32, &buf[p], 4);
	p += 4;

	tmp_time = i32;
	strftime(timestr, 20, "%Y-%m-%d %H:%M:%S", gmtime_r(&tmp_time, &tmp_tm));
	timestr[19] = 0;

	pcktprintf("\tTimestamp: %s\n", timestr);


	for (i=0; i<6; i++) {
		pcktprintf("\tMPPT-%d (%s):\n", i + 1, const_mppt_panel_names[i]);

		memcpy(&i32, &buf[p], 4);
		p += 4;

		tmp_time = i32;
		strftime(timestr, 20, "%Y-%m-%d %H:%M:%S", gmtime_r(&tmp_time, &tmp_tm));
		timestr[19] = 0;

		pcktprintf("\t\tTimestamp: %s\n", timestr);

		memcpy(&u16, &buf[p], 2);
		p += 2;
		tmpd = ((int16_t)u16) / 10.0;
		pcktprintf("\t\tTemperature: %.1fC°\n", tmpd);
		memcpy(&u16, &buf[p], 2);
		p += 2;
		tmpd = u16 / 1000.0;
		pcktprintf("\t\tLight sensor: %.3fV\n", tmpd);
		memcpy(&u16, &buf[p], 2);
		p += 2;
		pcktprintf("\t\tInput current: %humA\n", u16);
		memcpy(&u16, &buf[p], 2);
		p += 2;
		tmpd = u16 / 1000.0;
		pcktprintf("\t\tInput voltage: %.3fV\n", tmpd);
		memcpy(&u16, &buf[p], 2);
		p += 2;
		pcktprintf("\t\tOutput current: %humA\n", u16);
		memcpy(&u16, &buf[p], 2);
		p += 2;
		tmpd = u16 / 1000.0;
		pcktprintf("\t\tOutput voltage: %.3fV\n", tmpd);
		memcpy(&u8, &buf[p], 1);
		p += 1;
		pcktprintf("\t\tPanel number: %hhu\n", ((u8 >> 5) & 0x07));
		switch((u8 >> 3) & 0x03) {
			case 0: pcktprintf("\t\tAntenna status: closed\n");
				break;
			case 1: pcktprintf("\t\tAntenna status: open\n");
				break;
			case 2: pcktprintf("\t\tAntenna status: not monitored\n");
				break;
			case 3: pcktprintf("\t\tAntenna status: INVALID!\n");
				break;
		}
	}


	pcktprintf("\tACK-INFO: ");
	for (i=0; i<3; i++) {
		memcpy(&u16, &buf[p], 2);
		p += 2;
		memcpy(&u8, &buf[p], 1);
		p += 1;
		if (i == 0) {
			store_ackd_serial(u16, u8);
		}
		pcktprintf("%hu (%.0f dBm RSSI)\t", u16, helper_ACK_INFO_RSSI_CONVERT(u8));
	}
	pcktprintf("\n");

	pcktprint_close();
}

void pckt_proc_DOWNLINK_PCKT_TYPE_TELEMETRY_2(pckt_t * pckt)
{
	if (pckt == NULL) return;
	tsprintf("\tPacket type: \x1b[1m\x1b[32mTelemetry 2\n\x1b[0m");
	if (pckt->pckt_len < 128) {
		tsprintf("\tERROR: too short\n");
		return;
	}
	pcktprint_open("ct2");
	int pphex;
	for (pphex=0; pphex < pckt->pckt_len; pphex++) pcktonlyfileprintf("%02hhX", pckt->pckt[pphex]);
	pcktonlyfileprintf("\n");
	pcktonlyfileprintf("\tPacket type: Telemetry 2\n");

	int32_t i32;
	uint16_t u16;
	uint8_t u8;
	int i,p,n;
	struct tm tmp_tm;
	time_t tmp_time;
	char timestr[20];
	double tmpd;

	uint8_t * buf = pckt->pckt;



	p = 1;
	memcpy(&i32, &buf[p], 4);
	p += 4;

	tmp_time = i32;
	strftime(timestr, 20, "%Y-%m-%d %H:%M:%S", gmtime_r(&tmp_time, &tmp_tm));
	timestr[19] = 0;

	pcktprintf("\tTimestamp: %s\n", timestr);

	for (i=0; i<2; i++) {
		pcktprintf("\tPCU-%d DEP telemetry:\n", i+1);

		memcpy(&i32, &buf[p], 4);
		p += 4;

		tmp_time = i32;
		strftime(timestr, 20, "%Y-%m-%d %H:%M:%S", gmtime_r(&tmp_time, &tmp_tm));
		timestr[19] = 0;

		pcktprintf("\t\tTimestamp: %s\n", timestr);

		memcpy(&u8, &buf[p], 1);
		p += 1;
		pcktprintf("\t\tDeployment Switch 1 status: %d\n", (u8 & 0x80) ? 1 : 0);
		pcktprintf("\t\tDeployment Switch 2 status: %d\n", (u8 & 0x40) ? 1 : 0);
		pcktprintf("\t\tRemove Before Flight status: %d\n", (u8 & 0x20) ? 1 : 0);
		pcktprintf("\t\tPCU deployment status: %d\n", (u8 & 0x08) ? 1 : 0);
		pcktprintf("\t\tAntenna deployment status: %d\n", (u8 & 0x04) ? 1 : 0);

		memcpy(&u16, &buf[p], 2);
		p += 2;
		pcktprintf("\t\tPCU boot counter: %hu\n", u16);

		memcpy(&u16, &buf[p], 2);
		p += 2;
		pcktprintf("\t\tPCU uptime: %hu minutes\n", u16);
	}

	for (i=0; i<2; i++) {
		pcktprintf("\tPCU-%d SDC telemetry:\n", i+1);

		memcpy(&i32, &buf[p], 4);
		p += 4;

		tmp_time = i32;
		strftime(timestr, 20, "%Y-%m-%d %H:%M:%S", gmtime_r(&tmp_time, &tmp_tm));
		timestr[19] = 0;

		pcktprintf("\t\tTimestamp: %s\n", timestr);

		memcpy(&u16, &buf[p], 2);
		p += 2;
		pcktprintf("\t\tSDC1 input current: %humA\n", u16);

		memcpy(&u16, &buf[p], 2);
		p += 2;
		pcktprintf("\t\tSDC1 output current: %humA\n", u16);

		memcpy(&u16, &buf[p], 2);
		p += 2;
		tmpd = u16 / 1000.0;
		pcktprintf("\t\tSDC1 output voltage: %.3fV\n", tmpd);

		memcpy(&u16, &buf[p], 2);
		p += 2;
		pcktprintf("\t\tSDC2 input current: %humA\n", u16);

		memcpy(&u16, &buf[p], 2);
		p += 2;
		pcktprintf("\t\tSDC2 output current: %humA\n", u16);

		memcpy(&u16, &buf[p], 2);
		p += 2;
		tmpd = u16 / 1000.0;
		pcktprintf("\t\tSDC2 output voltage: %.3fV\n", tmpd);

		memcpy(&u8, &buf[p], 1);
		p += 1;
		pcktprintf("\t\tSDC1 current limiter overcurrent status: %d\n", (u8 & 0x80) ? 1 : 0);
		pcktprintf("\t\tSDC1 voltage limiter overvoltage status: %d\n", (u8 & 0x40) ? 1 : 0);
		pcktprintf("\t\tSDC2 current limiter overcurrent status: %d\n", (u8 & 0x20) ? 1 : 0);
		pcktprintf("\t\tSDC2 voltage limiter overvoltage status: %d\n", (u8 & 0x10) ? 1 : 0);
	}

	for (i=0; i<2; i++) {
#ifdef BUILD_ATL1
		pcktprintf("\tPCU-%d BAT telemetry: not used on ATL-1\n", i+1);
		p += 11;
#endif
#ifdef BUILD_SMOGP
		pcktprintf("\tPCU-%d BAT telemetry:\n", i+1);

		memcpy(&i32, &buf[p], 4);
		p += 4;

		tmp_time = i32;
		strftime(timestr, 20, "%Y-%m-%d %H:%M:%S", gmtime_r(&tmp_time, &tmp_tm));
		timestr[19] = 0;

		pcktprintf("\t\tTimestamp: %s\n", timestr);

		memcpy(&u16, &buf[p], 2);
		p += 2;
		tmpd = u16 / 1000.0;
		pcktprintf("\t\tBattery voltage: %.3fV\n", tmpd);

		memcpy(&u16, &buf[p], 2);
		p += 2;
		pcktprintf("\t\tBattery charge current: %humA\n", u16);

		memcpy(&u16, &buf[p], 2);
		p += 2;
		pcktprintf("\t\tBattery discharge current: %humA\n", u16);

		memcpy(&u8, &buf[p], 1);
		p += 1;
		pcktprintf("\t\tBattery charge overcurrent status: %d\n", (u8 & 0x80) ? 1 : 0);
		pcktprintf("\t\tBattery charge overvoltage status: %d\n", (u8 & 0x40) ? 1 : 0);
		pcktprintf("\t\tBattery discharge overcurrent status: %d\n", (u8 & 0x20) ? 1 : 0);
		pcktprintf("\t\tBattery discharge overvoltage status: %d\n", (u8 & 0x10) ? 1 : 0);
		pcktprintf("\t\tBattery charge enabled: %d\n", (u8 & 0x08) ? 1 : 0);
		pcktprintf("\t\tBattery discharge enabled: %d\n", (u8 & 0x04) ? 1 : 0);
#endif
	}

	for (i=0; i<2; i++) {
		pcktprintf("\tPCU-%d BUS telemetry:\n", i+1);

		memcpy(&i32, &buf[p], 4);
		p += 4;

		tmp_time = i32;
		strftime(timestr, 20, "%Y-%m-%d %H:%M:%S", gmtime_r(&tmp_time, &tmp_tm));
		timestr[19] = 0;

		pcktprintf("\t\tTimestamp: %s\n", timestr);

		memcpy(&u16, &buf[p], 2);
		p += 2;
		tmpd = u16 / 1000.0;
		//pcktprintf("\t\tUnregulated bus voltage: %.3fV\n", tmpd);
		pcktonlyfileprintf("\t\tUnregulated bus voltage: %.3fV\n", tmpd);
		tsprintf("\t\t\x1b[1m\x1b[33mUnregulated bus voltage: %.3fV\n\x1b[0m", tmpd);

		memcpy(&u16, &buf[p], 2);
		p += 2;
		tmpd = u16 / 1000.0;
		pcktprintf("\t\tRegulated bus voltage: %.3fV\n", tmpd);

		memcpy(&u16, &buf[p], 2);
		p += 2;
		pcktprintf("\t\tOBC 1 current consumption: %humA\n", u16);

		memcpy(&u16, &buf[p], 2);
		p += 2;
		pcktprintf("\t\tOBC 2 current consumption: %humA\n", u16);

		memcpy(&u8, &buf[p], 1);
		p += 1;
		pcktprintf("\t\tOBC 1 current limiter overcurrent status: %d\n", (u8 & 0x80) ? 1 : 0);
		pcktprintf("\t\tOBC 2 current limiter overcurrent status: %d\n", (u8 & 0x40) ? 1 : 0);
	}


	pcktprintf("\tACK-INFO: ");
	for (i=0; i<3; i++) {
		memcpy(&u16, &buf[p], 2);
		p += 2;
		memcpy(&u8, &buf[p], 1);
		p += 1;
		if (i == 0) {
			store_ackd_serial(u16, u8);
		}
		pcktprintf("%hu (%.0f dBm RSSI)\t", u16, helper_ACK_INFO_RSSI_CONVERT(u8));
	}
	pcktprintf("\n");

	pcktprint_close();
}

void pckt_proc_DOWNLINK_PCKT_TYPE_TELEMETRY_3(pckt_t * pckt)
{
	if (pckt == NULL) return;
	tsprintf("\tPacket type: \x1b[1m\x1b[32mTelemetry 3\n\x1b[0m");
	if (pckt->pckt_len < 128) {
		tsprintf("\tERROR: too short\n");
		return;
	}
	pcktprint_open("ct3");
	int pphex;
	for (pphex=0; pphex < pckt->pckt_len; pphex++) pcktonlyfileprintf("%02hhX", pckt->pckt[pphex]);
	pcktonlyfileprintf("\n");
	pcktonlyfileprintf("\tPacket type: Telemetry 3\n");

	int32_t i32;
	uint32_t u32;
	uint16_t u16;
	uint8_t u8;
	int i,p,n;
	struct tm tmp_tm;
	time_t tmp_time;
	char timestr[20];
	double tmpd;
	int16_t vector[3];

	uint8_t * buf = pckt->pckt;

	uint8_t curr_obc;



	p = 1;
	memcpy(&i32, &buf[p], 4);
	p += 4;

	tmp_time = i32;
	strftime(timestr, 20, "%Y-%m-%d %H:%M:%S", gmtime_r(&tmp_time, &tmp_tm));
	timestr[19] = 0;

	pcktprintf("\tTimestamp: %s\n", timestr);


	curr_obc = ((buf[p + 58] & 0x02) ? 1 : 0);	// peek forward

	memcpy(&u16, &buf[p], 2);
	p += 2;
	tmpd = u16 / 1000.0;
	pcktprintf("\tOBC supply voltage: %.3fV\n", tmpd);

	memcpy(&u16, &buf[p], 2);
	p += 2;
	if (curr_obc == 0) {
		pcktprintf("\tRTCC-1 Temperature: %hdC°\n", ((int16_t)u16));
	}

	memcpy(&u16, &buf[p], 2);
	p += 2;
	if (curr_obc == 1) {
		pcktprintf("\tRTCC-2 Temperature: %hdC°\n", ((int16_t)u16));
	}

	p += 2;	//OBC temperature not available

	memcpy(&u16, &buf[p], 2);
	p += 2;
	tmpd = ((int16_t)u16) / 10.0;
	pcktprintf("\tEPS2 panel A temperature 1: %.1fC°\n", tmpd);
	memcpy(&u16, &buf[p], 2);
	p += 2;
	tmpd = ((int16_t)u16) / 10.0;
	pcktprintf("\tEPS2 panel A temperature 2: %.1fC°\n", tmpd);

	memcpy(&u8, &buf[p], 1);
	p += 1;
	pcktprintf("\tCurrent COM data rate = %hhu, TX power level = %hhu\n", ((u8 >> 5) & 0x07), ((u8 >> 3) & 0x03));

	memcpy(&u16, &buf[p], 2);
	p += 2;
	pcktprintf("\tCurrent COM TX current consumption: %humA\n", u16);

	memcpy(&u16, &buf[p], 2);
	p += 2;
	pcktprintf("\tCurrent COM RX current consumption: %humA\n", u16);

	memcpy(&u8, &buf[p], 1);
	p += 1;
	pcktprintf("\tCOM 1 overcurrent status: %d\n", (u8 & 0x80) ? 1 : 0);
	pcktprintf("\tCOM 1 limiter switch status: %d\n", (u8 & 0x20) ? 1 : 0);
	pcktprintf("\tCOM 1 limiter switch override status: %d\n", (u8 & 0x08) ? 1 : 0);
	pcktprintf("\tCOM 2 overcurrent status: %d\n", (u8 & 0x40) ? 1 : 0);
	pcktprintf("\tCOM 2 limiter switch status: %d\n", (u8 & 0x10) ? 1 : 0);
	pcktprintf("\tCOM 2 limiter switch override status: %d\n", (u8 & 0x04) ? 1 : 0);

	if (curr_obc == 1) p += 20;	// if OBC2 skip first part

	memcpy(&vector[0], &buf[p], 2);
	p += 2;
	memcpy(&vector[1], &buf[p], 2);
	p += 2;
	memcpy(&vector[2], &buf[p], 2);
	p += 2;
	pcktprintf("\tMSEN Gyroscope: % 5.0f\t% 5.0f\t% 5.0f\n", 1.0 * vector[0], 1.0 * vector[1], 1.0 * vector[2]);

	memcpy(&vector[0], &buf[p], 2);
	p += 2;
	memcpy(&vector[1], &buf[p], 2);
	p += 2;
	memcpy(&vector[2], &buf[p], 2);
	p += 2;
	pcktprintf("\tMSEN Magneto:   % 5.0f\t% 5.0f\t% 5.0f\n", 1.0 * vector[0], 1.0 * vector[1], 1.0 * vector[2]);

	p += 6;	// accelerometer not used

	memcpy(&u16, &buf[p], 2);
	p += 2;
	tmpd = ((int16_t)u16) / 10.0;
	pcktprintf("\tMSEN Temperature: %.1fC°\n", tmpd);

	if (curr_obc == 0) p += 20;	// if OBC1 skip second part

	memcpy(&u8, &buf[p], 1);
	p += 1;
	if (curr_obc == 1) {
		pcktprintf("\tMSEN  functional: %d\n", (u8 & 0x40) ? 1 : 0);
		pcktprintf("\tFLASH functional: %d\n", (u8 & 0x10) ? 1 : 0);
		pcktprintf("\tRTCC  functional: %d\n", (u8 & 0x04) ? 1 : 0);
		pcktprintf("\t Current OBC: OBC-2\n");
	} else {
		pcktprintf("\tMSEN  functional: %d\n", (u8 & 0x80) ? 1 : 0);
		pcktprintf("\tFLASH functional: %d\n", (u8 & 0x20) ? 1 : 0);
		pcktprintf("\tRTCC  functional: %d\n", (u8 & 0x08) ? 1 : 0);
		pcktprintf("\tCurrent OBC: OBC-1\n");
	}

	pcktprintf("\tCurrent COM: COM-%d\n", (u8 & 0x01) ? 2 : 1);


	// COM telemetry
	memcpy(&i32, &buf[p], 4);
	p += 4;

	tmp_time = i32;
	strftime(timestr, 20, "%Y-%m-%d %H:%M:%S", gmtime_r(&tmp_time, &tmp_tm));
	timestr[19] = 0;

	pcktprintf("\tLast COM telemetry: %s\n", timestr);

	memcpy(&u8, &buf[p], 1);
	p += 1;
	pcktprintf("\tSWR bridge reading: %hhu\n", u8);

	memcpy(&u8, &buf[p], 1);
	p += 1;
	pcktprintf("\tLast received packet RSSI: %hhd\n", (int8_t)u8);

	memcpy(&u8, &buf[p], 1);
	p += 1;
	pcktprintf("\tSpectrum analyzer status: %hhu\n", u8);

	memcpy(&u16, &buf[p], 2);
	p += 2;
	tmpd = u16 / 1000.0;
	pcktprintf("\tActive COM voltage: %.3fV\n", tmpd);

	memcpy(&u16, &buf[p], 2);
	p += 2;
	tmpd = ((int16_t)u16) / 10.0;
	pcktprintf("\tActive COM temperature: %.1fC°\n", tmpd);

	memcpy(&u16, &buf[p], 2);
	p += 2;
	tmpd = ((int16_t)u16) / 10.0;
	pcktprintf("\tActive COM spectrum analyzer temperature: %.1fC°\n", tmpd);

#ifdef BUILD_ATL1
	p += 32;	// TID telemetry not available on ATL-1
#endif
#ifdef BUILD_SMOGP
	// TID telemetry	p += 32;
	for (i=0; i<2; i++) {
		memcpy(&i32, &buf[p], 4);
		p += 4;
		tmp_time = i32;
		strftime(timestr, 20, "%Y-%m-%d %H:%M:%S", gmtime_r(&tmp_time, &tmp_tm));
		timestr[19] = 0;
		pcktprintf("\tTID-%d measurement:\n", (i+1));
		pcktprintf("\t\tTimestamp: %s\n", timestr);
		memcpy(&u16, &buf[p], 2);
		p += 2;
		tmpd = ((int16_t)u16) / 10.0;
		pcktprintf("\t\tTemperature: %.1fC°\n", tmpd);
		memcpy(&u16, &buf[p], 2);
		p += 2;
		tmpd = u16 / 1000.0;
		pcktprintf("\t\tVoltage: %.3fV\n", tmpd);
		u32 = (buf[p] << 16) + (buf[p+1] << 8) + (buf[p+2] << 0);
		p += 3;
		pcktprintf("\t\tRadFET-1 voltage: %uuV\n", u32);
		u32 = (buf[p] << 16) + (buf[p+1] << 8) + (buf[p+2] << 0);
		p += 3;
		pcktprintf("\t\tRadFET-2 voltage: %uuV\n", u32);
		memcpy(&u16, &buf[p], 2);
		p += 2;
		pcktprintf("\t\tMeasurement serial number: %hu\n", u16);
	}
#endif

	pcktprintf("\tACK-INFO: ");
	for (i=0; i<3; i++) {
		memcpy(&u16, &buf[p], 2);
		p += 2;
		memcpy(&u8, &buf[p], 1);
		p += 1;
		if (i == 0) {
			store_ackd_serial(u16, u8);
		}
		pcktprintf("%hu (%.0f dBm RSSI)\t", u16, helper_ACK_INFO_RSSI_CONVERT(u8));
	}
	pcktprintf("\n");

	pcktprint_close();
}

void pckt_proc_DOWNLINK_PCKT_TYPE_BEACON(pckt_t * pckt)
{
	if (pckt == NULL) return;
	tsprintf("\tPacket type: \x1b[1m\x1b[32mBeacon\n\x1b[0m");
	if (pckt->pckt_len < 128) {
		tsprintf("\tERROR: too short\n");
		return;
	}
	pcktprint_open("bcn");
	int pphex;
	for (pphex=0; pphex < pckt->pckt_len; pphex++) pcktonlyfileprintf("%02hhX", pckt->pckt[pphex]);
	pcktonlyfileprintf("\n");
	pcktonlyfileprintf("\tPacket type: Beacon\n");


	int32_t i32;
	uint16_t u16;
	uint32_t u32;
	uint8_t u8;
	int i,p,n;
	struct tm tmp_tm;
	time_t tmp_time;
	char timestr[20];

	uint8_t * buf = pckt->pckt;

	char beacon_msg[81];


	p = 1;
	memcpy(&i32, &buf[p], 4);
	p += 4;

	tmp_time = i32;
	strftime(timestr, 20, "%Y-%m-%d %H:%M:%S", gmtime_r(&tmp_time, &tmp_tm));
	timestr[19] = 0;

	pcktprintf("\tTimestamp: %s\n", timestr);

	memcpy(beacon_msg, &buf[p], 80);
	p += 80;
	beacon_msg[80] = 0;
	pcktprintf("\tBeacon message: %s\n", beacon_msg);


	pcktprintf("\tUP-Link statistics:\n");

	memcpy(&i32, &buf[p], 4);
	p += 4;
	pcktprintf("\t\tValid packets: %d\n", i32);

	memcpy(&u16, &buf[p], 2);
	p += 2;
	pcktprintf("\t\tRX-Error / wrong size: %hu\n", u16);

	memcpy(&u16, &buf[p], 2);
	p += 2;
	pcktprintf("\t\tRX-Error / golay-failed: %hu\n", u16);

	memcpy(&u16, &buf[p], 2);
	p += 2;
	pcktprintf("\t\tRX-Error / wrong signature: %hu\n", u16);

	memcpy(&u16, &buf[p], 2);
	p += 2;
	pcktprintf("\t\tRX-Error / invalid serial: %hu\n", u16);

	memcpy(&u32, &buf[p], 4);
	p += 4;
	pcktprintf("\tOBC-COM TRX error statistic: %u\n", u32);


	pcktprintf("\tACK-INFO: ");
	for (i=0; i<3; i++) {
		memcpy(&u16, &buf[p], 2);
		p += 2;
		memcpy(&u8, &buf[p], 1);
		p += 1;
		if (i == 0) {
			store_ackd_serial(u16, u8);
		}
		pcktprintf("%hu (%.0f dBm RSSI)\t", u16, helper_ACK_INFO_RSSI_CONVERT(u8));
	}
	pcktprintf("\n");

	pcktprint_close();
}

typedef struct {
	int32_t timestamp;
	double startfreq;
	double stepfreq;
	uint8_t rbw;
	uint8_t pckt_index;
	uint8_t pckt_cnt;
	uint16_t measid;
	uint16_t spectrum_len;
	uint8_t * data;
} asap_spectrum_t;

void pckt_proc_DOWNLINK_PCKT_TYPE_SPECTRUM_RESULT(pckt_t * pckt)
{
	static asap_spectrum_t * asap_spectrum = NULL;

	if (pckt == NULL) return;
	tsprintf("\tPacket type: \x1b[1m\x1b[32mSpectrum result\n\x1b[0m");
	if (pckt->pckt_len < 256) {
		tsprintf("\tERROR: too short\n");
		return;
	}
	pcktprint_open("spf");
	int pphex;
	for (pphex=0; pphex < pckt->pckt_len; pphex++) pcktonlyfileprintf("%02hhX", pckt->pckt[pphex]);
	pcktonlyfileprintf("\n");
	pcktonlyfileprintf("\tPacket type: Spectrum result\n");


	int32_t i32;
	uint16_t u16;
	uint8_t u8;
	int i,p,n;
	struct tm tmp_tm;
	time_t tmp_time;
	char timestr[20];

	uint32_t freqs[2];
	uint8_t rbw, pckt_cnt, pckt_index;
	uint16_t spectrum_len, measid;
	double tmpd;
	double startfreq, stopfreq,stepfreq;

	uint8_t * buf = pckt->pckt;


	p = 1;
	memcpy(&i32, &buf[p], 4);
	p += 4;

	tmp_time = i32;
	strftime(timestr, 20, "%Y-%m-%d %H:%M:%S", gmtime_r(&tmp_time, &tmp_tm));
	timestr[19] = 0;

	pcktprintf("\tTimestamp: %s\n", timestr);

	memcpy(&freqs[0], &buf[p], 4);
	p += 4;
	memcpy(&freqs[1], &buf[p], 4);
	p += 4;
	memcpy(&rbw, &buf[p], 1);
	p += 1;
	memcpy(&pckt_index, &buf[p], 1);
	p += 1;
	memcpy(&pckt_cnt, &buf[p], 1);
	p += 1;
	memcpy(&spectrum_len, &buf[p], 2);
	p += 2;

	p += 2;

	memcpy(&measid, &buf[p], 2);
	p += 2;

	startfreq = freqs[0];
	stepfreq = freqs[1];
	startfreq += 224.0 * pckt_index * stepfreq;
	stopfreq = startfreq + (stepfreq * spectrum_len);

	pcktprintf("\tMeasured %hu samples between %.6fMHz ... %.6fMHz with %.3fkHz stepsize and %hhu RBW\n", spectrum_len, startfreq / 1000000.0, stopfreq / 1000000.0, stepfreq / 1000.0, rbw);
	pcktprintf("\tSending fragment %hhu/%hhu of measurement \"%hu\"...\n", pckt_index, pckt_cnt, measid);

	pcktprint_close();

	if (asap_spectrum == NULL) {
		if (pckt_index == 0) {
			asap_spectrum = calloc(1, sizeof(asap_spectrum_t));
			if (asap_spectrum == NULL) goto spectrum_error;
			asap_spectrum->timestamp = tmp_time;
			asap_spectrum->startfreq = freqs[0];
			asap_spectrum->stepfreq = freqs[1];
			asap_spectrum->rbw = rbw;
			asap_spectrum->pckt_index = 0;
			asap_spectrum->pckt_cnt = pckt_cnt;
			asap_spectrum->measid = measid;
			asap_spectrum->spectrum_len = 0;
			asap_spectrum->data = malloc(pckt_cnt * 224);
			if (asap_spectrum->data == NULL) goto spectrum_error;
			for (i=0; i<(pckt_cnt * 224); i++) asap_spectrum->data[i] = 0;
		}
	} else {
		if (pckt_index == 0) {	// ?? -> discard
			free(asap_spectrum->data);
			free(asap_spectrum);
			asap_spectrum = calloc(1, sizeof(asap_spectrum_t));
			if (asap_spectrum == NULL) goto spectrum_error;
			asap_spectrum->timestamp = tmp_time;
			asap_spectrum->startfreq = freqs[0];
			asap_spectrum->stepfreq = freqs[1];
			asap_spectrum->rbw = rbw;
			asap_spectrum->pckt_index = 0;
			asap_spectrum->pckt_cnt = pckt_cnt;
			asap_spectrum->measid = measid;
			asap_spectrum->spectrum_len = 0;
			asap_spectrum->data = malloc(pckt_cnt * 224);
			if (asap_spectrum->data == NULL) goto spectrum_error;
			for (i=0; i<(pckt_cnt * 224); i++) asap_spectrum->data[i] = 0;
		} else {
			if (asap_spectrum->timestamp != tmp_time) goto spectrum_error;
			if (asap_spectrum->startfreq != freqs[0]) goto spectrum_error;
			if (asap_spectrum->stepfreq != freqs[1]) goto spectrum_error;
			if (asap_spectrum->rbw != rbw) goto spectrum_error;
			if (asap_spectrum->pckt_cnt != pckt_cnt) goto spectrum_error;
			if (asap_spectrum->measid != measid) goto spectrum_error;
		}
	}

	if (asap_spectrum == NULL) goto spectrum_error;
	if (asap_spectrum->data == NULL) goto spectrum_error;

	asap_spectrum->pckt_index = pckt_index;
	if (pckt_index >= pckt_cnt) goto spectrum_error;

	asap_spectrum->spectrum_len = pckt_index * 224 + spectrum_len;

	if (asap_spectrum->spectrum_len > (asap_spectrum->pckt_cnt * 224)) goto spectrum_error;
	if (spectrum_len > 224) goto spectrum_error;

	memcpy(&asap_spectrum->data[pckt_index * 224], &buf[22], spectrum_len);

	if (pckt_index == (pckt_cnt - 1)) {	// done!
		char tmp_str[256];
		int tmp_str_len;
		strftime(timestr, 20, "%Y%m%d%H%M%S", gmtime_r(&tmp_time, &tmp_tm));
		sprintf(tmp_str, "spectrum/asap_spectrum_analysis_%hu_%s.csv", measid, timestr);
		FILE * fsa = fopen(tmp_str, "w+");
		if (fsa != NULL) {
			strftime(timestr, 20, "%Y-%m-%d %H:%M:%S", gmtime_r(&tmp_time, &tmp_tm));
			timestr[19] = 0;
			tmp_str_len = sprintf(tmp_str, "Timestamp;%s\n", timestr);
			if (tmp_str_len < 0) goto spectrum_error;
			fwrite(tmp_str, 1, tmp_str_len, fsa);

			tmp_str_len = sprintf(tmp_str, "ID;%hu\nRBW;%hhu\nFrequency;Amplitude\n", asap_spectrum->measid, asap_spectrum->rbw);
			if (tmp_str_len < 0) goto spectrum_error;
			fwrite(tmp_str, 1, tmp_str_len, fsa);

			for (i=0; i<asap_spectrum->spectrum_len; i++) {
				tmp_str_len = sprintf(tmp_str, "%f;%hhu\n", asap_spectrum->startfreq + i*asap_spectrum->stepfreq, asap_spectrum->data[i]);
				if (tmp_str_len < 0) goto spectrum_error;
				fwrite(tmp_str, 1, tmp_str_len, fsa);
			}

			fclose(fsa);
		} else {
			tsprintf("ERROR: can not open \"%s\"\n", tmp_str);
		}

		free(asap_spectrum->data);
		free(asap_spectrum);
		asap_spectrum = NULL;
	}

	return;

   spectrum_error:
	if (asap_spectrum != NULL) {
		free(asap_spectrum->data);
		free(asap_spectrum);
	}
	asap_spectrum = NULL;
	return;
}

void pckt_proc_DOWNLINK_PCKT_TYPE_FILE_INFO(pckt_t * pckt)
{
	if (pckt == NULL) return;
	tsprintf("\tPacket type: \x1b[1m\x1b[32mFile Info\n\x1b[0m");
	if (pckt->pckt_len < 128) {
		tsprintf("\tERROR: too short\n");
		return;
	}
	pcktprint_open("fin");
	int pphex;
	for (pphex=0; pphex < pckt->pckt_len; pphex++) pcktonlyfileprintf("%02hhX", pckt->pckt[pphex]);
	pcktonlyfileprintf("\n");
	pcktonlyfileprintf("\tPacket type: File Info\n");


	int32_t i32;
	uint16_t u16;
	uint8_t u8;
	int i,p,n;
	struct tm tmp_tm;
	time_t tmp_time;
	char timestr[20];

	uint8_t * buf = pckt->pckt;

	uint8_t file_id, file_type;
	uint16_t page_addr;
	uint32_t file_size;
	char file_name[11];

	p = 1;
	memcpy(&i32, &buf[p], 4);
	p += 4;

	tmp_time = i32;
	strftime(timestr, 20, "%Y-%m-%d %H:%M:%S", gmtime_r(&tmp_time, &tmp_tm));
	timestr[19] = 0;

	pcktprintf("\tTimestamp: %s\n", timestr);

	for (i=0; i<5; i++) {
		memcpy(&file_id, &buf[p], 1);
		p += 1;
		memcpy(&file_type, &buf[p], 1);
		p += 1;
		memcpy(&page_addr, &buf[p], 2);
		p += 2;

		file_size = 0;
		memcpy(&u8, &buf[p], 1);
		p += 1;
		file_size += u8;
		file_size <<= 8;
		memcpy(&u8, &buf[p], 1);
		p += 1;
		file_size += u8;
		file_size <<= 8;
		memcpy(&u8, &buf[p], 1);
		p += 1;
		file_size += u8;

		memcpy(&i32, &buf[p], 4);
		p += 4;
		tmp_time = i32;
		strftime(timestr, 20, "%Y-%m-%d %H:%M:%S", gmtime_r(&tmp_time, &tmp_tm));
		timestr[19] = 0;

		memcpy(file_name, &buf[p], 10);
		p += 10;
		file_name[10] = 0;

		pcktprintf("\tFile ID=%hhu, type=%hhu, name=\"%s\" size=%u, starting at page %hu. Timestamp: %s\n", file_id, file_type, file_name, file_size, page_addr, timestr);
	}

	pcktprint_close();
}

void pckt_proc_DOWNLINK_PCKT_TYPE_FILE_FRAGMENT(pckt_t * pckt)
{
	if (pckt == NULL) return;
	tsprintf("\tPacket type: \x1b[1m\x1b[32mFile Fragment\n\x1b[0m");
	if (pckt->pckt_len < 256) {
		tsprintf("\tERROR: too short\n");
		return;
	}
	pcktprint_open("dlf");
	int pphex;
	for (pphex=0; pphex < pckt->pckt_len; pphex++) pcktonlyfileprintf("%02hhX", pckt->pckt[pphex]);
	pcktonlyfileprintf("\n");
	pcktonlyfileprintf("\tPacket type: File Fragment\n");


	int32_t i32;
	uint16_t u16;
	uint8_t u8;
	int i,p,n,c;
	struct tm tmp_tm;
	time_t tmp_time;
	char timestr[20];

	uint8_t * buf = pckt->pckt;

	uint16_t pckt_index, pckt_cnt;
	uint8_t file_type;
	uint16_t page_addr;
	uint32_t file_size;
	char file_name[11];

	FILE * file_shadow = NULL;
	FILE * file_content = NULL;
	char file_shadow_name[128];
	char file_content_name[128];

	char file_shadow_content[256];

	p = 1;
	memcpy(&i32, &buf[p], 4);
	p += 4;

	tmp_time = i32;
	strftime(timestr, 20, "%Y-%m-%d %H:%M:%S", gmtime_r(&tmp_time, &tmp_tm));
	timestr[19] = 0;

	pcktprintf("\tTimestamp: %s\n", timestr);

	memcpy(&pckt_index, &buf[p], 2);
	p += 2;
	memcpy(&pckt_cnt, &buf[p], 2);
	p += 2;
	memcpy(&file_type, &buf[p], 1);
	p += 1;
	memcpy(&page_addr, &buf[p], 2);
	p += 2;

	file_size = 0;
	memcpy(&u8, &buf[p], 1);
	p += 1;
	file_size += u8;
	file_size <<= 8;
	memcpy(&u8, &buf[p], 1);
	p += 1;
	file_size += u8;
	file_size <<= 8;
	memcpy(&u8, &buf[p], 1);
	p += 1;
	file_size += u8;

	memcpy(&i32, &buf[p], 4);
	p += 4;
	tmp_time = i32;
	strftime(timestr, 20, "%Y-%m-%d %H:%M:%S", gmtime_r(&tmp_time, &tmp_tm));
	timestr[19] = 0;

	memcpy(file_name, &buf[p], 10);
	p += 10;
	file_name[10] = 0;

	pcktprintf("\tDownloading %hu/%hu fragment of the file \"%s\" with type=%hhu, size=%u at page %hu. Timestamp: %s\n", pckt_index, pckt_cnt, file_name, file_type, file_size, page_addr, timestr);

	if ((pckt_cnt == 0) || (pckt_index >= pckt_cnt) || ((file_size / 217) > pckt_cnt)) {
		pcktprintf("\n\tERROR: inconsistent file parameter(s)!\n");
		pcktprint_close();
		return;
	}

	pcktprint_close();





	strftime(timestr, 20, "%Y%m%d%H%M%S", gmtime_r(&tmp_time, &tmp_tm));

	if (strlen(file_name) == 0) {
		sprintf(file_shadow_name, "partial/.noname_%02hX_%s.dat", page_addr / 128, timestr);
		sprintf(file_content_name, "partial/noname_%02hX_%s.dat", page_addr / 128, timestr);
	} else {
		sprintf(file_shadow_name, "partial/.%s_%02hX_%s.dat", file_name, page_addr / 128, timestr);
		sprintf(file_content_name, "partial/%s_%02hX_%s.dat", file_name, page_addr / 128, timestr);
	}

	file_shadow = fopen(file_shadow_name, "rb+");
	file_content = fopen(file_content_name, "rb+");

	if (file_shadow == NULL) {	// new file must be created
		if (file_content != NULL) goto fileop_error;		// neither or both can be NULL (NULL == nonexistent)

		file_shadow = fopen(file_shadow_name, "wb+");
		file_content = fopen(file_content_name, "wb+");

		for (i=0; i<pckt_cnt; i++) file_shadow_content[i] = ' ';

		fwrite(file_shadow_content, 1, pckt_cnt, file_shadow);

		fseek(file_content, file_size - 1, SEEK_SET);	// expand file allocation
		fputc('\0', file_content);

	}

	if ((file_shadow == NULL) || (file_content == NULL)) goto fileop_error;		// at this point both file should be opened

	fseek(file_shadow, pckt_index, SEEK_SET);
	c = fgetc(file_shadow);
	if (c < 0) goto fileop_error;
	if ((c != ' ') && (c != '=')) goto fileop_error;

	fseek(file_shadow, pckt_index, SEEK_SET);
	fputc('=', file_shadow);

	fseek(file_content, pckt_index * 217, SEEK_SET);

	int block_size;

	if (pckt_index < (pckt_cnt - 1)) {	// not the last block
		block_size = 217;
	} else {	// last block, maybe truncated...
		block_size = 217 - ((217 * pckt_cnt) - file_size);
		if ((block_size < 0) || (block_size > 217)) goto fileop_error;
	}

	fwrite(&buf[29], 1, block_size, file_content);

	fseek(file_shadow, 0, SEEK_SET);

	for (i=0; i<pckt_cnt; i++) {
		c = fgetc(file_shadow);
		if (c < 0) goto fileop_error;
		if (c == ' ') break;
	}

	fclose(file_shadow);
	fclose(file_content);

	if (i == pckt_cnt) {	// complete file
		tsprintf("Download of \"%s\" completed!\n", file_name);
		remove(file_shadow_name);	// delete shadow file
		memcpy(file_shadow_name, "download/", 9);	// reuse string: replace "partial/." to "download/"
		rename(file_content_name, file_shadow_name);
	}

	return;
   fileop_error:
	if (file_shadow != NULL) fclose(file_shadow);
	if (file_content != NULL) fclose(file_content);
	tsprintf("ERROR: file operation failed for download\n");
	return;
}



char * ET1_local_function_mppt_bus_state_interpretter(int8_t x)
{
	switch (x) {
		case 0: return "no data available yet";
		case 1: return "valid data";
		case -1: return "channel number mismatch";
		case -2: return "checksum error";
		case -3: return "no response";
		case -4: return "BUS ERROR";
		default: return "invalid status value!";
	}
}

char * ET1_local_function_pcudoz_bus_state_interpretter(int8_t x)
{
	switch (x) {
		case 0: return "unknown";
		case 1: return "OK";
		case -1: return "bit error";
		case -2: return "no response";
		case -3: return "BUS ERROR";
		default: return "invalid status value!";
	}
}




void pckt_proc_DOWNLINK_PCKT_TYPE_SMOGP_TELEMETRY_1(pckt_t * pckt)
{
	if (pckt == NULL) return;
	tsprintf("\tPacket type: \x1b[1m\x1b[32mSMOGP-Telemetry 1\n\x1b[0m");
	if (pckt->pckt_len < 128) {
		tsprintf("\tERROR: too short\n");
		return;
	}
	pcktprint_open("bt1");
	int pphex;
	for (pphex=0; pphex < pckt->pckt_len; pphex++) pcktonlyfileprintf("%02hhX", pckt->pckt[pphex]);
	pcktonlyfileprintf("\n");
	pcktonlyfileprintf("\tPacket type: SMOGP-Telemetry 1\n");


	int32_t i32;
	uint16_t u16;
	uint8_t u8;
	int i,p,n;
	struct tm tmp_tm;
	time_t tmp_time;
	char timestr[20];
	double tmpd;

	uint8_t * buf = pckt->pckt;



	p = 1;

	memcpy(&i32, &buf[p], 4);
	p += 4;
	pcktprintf("\tUptime: %d\n", i32);

	memcpy(&i32, &buf[p], 4);
	p += 4;

	tmp_time = i32;
	strftime(timestr, 20, "%Y-%m-%d %H:%M:%S", gmtime_r(&tmp_time, &tmp_tm));
	timestr[19] = 0;

	pcktprintf("\tSystem time: %s\n", timestr);

	memcpy(&u8, &buf[p], 1);
	p += 1;
	if (u8 == '0') pcktprintf("\tOBC ID: 1\n");
	else if (u8 == '1') pcktprintf("\tOBC ID: 2\n");
	else pcktprintf("\tOBC ID: invalid!\n");

	memcpy(&u8, &buf[p], 1);
	p += 1;
	if (u8 == 'E') pcktprintf("\tOscillator: external\n");
	else if (u8 == 'I') pcktprintf("\tOscillator: internal\n");
	else pcktprintf("\tOscillator: invalid!\n");

	memcpy(&u8, &buf[p], 1);
	p += 1;
	if (u8 == 'V') {
		memcpy(&i32, &buf[p], 4);
		p += 4;

		tmp_time = i32;
		strftime(timestr, 20, "%Y-%m-%d %H:%M:%S", gmtime_r(&tmp_time, &tmp_tm));
		timestr[19] = 0;

		pcktprintf("\tADC results at %s:\n", timestr);

		pcktprintf("\tADC results with internal reference:\n");
			memcpy(&u16, &buf[p], 2);
			p += 2;
			tmpd = u16 / 1000.0;
			pcktprintf("\t\tVcc: %.3fV\n", tmpd);
			memcpy(&u16, &buf[p], 2);
			p += 2;
			tmpd = u16 / 1000.0;
			pcktprintf("\t\tVbg: %.3fV\n", tmpd);
			memcpy(&u16, &buf[p], 2);
			p += 2;
			tmpd = u16 / 1000.0;
			pcktprintf("\t\tVcore: %.3fV\n", tmpd);
			memcpy(&u16, &buf[p], 2);
			p += 2;
			tmpd = u16 / 1000.0;
			pcktprintf("\t\tVextref: %.3fV\n", tmpd);
			memcpy(&u16, &buf[p], 2);
			p += 2;
			tmpd = u16 / 1000.0;
			pcktprintf("\t\tAN1: %.3fV\n", tmpd);
			memcpy(&u16, &buf[p], 2);
			p += 2;
			tmpd = u16 / 1000.0;
			pcktprintf("\t\tAN2: %.3fV\n", tmpd);
			memcpy(&u16, &buf[p], 2);
			p += 2;
			tmpd = u16 / 1000.0;
			pcktprintf("\t\tAN3: %.3fV\n", tmpd);
			memcpy(&u16, &buf[p], 2);
			p += 2;
			tmpd = u16 / 1000.0;
			pcktprintf("\t\tAN4: %.3fV\n", tmpd);

		pcktprintf("\tADC results with external reference:\n");
			memcpy(&u16, &buf[p], 2);
			p += 2;
			tmpd = u16 / 1000.0;
			pcktprintf("\t\tVcc: %.3fV\n", tmpd);
			memcpy(&u16, &buf[p], 2);
			p += 2;
			tmpd = u16 / 1000.0;
			pcktprintf("\t\tVbg: %.3fV\n", tmpd);
			memcpy(&u16, &buf[p], 2);
			p += 2;
			tmpd = u16 / 1000.0;
			pcktprintf("\t\tVcore: %.3fV\n", tmpd);
			memcpy(&u16, &buf[p], 2);
			p += 2;
			tmpd = u16 / 1000.0;
			pcktprintf("\t\tVextref: %.3fV\n", tmpd);
			memcpy(&u16, &buf[p], 2);
			p += 2;
			tmpd = u16 / 1000.0;
			pcktprintf("\t\tAN1: %.3fV\n", tmpd);
			memcpy(&u16, &buf[p], 2);
			p += 2;
			tmpd = u16 / 1000.0;
			pcktprintf("\t\tAN2: %.3fV\n", tmpd);
			memcpy(&u16, &buf[p], 2);
			p += 2;
			tmpd = u16 / 1000.0;
			pcktprintf("\t\tAN3: %.3fV\n", tmpd);
			memcpy(&u16, &buf[p], 2);
			p += 2;
			tmpd = u16 / 1000.0;
			pcktprintf("\t\tAN4: %.3fV\n", tmpd);
	} else {
		pcktprintf("\tADC results: not valid!\n");
		p += 37;
	}

	memcpy(&u8, &buf[p], 1);
	p += 1;
	if (u8 & 0x80) pcktprintf("\tSPI-MUX status: functional\n");
	else pcktprintf("\tSPI-MUX status: not functional\n");
	pcktprintf("\t\tMSEN CS pin select: %s\n", ((u8 & 0x40) ? "FAILED" : "OK"));
	pcktprintf("\t\tRTCC CS pin select: %s\n", ((u8 & 0x20) ? "FAILED" : "OK"));
	pcktprintf("\t\tFLASH CS pin select: %s\n", ((u8 & 0x10) ? "FAILED" : "OK"));
	pcktprintf("\t\tAll CS pin deselect: %s\n", ((u8 & 0x08) ? "FAILED" : "OK"));
	pcktprintf("\t\tMISO pin test: %s\n", ((u8 & 0x04) ? "FAILED" : "OK"));
	pcktprintf("\t\tMOSI pin test: %s\n", ((u8 & 0x02) ? "FAILED" : "OK"));
	pcktprintf("\t\tSCLK pin test: %s\n", ((u8 & 0x01) ? "FAILED" : "OK"));

	memcpy(&u8, &buf[p], 1);
	p += 1;
	if (u8 == 0) pcktprintf("\tSPI/FLASH status: not functional\n");
	else pcktprintf("\tSPI/FLASH status: functional, startcount == %hhu\n", u8);

	memcpy(&u8, &buf[p], 1);
	p += 1;
	if (u8 == 0) pcktprintf("\tSPI/MSEN status: not functional\n");
	else pcktprintf("\tSPI/MSEN status: functional, startcount == %hhu\n", u8);

	memcpy(&u8, &buf[p], 1);
	p += 1;
	if (u8 == 0) pcktprintf("\tSPI/RTCC status: not functional\n");
	else pcktprintf("\tSPI/RTCC status: functional, startcount == %hhu\n", u8);

	memcpy(&u8, &buf[p], 1);
	p += 1;
	pcktprintf("\tRandom number: %02hhX\n", u8);

	for (i=0; i<6; i++) {
		memcpy(&u8, &buf[p], 1);
		p += 1;
		pcktprintf("\tMPPT-%d (%s) bus status: %s\n", (i+1), const_mppt_panel_names[i], ET1_local_function_mppt_bus_state_interpretter((int8_t)u8) );
	}

	for (i=0; i<2; i++) {
		memcpy(&u8, &buf[p], 1);
		p += 1;
		pcktprintf("\tTID-%d bus status: %s\n", (i+1), ET1_local_function_pcudoz_bus_state_interpretter((int8_t)u8) );
	}

	for (i=0; i<2; i++) {
		memcpy(&u8, &buf[p], 1);
		p += 1;
		pcktprintf("\tPCU-%d bus status: %s\n", (i+1), ET1_local_function_pcudoz_bus_state_interpretter((int8_t)u8) );
	}

	memcpy(&u8, &buf[p], 1);
	p += 1;
	pcktprintf("\tCurrent COM: COM-%hhu\n", u8);

	memcpy(&i32, &buf[p], 4);
	p += 4;
	pcktprintf("\tCOM uptime: %d seconds\n", i32);


	static const int const_tx_pwr_level_to_mw[] = {10, 11, 12, 13, 14, 15, 16, 25, 29, 33, 38, 42, 46, 50, 75, 100};

	memcpy(&u8, &buf[p], 1);
	p += 1;
	if (u8 > 15) {
		pcktprintf("\tCOM-TX power level: INVALID (%hhu)\n", u8);
	} else {
		pcktprintf("\tCOM-TX power level: %dmW (%hhu)\n", const_tx_pwr_level_to_mw[u8], u8);
	}

	memcpy(&u16, &buf[p], 2);
	p += 2;
	pcktprintf("\tCOM-TX Current: %humA\n", u16);

	memcpy(&u16, &buf[p], 2);
	p += 2;
	pcktprintf("\tCOM-RX Current: %humA\n", u16);

	memcpy(&u16, &buf[p], 2);
	p += 2;
	tmpd = u16 / 1000.0;
	pcktprintf("\tCOM-TX Voltage Drop: %.3fV\n", tmpd);

	memcpy(&u16, &buf[p], 2);
	p += 2;
	pcktprintf("\tScheduled Spectrum Analysis queue: %hu request(s)\n", u16);

	memcpy(&u16, &buf[p], 2);
	p += 2;
	pcktprintf("\tScheduled File Download queue: %hu request(s)\n", u16);


	int32_t emm_rc, emm_sleep;
	uint8_t emm_mp;
	memcpy(&u8, &buf[p], 1);
	p += 1;
	memcpy(&emm_mp, &buf[p], 1);
	p += 1;
	memcpy(&emm_rc, &buf[p], 4);
	p += 4;
	memcpy(&emm_sleep, &buf[p], 4);
	p += 4;
	switch(u8) {
		case 0:	tsprintf("\x1b[1m\x1b[32m");	// bold green
			pcktprintf("\tEnergy Management Mode: normal (morse period: %hhu; radio cycle: %.1fs; sleep: 0)", emm_mp, emm_rc/1e6);
			break;
		case 1:	tsprintf("\x1b[1m\x1b[32m");	// bold green
			pcktprintf("\tEnergy Management Mode: normal, reduced (morse period: %hhu; radio cycle: %.1fs; sleep: %.1fs)", emm_mp, emm_rc/1e6, emm_sleep/1e6);
			break;
		case 2:	tsprintf("\x1b[1m\x1b[33m");	// bold yellow
			pcktprintf("\tEnergy Management Mode: energy saving (NO morse; radio cycle: %.1fs; sleep: %.1fs", emm_rc/1e6, emm_sleep/1e6);
			break;
		case 3:	tsprintf("\x1b[1m\x1b[101m");	// red background
			pcktprintf("\tEnergy Management Mode: EMERGENCY (NO morse; radio cycle: %.1fs; sleep: %.1fs", emm_rc/1e6, emm_sleep/1e6);
			break;
		default: tsprintf("\x1b[1m\x1b[101m");	// red background
			pcktprintf("\tEnergy Management Mode: INVALID!");
			break;
	}
	tsprintf("\x1b[0m");
	pcktprintf("\n");

	memcpy(&i32, &buf[p], 4);
	p += 4;
	if (i32 == -1) {
		pcktprintf("\tLast Telecommand: none received yet!\n");
	} else {
		pcktprintf("\tLast Telecommand: %d seconds ago\n", i32);
	}

	memcpy(&u16, &buf[p], 2);
	p += 2;
	pcktprintf("\tAutomatic Antenna Opening Attempts: %hu\n", u16);

	int32_t cpui, cpuw;
	memcpy(&u16, &buf[p], 2);
	p += 2;
	memcpy(&cpui, &buf[p], 4);
	p += 4;
	memcpy(&cpuw, &buf[p], 4);
	p += 4;
	pcktprintf("\tCPU usage: %uus iddle, %uus work over %hu cycles\n", cpui, cpuw, u16);
	double load, cyc;
	load = (double)cpui / ((double)cpui + (double)cpuw);
	cyc = u16;
	pcktprintf("\t\tLoad: %.1f%%\n\t\tIddle/c: %.3fus\n\t\tWork/c: %.3fus\n", load * 100.0, cpui/cyc, cpuw/cyc);

	uint32_t chksum, chksum_prev_diff;
	memcpy(&chksum, &buf[p], 4);
	p += 4;
	memcpy(&chksum_prev_diff, &buf[p], 4);
	p += 4;
	pcktprintf("\tOBC-FLASH checksum: %08X (%08X)\n", chksum, chksum_prev_diff);

	memcpy(&u16, &buf[p], 2);
	p += 2;
	pcktprintf("\tScheduled Datalog queue: %hu request(s)\n", u16);

	memcpy(&u16, &buf[p], 2);
	p += 2;
	pcktprintf("\tCurrent Scheduled Datalog: 0x%04hX\n", u16);

	pcktprint_close();
}

void pckt_proc_DOWNLINK_PCKT_TYPE_SMOGP_TELEMETRY_2(pckt_t * pckt)
{
	if (pckt == NULL) return;
	tsprintf("\tPacket type: \x1b[1m\x1b[32mSMOGP-Telemetry 2\n\x1b[0m");
	if (pckt->pckt_len < 128) {
		tsprintf("\tERROR: too short\n");
		return;
	}
	pcktprint_open("bt2");
	int pphex;
	for (pphex=0; pphex < pckt->pckt_len; pphex++) pcktonlyfileprintf("%02hhX", pckt->pckt[pphex]);
	pcktonlyfileprintf("\n");
	pcktonlyfileprintf("\tPacket type: SMOGP-Telemetry 2\n");


	int32_t i32;
	uint16_t u16;
	uint8_t u8;
	int i,p,n;
	struct tm tmp_tm;
	time_t tmp_time;
	char timestr[20];

	int16_t vector[6];
	float float32;
	double tmpd, dvector[3];

	uint8_t * buf = pckt->pckt;



	p = 1;

	memcpy(&u8, &buf[p], 1);

	if (u8 == 'V') {
		p += 1;	// (valid flag)

		memcpy(&i32, &buf[p], 4);
		p += 4;

		tmp_time = i32;
		strftime(timestr, 20, "%Y-%m-%d %H:%M:%S", gmtime_r(&tmp_time, &tmp_tm));
		timestr[19] = 0;

		pcktprintf("\tTimestamp: %s\n", timestr);

		memcpy(&float32, &buf[p], 4);
		p += 4;
		tmpd = float32;
		pcktprintf("\tTemperature: %.2fC°\n", tmpd);

		for (i=0; i<3; i++) {
			memcpy(&float32, &buf[p], 4);
			p += 4;
			dvector[i] = float32;
		}
		pcktprintf("\tMSEN Gyroscope:   %f,\t%f,\t%f\n", dvector[0], dvector[1], dvector[2]);

		for (i=0; i<3; i++) {
			memcpy(&u16, &buf[p], 2);
			p += 2;
			vector[i] = (int16_t)u16;
		}
		pcktprintf("\tMSEN Magneto RAW:   % 5.0f\t% 5.0f\t% 5.0f\n", 1.0 * vector[0], 1.0 * vector[1], 1.0 * vector[2]);

		memcpy(&u8, &buf[p], 1);
		p += 1;

		if (u8 == 'Y') {
			for (i=0; i<6; i++) {
				memcpy(&u16, &buf[p], 2);
				p += 2;
				vector[i] = (int16_t)u16;
			}
			pcktprintf("\tMSEN Magneto MIN-MAX: [% 5.0f ... % 5.0f], [% 5.0f ... % 5.0f], [% 5.0f ... % 5.0f]\n", 1.0 * vector[0], 1.0 * vector[3], 1.0 * vector[1], 1.0 * vector[4], 1.0 * vector[2], 1.0 * vector[5]);
		} else {
			pcktprintf("\tMSEN Magneto MIN-MAX not valid yet!\n");
			p += 12;
		}

		for (i=0; i<3; i++) {
			memcpy(&float32, &buf[p], 4);
			p += 4;
			dvector[i] = float32;
		}
		pcktprintf("\tMSEN Magneto Scale:   % 5.0f\t% 5.0f\t% 5.0f\n", dvector[0], dvector[1], dvector[2]);

		for (i=0; i<3; i++) {
			memcpy(&float32, &buf[p], 4);
			p += 4;
			dvector[i] = float32;
		}
		pcktprintf("\tMSEN Magneto:   % 5.0f\t% 5.0f\t% 5.0f\n", dvector[0], dvector[1], dvector[2]);

	} else {
		p += 64;
		pcktprintf("\tNo MSEN data are available!\n");
	}

	pcktprintf("\tACK-INFO: ");
	for (i=0; i<17; i++) {
		memcpy(&u16, &buf[p], 2);
		p += 2;
		memcpy(&u8, &buf[p], 1);
		p += 1;
		if (i == 0) {
			store_ackd_serial(u16, u8);
		}
		pcktprintf("%hu (%.0f dBm RSSI)\t", u16, helper_ACK_INFO_RSSI_CONVERT(u8));
	}
	pcktprintf("\n");


	pcktprint_close();
}

void pckt_proc_DOWNLINK_PCKT_TYPE_ATL_TELEMETRY_1(pckt_t * pckt)
{
	if (pckt == NULL) return;
	tsprintf("\tPacket type: \x1b[1m\x1b[32mATL-Telemetry 1\n\x1b[0m");
	if (pckt->pckt_len < 128) {
		tsprintf("\tERROR: too short\n");
		return;
	}
	pcktprint_open("at1");
	int pphex;
	for (pphex=0; pphex < pckt->pckt_len; pphex++) pcktonlyfileprintf("%02hhX", pckt->pckt[pphex]);
	pcktonlyfileprintf("\n");
	pcktonlyfileprintf("\tPacket type: ATL-Telemetry 1\n");


	int32_t i32;
	uint16_t u16;
	uint8_t u8;
	int i,p,n;
	struct tm tmp_tm;
	time_t tmp_time;
	char timestr[20];
	double tmpd;

	uint8_t * buf = pckt->pckt;

	p = 1;

	memcpy(&i32, &buf[p], 4);
	p += 4;
	pcktprintf("\tUptime: %d\n", i32);

	memcpy(&i32, &buf[p], 4);
	p += 4;

	tmp_time = i32;
	strftime(timestr, 20, "%Y-%m-%d %H:%M:%S", gmtime_r(&tmp_time, &tmp_tm));
	timestr[19] = 0;

	pcktprintf("\tSystem time: %s\n", timestr);

	memcpy(&u8, &buf[p], 1);
	p += 1;
	if (u8 == '0') pcktprintf("\tOBC ID: 1\n");
	else if (u8 == '1') pcktprintf("\tOBC ID: 2\n");
	else pcktprintf("\tOBC ID: invalid!\n");

	memcpy(&u8, &buf[p], 1);
	p += 1;
	if (u8 == 'E') pcktprintf("\tOscillator: external\n");
	else if (u8 == 'I') pcktprintf("\tOscillator: internal\n");
	else pcktprintf("\tOscillator: invalid!\n");

	memcpy(&u8, &buf[p], 1);
	p += 1;
	if (u8 == 'V') {
		memcpy(&i32, &buf[p], 4);
		p += 4;

		tmp_time = i32;
		strftime(timestr, 20, "%Y-%m-%d %H:%M:%S", gmtime_r(&tmp_time, &tmp_tm));
		timestr[19] = 0;

		pcktprintf("\tADC results at %s:\n", timestr);

		pcktprintf("\tADC results with internal reference:\n");
			memcpy(&u16, &buf[p], 2);
			p += 2;
			tmpd = u16 / 1000.0;
			pcktprintf("\t\tVcc: %.3fV\n", tmpd);
			memcpy(&u16, &buf[p], 2);
			p += 2;
			tmpd = u16 / 1000.0;
			pcktprintf("\t\tVbg: %.3fV\n", tmpd);
			memcpy(&u16, &buf[p], 2);
			p += 2;
			tmpd = u16 / 1000.0;
			pcktprintf("\t\tVcore: %.3fV\n", tmpd);
			memcpy(&u16, &buf[p], 2);
			p += 2;
			tmpd = u16 / 1000.0;
			pcktprintf("\t\tVextref: %.3fV\n", tmpd);
			memcpy(&u16, &buf[p], 2);
			p += 2;
			tmpd = u16 / 1000.0;
			pcktprintf("\t\tAN1: %.3fV\n", tmpd);
			memcpy(&u16, &buf[p], 2);
			p += 2;
			tmpd = u16 / 1000.0;
			pcktprintf("\t\tAN2: %.3fV\n", tmpd);
			memcpy(&u16, &buf[p], 2);
			p += 2;
			tmpd = u16 / 1000.0;
			pcktprintf("\t\tAN3: %.3fV\n", tmpd);
			memcpy(&u16, &buf[p], 2);
			p += 2;
			tmpd = u16 / 1000.0;
			pcktprintf("\t\tAN4: %.3fV\n", tmpd);

		pcktprintf("\tADC results with external reference:\n");
			memcpy(&u16, &buf[p], 2);
			p += 2;
			tmpd = u16 / 1000.0;
			pcktprintf("\t\tVcc: %.3fV\n", tmpd);
			memcpy(&u16, &buf[p], 2);
			p += 2;
			tmpd = u16 / 1000.0;
			pcktprintf("\t\tVbg: %.3fV\n", tmpd);
			memcpy(&u16, &buf[p], 2);
			p += 2;
			tmpd = u16 / 1000.0;
			pcktprintf("\t\tVcore: %.3fV\n", tmpd);
			memcpy(&u16, &buf[p], 2);
			p += 2;
			tmpd = u16 / 1000.0;
			pcktprintf("\t\tVextref: %.3fV\n", tmpd);
			memcpy(&u16, &buf[p], 2);
			p += 2;
			tmpd = u16 / 1000.0;
			pcktprintf("\t\tAN1: %.3fV\n", tmpd);
			memcpy(&u16, &buf[p], 2);
			p += 2;
			tmpd = u16 / 1000.0;
			pcktprintf("\t\tAN2: %.3fV\n", tmpd);
			memcpy(&u16, &buf[p], 2);
			p += 2;
			tmpd = u16 / 1000.0;
			pcktprintf("\t\tAN3: %.3fV\n", tmpd);
			memcpy(&u16, &buf[p], 2);
			p += 2;
			tmpd = u16 / 1000.0;
			pcktprintf("\t\tAN4: %.3fV\n", tmpd);
	} else {
		pcktprintf("\tADC results: not valid!\n");
		p += 37;
	}

	memcpy(&u8, &buf[p], 1);
	p += 1;
	if (u8 & 0x80) pcktprintf("\tSPI-MUX status: functional\n");
	else pcktprintf("\tSPI-MUX status: not functional\n");
	pcktprintf("\t\tMSEN CS pin select: %s\n", ((u8 & 0x40) ? "FAILED" : "OK"));
	pcktprintf("\t\tRTCC CS pin select: %s\n", ((u8 & 0x20) ? "FAILED" : "OK"));
	pcktprintf("\t\tFLASH CS pin select: %s\n", ((u8 & 0x10) ? "FAILED" : "OK"));
	pcktprintf("\t\tAll CS pin deselect: %s\n", ((u8 & 0x08) ? "FAILED" : "OK"));
	pcktprintf("\t\tMISO pin test: %s\n", ((u8 & 0x04) ? "FAILED" : "OK"));
	pcktprintf("\t\tMOSI pin test: %s\n", ((u8 & 0x02) ? "FAILED" : "OK"));
	pcktprintf("\t\tSCLK pin test: %s\n", ((u8 & 0x01) ? "FAILED" : "OK"));

	memcpy(&u8, &buf[p], 1);
	p += 1;
	if (u8 == 0) pcktprintf("\tSPI/FLASH status: not functional\n");
	else pcktprintf("\tSPI/FLASH status: functional, startcount == %hhu\n", u8);

	memcpy(&u8, &buf[p], 1);
	p += 1;
	if (u8 == 0) pcktprintf("\tSPI/MSEN status: not functional\n");
	else pcktprintf("\tSPI/MSEN status: functional, startcount == %hhu\n", u8);

	memcpy(&u8, &buf[p], 1);
	p += 1;
	if (u8 == 0) pcktprintf("\tSPI/RTCC status: not functional\n");
	else pcktprintf("\tSPI/RTCC status: functional, startcount == %hhu\n", u8);

	memcpy(&u8, &buf[p], 1);
	p += 1;
	pcktprintf("\tRandom number: %02hhX\n", u8);

	for (i=0; i<6; i++) {
		memcpy(&u8, &buf[p], 1);
		p += 1;
		pcktprintf("\tMPPT-%d (%s) bus status: %s\n", (i+1), const_mppt_panel_names[i], ET1_local_function_mppt_bus_state_interpretter((int8_t)u8) );
	}

	for (i=0; i<2; i++) {
		memcpy(&u8, &buf[p], 1);
		p += 1;
		pcktprintf("\tACCU-%d bus status: %s\n", (i+1), ET1_local_function_pcudoz_bus_state_interpretter((int8_t)u8) );
	}

	for (i=0; i<2; i++) {
		memcpy(&u8, &buf[p], 1);
		p += 1;
		pcktprintf("\tPCU-%d bus status: %s\n", (i+1), ET1_local_function_pcudoz_bus_state_interpretter((int8_t)u8) );
	}

	memcpy(&u8, &buf[p], 1);
	p += 1;
	pcktprintf("\tCurrent COM: COM-%hhu\n", u8);

	memcpy(&i32, &buf[p], 4);
	p += 4;
	pcktprintf("\tCOM uptime: %d seconds\n", i32);


	static const int const_tx_pwr_level_to_mw[] = {10, 25, 50, 100};

	memcpy(&u8, &buf[p], 1);
	p += 1;
	if (u8 > 3) {
		pcktprintf("\tCOM-TX power level: INVALID (%hhu)\n", u8);
	} else {
		pcktprintf("\tCOM-TX power level: %dmW (%hhu)\n", const_tx_pwr_level_to_mw[u8], u8);
	}

	memcpy(&u16, &buf[p], 2);
	p += 2;
	pcktprintf("\tCOM-TX Current: %humA\n", u16);

	memcpy(&u16, &buf[p], 2);
	p += 2;
	pcktprintf("\tCOM-RX Current: %humA\n", u16);

	memcpy(&u16, &buf[p], 2);
	p += 2;
	tmpd = u16 / 1000.0;
	pcktprintf("\tCOM-TX Voltage Drop: %.3fV\n", tmpd);

	memcpy(&u16, &buf[p], 2);
	p += 2;
	pcktprintf("\tScheduled Spectrum Analysis queue: %hu request(s)\n", u16);

	memcpy(&u16, &buf[p], 2);
	p += 2;
	pcktprintf("\tScheduled File Download queue: %hu request(s)\n", u16);


	int32_t emm_rc, emm_sleep;
	uint8_t emm_mp;
	memcpy(&u8, &buf[p], 1);
	p += 1;
	memcpy(&emm_mp, &buf[p], 1);
	p += 1;
	memcpy(&emm_rc, &buf[p], 4);
	p += 4;
	memcpy(&emm_sleep, &buf[p], 4);
	p += 4;
	switch(u8) {
		case 0:	tsprintf("\x1b[1m\x1b[32m");	// bold green
			pcktprintf("\tEnergy Management Mode: normal (morse period: %hhu; radio cycle: %.1fs; sleep: 0)", emm_mp, emm_rc/1e6);
			break;
		case 1:	tsprintf("\x1b[1m\x1b[32m");	// bold green
			pcktprintf("\tEnergy Management Mode: normal, reduced (morse period: %hhu; radio cycle: %.1fs; sleep: %.1fs)", emm_mp, emm_rc/1e6, emm_sleep/1e6);
			break;
		case 2:	tsprintf("\x1b[1m\x1b[33m");	// bold yellow
			pcktprintf("\tEnergy Management Mode: energy saving (NO morse; radio cycle: %.1fs; sleep: %.1fs", emm_rc/1e6, emm_sleep/1e6);
			break;
		case 3:	tsprintf("\x1b[1m\x1b[101m");	// red background
			pcktprintf("\tEnergy Management Mode: EMERGENCY (NO morse; radio cycle: %.1fs; sleep: %.1fs", emm_rc/1e6, emm_sleep/1e6);
			break;
		default: tsprintf("\x1b[1m\x1b[101m");	// red background
			pcktprintf("\tEnergy Management Mode: INVALID!");
			break;
	}
	tsprintf("\x1b[0m");
	pcktprintf("\n");

	memcpy(&i32, &buf[p], 4);
	p += 4;
	if (i32 == -1) {
		pcktprintf("\tLast Telecommand: none received yet!\n");
	} else {
		pcktprintf("\tLast Telecommand: %d seconds ago\n", i32);
	}

	memcpy(&u16, &buf[p], 2);
	p += 2;
	pcktprintf("\tAutomatic Antenna Opening Attempts: %hu\n", u16);

	int32_t cpui, cpuw;
	memcpy(&u16, &buf[p], 2);
	p += 2;
	memcpy(&cpui, &buf[p], 4);
	p += 4;
	memcpy(&cpuw, &buf[p], 4);
	p += 4;
	pcktprintf("\tCPU usage: %uus iddle, %uus work over %hu cycles\n", cpui, cpuw, u16);
	double load, cyc;
	load = (double)cpui / ((double)cpui + (double)cpuw);
	cyc = u16;
	pcktprintf("\t\tLoad: %.1f%%\n\t\tIddle/c: %.3fus\n\t\tWork/c: %.3fus\n", load * 100.0, cpui/cyc, cpuw/cyc);

	uint32_t chksum, chksum_prev_diff;
	memcpy(&chksum, &buf[p], 4);
	p += 4;
	memcpy(&chksum_prev_diff, &buf[p], 4);
	p += 4;
	pcktprintf("\tOBC-FLASH checksum: %08X (%08X)\n", chksum, chksum_prev_diff);

	memcpy(&u16, &buf[p], 2);
	p += 2;
	pcktprintf("\tScheduled Datalog queue: %hu request(s)\n", u16);

	memcpy(&u16, &buf[p], 2);
	p += 2;
	pcktprintf("\tCurrent Scheduled Datalog: 0x%04hX\n", u16);

	pcktprint_close();
}

void pckt_proc_DOWNLINK_PCKT_TYPE_ATL_TELEMETRY_2(pckt_t * pckt)
{
	if (pckt == NULL) return;
	tsprintf("\tPacket type: \x1b[1m\x1b[32mATL-Telemetry 2\n\x1b[0m");
	if (pckt->pckt_len < 128) {
		tsprintf("\tERROR: too short\n");
		return;
	}
	pcktprint_open("at2");
	int pphex;
	for (pphex=0; pphex < pckt->pckt_len; pphex++) pcktonlyfileprintf("%02hhX", pckt->pckt[pphex]);
	pcktonlyfileprintf("\n");
	pcktonlyfileprintf("\tPacket type: ATL-Telemetry 2\n");


	int32_t i32;
	uint16_t u16;
	uint8_t u8;
	int i,p,n;
	struct tm tmp_tm;
	time_t tmp_time;
	char timestr[20];

	int16_t vector[6];
	float float32;
	double tmpd, dvector[3];

	uint8_t * buf = pckt->pckt;



	p = 1;

	memcpy(&u8, &buf[p], 1);

	if (u8 == 'V') {
		p += 1;	// (valid flag)

		memcpy(&i32, &buf[p], 4);
		p += 4;

		tmp_time = i32;
		strftime(timestr, 20, "%Y-%m-%d %H:%M:%S", gmtime_r(&tmp_time, &tmp_tm));
		timestr[19] = 0;

		pcktprintf("\tTimestamp: %s\n", timestr);

		memcpy(&float32, &buf[p], 4);
		p += 4;
		tmpd = float32;
		pcktprintf("\tTemperature: %.2fC°\n", tmpd);

		for (i=0; i<3; i++) {
			memcpy(&float32, &buf[p], 4);
			p += 4;
			dvector[i] = float32;
		}
		pcktprintf("\tMSEN Gyroscope:   %f,\t%f,\t%f\n", dvector[0], dvector[1], dvector[2]);

		for (i=0; i<3; i++) {
			memcpy(&u16, &buf[p], 2);
			p += 2;
			vector[i] = (int16_t)u16;
		}
		pcktprintf("\tMSEN Magneto RAW:   % 5.0f\t% 5.0f\t% 5.0f\n", 1.0 * vector[0], 1.0 * vector[1], 1.0 * vector[2]);

		memcpy(&u8, &buf[p], 1);
		p += 1;

		if (u8 == 'Y') {
			for (i=0; i<6; i++) {
				memcpy(&u16, &buf[p], 2);
				p += 2;
				vector[i] = (int16_t)u16;
			}
			pcktprintf("\tMSEN Magneto MIN-MAX: [% 5.0f ... % 5.0f], [% 5.0f ... % 5.0f], [% 5.0f ... % 5.0f]\n", 1.0 * vector[0], 1.0 * vector[3], 1.0 * vector[1], 1.0 * vector[4], 1.0 * vector[2], 1.0 * vector[5]);
		} else {
			pcktprintf("\tMSEN Magneto MIN-MAX not valid yet!\n");
			p += 12;
		}

		for (i=0; i<3; i++) {
			memcpy(&float32, &buf[p], 4);
			p += 4;
			dvector[i] = float32;
		}
		pcktprintf("\tMSEN Magneto Scale:   % 5.0f\t% 5.0f\t% 5.0f\n", dvector[0], dvector[1], dvector[2]);

		for (i=0; i<3; i++) {
			memcpy(&float32, &buf[p], 4);
			p += 4;
			dvector[i] = float32;
		}
		pcktprintf("\tMSEN Magneto:   % 5.0f\t% 5.0f\t% 5.0f\n", dvector[0], dvector[1], dvector[2]);

	} else {
		p += 64;
		pcktprintf("\tNo MSEN data are available!\n");
	}

	pcktprintf("\tACK-INFO: ");
	for (i=0; i<17; i++) {
		memcpy(&u16, &buf[p], 2);
		p += 2;
		memcpy(&u8, &buf[p], 1);
		p += 1;
		if (i == 0) {
			store_ackd_serial(u16, u8);
		}
		pcktprintf("%hu (%.0f dBm RSSI)\t", u16, helper_ACK_INFO_RSSI_CONVERT(u8));
	}
	pcktprintf("\n");


	pcktprint_close();
}

// moved to "main.h":
//double helper_DOWNLINK_PCKT_TYPE_ATL_TELEMETRY_3_calc_battery_current(int panel, uint16_t adc)
//double helper_DOWNLINK_PCKT_TYPE_ATL_TELEMETRY_3_calc_temperature(int panel, uint16_t adc_ref, uint16_t adc)


void pckt_proc_DOWNLINK_PCKT_TYPE_ATL_TELEMETRY_3(pckt_t * pckt)
{
	if (pckt == NULL) return;
	tsprintf("\tPacket type: \x1b[1m\x1b[32mATL-Telemetry 3\n\x1b[0m");
	if (pckt->pckt_len < 128) {
		tsprintf("\tERROR: too short\n");
		return;
	}
	pcktprint_open("at3");
	int pphex;
	for (pphex=0; pphex < pckt->pckt_len; pphex++) pcktonlyfileprintf("%02hhX", pckt->pckt[pphex]);
	pcktonlyfileprintf("\n");
	pcktonlyfileprintf("\tPacket type: ATL-Telemetry 3\n");

	int32_t i32;
	uint32_t u32;
	uint16_t u16, u16_vect[6];
	uint8_t u8;
	int i,p,n;
	struct tm tmp_tm;
	time_t tmp_time;
	char timestr[20];

	uint8_t * buf = pckt->pckt;

	static const char * const_accu_panel_names[] = {"porszal (A2)", "kapton", "szalas (SZ2)", "por (K2)"};

	p = 1;
	memcpy(&i32, &buf[p], 4);
	p += 4;

	tmp_time = i32;
	strftime(timestr, 20, "%Y-%m-%d %H:%M:%S", gmtime_r(&tmp_time, &tmp_tm));
	timestr[19] = 0;

	pcktprintf("\tTimestamp: %s\n", timestr);

	for (i=0; i<4; i++) {
		if (buf[p] == 1) {
			p += 1;

			memcpy(&u8, &buf[p], 1);
			p += 1;

			memcpy(&i32, &buf[p], 4);
			p += 4;

			tmp_time = i32;
			strftime(timestr, 20, "%Y-%m-%d %H:%M:%S", gmtime_r(&tmp_time, &tmp_tm));
			timestr[19] = 0;

			memcpy(&u16, &buf[p], 2);
			p += 2;

			for (n=0; n<6; n++) {
				memcpy(&u16_vect[n], &buf[p], 2);
				p += 2;
			}

			double ibat = helper_DOWNLINK_PCKT_TYPE_ATL_TELEMETRY_3_calc_battery_current(i, u16);
			double temp[5];
			for (n=0; n<5; n++) {
				temp[n] = helper_DOWNLINK_PCKT_TYPE_ATL_TELEMETRY_3_calc_temperature(i, u16_vect[0], u16_vect[n + 1], n);
			}

			pcktprintf("\tACCU-%d (%s) measurement: valid at %s [on 1WBus-%hhu]\n\t\tADC values: %hu, %hu, %hu, %hu, %hu, %hu, %hu\n\t\tBattery Current: %.3fA\n\t\tTemperature 1 (right) : %.2f°C\n\t\tTemperature 2 (Top): %.2f°C\n\t\tTemperature 3 (left): %.2f°C\n\t\tTemperature 4 (back): %.2f°C\n\t\tTemperature 5 (panel): %.2f°C\n", (i+1), const_accu_panel_names[i], timestr, (u8+1), u16, u16_vect[0], u16_vect[1], u16_vect[2], u16_vect[3], u16_vect[4], u16_vect[5], ibat, temp[0], temp[1], temp[2], temp[3], temp[4]);
		} else {
			pcktprintf("\tACCU-%d (%s) measurement: not valid\n", (i+1), const_accu_panel_names[i]);
			p += 20;
		}
	}

	pcktprint_close();
}





int main(int argc , char *argv[])
{
	pthread_t input_pt, rx_pckt_proc_pt, rx_pckt_data_pt;



	tsprintf_init(false, true, stderr);
	tsprintf("\x1b[1m\x1b[102m**  GND-SW GORGON  **\x1b[0m\n");

	if (block_fifo_init(&received_raw_packet_fifo, sizeof(pckt_t), 256) < 0) global_break("FIFO init failed\n");
	if (block_fifo_init(&received_decoded_packet_fifo, sizeof(pckt_t), 256) < 0) global_break("FIFO init failed\n");


	pthread_create(&input_pt, NULL, input_ptfun, NULL);
	pthread_create(&rx_pckt_proc_pt, NULL, rx_pckt_proc_ptfun, NULL);
	pthread_create(&rx_pckt_data_pt, NULL, rx_pckt_data_ptfun, NULL);

	// WORK IN THE THREADS

	pthread_join(rx_pckt_data_pt, NULL);
	pthread_join(rx_pckt_proc_pt, NULL);
	pthread_join(input_pt, NULL);

}
