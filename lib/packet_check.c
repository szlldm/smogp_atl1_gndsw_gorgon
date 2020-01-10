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

#include <stdint.h>
#include <string.h>
#include "packet_check.h"

#define DOWNLINK_PCKT_TYPE_TELEMETRY_1		(1)
#define DOWNLINK_PCKT_TYPE_TELEMETRY_2		(2)
#define DOWNLINK_PCKT_TYPE_TELEMETRY_3		(3)
#define DOWNLINK_PCKT_TYPE_BEACON		(4)
#define DOWNLINK_PCKT_TYPE_SPECTRUM_RESULT	(5)
#define DOWNLINK_PCKT_TYPE_FILE_INFO		(6)
#define DOWNLINK_PCKT_TYPE_FILE_FRAGMENT	(7)

#define DOWNLINK_PCKT_TYPE_SMOGP_TELEMETRY_1	(33)
#define DOWNLINK_PCKT_TYPE_SMOGP_TELEMETRY_2	(34)

#define DOWNLINK_PCKT_TYPE_ATL_TELEMETRY_1	(129)
#define DOWNLINK_PCKT_TYPE_ATL_TELEMETRY_2	(130)
#define DOWNLINK_PCKT_TYPE_ATL_TELEMETRY_3	(131)


int packet_check_DOWNLINK_PCKT_TYPE_TELEMETRY_1(uint8_t * pckt, int pckt_len)
{
	int i,p;

	if (pckt_len != 128) return -1;

	p = 5;
	for (i=0; i<6; i++) {
		if (pckt[p+16] & 0x07) return -1;	// check status byte unused lower bits
		p += 17;
	}

	p = 116;
	// check last two unused bytes
	for (i=0; i<2; i++) {
		if (pckt[p] != 0) return -1;
		p += 1;
	}

	return 0;
}

int packet_check_DOWNLINK_PCKT_TYPE_TELEMETRY_2(uint8_t * pckt, int pckt_len)
{
	int i,p;

	if (pckt_len != 128) return -1;

	p = 5;
	for (i=0; i<2; i++) {
		if (pckt[p+4] & 0x03) return -1;	// check status byte unused lower bits
		p += 9;
	}
	for (i=0; i<2; i++) {
		if (pckt[p+16] & 0x0F) return -1;	// check status byte unused lower bits
		p += 17;
	}
	for (i=0; i<2; i++) {
		if (pckt[p+10] & 0x03) return -1;	// check status byte unused lower bits
		p += 11;
	}
	for (i=0; i<2; i++) {
		if (pckt[p+12] & 0x3F) return -1;	// check status byte unused lower bits
		p += 13;
	}

	p = 114;
	// check last four unused bytes
	for (i=0; i<4; i++) {
		if (pckt[p] != 0) return -1;
		p += 1;
	}

	return 0;
}

int packet_check_DOWNLINK_PCKT_TYPE_TELEMETRY_3(uint8_t * pckt, int pckt_len)
{
	int i;
	int16_t i16;

	if (pckt_len != 128) return -1;

	memcpy(&i16, &pckt[11], sizeof(int16_t));
	if (i16 != INT16_MAX) return -1;	// value check

	if (pckt[17] & 0x07) return -1;	// check status byte unused lower bits
	if (pckt[22] & 0x03) return -1;	// check status byte unused lower bits
	if (!((pckt[70] == 1) || (pckt[70] == 0))) return -1;	// valid value check

	return 0;
}

int packet_check_DOWNLINK_PCKT_TYPE_BEACON(uint8_t * pckt, int pckt_len)
{
	int i,p;

	if (pckt_len != 128) return -1;

	p = 110;
	// check last eight unused bytes
	for (i=0; i<8; i++) {
		if (pckt[p] != 0) return -1;
		p += 1;
	}

	return 0;
}

int packet_check_DOWNLINK_PCKT_TYPE_SPECTRUM_RESULT(uint8_t * pckt, int pckt_len)
{
	int i,p;

	if (pckt_len != 256) return -1;

	if (pckt[13] > 9) return -1;	// value check
	p = 18;
	// check unused bytes
	for (i=0; i<2; i++) {
		if (pckt[p] != 0) return -1;
		p += 1;
	}

	return 0;
}

int packet_check_DOWNLINK_PCKT_TYPE_FILE_INFO(uint8_t * pckt, int pckt_len)
{
	int i,p;

	if (pckt_len != 128) return -1;

	p = 5;
	for (i=0; i<5; i++) {
		if (pckt[p+4] != 0) return -1;	// value check
		p += 21;
	}

	return 0;
}

int packet_check_DOWNLINK_PCKT_TYPE_FILE_FRAGMENT(uint8_t * pckt, int pckt_len)
{
	int i,p;

	if (pckt_len != 256) return -1;

	if (pckt[12] != 0) return -1;	// value check

	uint16_t pckt_index, pckt_cnt;
	uint32_t file_size;

	memcpy(&pckt_index, &pckt[5], 2);
	memcpy(&pckt_cnt, &pckt[7], 2);

	file_size = 0;
	file_size += pckt[12];
	file_size <<= 8;
	file_size += pckt[13];
	file_size <<= 8;
	file_size += pckt[14];

	// max file size: 217*128 = 27776 == 27.125kB
	if (pckt_cnt == 0) return -1;
	if (pckt_cnt > 128) return -1;
	if (pckt_index >= pckt_cnt) return -1;
	if ((file_size / 217) > pckt_cnt) return -1;
	if (((file_size / 217) + 1) < pckt_cnt) return -1;

	return 0;
}

int packet_check_DOWNLINK_PCKT_TYPE_SMOGP_TELEMETRY_1(uint8_t * pckt, int pckt_len)
{
	int i,p;

	if (pckt_len != 128) return -1;

	if (!((pckt[9] == '1') || (pckt[9] == '0'))) return -1;		// valid value check
	if (!((pckt[10] == 'E') || (pckt[10] == 'I'))) return -1;	// valid value check
	if (!((pckt[11] == 'V') || (pckt[11] == 'N'))) return -1;	// valid value check
	if (!((pckt[63] == 2) || (pckt[63] == 1))) return -1;		// valid value check
	// check unused byte
	p = 117;
	if (pckt[p] != 0) return -1;	// value check

	return 0;
}

int packet_check_DOWNLINK_PCKT_TYPE_SMOGP_TELEMETRY_2(uint8_t * pckt, int pckt_len)
{
	int i,p;

	if (pckt_len != 128) return -1;

	if (!((pckt[1] == 'V') || (pckt[1] == 'N'))) return -1;	// valid value check
	p = 116;
	// check unused bytes
	for (i=0; i<2; i++) {
		if (pckt[p] != 0) return -1;
		p += 1;
	}

	return 0;
}

int packet_check_DOWNLINK_PCKT_TYPE_ATL_TELEMETRY_1(uint8_t * pckt, int pckt_len)
{
	return packet_check_DOWNLINK_PCKT_TYPE_SMOGP_TELEMETRY_1(pckt, pckt_len);
}

int packet_check_DOWNLINK_PCKT_TYPE_ATL_TELEMETRY_2(uint8_t * pckt, int pckt_len)
{
	return packet_check_DOWNLINK_PCKT_TYPE_SMOGP_TELEMETRY_2(pckt, pckt_len);
}

int packet_check_DOWNLINK_PCKT_TYPE_ATL_TELEMETRY_3(uint8_t * pckt, int pckt_len)
{
	int i,p;

	if (pckt_len != 128) return -1;

	p = 85;
	// check unused bytes
	for (i=0; i<33; i++) {
		if (pckt[p] != 0) return -1;
		p += 1;
	}

	return 0;
}


int packet_check(uint8_t * pckt, int pckt_len)
{
	if (pckt_len < 128) return -1;

	if (pckt[0] == DOWNLINK_PCKT_TYPE_TELEMETRY_1) return packet_check_DOWNLINK_PCKT_TYPE_TELEMETRY_1(pckt, pckt_len);
	if (pckt[0] == DOWNLINK_PCKT_TYPE_TELEMETRY_2) return packet_check_DOWNLINK_PCKT_TYPE_TELEMETRY_2(pckt, pckt_len);
	if (pckt[0] == DOWNLINK_PCKT_TYPE_TELEMETRY_3) return packet_check_DOWNLINK_PCKT_TYPE_TELEMETRY_3(pckt, pckt_len);
	if (pckt[0] == DOWNLINK_PCKT_TYPE_BEACON) return packet_check_DOWNLINK_PCKT_TYPE_BEACON(pckt, pckt_len);
	if (pckt[0] == DOWNLINK_PCKT_TYPE_SPECTRUM_RESULT) return packet_check_DOWNLINK_PCKT_TYPE_SPECTRUM_RESULT(pckt, pckt_len);
	if (pckt[0] == DOWNLINK_PCKT_TYPE_FILE_INFO) return packet_check_DOWNLINK_PCKT_TYPE_FILE_INFO(pckt, pckt_len);
	if (pckt[0] == DOWNLINK_PCKT_TYPE_FILE_FRAGMENT) return packet_check_DOWNLINK_PCKT_TYPE_FILE_FRAGMENT(pckt, pckt_len);
	if (pckt[0] == DOWNLINK_PCKT_TYPE_SMOGP_TELEMETRY_1) return packet_check_DOWNLINK_PCKT_TYPE_SMOGP_TELEMETRY_1(pckt, pckt_len);
	if (pckt[0] == DOWNLINK_PCKT_TYPE_SMOGP_TELEMETRY_2) return packet_check_DOWNLINK_PCKT_TYPE_SMOGP_TELEMETRY_2(pckt, pckt_len);
	if (pckt[0] == DOWNLINK_PCKT_TYPE_ATL_TELEMETRY_1) return packet_check_DOWNLINK_PCKT_TYPE_ATL_TELEMETRY_1(pckt, pckt_len);
	if (pckt[0] == DOWNLINK_PCKT_TYPE_ATL_TELEMETRY_2) return packet_check_DOWNLINK_PCKT_TYPE_ATL_TELEMETRY_2(pckt, pckt_len);
	if (pckt[0] == DOWNLINK_PCKT_TYPE_ATL_TELEMETRY_3) return packet_check_DOWNLINK_PCKT_TYPE_ATL_TELEMETRY_3(pckt, pckt_len);

	return -1;
}


/*uint8_t hex_to_num(char c)*/
/*{*/
/*	if ((c >= '0') && (c <= '9')) return (c-'0');*/
/*	if ((c >= 'A') && (c <= 'F')) return (c+10-'A');*/
/*	return 0;*/
/*}*/

/*int main()*/
/*{*/
/*	char * test_packets[] = {*/
/*			"01D88B00009B890000BF002B071B0036070C004C1030A1890000CE00290909008A0506001410909D8900008400F8030100EA010500C20B709E8900001601FD080500EF010500F40D50A489000034F500000000000000000000B0A089000052000509070075040500DC0FC8D201FF0000000000000000277B6EF2C642711A3ABF",*/
/*			"02D98B0000D38B0000E801003302D48B0000E801002B02D78B00001D0020007D0D00007800000200D88B00001F0021007E0D0000780002020000000000000000000000000000000000000000000000D08B0000990FB60C0F00000000D18B0000A50FBF0C0900000000D201FF0000000000000000000098CAAE954A79307B0EB2",*/
/*			"03DA8B0000FE0C12000000FF7F98009D007874001700201E000200ECFFE2008401CEFF000000000000A7000000000000000000000000000000000000000000A8DA8B000008FF01C30C0701D8000000000000000000000000000000000000000000000000000000000000000000D201FF000000000000D17772ACF723ED64A0CD",*/
/*			"81DB8B0000DB8B0000304556DA8B0000FE0CB0044707410806001D00E802E802970C8F040507000806001E00D002D2028001010145FDFDFDFDFDFD0101010101DA8B00000374001700880000000000010E40F40F0200000000FFFFFFFF0A004B10B1443F015EC70F022422FD61FFFFFFFF00000000008E1F7F98C6B51EDAFBA7",*/
/*			"8256DB8B000024E78541BC12D941EE4A6441199C9BC16F01FD00C5005975FF1F009BFFB0015602FF016766393E00003A3ECDCC323E01541F42004831C1CDCCB2BFD201FF0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000430BF93CBCAE5F8B340E",*/
/*			"83DC8B00000100DB8B000000818BA8A945FA429D49BA474F4A0100DC8B00005D7925A83146EF423A491547F04A0100D98B00007781D6A7C645DA42C249BA47844A0100D98B0000627F08A914459C42DB48F04782490000000000000000000000000000000000000000000000000000000000000000004F7781F9391AA8F7F2B4",*/
/*			"04DD8B000048656C6C6F20576F726C64212041544C2D3120697320616C6976652120000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000D1020000D201FF0000000000000000000000000000352E65EDF55BBD3BE2ED",*/
/*		};*/

/*	int i,k;*/
/*	uint8_t pckt[128];*/
/*	for (i=0; i<7; i++) {*/
/*		for (k=0; k<128; k++) {*/
/*			pckt[k] = hex_to_num(test_packets[i][k*2]) << 4;*/
/*			pckt[k] += hex_to_num(test_packets[i][k*2+1]);*/
/*		}*/
/*		printf("Test %d = %d\n", i+1, packet_check(pckt, 128));*/
/*	}*/

/*}*/



