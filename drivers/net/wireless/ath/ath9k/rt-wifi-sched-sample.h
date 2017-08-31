/*
 * Copyright (c) 2015, The University of Texas at Austin,
 * Department of Computer Science, Cyberphysical Group
 * http://www.cs.utexas.edu/~cps/ All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* This is a configuration file for each RT-WiFi node. 
 * Copy this file to rt-wifi-sched.h, and modify it accordingly before build
 * RT-WiFi module */
#ifndef RT_WIFI_SCHED_H
#define RT_WIFI_SCHED_H

#include "rt-wifi.h"

/* Authorized stations */
static struct rt_wifi_sta RT_WIFI_STAS[] = {
	{ .mac_addr = {0x00, 0x22, 0x5F, 0xF6, 0x11, 0x21} },  // i7
	{ .mac_addr = {0x00, 0x26, 0xB6, 0xF7, 0x72, 0x10} },  // i5
	{ .mac_addr = {0x00, 0x22, 0x5F, 0xF6, 0x0E, 0xA7} },  // i3
	{ .mac_addr = {0x00, 0x21, 0x63, 0xC9, 0x0C, 0xDB} }   // Chr Box
};
static u16 RT_WIFI_STAS_NUM = ARRAY_SIZE(RT_WIFI_STAS);

/* Local ID of a station in the RT_WIFI_STAS array. For example, the local ID
 * is 0x01 for i5 machine in the given RT_WIFI_STAS array. */
#define RT_WIFI_LOCAL_ID 0xFF

/* RT-WiFi TDMA schedule */
static struct rt_wifi_sched RT_WIFI_PRE_CONFIG_SCHED[] = {
	{ .type = RT_WIFI_RX, 0,  .sta_id = 0x00 },
	{ .type = RT_WIFI_TX, 1,  .sta_id = 0x00 },
	{ .type = RT_WIFI_RX, 2,  .sta_id = 0x00 },
	{ .type = RT_WIFI_TX, 3,  .sta_id = 0x00 },
	{ .type = RT_WIFI_RX, 4,  .sta_id = 0x00 },
	{ .type = RT_WIFI_TX, 5,  .sta_id = 0x00 },
	{ .type = RT_WIFI_RX, 6,  .sta_id = 0x00 },
	{ .type = RT_WIFI_TX, 7,  .sta_id = 0x00 },
	{ .type = RT_WIFI_RX, 8,  .sta_id = 0x00 },
	{ .type = RT_WIFI_TX, 9,  .sta_id = 0x00 },
	{ .type = RT_WIFI_RX, 10, .sta_id = 0x00 },
	{ .type = RT_WIFI_TX, 11, .sta_id = 0x00 },
	{ .type = RT_WIFI_RX, 12, .sta_id = 0x00 },
	{ .type = RT_WIFI_TX, 13, .sta_id = 0x00 },
	{ .type = RT_WIFI_RX, 14, .sta_id = 0x00 },
	{ .type = RT_WIFI_TX, 15, .sta_id = 0x00 },
	{ .type = RT_WIFI_RX, 16, .sta_id = 0x00 },
	{ .type = RT_WIFI_TX, 17, .sta_id = 0x00 },
	{ .type = RT_WIFI_RX, 18, .sta_id = 0x00 },
	{ .type = RT_WIFI_TX, 19, .sta_id = 0x00 },
	{ .type = RT_WIFI_RX, 20, .sta_id = 0x00 },
	{ .type = RT_WIFI_TX, 21, .sta_id = 0x00 },
	{ .type = RT_WIFI_RX, 22, .sta_id = 0x00 },
	{ .type = RT_WIFI_TX, 23, .sta_id = 0x00 },
	{ .type = RT_WIFI_RX, 24, .sta_id = 0x00 },
	{ .type = RT_WIFI_TX, 25, .sta_id = 0x00 },
	{ .type = RT_WIFI_RX, 26, .sta_id = 0x00 },
	{ .type = RT_WIFI_TX, 27, .sta_id = 0x00 },
	{ .type = RT_WIFI_RX, 28, .sta_id = 0x00 },
	{ .type = RT_WIFI_TX, 29, .sta_id = 0x00 },
	{ .type = RT_WIFI_RX, 30, .sta_id = 0x00 },
	{ .type = RT_WIFI_TX, 31, .sta_id = 0x00 },
	{ .type = RT_WIFI_RX, 32, .sta_id = 0x00 },
	{ .type = RT_WIFI_TX, 33, .sta_id = 0x00 },
	{ .type = RT_WIFI_RX, 34, .sta_id = 0x00 },
	{ .type = RT_WIFI_TX, 35, .sta_id = 0x00 },
	{ .type = RT_WIFI_RX, 36, .sta_id = 0x00 },
	{ .type = RT_WIFI_TX, 37, .sta_id = 0x00 },
	{ .type = RT_WIFI_RX, 38, .sta_id = 0x00 },
	{ .type = RT_WIFI_TX, 39, .sta_id = 0x00 },
	{ .type = RT_WIFI_RX, 40, .sta_id = 0x00 },
	{ .type = RT_WIFI_TX, 41, .sta_id = 0x00 },
	{ .type = RT_WIFI_RX, 42, .sta_id = 0x00 },
	{ .type = RT_WIFI_TX, 43, .sta_id = 0x00 },
	{ .type = RT_WIFI_RX, 44, .sta_id = 0x00 },
	{ .type = RT_WIFI_TX, 45, .sta_id = 0x00 },
	{ .type = RT_WIFI_RX, 46, .sta_id = 0x00 },
	{ .type = RT_WIFI_TX, 47, .sta_id = 0x00 },
	{ .type = RT_WIFI_BEACON, 48},
	{ .type = RT_WIFI_BEACON, 49},
};

#endif /* RT_WIFI_SCHED_H */
