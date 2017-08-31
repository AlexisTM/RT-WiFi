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
 * Copy this file to rt-wifi-local.h, and modify it accordingly before build
 * RT-WiFi module */
#ifndef RT_WIFI_LOCAL_H
#define RT_WIFI_LOCAL_H

/* TDMA configuration */
/* TDMA slot size, see definition in rt-wifi.h */
#define RT_WIFI_TIME_SLOT_LEN 7

/* Transmission configuration */
/* co-existence */
#define RT_WIFI_ENABLE_COEX	0
/* # of in-slot transmission count */
#define RT_WIFI_NUM_OF_TRIES	1
/* Whether the sender will wait for ACK or not */
#define RT_WIFI_ENABLE_ACK	1


/* Debug or log information */
/* Uncomment the following flags to enable the corresponding logging.*/
/* Please note that logging may influence system performance. */
/* For low prioirty logging, please use trace_printk() instead. */
// #define RT_WIFI_DEBUG_ENABLE
// #define RT_WIFI_ALERT_ENABLE

#endif /* RT_WIFI_LOCAL_H */
