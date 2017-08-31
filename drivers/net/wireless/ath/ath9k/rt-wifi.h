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
#ifndef RT_WIFI_H
#define RT_WIFI_H

#include <linux/nl80211.h>
#include "ath9k.h"
#include "rt-wifi-local.h"

/* rt-wifi constant */
#define RT_WIFI_TIMER_OFFSET  50	/* compensate the isr invoking time */
#define RT_WIFI_TSF_SYNC_OFFSET 1000 /* max sync error in micro-sec */
#define RT_WIFI_BEACON_VEN_EXT_SIZE 17  // Size of vendor extension in beacon frame
#define BEACON_FCS_SIZE 4    		// Size of frame check sequence in byte
#define RT_WIFI_BEACON_TAG    0xFE
#define RT_WIFI_BEACON_INTVAL 102400         /* in terms of microsec, default beacon time interval 100 time units
						where 1TU = 1024 micro-sec */
#define RT_WIFI_BEACON_DELAY_OFFSET 60    /* in terms of micro-sec, this value is highly dependent on beacon data rate, 60us for 24Mbps */

#define RT_WIFI_KFIFO_SIZE 2048
// Make sure this # is the same as ath_tx_buf_size!!
// We do not take care of FIFO overflow.

#define RT_WIFI_TIME_SLOT_64TU  0
#define RT_WIFI_TIME_SLOT_32TU  1
#define RT_WIFI_TIME_SLOT_16TU  2
#define RT_WIFI_TIME_SLOT_8TU   3
#define RT_WIFI_TIME_SLOT_4TU   4
#define RT_WIFI_TIME_SLOT_2TU   5
#define RT_WIFI_TIME_SLOT_1TU   6
#define RT_WIFI_TIME_SLOT_512us 7
#define RT_WIFI_TIME_SLOT_256us 8
#define RT_WIFI_TIME_SLOT_128us 9

/* Macro for debugging */
#ifdef RT_WIFI_DEBUG_ENABLE
#define RT_WIFI_DEBUG(fmt, args...) printk(KERN_DEBUG "RT_WIFI: " fmt, ##args)
#else
#define RT_WIFI_DEBUG(fmt, args...)
#endif

/* Macro for alert message*/
#ifdef RT_WIFI_ALERT_ENABLE
#define RT_WIFI_ALERT(fmt, args...) printk(KERN_ALERT "RT_WIFI: " fmt, ##args)
#else
#define RT_WIFI_ALERT(fmt, args...)
#endif


struct rt_wifi_sta {
	u8 mac_addr[ETH_ALEN];
};

/* link scheduling */
enum rt_wifi_link_type{
	RT_WIFI_BEACON,
	RT_WIFI_TX,
	RT_WIFI_RX,
	RT_WIFI_SHARED,
};

struct rt_wifi_sched{
	enum rt_wifi_link_type type;
	u16 offset;
	u8 sta_id;
};

/* rt-wifi function prototype*/
void ath_rt_wifi_tasklet(struct ath_softc *sc);
void ath_rt_wifi_ap_start_timer(struct ath_softc *sc, u32 bcon_intval, u32 nexttbtt);
void ath_rt_wifi_sta_start_timer(struct ath_softc *sc);
void ath_rt_wifi_tx(struct ath_softc *sc, struct ath_buf *new_buf);
void ath_rt_wifi_rx_beacon(struct ath_softc *sc, struct sk_buff *skb);
struct ath_buf* ath_rt_wifi_get_buf_sta(struct ath_softc *sc);
bool rt_wifi_authorized_sta(u8 *addr);
inline bool rt_wifi_dst_sta(u8 *addr, u8 sta_id);
void ath9k_gen_timer_stop(struct ath_hw *ah, struct ath_gen_timer *timer);
/* existing functions not in rt-wifi*/
bool bf_is_ampdu_not_probing(struct ath_buf *bf);

/* other marco */
#define WLAN_GET_SEQ_SEQ(seq) (((seq) & IEEE80211_SCTL_SEQ) >> 4 )
#define WLAN_GET_SEQ_FRAG(seq) ((seq) & IEEE80211_SCTL_FRAG)

#endif /* RT_WIFI_H */
