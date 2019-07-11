/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013, 2014 Damien P. George
 * Copyright (c) 2019 Analog Devices, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#ifndef MICROPY_INCLUDED_SC5XX_MODNETWORK_H
#define MICROPY_INCLUDED_SC5XX_MODNETWORK_H

#define MOD_NETWORK_IPADDR_BUF_SIZE (4)

#define MOD_NETWORK_AF_INET (2)
#define MOD_NETWORK_AF_INET6 (10)

#define MOD_NETWORK_SOCK_STREAM (1)
#define MOD_NETWORK_SOCK_DGRAM (2)
#define MOD_NETWORK_SOCK_RAW (3)

#if MICROPY_PY_LWIP

#define MICROPY_PY_LWIP_ENTER eth_disable_int();
#define MICROPY_PY_LWIP_EXIT  eth_enable_int();

struct netif;

typedef struct _mod_network_nic_type_t {
    mp_obj_base_t base;
    void (*poll_callback)(void *data, struct netif *netif);
} mod_network_nic_type_t;

extern const mp_obj_type_t network_gemac_type;

void mod_network_lwip_poll_wrapper(uint32_t ticks_ms);
mp_obj_t mod_network_nic_ifconfig(struct netif *netif, size_t n_args, const mp_obj_t *args);

#else

#error "For sc5xx port, modnetwork must be used with lwip"

#endif

void mod_network_init(void);
void mod_network_deinit(void);
void mod_network_register_nic(mp_obj_t nic);
mp_obj_t mod_network_find_nic(const uint8_t *ip);

#endif // MICROPY_INCLUDED_SC5XX_MODNETWORK_H
