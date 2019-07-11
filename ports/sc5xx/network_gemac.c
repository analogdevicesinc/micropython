/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Damien P. George
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

#include "py/runtime.h"
#include "py/mphal.h"
#include "eth.h"
#include "modnetwork.h"

#if defined(MICROPY_HW_EMAC)

#include "lwip/netif.h"

typedef struct _network_gemac_obj_t {
    mp_obj_base_t base;
    eth_t *eth;
} network_gemac_obj_t;

STATIC const network_gemac_obj_t network_gemac_eth0 = { { &network_gemac_type }, &eth_instance };

STATIC void network_gemac_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    network_gemac_obj_t *self = MP_OBJ_TO_PTR(self_in);
    struct netif *netif = eth_netif(self->eth);
    int status = eth_link_status(self->eth);
    mp_printf(print, "<ETH %u %u.%u.%u.%u>",
        status,
        netif->ip_addr.addr & 0xff,
        netif->ip_addr.addr >> 8 & 0xff,
        netif->ip_addr.addr >> 16 & 0xff,
        netif->ip_addr.addr >> 24
    );
}

STATIC mp_obj_t network_gemac_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 0, 0, false);
    const network_gemac_obj_t *self = &network_gemac_eth0;
    eth_init(self->eth);
    return MP_OBJ_FROM_PTR(self);
}

STATIC mp_obj_t network_gemac_active(size_t n_args, const mp_obj_t *args) {
    network_gemac_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    if (n_args == 1) {
        return mp_obj_new_bool(eth_link_status(self->eth));
    } else {
        int ret;
        if (mp_obj_is_true(args[1])) {
            ret = eth_start(self->eth);
        } else {
            ret = eth_stop(self->eth);
        }
        if (ret < 0) {
            mp_raise_OSError(-ret);
        }
        return mp_const_none;
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(network_gemac_active_obj, 1, 2, network_gemac_active);

STATIC mp_obj_t network_gemac_isconnected(mp_obj_t self_in) {
    network_gemac_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_bool(eth_link_status(self->eth) == 3);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(network_gemac_isconnected_obj, network_gemac_isconnected);

STATIC mp_obj_t network_gemac_ifconfig(size_t n_args, const mp_obj_t *args) {
    network_gemac_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    return mod_network_nic_ifconfig(eth_netif(self->eth), n_args - 1, args + 1);
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(network_gemac_ifconfig_obj, 1, 2, network_gemac_ifconfig);

STATIC mp_obj_t network_gemac_status(size_t n_args, const mp_obj_t *args) {
    network_gemac_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    (void)self;

    if (n_args == 1) {
        // No arguments: return link status
        return MP_OBJ_NEW_SMALL_INT(eth_link_status(self->eth));
    }

    mp_raise_ValueError("unknown status param");
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(network_gemac_status_obj, 1, 2, network_gemac_status);

STATIC mp_obj_t network_gemac_config(size_t n_args, const mp_obj_t *args, mp_map_t *kwargs) {
    network_gemac_obj_t *self = MP_OBJ_TO_PTR(args[0]);

    if (kwargs->used == 0) {
        // Get config value
        if (n_args != 2) {
            mp_raise_TypeError("must query one param");
        }

        switch (mp_obj_str_get_qstr(args[1])) {
            case MP_QSTR_mac: {
                return mp_obj_new_bytes(&eth_netif(self->eth)->hwaddr[0], 6);
            }
            default:
                mp_raise_ValueError("unknown config param");
        }
    } else {
        // Set config value(s)
        if (n_args != 1) {
            mp_raise_TypeError("can't specify pos and kw args");
        }

        for (size_t i = 0; i < kwargs->alloc; ++i) {
            if (MP_MAP_SLOT_IS_FILLED(kwargs, i)) {
                mp_map_elem_t *e = &kwargs->table[i];
                switch (mp_obj_str_get_qstr(e->key)) {
                    case MP_QSTR_trace: {
                        eth_set_trace(self->eth, mp_obj_get_int(e->value));
                        break;
                    }
                    default:
                        mp_raise_ValueError("unknown config param");
                }
            }
        }

        return mp_const_none;
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(network_gemac_config_obj, 1, network_gemac_config);

STATIC const mp_rom_map_elem_t network_gemac_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_active), MP_ROM_PTR(&network_gemac_active_obj) },
    { MP_ROM_QSTR(MP_QSTR_isconnected), MP_ROM_PTR(&network_gemac_isconnected_obj) },
    { MP_ROM_QSTR(MP_QSTR_ifconfig), MP_ROM_PTR(&network_gemac_ifconfig_obj) },
    { MP_ROM_QSTR(MP_QSTR_status), MP_ROM_PTR(&network_gemac_status_obj) },
    { MP_ROM_QSTR(MP_QSTR_config), MP_ROM_PTR(&network_gemac_config_obj) },
};
STATIC MP_DEFINE_CONST_DICT(network_gemac_locals_dict, network_gemac_locals_dict_table);

const mp_obj_type_t network_gemac_type = {
    { &mp_type_type },
    .name = MP_QSTR_gemac,
    .print = network_gemac_print,
    .make_new = network_gemac_make_new,
    .locals_dict = (mp_obj_dict_t*)&network_gemac_locals_dict,
};

#endif // defined(MICROPY_HW_ETH_MDC)
