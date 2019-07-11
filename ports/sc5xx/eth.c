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

#include <string.h>
#include <services/int/adi_int.h>
#include <adi_osal.h>
#include "py/mphal.h"
#include "py/mperrno.h"
#include "lib/netutils/netutils.h"
#include "eth.h"
#include "modnetwork.h"
#include "systick.h"

#if defined(MICROPY_HW_EMAC)

#include "lwip/etharp.h"
#include "lwip/dns.h"
#include "lwip/dhcp.h"
#include "netif/ethernet.h"

#include "gemac/adi_ether.h"
#include "gemac/adi_ether_gemac.h"

#define MICROPY_HW_EMAC0_CRS          (pin_A7)
#define MICROPY_HW_EMAC0_MDC          (pin_A2)
#define MICROPY_HW_EMAC0_MDIO         (pin_A3)
#define MICROPY_HW_EMAC0_PTPAUXIN0    (pin_B3)
#define MICROPY_HW_EMAC0_PTPCLKIN0    (pin_B2)
#define MICROPY_HW_EMAC0_PTPPPS0      (pin_B1)
#define MICROPY_HW_EMAC0_PTPPPS1      (pin_B0)
#define MICROPY_HW_EMAC0_PTPPPS2      (pin_A15)
#define MICROPY_HW_EMAC0_PTPPPS3      (pin_A14)
#define MICROPY_HW_EMAC0_RXCLK_REFCLK (pin_A6)
#define MICROPY_HW_EMAC0_RXD0         (pin_A4)
#define MICROPY_HW_EMAC0_RXD1         (pin_A5)
#define MICROPY_HW_EMAC0_RXD2         (pin_A8)
#define MICROPY_HW_EMAC0_RXD3         (pin_A9)
#define MICROPY_HW_EMAC0_TXCLK        (pin_A11)
#define MICROPY_HW_EMAC0_TXD0         (pin_A0)
#define MICROPY_HW_EMAC0_TXD1         (pin_A1)
#define MICROPY_HW_EMAC0_TXD2         (pin_A12)
#define MICROPY_HW_EMAC0_TXD3         (pin_A13)
#define MICROPY_HW_EMAC0_TXEN         (pin_A10)

#define AF0_EMAC0 0

#define PRINTF(x) mp_printf(&mp_plat_print, x)

// EMAC0 Driver Entry point
#define EMAC_DRIVER_ENTRY (GEMAC0DriverEntry)
// Number of receive DMA descriptors
#define EMAC_NUM_RECV_DESC    (40)
// Number of transmit DMA descriptors
#define EMAC_NUM_XMIT_DESC    (40)

#define ETHERNET_MTU        (2000)
#define ETHERNET_MAX_SIZE   (ETHERNET_MTU + 22 + sizeof(uint16_t))
#define NUM_TX_BUFFS        (128)
#define NUM_RX_BUFFS        (128)

// DMA memory
static uint8_t base_mem_size[32]  __attribute__((aligned(32)));
// Receive DMA descriptor memory
static uint8_t mem_receive[EMAC_NUM_RECV_DESC*32]  __attribute__((aligned(32)));
// Transmit DMA descriptor memory
static uint8_t mem_transmit[EMAC_NUM_XMIT_DESC*32]  __attribute__((aligned(32)));

// Transmission buffer
static uint8_t txpkt[NUM_TX_BUFFS][ETHERNET_MAX_SIZE] __attribute__((aligned(32)));
static ADI_ETHER_BUFFER txbuff[NUM_TX_BUFFS] __attribute__((aligned(32)));
static volatile uint32_t tx_pkt_head;
static volatile uint32_t tx_pkt_tail;
static volatile uint32_t tx_pkt_count;

// Receive buffer
static uint8_t rxpkt[NUM_RX_BUFFS][ETHERNET_MAX_SIZE] __attribute__((aligned(32)));
static ADI_ETHER_BUFFER rxbuff[NUM_RX_BUFFS] __attribute__((aligned(32)));
static ADI_ETHER_BUFFER *pkt_received_buffer;
static ADI_ETHER_BUFFER *pkt_received_buffer_head;
static volatile bool pkt_received;

static volatile bool link_ready = false;

static volatile int tx_count = 0;
static volatile int rx_count = 0;
static volatile int tx_drop_count = 0;
static volatile int rx_drop_count = 0;

// Initialize the Memory Table
ADI_ETHER_MEM memtable =
{
    mem_receive, sizeof(mem_receive),
    mem_transmit, sizeof(mem_transmit),
    base_mem_size, sizeof(base_mem_size)
};

typedef struct _eth_t {
    mod_network_nic_type_t base;
    ADI_ETHER_HANDLE h_ethernet;
    uint32_t trace_flags;
    struct netif netif;
    struct dhcp dhcp_struct;
} eth_t;

eth_t eth_instance;

void eth_mac_deinit(eth_t *self);
void eth_irq_callback(void *pDev, uint32_t event, void *param);

static void reset_tx_buff(ADI_ETHER_BUFFER *txbuff)
{
    txbuff->ElementCount = 0;
    txbuff->ElementWidth = 1;
    txbuff->ProcessedFlag = 0;
    txbuff->nChannel = 0;
    txbuff->pNext = NULL;
}

static void reset_rx_buff(ADI_ETHER_BUFFER *rxbuff)
{
    rxbuff->ElementCount = ETHERNET_MAX_SIZE;
    rxbuff->ElementWidth = 1;
    rxbuff->ProcessedFlag = 0;
    rxbuff->nChannel = 0;
}

void eth_init(eth_t *self) {
    // Read MAC address from OCOTP memory.
    // Note only SAM boards have MAC addresses, EZKITs don't.
    uint32_t gp1[16];
    uint32_t *addr = (uint32_t *)0x24000300;
    for (int i = 0; i < 16; i++)
        gp1[i] = *addr++;
    uint8_t *mac = (uint8_t *)gp1;
    self->netif.hwaddr[0] = mac[3];
    self->netif.hwaddr[1] = mac[2];
    self->netif.hwaddr[2] = mac[1];
    self->netif.hwaddr[3] = mac[0];
    self->netif.hwaddr[4] = mac[7];
    self->netif.hwaddr[5] = mac[6];
    self->netif.hwaddr_len = 6;

    //self->base.poll_callback = eth_polling_callback;

    // Initialize tx buffer
    memset(&txpkt, 0, sizeof(txpkt));
    memset(&txbuff, 0, sizeof(txbuff));
    for (int i = 0; i < NUM_TX_BUFFS; i++) {
        txbuff[i].Data = &txpkt[i];
        reset_tx_buff(&txbuff[i]);
        txbuff[i].ProcessedFlag = 1; // By default they are all available
    }

    // Initialize rx buffer
    memset(&rxpkt, 0, sizeof(rxpkt));
    memset(&rxbuff, 0, sizeof(rxbuff));
    for (int i = 0; i < NUM_RX_BUFFS; i++) {
        reset_rx_buff(&rxbuff[i]);
        rxbuff[i].Data = &rxpkt[i][0];
        rxbuff[i].pNext = NULL;
        if (i > 0) {
            rxbuff[i-1].pNext = &rxbuff[i];
        }
    }
    pkt_received = false;

    mp_hal_pin_config((pin_obj_t *)MICROPY_HW_EMAC0_CRS, GPIO_MODE_AF_PP, 0, AF0_EMAC0);
    mp_hal_pin_config((pin_obj_t *)MICROPY_HW_EMAC0_MDC, GPIO_MODE_AF_PP, 0, AF0_EMAC0);
    mp_hal_pin_config((pin_obj_t *)MICROPY_HW_EMAC0_MDIO, GPIO_MODE_AF_PP, 0, AF0_EMAC0);
    mp_hal_pin_config((pin_obj_t *)MICROPY_HW_EMAC0_PTPAUXIN0, GPIO_MODE_AF_PP, 0, AF0_EMAC0);
    mp_hal_pin_config((pin_obj_t *)MICROPY_HW_EMAC0_PTPCLKIN0, GPIO_MODE_AF_PP, 0, AF0_EMAC0);
    mp_hal_pin_config((pin_obj_t *)MICROPY_HW_EMAC0_PTPPPS0, GPIO_MODE_AF_PP, 0, AF0_EMAC0);
    mp_hal_pin_config((pin_obj_t *)MICROPY_HW_EMAC0_PTPPPS1, GPIO_MODE_AF_PP, 0, AF0_EMAC0);
    mp_hal_pin_config((pin_obj_t *)MICROPY_HW_EMAC0_PTPPPS2, GPIO_MODE_AF_PP, 0, AF0_EMAC0);
    mp_hal_pin_config((pin_obj_t *)MICROPY_HW_EMAC0_PTPPPS3, GPIO_MODE_AF_PP, 0, AF0_EMAC0);
    mp_hal_pin_config((pin_obj_t *)MICROPY_HW_EMAC0_RXCLK_REFCLK, GPIO_MODE_AF_PP, 0, AF0_EMAC0);
    mp_hal_pin_config((pin_obj_t *)MICROPY_HW_EMAC0_RXD0, GPIO_MODE_AF_PP, 0, AF0_EMAC0);
    mp_hal_pin_config((pin_obj_t *)MICROPY_HW_EMAC0_RXD1, GPIO_MODE_AF_PP, 0, AF0_EMAC0);
    mp_hal_pin_config((pin_obj_t *)MICROPY_HW_EMAC0_RXD2, GPIO_MODE_AF_PP, 0, AF0_EMAC0);
    mp_hal_pin_config((pin_obj_t *)MICROPY_HW_EMAC0_RXD3, GPIO_MODE_AF_PP, 0, AF0_EMAC0);
    mp_hal_pin_config((pin_obj_t *)MICROPY_HW_EMAC0_TXCLK, GPIO_MODE_AF_PP, 0, AF0_EMAC0);
    mp_hal_pin_config((pin_obj_t *)MICROPY_HW_EMAC0_TXD0, GPIO_MODE_AF_PP, 0, AF0_EMAC0);
    mp_hal_pin_config((pin_obj_t *)MICROPY_HW_EMAC0_TXD1, GPIO_MODE_AF_PP, 0, AF0_EMAC0);
    mp_hal_pin_config((pin_obj_t *)MICROPY_HW_EMAC0_TXD2, GPIO_MODE_AF_PP, 0, AF0_EMAC0);
    mp_hal_pin_config((pin_obj_t *)MICROPY_HW_EMAC0_TXD3, GPIO_MODE_AF_PP, 0, AF0_EMAC0);
    mp_hal_pin_config((pin_obj_t *)MICROPY_HW_EMAC0_TXEN, GPIO_MODE_AF_PP, 0, AF0_EMAC0);
}

void eth_set_trace(eth_t *self, uint32_t value) {
    self->trace_flags = value;
}

void eth_disable_int() {
    sys_tick_disable_irq();
}

void eth_enable_int() {
    sys_tick_enable_irq();
}

int eth_mac_init(eth_t *self) {

    ADI_ETHER_RESULT ether_result;
    ADI_ETHER_DEV_INIT ether_init_data= { true, &memtable, NULL }; // data-cache, driver memory

#ifdef  __ADSPSC589_FAMILY__
    // Bring the phy out of reset
    *pREG_PORTB_FER_CLR  = BITM_PORT_FER_CLR_PX14;
    *pREG_PORTB_DIR_SET  = BITM_PORT_DIR_SET_PX14; //choose output mode

    *pREG_PORTB_DATA_CLR = BITM_PORT_DATA_CLR_PX14; //drive 0
    mp_hal_delay_ms(50);

    *pREG_PORTB_DATA_SET = BITM_PORT_DATA_SET_PX14; //drive 1
    mp_hal_delay_ms(50);
#endif

#ifdef  __ADSPSC573_FAMILY__
    // Bring the phy out of reset
    *pREG_PORTA_FER_CLR  = BITM_PORT_FER_CLR_PX5;
    *pREG_PORTA_DIR_SET  = BITM_PORT_DIR_SET_PX5; //choose output mode

    *pREG_PORTA_DATA_CLR = BITM_PORT_DATA_CLR_PX5; //drive 0
    SLEEP_MSEC(500)

    *pREG_PORTA_DATA_SET = BITM_PORT_DATA_SET_PX5; //drive 1
    SLEEP_MSEC(500);
#endif

    // Select RGMII for EMAC0
#ifdef __ADSPSC573_FAMILY__
    *pREG_PADS0_PCFG0 &= ~(BITM_PADS_PCFG0_EMACPHYISEL);
    *pREG_PADS0_PCFG0 |=  (1<<BITP_PADS_PCFG0_EMACPHYISEL);
#endif
#ifdef  __ADSPSC589_FAMILY__
    *pREG_PADS0_PCFG0 |= BITM_PADS_PCFG0_EMACPHYISEL;
#endif

    // Bring EMAC0 out of reset
    *pREG_PADS0_PCFG0 |= BITM_PADS_PCFG0_EMACRESET;

    // SPU Settings for EMAC0 and EMAC1
#ifdef __ADSPSC573_FAMILY__
    *pREG_SPU0_SECUREP40 = 0x03;
#endif
#ifdef __ADSPSC589_FAMILY__
    *pREG_SPU0_SECUREP55 = 0x03;
    *pREG_SPU0_SECUREP56 = 0x03;
#endif

    memset(mem_receive, 0, sizeof(mem_receive));
    memset(mem_transmit, 0, sizeof(mem_transmit));
    memset(base_mem_size, 0, sizeof(base_mem_size));

    // open ethernet device
    ether_result = adi_ether_Open(
            &(EMAC_DRIVER_ENTRY), // Driver Entry
            &ether_init_data, // Driver Initialization data
            eth_irq_callback, // Driver callback
            &self->h_ethernet // Pointer to get driver handle
            );

    if (ether_result != ADI_ETHER_RESULT_SUCCESS) {
        PRINTF("eth_mac_init: failed to open ethernet driver\n");
        return (-1);
    }

    // Set the MAC address
    ether_result = adi_ether_SetMACAddress(self->h_ethernet, (uint8_t*)&self->netif.hwaddr[0]);

    if (ether_result != ADI_ETHER_RESULT_SUCCESS) {
        PRINTF("eth_mac_init: failed to set MAC address\n");
        return (-1);
    }

    // Start the MAC
    ether_result = adi_ether_EnableMAC(self->h_ethernet);

    if (ether_result != ADI_ETHER_RESULT_SUCCESS) {
        PRINTF("eth_mac_init: failed to enable EMAC\n");
        return (-1);
    }

    // Give the receive buffer chain to the driver
    adi_ether_Read(self->h_ethernet, rxbuff);

    // Wait for link up here or not?

    return 0;
}

void eth_mac_deinit(eth_t *self) {
    // TODO: Figure out correct way to disable it.
    adi_ether_Close(self->h_ethernet);
}

// Receive a frame into lwIP
void eth_lwip_rx_frame(struct netif *netif, ADI_ETHER_BUFFER *rxbuff)
{
    uint16_t len;
    struct pbuf *p, *q;
    uint8_t *in;
    err_t err;

    // Get the total length from the first 2 bytes of the frame
    in = (uint8_t *)rxbuff->Data;
    len = *((uint16_t *)in);

    in += 2;
    len -= 2;

    /* Allocate an lwIP pbuf to carry the data, copy, and submit it */
    p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);

    if (p != NULL) {
        for (q = p; q != NULL; q = q->next) {
            memcpy(q->payload, in, q->len);
            in += q->len;
        }
        LINK_STATS_INC(link.recv);

        err = netif->input(p, netif);
        if (err != ERR_OK) {
            pbuf_free(p);
        }
    }
}

void eth_timer_polling() {
    static bool old_link_ready = false;

    /* Check for packet reception */
    adi_osal_EnterCriticalRegion();
    if (pkt_received == true) {
        pkt_received_buffer_head = pkt_received_buffer;
        while (pkt_received_buffer) {
            eth_lwip_rx_frame(&eth_instance.netif, pkt_received_buffer);
            reset_rx_buff(pkt_received_buffer);
            pkt_received_buffer = pkt_received_buffer->pNext;
        }
        adi_ether_Read(eth_instance.h_ethernet, pkt_received_buffer_head);
        pkt_received = false;
    }

    /* Check for packet transmission */
    if (link_ready && (tx_pkt_head != tx_pkt_tail)) {
        while (tx_pkt_tail != tx_pkt_head) {
            ADI_ETHER_RESULT etherResult = adi_ether_Write(eth_instance.h_ethernet, &txbuff[tx_pkt_tail]);
            if (etherResult == ADI_ETHER_RESULT_SUCCESS) {
                LINK_STATS_INC(link.xmit);
                tx_pkt_tail++;
                if (tx_pkt_tail == NUM_TX_BUFFS) {
                    tx_pkt_tail = 0;
                }
            }
        }
    }
    adi_osal_ExitCriticalRegion();

    if (link_ready != old_link_ready) {
        if (link_ready) {
            netif_set_link_up(&eth_instance.netif);
        } else {
            netif_set_link_down(&eth_instance.netif);
        }
        old_link_ready = link_ready;
    }
}

void eth_irq_callback(void *pDev, uint32_t event, void *param) {
    ADI_ETHER_BUFFER *p_buffer = (ADI_ETHER_BUFFER*)param;
    ADI_ETHER_BUFFER *append;
    uint32_t status;

    sys_tick_disable_irq();

    switch(event)
    {
        case ADI_ETHER_EVENT_FRAME_XMIT:
            break;

        case ADI_ETHER_EVENT_FRAME_RCVD:
            if (pkt_received == true) {
                append = (ADI_ETHER_BUFFER *)pkt_received_buffer;
                while (append->pNext) {
                    append = append->pNext;
                }
                append->pNext = p_buffer;
            } else {
                pkt_received_buffer = p_buffer;
                pkt_received = true;
            }
            break;

        case ADI_ETHER_EVENT_PHY_INTERRUPT:
            status = (uint32_t)param;
            if (status & ADI_ETHER_PHY_AN_COMPLETE) {
                link_ready = true;
            } else if (status & ADI_ETHER_PHY_LINK_DOWN) {
                link_ready = false;
            }
            break;

        default:
            break;
    }

    sys_tick_enable_irq();
}

/*******************************************************************************/
// ETH-LwIP bindings

#define TRACE_ASYNC_EV (0x0001)
#define TRACE_ETH_TX (0x0002)
#define TRACE_ETH_RX (0x0004)
#define TRACE_ETH_FULL (0x0008)

/*void eth_trace(eth_t *self, size_t len, const void *data, unsigned int flags) {
    if (((flags & NETUTILS_TRACE_IS_TX) && (self->trace_flags & TRACE_ETH_TX))
        || (!(flags & NETUTILS_TRACE_IS_TX) && (self->trace_flags & TRACE_ETH_RX))) {
        const uint8_t *buf;
        if (len == (size_t)-1) {
            // data is a pbuf
            const struct pbuf *pbuf = data;
            buf = pbuf->payload;
            len = pbuf->len; // restricted to print only the first chunk of the pbuf
        } else {
            // data is actual data buffer
            buf = data;
        }
        if (self->trace_flags & TRACE_ETH_FULL) {
            flags |= NETUTILS_TRACE_PAYLOAD;
        }
        netutils_ethernet_trace(MP_PYTHON_PRINTER, len, buf, flags);
    }
}*/

err_t eth_netif_output(struct netif *netif, struct pbuf *p) {
    uint16_t len;
    uint32_t check;
    struct pbuf *q;
    uint8_t *out;

    // Check for overflow.  Drop if necessary.
    check = tx_pkt_head + 1;
    if (check == NUM_TX_BUFFS) {
        check = 0;
    }
    if (check == tx_pkt_tail) {
        return(ERR_OK);
    }

    // Put the total length into the first 2 bytes of the frame
    len = p->tot_len + sizeof(uint16_t);

    out = (uint8_t *)txbuff[tx_pkt_head].Data;
    *((uint16_t *)out) = len;

    out += sizeof(uint16_t);

    // Insert the lwIP payload into the frame
    for (q = p; q != NULL; q = q->next) {
        memcpy(out, q->payload, q->len);
        out += q->len;
    }

    // Prepare the ADI_ETHER_BUFFER for transmission by the driver
    reset_tx_buff(&txbuff[tx_pkt_head]);
    txbuff[tx_pkt_head].ElementCount = len;

    /* Indicate to the polling loop the packet is ready to go */
    tx_pkt_head++;
    if (tx_pkt_head == NUM_TX_BUFFS) {
        tx_pkt_head = 0;
    }

    return (ERR_OK);
}

err_t eth_netif_init(struct netif *netif) {
    netif->linkoutput = eth_netif_output;
    netif->output = etharp_output;
    netif->mtu = 1500;
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_ETHERNET | NETIF_FLAG_IGMP;
    // Checksums only need to be checked on incoming frames, not computed on outgoing frames
    NETIF_SET_CHECKSUM_CTRL(netif,
        NETIF_CHECKSUM_CHECK_IP
        | NETIF_CHECKSUM_CHECK_UDP
        | NETIF_CHECKSUM_CHECK_TCP
        | NETIF_CHECKSUM_CHECK_ICMP
        | NETIF_CHECKSUM_CHECK_ICMP6);
    return ERR_OK;
}

void eth_lwip_init(eth_t *self) {
    ip_addr_t ipconfig[4];
    IP4_ADDR(&ipconfig[0], 0, 0, 0, 0);
    IP4_ADDR(&ipconfig[2], 0, 0, 0, 0);
    IP4_ADDR(&ipconfig[1], 255, 255, 255, 0);
    IP4_ADDR(&ipconfig[3], 8, 8, 8, 8);

    MICROPY_PY_LWIP_ENTER

    struct netif *n = &self->netif;
    n->name[0] = 'e';
    n->name[1] = '0';
    netif_add(n, &ipconfig[0], &ipconfig[1], &ipconfig[2], self, eth_netif_init, ethernet_input);
    netif_set_hostname(n, "MPY");
    netif_set_default(n);
    netif_set_up(n);

    dns_setserver(0, &ipconfig[3]);
    dhcp_set_struct(n, &self->dhcp_struct);
    dhcp_start(n);

    MICROPY_PY_LWIP_EXIT
}

void eth_lwip_deinit(eth_t *self) {
    MICROPY_PY_LWIP_ENTER
    for (struct netif *netif = netif_list; netif != NULL; netif = netif->next) {
        if (netif == &self->netif) {
            netif_remove(netif);
            netif->ip_addr.addr = 0;
            netif->flags = 0;
        }
    }
    MICROPY_PY_LWIP_EXIT
}

struct netif *eth_netif(eth_t *self) {
    return &self->netif;
}

int eth_link_status(eth_t *self) {
    struct netif *netif = &self->netif;
    if ((netif->flags & (NETIF_FLAG_UP | NETIF_FLAG_LINK_UP))
        == (NETIF_FLAG_UP | NETIF_FLAG_LINK_UP)) {
        if (netif->ip_addr.addr != 0) {
            return 3; // link up
        } else {
            return 2; // link no-ip;
        }
    } else {
        return 0; // link down
    }
}

int eth_start(eth_t *self) {
    eth_lwip_deinit(self);
    int ret = eth_mac_init(self);
    if (ret < 0) {
        return ret;
    }
    eth_lwip_init(self);
    return 0;
}

int eth_stop(eth_t *self) {
    eth_lwip_deinit(self);
    eth_mac_deinit(self);
    return 0;
}

#endif // defined(MICROPY_HW_ETH_MDC)
