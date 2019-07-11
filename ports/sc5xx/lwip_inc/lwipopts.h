#ifndef MICROPY_INCLUDED_SC5XX_LWIP_LWIPOPTS_H
#define MICROPY_INCLUDED_SC5XX_LWIP_LWIPOPTS_H

#include <stdint.h>

#define LWIP_DEBUG 1
#define TCP_DEBUG                       LWIP_DBG_OFF
#define ETHARP_DEBUG                    LWIP_DBG_OFF
#define PBUF_DEBUG                      LWIP_DBG_OFF
#define IP_DEBUG                        LWIP_DBG_OFF
#define TCPIP_DEBUG                     LWIP_DBG_OFF
#define DHCP_DEBUG                      LWIP_DBG_OFF
#define UDP_DEBUG                       LWIP_DBG_OFF

#define NO_SYS                          1
#define SYS_LIGHTWEIGHT_PROT            1
#define MEM_ALIGNMENT                   4

//#define LWIP_CHKSUM_ALGORITHM           3
//#define LWIP_CHECKSUM_CTRL_PER_NETIF    1

//#define LWIP_NETIF_TX_SINGLE_PBUF       1

#define LWIP_ARP                        1
#define LWIP_ETHERNET                   1
#define LWIP_NETCONN                    0
#define LWIP_SOCKET                     0
#define LWIP_STATS                      0
#define LWIP_NETIF_HOSTNAME             1

#define LWIP_IPV6                       0
#define LWIP_DHCP                       1
#define LWIP_DHCP_CHECK_LINK_UP         1
#define LWIP_DNS                        1
#define LWIP_IGMP                       1

//#define SO_REUSE                        1

#define MEM_SIZE (16000)
#define TCP_MSS (1460)
#define TCP_WND (8 * TCP_MSS)
#define TCP_SND_BUF (8 * TCP_MSS)
#define MEMP_NUM_TCP_SEG (32)

typedef uint32_t sys_prot_t;

#endif // MICROPY_INCLUDED_SC5XX_LWIP_LWIPOPTS_H
