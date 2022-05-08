#ifndef _SLCAN_H
#define _SLCAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "stdio.h"
#include "string.h"
#include "rtthread.h"
#include "rtdevice.h"

#ifdef RT_USING_CAN
#include "drivers/can.h"
#endif /* RT_USING_CAN */

/* maximum rx buffer len: extended CAN frame with timestamp */
#define SLCAN_MTU (sizeof("T1111222281122334455667788EA5F\r")+1)

#define SLCAN_CMD_LEN 1
#define SLCAN_STD_ID_LEN 3
#define SLCAN_EFF_ID_LEN 8


typedef struct rt_slcan
{
    rt_device_t  candev;
    rt_err_t     (*candev_rx_call)(rt_device_t dev, rt_size_t size);
    char *       candev_name;
    int          candev_oflag;
    uint32_t     candev_baud;
    uint32_t     candev_mode;

    rt_device_t  chardev;
    rt_err_t     (*chardev_rx_call)(rt_device_t dev, rt_size_t size);
    char *       chardev_name;
    int          chardev_oflag;

    struct rt_semaphore  rx_sem;
    struct rt_can_msg   can_msg;

    uint8_t      slcan_baud_index;
    uint8_t      slcan_mode_index;
    uint8_t      timestamp_isopen;
    uint8_t      candev_isopen;

    uint16_t     chardev_rx_remain;
    uint8_t      chardev_rx_buffer[SLCAN_MTU + SLCAN_MTU];
    uint8_t      chardev_tx_buffer[SLCAN_MTU];

}rt_slcan_t;


enum slcan_baud
{
    SLCAN_BAUD_10K = 0,
    SLCAN_BAUD_20K,
    SLCAN_BAUD_50K,
    SLCAN_BAUD_100K,
    SLCAN_BAUD_125K,
    SLCAN_BAUD_250K,
    SLCAN_BAUD_500K,
    SLCAN_BAUD_800K,
    SLCAN_BAUD_1000K,
    SLCAN_BAUD_INVALID,
};

enum slcan_mode
{
 SLCAN_MODE_NORMAL = 0,
 SLCAN_MODE_LISEN,
 SLCAN_MODE_LOOPBACK,
 SLCAN_MODE_LOOPBACKANLISEN,
};

#ifdef __cplusplus
}
#endif

#endif // _SLCAN_H
