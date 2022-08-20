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

/* RUN FLAG */
#define SLCAN_RFLAG_CAN_OPEN           0x0001
#define SLCAN_RFLAG_CHAR_OPEN          0x0002
#define SLCAN_RFLAG_EXIT               0x8000

/* SET FLAG */
#define SLCAN_SFLAG_MODE            0x0001
#define SLCAN_SFLAG_BUADRATE        0x0002
#define SLCAN_SFLAG_TIMESTAMP       0x0004
#define SLCAN_SFLAG_LOG             0x0008


typedef struct rt_slcan
{
    rt_device_t  candev;
    rt_err_t     (*candev_rx_call)(rt_device_t dev, rt_size_t size);
    const char * candev_name;
    int          candev_oflag;
    uint32_t     candev_baud;
    uint32_t     candev_mode;

    rt_device_t  serialdev;
    rt_err_t     (*serialdev_rx_call)(rt_device_t dev, rt_size_t size);
    const char * serialdev_name;
    int          serialdev_oflag;

    struct rt_semaphore  rx_sem;
    struct rt_can_msg   can_msg;

    uint8_t      slcan_baud_index;
    uint8_t      slcan_mode_index;

    uint16_t     run_state;
    uint16_t     run_flag;
    uint16_t     set_flag;

    uint16_t     char_rx_remain;
    uint8_t      char_rx_buffer[SLCAN_MTU + SLCAN_MTU];
    uint8_t      char_tx_buffer[SLCAN_MTU];

    rt_thread_t  tid;
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
    SLCAN_MODE_LISTEN,
    SLCAN_MODE_LOOPBACK,
    SLCAN_MODE_LOOPBACKANLISTEN,
};

void slcan_process_task(void *instance);

int slcan_instance_startup(rt_slcan_t * slcan_instance, char *name,  rt_uint32_t stack_size, rt_uint8_t  priority, rt_uint32_t tick);

void slcan_instance_exit(rt_slcan_t* slcan_instance);

rt_slcan_t* slcan_instance_create(const char * serialdev_name, const char * candev_name);

void slcan_instance_delete(rt_slcan_t * slcan_instance);

#ifdef __cplusplus
}
#endif

#endif // _SLCAN_H
