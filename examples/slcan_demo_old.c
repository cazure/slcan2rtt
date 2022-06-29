/*
* Change Logs:
* Date           Author       Notes
* 2022-01-15     chenbin
*/
#include "stdint.h"
#include "stdio.h"
#include "string.h"
#include "rtthread.h"
#include "rtdevice.h"

#define DBG_TAG "slcanapp"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#include "slcan.h"

extern void slcan_process_task(void *p);

rt_slcan_t slcan_instance0 = {0};

static rt_err_t slcan_rtcan_rx_call(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(&(slcan_instance0.rx_sem));
    return RT_EOK;
}

static rt_err_t slcan_rtchar_rx_call(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(&(slcan_instance0.rx_sem));
    return RT_EOK;
}


void slcan_app_init(void)
{
    static int slcan_app_init = 0;
    rt_thread_t tid;
    if(slcan_app_init > 0)
    {
        return;
    }
    slcan_app_init = 1;

    slcan_instance0.candev_name = "can2";
    slcan_instance0.candev = rt_device_find(slcan_instance0.candev_name);
    slcan_instance0.candev_rx_call = slcan_rtcan_rx_call;
    slcan_instance0.candev_oflag = RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX;

    slcan_instance0.chardev_name = "vcom";
    slcan_instance0.chardev = rt_device_find(slcan_instance0.chardev_name);
    slcan_instance0.chardev_rx_call = slcan_rtchar_rx_call;
    slcan_instance0.chardev_oflag = RT_DEVICE_FLAG_INT_RX;

    slcan_instance0.slcan_baud_index = SLCAN_BAUD_500K;
    slcan_instance0.slcan_mode_index = SLCAN_MODE_NORMAL;

    rt_sem_init(&(slcan_instance0.rx_sem), "rx_sem", 0, RT_IPC_FLAG_FIFO);

    tid = rt_thread_create("slcan_app",slcan_process_task, &slcan_instance0,2048,20, 10);
    if (tid != RT_NULL)
            rt_thread_startup(tid);
    return;
}


void slcan_init(uint8_t argc, char **argv)
{
    rt_kprintf("slcan init\n");
    slcan_app_init();
}
MSH_CMD_EXPORT(slcan_init, slcan init);
