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

rt_slcan_t * slcan_instance0 = NULL;

static rt_err_t slcan_instance0_rtcan_rx_call(rt_device_t dev, rt_size_t size)
{
    if(slcan_instance0 != NULL)
    {
        rt_sem_release(&(slcan_instance0->rx_sem));
    }
    return RT_EOK;
}

static rt_err_t slcan_instance0_rtchar_rx_call(rt_device_t dev, rt_size_t size)
{
    if(slcan_instance0 != NULL)
    {
        rt_sem_release(&(slcan_instance0->rx_sem));
    }
    return RT_EOK;
}


void slcan0_startup(uint8_t argc, char **argv)
{
    if(slcan_instance0 != NULL)
    {
        rt_kprintf("slcan is run\n");
        return;
    }
    rt_kprintf("slcan0 startup\n");

    slcan_instance0 = slcan_instance_create("uart2", "can1");
    if(slcan_instance0 != NULL)
    {
        //can device config
        slcan_instance0->candev_rx_call = slcan_instance0_rtcan_rx_call;
        slcan_instance0->candev_oflag = RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX;

        struct can_configure can_config = CANDEFAULTCONFIG;
        can_config.baud_rate = CAN1MBaud;
        rt_device_control(slcan_instance0->candev, RT_DEVICE_CTRL_CONFIG, &can_config);

        //serial device config
        slcan_instance0->chardev_rx_call = slcan_instance0_rtchar_rx_call;
        slcan_instance0->chardev_oflag = RT_DEVICE_FLAG_INT_RX;

        struct serial_configure serial_config;
        serial_config.baud_rate = BAUD_RATE_115200;
        serial_config.data_bits = DATA_BITS_8;
        serial_config.stop_bits = STOP_BITS_1;
        serial_config.bufsz = RT_SERIAL_RB_BUFSZ;
        serial_config.parity = PARITY_NONE;
        rt_device_control(slcan_instance0->chardev, RT_DEVICE_CTRL_CONFIG, &serial_config);

//        //slcan instance config
//        slcan_instance0->slcan_baud_index = SLCAN_BAUD_500K;
//        slcan_instance0->slcan_mode_index = SLCAN_MODE_NORMAL;
//        slcan_instance0->set_flag = SLCAN_SFLAG_MODE | SLCAN_SFLAG_BUADRATE | SLCAN_SFLAG_LOG;
        slcan_instance0->set_flag = SLCAN_SFLAG_LOG;

        slcan_instance_startup(slcan_instance0,"slcan0", 2048, 20, 10);
    }else {
        rt_kprintf("slcan0 startup fail\n");
    }
    return;
}
MSH_CMD_EXPORT(slcan0_startup, slcan0 startup);

void slcan0_exit(uint8_t argc, char **argv)
{
    rt_kprintf("slcan0 exit\n");
    slcan_instance_delete(slcan_instance0);
    slcan_instance0 = NULL;
}
MSH_CMD_EXPORT(slcan0_exit, slcan0 exit);


void slcan0_debug(uint8_t argc, char **argv)
{
    if(argc > 1)
    {
        int val = atoi(argv[1]);
        if(val > 0)
        {
            slcan_instance0->set_flag |= SLCAN_SFLAG_LOG;
        }else {
            slcan_instance0->set_flag &= ~SLCAN_SFLAG_LOG;
        }
        rt_kprintf("slcan0 debug set %d\n", val);
    }
}
MSH_CMD_EXPORT(slcan0_debug, slcan0 debug);

