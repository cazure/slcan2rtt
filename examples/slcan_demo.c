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

#define DBG_TAG "slcan0"
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

static rt_err_t slcan_instance0_rtserial_rx_call(rt_device_t dev, rt_size_t size)
{
    if(slcan_instance0 != NULL)
    {
        rt_sem_release(&(slcan_instance0->rx_sem));
    }
    return RT_EOK;
}


void slcan0_startup(const char * serialdev_name, const char * candev_name)
{
    if(slcan_instance0 != NULL)
    {
        rt_kprintf("slcan is run\n");
        return;
    }
    if( (serialdev_name == NULL) || (candev_name == NULL))
    {
        slcan_instance0 = slcan_instance_create("uart2", "can1");
    }else {
        slcan_instance0 = slcan_instance_create(serialdev_name, candev_name);
    }
    if(slcan_instance0 != NULL)
    {
        //can device config
        slcan_instance0->candev_rx_call = slcan_instance0_rtcan_rx_call;
        slcan_instance0->candev_oflag = RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX;

//        struct can_configure can_config = CANDEFAULTCONFIG;
//        can_config.baud_rate = CAN1MBaud;
//        rt_device_control(slcan_instance0->candev, RT_DEVICE_CTRL_CONFIG, &can_config);

        //serial device config
        slcan_instance0->serialdev_rx_call = slcan_instance0_rtserial_rx_call;
        slcan_instance0->serialdev_oflag = RT_DEVICE_FLAG_INT_RX;

//        struct serial_configure serial_config;
//        serial_config.baud_rate = BAUD_RATE_115200;
//        serial_config.data_bits = DATA_BITS_8;
//        serial_config.stop_bits = STOP_BITS_1;
//        serial_config.bufsz = RT_SERIAL_RB_BUFSZ;
//        serial_config.parity = PARITY_NONE;
//        rt_device_control(slcan_instance0->chardev, RT_DEVICE_CTRL_CONFIG, &serial_config);

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

void slcan0_stop(void)
{
    rt_kprintf("slcan0 stop\n");
    slcan_instance_delete(slcan_instance0);
    slcan_instance0 = NULL;
}



#ifdef RT_USING_FINSH
#include <finsh.h>
#include <stdlib.h>

void slcan0(uint8_t argc, char **argv)
{
    if(strcmp(argv[1],"start" ) == 0)
    {
        slcan0_startup(argv[2], argv[3]);
    }else
    if(strcmp(argv[1],"stop" ) == 0)
    {
        slcan0_stop();
    }else
    if(strcmp(argv[1],"debug" ) == 0)
    {
        int val = atoi(argv[2]);
        if(val > 0)
        {
            slcan_instance0->set_flag |= SLCAN_SFLAG_LOG;
        }else {
            slcan_instance0->set_flag &= ~SLCAN_SFLAG_LOG;
        }
        rt_kprintf("slcan0 debug set %d\n", val);
    }else {

        rt_kprintf("slcan0 usage:\n");
        rt_kprintf("slcan0 start [serial dev] [can dev]\n");
        rt_kprintf("slcan0 stop\n");
        rt_kprintf("slcan0 debug  [0-1]\n");
    }
    return;
}
MSH_CMD_EXPORT(slcan0, slcan0);


int cmd_canconfig(int argc, void **argv)
{
    //rt_device_t candev = NULL;
    rt_can_t candev = NULL;
    int num= 0;
    if (argc == 1)
    {
        candev = (rt_can_t)rt_device_find(argv[1]);
        if (!candev)
        {
            rt_kprintf(" Can't find can device %s\n", argv[1]);
            return -1;
        }
        rt_kprintf(" Finded can device: %s...", argv[1]);

        rt_kprintf("baud_rate: %d\n",candev->config.baud_rate);
        rt_kprintf("mode:      %d\n",candev->config.mode);
    }
    if (argc >= 2)
    {
        candev = (rt_can_t)rt_device_find(argv[1]);
        if (!candev)
        {
            rt_kprintf(" Can't find can device %s\n", argv[1]);
            return -1;
        }
        rt_kprintf("find device: %s\n", argv[1]);
        rt_kprintf("baud: %d\n",candev->config.baud_rate);
        rt_kprintf("mode: %d\n",candev->config.mode);

        //struct can_configure can_config = candev->config;
        int isconfig = 0;

        for(int i = 2; i < argc; i++)
        {
            if(strcmp(argv[i],"-b") == 0)
            {
                num = atoi(argv[i+1]);
                if(num > 0)
                {
                    candev->config.baud_rate = num;
                    rt_kprintf("set baud : %d\n",candev->config.baud_rate);
                    isconfig =1;
                }
            }else
            if(strcmp(argv[i],"-m") == 0)
            {
                num = atoi(argv[i+1]);
                if(num >= 0)
                {
                    candev->config.mode = num;
                    rt_kprintf("set mode : %d\n",candev->config.mode);
                    isconfig =1;
                }
            }
        }
        if(isconfig > 0)
        {
            rt_device_control((rt_device_t)candev, RT_DEVICE_CTRL_CONFIG, &(candev->config) );
        }
    }
    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS( cmd_canconfig, __cmd_canconfig, Can Device config.);
#endif

