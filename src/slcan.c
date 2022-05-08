/*
* Change Logs:
* Date           Author       Notes
* 2022-03-16     chenbin    
*/
#include "slcan.h"

#include "stdint.h"
#include "stdio.h"
#include "string.h"
#include "rtthread.h"
#include "rtdevice.h"

#ifdef RT_USING_CAN
#include "drivers/can.h"
#endif /* RT_USING_CAN */

#define DBG_TAG "slcan"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>


void slcan_rtcan_set_baud(rt_slcan_t* slcan_instance, uint8_t baud_index);
void slcan_rtcan_set_mode(rt_slcan_t* slcan_instance, uint8_t mode_index);

uint32_t  can_baud_sclcan2rtthread(uint8_t can_baud_index)
{
	uint32_t baud = CAN500kBaud;
	switch(can_baud_index)
	{
		case SLCAN_BAUD_10K:
			baud = CAN10kBaud;
			break;
		case SLCAN_BAUD_20K:
			baud = CAN20kBaud;
			break;
		case SLCAN_BAUD_50K:
			baud = CAN50kBaud;
			break;
		case SLCAN_BAUD_100K:
			baud = CAN100kBaud;
			break;
		case SLCAN_BAUD_125K:
			baud = CAN125kBaud;
			break;
		case SLCAN_BAUD_250K:
			baud = CAN250kBaud;
			break;
		case SLCAN_BAUD_500K:
			baud = CAN500kBaud;
			break;
		case SLCAN_BAUD_800K:
			baud = CAN800kBaud;
			break;
		case SLCAN_BAUD_1000K:
			baud = CAN1MBaud;
			break;
	}
	return baud;
}

uint32_t  can_mode_sclcan2rtthread(uint8_t can_mode_index)
{
	uint32_t mode = RT_CAN_MODE_NORMAL;
	switch(can_mode_index)
	{
		case SLCAN_MODE_NORMAL:
			mode = RT_CAN_MODE_NORMAL;
			break;
		case SLCAN_MODE_LISEN:
			mode = RT_CAN_MODE_LISEN;
			break;
		case SLCAN_MODE_LOOPBACK:
			mode = RT_CAN_MODE_LOOPBACK;
			break;
		case SLCAN_MODE_LOOPBACKANLISEN:
			mode = RT_CAN_MODE_LOOPBACKANLISEN;
			break;
	}
	return mode;
}


void slcan_rtcan_open(rt_slcan_t* slcan_instance)
{
	int rc = rt_device_open(slcan_instance->candev, slcan_instance->candev_oflag);
	RT_ASSERT(rc == RT_EOK);
	rt_device_set_rx_indicate(slcan_instance->candev, slcan_instance->candev_rx_call);
	LOG_D("open:%s  %d\n",slcan_instance->candev_name,rc);
	slcan_rtcan_set_baud(slcan_instance, slcan_instance->slcan_baud_index);
	slcan_rtcan_set_mode(slcan_instance, slcan_instance->slcan_mode_index);
}

void slcan_rtcan_close(rt_slcan_t* slcan_instance)
{
	int rc = rt_device_close(slcan_instance->candev);
	RT_ASSERT(rc == RT_EOK);
	LOG_D("close:%s  %d\n",slcan_instance->candev_name,rc);
}

void slcan_rtcan_set_baud(rt_slcan_t* slcan_instance, uint8_t baud_index)
{
	slcan_instance->slcan_baud_index = baud_index;
	slcan_instance->candev_baud = can_baud_sclcan2rtthread(slcan_instance->slcan_baud_index);
  LOG_D("set baud : %s [%d] %d\n", slcan_instance->candev_name, slcan_instance->slcan_baud_index, slcan_instance->candev_baud);
	rt_device_control(slcan_instance->candev, RT_CAN_CMD_SET_BAUD, (void *)slcan_instance->candev_baud);
}

void slcan_rtcan_set_mode(rt_slcan_t* slcan_instance, uint8_t mode_index)
{
	slcan_instance->slcan_mode_index = mode_index;
	slcan_instance->candev_mode = can_mode_sclcan2rtthread(slcan_instance->slcan_mode_index);
	LOG_D("set mode : %s [%d] %d\n", slcan_instance->candev_name, slcan_instance->slcan_mode_index, slcan_instance->candev_mode);
	rt_device_control(slcan_instance->candev, RT_CAN_CMD_SET_MODE, (void *)slcan_instance->candev_mode);
}

rt_size_t slcan_rtcan_read(rt_slcan_t* slcan_instance, rt_can_msg_t msg)
{
	return rt_device_read(slcan_instance->candev, 0, msg, sizeof(struct rt_can_msg));
}

rt_size_t slcan_rtcan_write(rt_slcan_t* slcan_instance, rt_can_msg_t msg)
{
	return rt_device_write(slcan_instance->candev, 0, msg, sizeof(struct rt_can_msg));
}


void slcan_rtchar_open(rt_slcan_t* slcan_instance)
{
	int rc = rt_device_open(slcan_instance->chardev, slcan_instance->chardev_oflag);
	RT_ASSERT(rc == RT_EOK);
	rt_device_set_rx_indicate(slcan_instance->chardev, slcan_instance->chardev_rx_call);
	LOG_D("open:%s  %d\n",slcan_instance->chardev_name,rc);
}

void slcan_rtchar_close(rt_slcan_t* slcan_instance)
{
	int rc = rt_device_close(slcan_instance->chardev);
	RT_ASSERT(rc == RT_EOK);
	LOG_D("close:%s  %d\n",slcan_instance->chardev_name,rc);
}

rt_size_t slcan_rtchar_read(rt_slcan_t* slcan_instance, void* buffer, rt_size_t size)
{
	return rt_device_read(slcan_instance->chardev, 0, buffer, size);
}

rt_size_t slcan_rtchar_write(rt_slcan_t* slcan_instance, void* buffer, rt_size_t size)
{
	return rt_device_write(slcan_instance->chardev, 0, buffer, size);
}


rt_size_t slcan_wait(rt_slcan_t* slcan_instance,rt_int32_t time)
{
	return rt_sem_take(&(slcan_instance->rx_sem), time);
}

static int asc2nibble(char c)
{
	if ((c >= '0') && (c <= '9'))
		return c - '0';

	if ((c >= 'A') && (c <= 'F'))
		return c - 'A' + 10;

	if ((c >= 'a') && (c <= 'f'))
		return c - 'a' + 10;

	return 16; /* error */
}

static int hexstr2int(char* strbuffer, uint8_t strlen)
{
	uint32_t numcount = 0;
	int8_t num = 0;
	int8_t index = 0;
	while(index < strlen)
	{
		num = asc2nibble(strbuffer[index]);
		if( (0x00 <= num) && (num <= 0x0F) )
		{
			numcount = numcount << 4;
			numcount |= (num & 0x0F);
		}
		index++;
	}
	return numcount;
}

int slcan_parse_ascii(rt_slcan_t* slcan_instance, uint8_t *buffer, rt_size_t size)
{
	int bytes_count;
	int bytes_used;
	int bytes_remain = 0;
	int rx_out_len = 0;
	int tmp = 0;

	char str = 0;
	char cmd = 0;
	char * buf = (char *)buffer;
	rt_can_msg_t msg = &(slcan_instance->can_msg);
	char replybuf[10] = {0}; /* for answers to received commands */
	/* reset incomplete message offset */
	bytes_count = size;

parse_restart:
	rx_out_len = 0;
	/* remove trailing '\r' characters to be robust against some apps */
	while (buf[0] == '\r' && bytes_count > 0) {
		for (tmp = 0; tmp < bytes_count; tmp++)
		{
			buf[tmp] = buf[tmp+1];
		}
		bytes_count--;
	}

	if (!bytes_count)
	{
		bytes_remain = 0;
		goto parse_exit;
	}
	
	/* check if we can detect a complete SLCAN message including '\r' */
	for (tmp = 0; tmp < bytes_count; tmp++) {
		if (buf[tmp] == '\r')
			break;
	}

	/* no '\r' found in the message buffer? */
	if (tmp == bytes_count) {
		/* save incomplete message */
		bytes_remain = bytes_count;
		goto parse_exit;
	}
	cmd = buf[0];
	//buf[bytes_count] = 0;

	/* check for filter configuration commands */
	if (cmd == 'm' || cmd == 'M') {
		buf[9] = 0; /* terminate filter string */
		rx_out_len = 1;
		bytes_used = 9;
		goto rx_out_ack;
	}

	/* check for timestamp on/off command */
	if (cmd == 'Z') {
		slcan_instance->timestamp_isopen = buf[1] & 0x01;
		rx_out_len = 1;
		bytes_used = 2;
		goto rx_out_ack;
	}

	/* check for 'O'pen command */
	if (cmd == 'O') {
		slcan_rtcan_open(slcan_instance);
		slcan_instance->candev_isopen = 1;
		rx_out_len = 1;
		bytes_used = 1;
		goto rx_out_ack;
	}

	/* check for 'C'lose command */
	if (cmd == 'C') {
		slcan_rtcan_close(slcan_instance);
		slcan_instance->candev_isopen = 0;
		rx_out_len = 1;
		bytes_used = 1;
		goto rx_out_ack;
	}

	/* check for 'V'ersion command */
	if (cmd == 'V') {
		rt_sprintf(replybuf, "V1013\r");
		rx_out_len = strlen(replybuf);
		bytes_used = 1;
		goto rx_out;
	}
	/* check for 'v'ersion command */
	if (cmd == 'v') {
		rt_sprintf(replybuf, "v1014\r");
		rx_out_len = strlen(replybuf);
		bytes_used = 1;
		goto rx_out;
	}

	/* check for serial 'N'umber command */
	if (cmd == 'N') {
		rt_sprintf(replybuf, "N4242\r");
		rx_out_len = strlen(replybuf);
		bytes_used = 1;
		goto rx_out;
	}

	/* check for read status 'F'lags */
	if (cmd == 'F') {
		rt_sprintf(replybuf, "F00\r");
		rx_out_len = strlen(replybuf);
		bytes_used = 1;
		goto rx_out;
	}
	if (cmd == 'S') {
		
		rx_out_len = 1;
		bytes_used = 2;
		goto rx_out_ack;
	}

	/* correctly answer unsupported commands */
	if (cmd == 'U') {
		rx_out_len = 1;
		bytes_used = 2;
		goto rx_out_ack;
	}
	if (cmd == 's') {
		rx_out_len = 1;
		bytes_used = 5;
		goto rx_out_ack;
	}
	if (cmd == 'P' || cmd == 'A') {
		rx_out_len = 1;
		bytes_used = 1;
		goto rx_out_nack;
	}
	if (cmd == 'X') {
		rx_out_len = 1;
		bytes_used = 2;
		if (buf[1] & 0x01)
			goto rx_out_ack;
		else
			goto rx_out_nack;
	}

	/* catch unknown commands */
	if ((cmd != 't') && (cmd != 'T') &&
	    (cmd != 'r') && (cmd != 'R')) {
		bytes_used = bytes_count-1;
		rx_out_len = 1;
		goto rx_out_nack;
	}
	
	if( (cmd == 'r') || (cmd == 't') )
	{
		str = buf[4];  //dlc str
		if (!(str >= '0' && str < '9'))
		{
			bytes_used = bytes_count-1;
			rx_out_len = 1;
			goto rx_out_nack;
		}
		tmp =  hexstr2int( &(buf[1]),3);
		msg->id = tmp;
		msg->ide = RT_CAN_STDID; //11bit id
		msg->len = str - '0';
		bytes_used = 5;  // "Riii********\a" 
		if(cmd == 'r')
		{
			msg->rtr = RT_CAN_RTR;
		}else
		{
			msg->rtr = RT_CAN_DTR;
			if(msg->len > 0)
			{
				bytes_used += (msg->len * 2); //add dlc
				for (tmp = 0; tmp < msg->len; tmp++)
				{
					 msg->data[tmp] = hexstr2int( &(buf[5 + (tmp *2) ]), 2);
				}
			}
		}
	}else
	if( (cmd == 'R') || (cmd == 'T') )
	{
		str = buf[9];  //dlc str
		
		if (!(str >= '0' && str < '9'))
		{
			bytes_used = bytes_count-1;
		  rx_out_len = 1;
			goto rx_out_nack;
		}
		tmp =  hexstr2int( &(buf[1]),8);
		msg->id = tmp;
		msg->ide = RT_CAN_EXTID; //29bit id
		msg->len = str - '0';
		bytes_used = 10; // "Riiiiiiii********\a" 
		if(cmd == 'R')
		{
			msg->rtr = RT_CAN_RTR;
		}else
		{
			msg->rtr = RT_CAN_DTR;
			if(msg->len > 0)
			{
				bytes_used += (msg->len * 2); //add dlc
				for (tmp = 0; tmp < msg->len; tmp++)
				{
					 msg->data[tmp] = hexstr2int( &(buf[10 + (tmp *2) ]), 2);
				}
			}

		}
	}
	slcan_rtcan_write(slcan_instance,msg);
	rx_out_len = 1;
	
rx_out_ack:
	replybuf[0] = '\r';
	goto rx_out;
rx_out_nack:
	replybuf[0] = '\a';
rx_out:
	if(rx_out_len > 0)
	{
		slcan_rtchar_write(slcan_instance, replybuf, rx_out_len);
	}
	/* check if there is another command in this buffer */
	if (bytes_count > bytes_used) {
		for (tmp = 0, bytes_used++; bytes_used+tmp < bytes_count; tmp++)
		{
			buf[tmp] = buf[bytes_count+tmp];
		}
		bytes_count = tmp;
		goto parse_restart;
	}
parse_exit:
	return bytes_remain;
}


void slcan_process_serial(rt_slcan_t* slcan_instance)
{
	int rc = 0;
	uint8_t *rx_buffer = slcan_instance->chardev_rx_buffer;
	int buffer_len = slcan_instance->chardev_rx_remain;
	
	rc = slcan_rtchar_read( slcan_instance, &(rx_buffer[buffer_len]), (sizeof(slcan_instance->chardev_rx_buffer) - buffer_len) );
	if((buffer_len <= 0) && (rc <= 0) )
	{
		return;
	}
	buffer_len += rc;
	if(buffer_len > 0)
	{
		LOG_D("[%s >> %s] (%d %d) %s  \n",slcan_instance->chardev_name, slcan_instance->candev_name,buffer_len,rc, rx_buffer);
	}
	slcan_instance->chardev_rx_remain = 0;
	rc = slcan_parse_ascii(slcan_instance, slcan_instance->chardev_rx_buffer, buffer_len);
	slcan_instance->chardev_rx_remain = rc;
	rt_memset(&(slcan_instance->chardev_rx_buffer[slcan_instance->chardev_rx_remain]), 0, (sizeof(slcan_instance->chardev_rx_buffer) - slcan_instance->chardev_rx_remain));
	
	LOG_D("[%s >> %s] (%d ) %s\n",slcan_instance->chardev_name, slcan_instance->candev_name,slcan_instance->chardev_rx_remain, rx_buffer);
	
}



static int slcan_can2ascii(rt_slcan_t* slcan_instance, rt_can_msg_t msg, char *tx_buffer)
{
	int pos = 0;
	uint32_t temp = 0;
	rt_memset(tx_buffer, 0 , SLCAN_MTU);
	
	/* RTR frame */
	if(msg->rtr == RT_CAN_RTR) 
	{
		if(msg->ide == RT_CAN_EXTID)
		{
			tx_buffer[pos] = 'R';
		}else
		{
			tx_buffer[pos] = 'r';
		}
	}else /* data frame */
	{
		if(msg->ide == RT_CAN_EXTID)
		{
			tx_buffer[pos] = 'T';
		}else
		{
			tx_buffer[pos] = 't';
		}
	}
	pos++;
	/* id */
	temp =  msg->id;
	if(msg->ide == RT_CAN_EXTID)
	{
		rt_snprintf(&(tx_buffer[pos]), 9, "%08X", temp);
		pos += 8;
	}else
	{
		rt_snprintf(&(tx_buffer[pos]), 4, "%03X", temp);
		pos += 3;
	}
	/* len */
	tx_buffer[pos] = '0' + (msg->len & 0x0f);
	pos++;
	/* data */
	if(msg->rtr != RT_CAN_RTR)
	{
		for(int i=0; i < msg->len; i++)
		{
			rt_snprintf(&(tx_buffer[pos]), 3, "%02X", msg->data[i]);
			pos+=2;
		}
	}
	/* timestamp */
	if(slcan_instance->timestamp_isopen > 0)
	{
		uint32_t tick = rt_tick_get();
		rt_sprintf( &tx_buffer[pos], "%04lX", (tick & 0x0000FFFF) );
		pos += 4;
	}
	/* end */
	tx_buffer[pos] = '\r';
	pos++;
	tx_buffer[pos] = 0;
	return pos;
}


void slcan_process_can(rt_slcan_t* slcan_instance)
{
	int read_rc = 0;
	int tx_len = 0;
	do{
		slcan_instance->can_msg.hdr = -1;
		read_rc = slcan_rtcan_read(slcan_instance, &(slcan_instance->can_msg) );
		if(read_rc != sizeof(struct rt_can_msg) )
		{
			return;
		}
		tx_len = slcan_can2ascii(slcan_instance, &(slcan_instance->can_msg) , (char *)slcan_instance->chardev_tx_buffer );
		if(tx_len <= 0)
		{
			return;
		}
		slcan_rtchar_write(slcan_instance, slcan_instance->chardev_tx_buffer, tx_len);
		LOG_D("[%s << %s] %s\n",slcan_instance->chardev_name, slcan_instance->candev_name, slcan_instance->chardev_tx_buffer);
	}while(1);
}


void slcan_process_task(void *p)
{
	rt_slcan_t* slcan_instance = p;
	LOG_D("[%s <=> %s] slcan start\n",slcan_instance->chardev_name, slcan_instance->candev_name);
	slcan_rtchar_open(slcan_instance);
	slcan_rtcan_open(slcan_instance);
	slcan_instance->candev_isopen = 1;
	while(1)
	{
		slcan_wait(slcan_instance,RT_WAITING_FOREVER);
		slcan_process_can(slcan_instance);
		slcan_process_serial(slcan_instance);
	}
}


