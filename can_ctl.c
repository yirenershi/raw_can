#ifndef CAN_CTL_C_
#define CAN_CTL_C_
#endif

#include "sf_Driver.h"
#include "App_Inc.h"
#include "can_ctl.h"
#include "timing_task.h"

void can_ctl_init(TY_CAN_CTL *p_can, uint32 bit_rate)
{
	memset(p_can, 0, sizeof(TY_CAN_CTL));
	
	p_can->sm = CAN_CTL_SM_STOP;
	
	p_can->timing_tm_max =0;
	p_can->timing_tm_min =(uint32)(-1);	

	p_can->sync_tm_max =0;
	p_can->sync_tm_min =(uint32)(-1);
}

void can_ctl_enable(TY_CAN_CTL *p_can)
{
	p_can->sm = CAN_CTL_SM_IDLE;
}

void can_ctl_disable(TY_CAN_CTL *p_can)
{
	p_can->sm = CAN_CTL_SM_STOP;
}

bool can_ctl_wr(TY_CAN_CTL *p_can, uint32 id, uint8 len, uint8 *p_dat)
{
	uint16 tmp;
	uint8 cnt;
	
	if(len > 8)
		len = 8;
	tmp = (p_can->tx_fifo_wr+1)&TX_FIFO_LEN_MSK;
	if(tmp != p_can->tx_fifo_rd)
	{
		p_can->tx_fifo[p_can->tx_fifo_wr].id = id;
		p_can->tx_fifo[p_can->tx_fifo_wr].len = len;
		for(cnt=0; cnt<len; cnt++)
			p_can->tx_fifo[p_can->tx_fifo_wr].dat[cnt] = p_dat[cnt];
		p_can->tx_fifo[p_can->tx_fifo_wr].flag = 0x83;
		p_can->tx_fifo[p_can->tx_fifo_wr].crc = can_frame_crc(&p_can->tx_fifo[p_can->tx_fifo_wr]);
		p_can->tx_fifo_wr = tmp;
		return true;
	}
	return false;
}


bool can_ctl_rd(TY_CAN_CTL *p_can, uint32 *p_ts, uint32 *p_id, uint8 *p_len, uint8 *p_dat)
{
	uint8 cnt;
	
	if(p_can->rx_fifo_wr != p_can->rx_fifo_rd)
	{
		*p_ts = p_can->rx_fifo[p_can->rx_fifo_rd].ts;
		*p_id = p_can->rx_fifo[p_can->rx_fifo_rd].id;
		*p_len = p_can->rx_fifo[p_can->rx_fifo_rd].len;
		for(cnt=0; cnt<*p_len; cnt++)
			p_dat[cnt] = p_can->rx_fifo[p_can->rx_fifo_rd].dat[cnt];
		p_can->rx_fifo_rd = (p_can->rx_fifo_rd+1)&RX_FIFO_LEN_MAK;
		return true;
	}
	return false;
}	
														