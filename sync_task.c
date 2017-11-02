#ifndef SYNC_TASK_C_
#define SYNC_TASK_C_
#endif
#include "sf_Driver.h"
#include "App_Inc.h"
#include "can_ctl.h"
#include "timing_task.h"

/*
 * 同步信号处理任务
 */
#pragma define_section code_in_ram ".app_code_ram" abs32 RWX
#pragma section code_in_ram begin
void sync_task(TY_CAN_CTL *p_can)
{
	p_can->sync_cnt ++;
	switch(p_can->sm)
	{
		case CAN_CTL_SM_IDLE:
		case CAN_CTL_SM_RX_WAIT:
		    p_can->sm = CAN_CTL_SM_RX_GO;
		    PIT_SET_LDVAL(1);
		    break;
		case CAN_CTL_SM_TX_GAP:
		case CAN_CTL_SM_RX:
		case CAN_CTL_SM_RX_ACK:
		case CAN_CTL_SM_TX_ARB:
		case CAN_CTL_SM_TX:
		case CAN_CTL_SM_TX_ACK:
			// 发送同步信号
			p_can->sync_tx ++;
			break;
		default:
			// 发送同步信号
			p_can->sync_tx ++;
			break;
	}
}
#pragma section code_in_ram end



