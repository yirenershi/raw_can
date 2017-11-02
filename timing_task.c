#ifndef TIMING_TASK_C_
#define TIMING_TASK_C_
#endif

#include "sf_Driver.h"
#include "App_Inc.h"
#include "can_ctl.h"
#include "timing_task.h"




#pragma define_section code_in_ram ".app_code_ram" abs32 RWX

#pragma section code_in_ram begin
// M4的时钟为48mhz
// 定时时间us(x)
// 重载值=x*1000/(1000/48)=x*48
#define PIT3_LD_VAL_TQ		(5*48-1)		// TQ
#define PIT3_LD_VAL_BIT		(50*48-1)		// BIT
#define PIT3_LD_VAL_IDLE	(1000*48-1)		// MS
void timing_task(TY_CAN_CTL *p_can)
{    
	uint32 tm_s=PIT_GET_CNT();//getTimeTicks();
	uint32 tm_sm=(p_can->sm<<24)+(p_can->tq_cnt<<16)+(p_can->bit_rx_parse.valid_bit_st<<8)+(p_can->bit_rx_parse.frm_parse_st);
    uint16 tmp;
    bool   b_res;
    uint8  sm = p_can->sm;
    
   	p_can->timing_cnt ++;
	switch(p_can->sm)
	{
		case CAN_CTL_SM_STOP:
		{
			PIT_SET_LDVAL(PIT3_LD_VAL_IDLE);
			break;
		}
		case CAN_CTL_SM_IDLE:
		{
			// 有待发数据就发送数据，否则降低终端频率
		    if(p_can->tx_fifo_rd != p_can->tx_fifo_wr)
			{
				p_can->p_tx_bit_parse.frame = p_can->tx_fifo[p_can->tx_fifo_rd];
				p_can->p_tx_bit_parse.sm = 0;
				p_can->tx_cnt ++;
				
				p_can->tq_cnt = 1;
				if(bit_tx_parse(&p_can->p_tx_bit_parse, &p_can->tx_bit))
				{
					TX_PIN_SET(p_can->tx_bit);
				}
				
				p_can->bit_rx_parse.can_frm.ts = tm_s;
				PIT_SET_LDVAL(PIT3_LD_VAL_TQ);
				p_can->sm = CAN_CTL_SM_TX_ARB;
			}
			else
			{
				TX_PIN_SET(1);
				PIT_SET_LDVAL(PIT3_LD_VAL_IDLE);
			}
			break;
		}
		case CAN_CTL_SM_RX_WAIT:
		{
			p_can->tq_cnt ++;
			if(p_can->tq_cnt >= 7)
			{
				p_can->tq_cnt = 0;
				p_can->sm = CAN_CTL_SM_IDLE;
			}
			break;
		}
		case CAN_CTL_SM_RX_GO:
		{
			// 2000/20-1=99
			PIT_SET_LDVAL(PIT3_LD_VAL_TQ);
			// 此时对应于SOF的下降沿
			// 准备接收数据
			p_can->bit_rx_parse.can_frm.ts = tm_s;
			p_can->tq_cnt = 0;
			p_can->rx_flag = 0;
			p_can->sm = CAN_CTL_SM_RX;
		}
		case CAN_CTL_SM_RX:
		{
			// |_|_|_|_|_|_|_|_|_|_|
			// 0 1 2 3 4 5 6 7 8 9 0   
			// 1、边沿0、1，出现同步信号是正常信号，不需要处理
			// 2、边沿2、3，出现同步信号需要再同步来矫正，直接认为当前TQ=1
			// 3、边沿8、9，出现同步信号需要再同步来矫正，直接认为当前TQ=0
			// 4、其余边沿，出现同步信号属于不可挽回的错误
			// #、在每个TQ=0时要复位tq_sync
			switch(p_can->tq_cnt)
			{
			    case 0:
			        p_can->tq_sync = 0;
			    case 1:
			        if(p_can->sync_tx != p_can->sync_rx)
			        {
			            p_can->sync_rx = p_can->sync_tx;
			        }
			        p_can->tq_cnt ++;
			        break;
			    case 2:
			    case 3:
			        if(p_can->sync_tx != p_can->sync_rx)
			        {
			            p_can->sync_rx  = p_can->sync_tx;
			            if(p_can->tq_sync)
			            {
			               	p_can->tq_cnt ++;
			            }
			            else
			            {
			            	p_can->head_rsync ++;
			                p_can->tq_cnt = 2;
			                p_can->tq_sync = 1;
			            }
			        }
			        else
			        {
			        	p_can->tq_cnt ++;
			        }
			        break;
			     case 4:
			     case 5:
			     case 6:
			        if(p_can->sync_tx != p_can->sync_rx)
			        {
			            p_can->sync_rx = p_can->sync_tx;
			            p_can->sync_err ++;
			        }
			        p_can->tq_cnt ++;
			        break;
			     case 7:
			        // sample point
			        p_can->bit_stat = RX_PIN_STAT();
#ifdef CAN_CTL_DEBUG				        
			        if(!p_can->rx_bit_lock)
			        {
			        	p_can->rx_bit_ts[p_can->rx_bit_wr] = CAN_CTL_GET_TIME();
				        p_can->rx_bit[p_can->rx_bit_wr] = p_can->bit_stat;
				        p_can->rx_bit_wr = (p_can->rx_bit_wr+1)&BIT_BUF_LEN_MSK;
				    }
#endif			        
			        b_res = bit_rx_parse(&p_can->bit_rx_parse, p_can->bit_stat);
			        	
			        // 下一位是ACK，认为已经解码到数据
			        if(b_res)
			        {
			        	p_can->rx_flag = 1;
			        	
			        	if(p_can->sync_tx != p_can->sync_rx)
				        {
				            p_can->sync_rx = p_can->sync_tx;
				            p_can->sync_err ++;
				        }
					    p_can->tq_cnt ++;
			        }
			        else
			        {
			        	p_can->rx_flag = 0;
				        // bit_parse函数异常情况下会把状态机复位
			        	if(0!=p_can->bit_rx_parse.frm_parse_st)
				        {
					        if(p_can->sync_tx != p_can->sync_rx)
					        {
					            p_can->sync_rx = p_can->sync_tx;
					            p_can->sync_err ++;
					        }
						    p_can->tq_cnt ++;
						}
						else
						{
							p_can->rx_err_bit_parse ++;
							p_can->tq_cnt = 0;
							p_can->sm = CAN_CTL_SM_RX_RST;
						}
					}
			        break;
			     case 8:
			     	// 为了平衡负荷，计算校验和放到了这里
		     		if(p_can->bit_rx_parse.valid_bit)
		     		{
		     			p_can->crc = can_crc(p_can->crc, p_can->bit_stat);
		     		}

			     	// 检查数据有效性
			     	if(p_can->rx_flag)
			     	{
			     		// 检查CRC
			     		if(p_can->crc == p_can->bit_rx_parse.can_frm.crc)
			     		{
			     			p_can->rx_crc_ok = 1;
				        }
				        else
				        {
				        	p_can->rx_err_crc ++;
				        	p_can->rx_crc_ok = 0;
				        }
			     	}
			     	
			        if(p_can->sync_tx != p_can->sync_rx)
			        {
			            p_can->sync_rx = p_can->sync_tx;
			            if(p_can->tq_sync)
			            {
			                p_can->tq_cnt ++;
			            }
			            else
			            {
			            	p_can->tail_rsync ++;
			                p_can->tq_cnt = 0;
			                // 下一位是ACK
			                if(p_can->rx_flag)
        			        { 
        			        	p_can->sm_rx_2_rx_ack ++;
        			            p_can->sm = CAN_CTL_SM_RX_ACK;
        			        }
			            }
			        }
			        else
			        {
			            p_can->tq_cnt ++;
			        }
			        break;
                case 9:
                	// if next bit is ack, do not check resync 
                	// else check resync 
			        if(p_can->rx_flag)
			        {
			        	if(p_can->sync_tx != p_can->sync_rx)
				        {
				            p_can->sync_rx = p_can->sync_tx;
				        }
				        p_can->tq_cnt = 0;
			        	p_can->sm_rx_2_rx_ack ++;
			            p_can->sm = CAN_CTL_SM_RX_ACK;
			        }
			        else
			    	{
			    		if(p_can->sync_tx != p_can->sync_rx)
				        {
				            p_can->sync_rx = p_can->sync_tx;
				            if(p_can->tq_sync)
				            {
				                p_can->tq_cnt =0;
				            }
				            else
				            {
				            	p_can->tq9_rsync ++;
				            	p_can->tq_cnt =1;
				            }
				        }
				        else
				        {
				        	p_can->tq_cnt = 0;
				        }
			    	}                
			        break;
                default:
                    p_can->rx_err_sm ++;
                    p_can->tq_cnt = 0;
			        p_can->sm = CAN_CTL_SM_RX_RST;
                    break;
			}			
			break;
		}
        case CAN_CTL_SM_RX_ACK:	// 3
        {
            switch(p_can->tq_cnt)
			{
                case 0: 
                	// 发送应答信号
                	if(p_can->rx_crc_ok)
                	{
			        	TX_PIN_SET(0);
			        	tmp = (p_can->rx_fifo_wr+1)&0x07;
				        if(p_can->rx_fifo_rd != tmp)
				        {
			        		p_can->rx_fifo_ok = 1;
			        	}
			        	else
			        	{
			        		p_can->rx_ov_cnt ++;
			        		p_can->rx_fifo_ok = 0;
			        	}
			        	p_can->rx_cnt ++;
			        }
			        else
			        {
			        	TX_PIN_SET(1);
			        	p_can->rx_fifo_ok = 0;
			        }
			        p_can->sm_ack ++;
			        p_can->tq_cnt ++;
			        break;
                case 1:
                	if(p_can->rx_fifo_ok)
                	{
                		p_can->rx_fifo[p_can->rx_fifo_wr].ts = p_can->bit_rx_parse.can_frm.ts;
                		p_can->rx_fifo[p_can->rx_fifo_wr].id = p_can->bit_rx_parse.can_frm.id;
                	}
                	p_can->tq_cnt ++;
                	break;
                case 2:
                	if(p_can->rx_fifo_ok)
                	{
                		p_can->rx_fifo[p_can->rx_fifo_wr].len = p_can->bit_rx_parse.can_frm.len;
                		p_can->rx_fifo[p_can->rx_fifo_wr].flag = p_can->bit_rx_parse.can_frm.flag;
                	}
                	p_can->tq_cnt ++;
                	break;
			    case 3:
			    	if(p_can->rx_fifo_ok)
                	{
                		p_can->rx_fifo[p_can->rx_fifo_wr].dat[0] = p_can->bit_rx_parse.can_frm.dat[0];
                		p_can->rx_fifo[p_can->rx_fifo_wr].dat[1] = p_can->bit_rx_parse.can_frm.dat[1];
                	}
                	p_can->tq_cnt ++;
                	break;
			    case 4:
			    	if(p_can->rx_fifo_ok)
                	{
                		p_can->rx_fifo[p_can->rx_fifo_wr].dat[2] = p_can->bit_rx_parse.can_frm.dat[2];
                		p_can->rx_fifo[p_can->rx_fifo_wr].dat[3] = p_can->bit_rx_parse.can_frm.dat[3];
                	}
                	p_can->tq_cnt ++;
                	break;
			    case 5:
			    	if(p_can->rx_fifo_ok)
                	{
                		p_can->rx_fifo[p_can->rx_fifo_wr].dat[4] = p_can->bit_rx_parse.can_frm.dat[4];
                		p_can->rx_fifo[p_can->rx_fifo_wr].dat[5] = p_can->bit_rx_parse.can_frm.dat[5];
                	}
                	p_can->tq_cnt ++;
                	break;
			    case 6:
			    	if(p_can->rx_fifo_ok)
                	{
                		p_can->rx_fifo[p_can->rx_fifo_wr].dat[6] = p_can->bit_rx_parse.can_frm.dat[6];
                		p_can->rx_fifo[p_can->rx_fifo_wr].dat[7] = p_can->bit_rx_parse.can_frm.dat[7];
                	}
                	p_can->tq_cnt ++;
                	break;
                case 7:
                	if(p_can->rx_fifo_ok)
                	{
                		p_can->rx_fifo[p_can->rx_fifo_wr].crc = p_can->bit_rx_parse.can_frm.crc;
                	}
			        p_can->tq_cnt ++;
			        break;
			    case 8:
			    	if(p_can->rx_fifo_ok)
                	{
                		p_can->rx_fifo_wr = (p_can->rx_fifo_wr+1)&RX_FIFO_LEN_MAK;
                	}
			        p_can->tq_cnt ++;
			        break;
			    case 9: 
			        if(p_can->sync_tx != p_can->sync_rx)
			        {
			            p_can->sync_rx = p_can->sync_tx;
			        }
			        p_can->tq_cnt =0;	
			        p_can->sm = CAN_CTL_SM_RX_RST;
			        break;
			    default:
			    	p_can->rx_err_sm ++;
			    	p_can->tq_cnt = 0;
			        p_can->sm = CAN_CTL_SM_RX_RST;
			        break;
			}
            break;
        }
        case CAN_CTL_SM_RX_RST:
        	// 接收异常或者接收完毕后的中间态，用来复位接收相关的变量
        	// 持续4个BIT
        	switch(p_can->tq_cnt)
        	{
        		case 0:
        			TX_PIN_SET(1);
        			PIT_SET_LDVAL(PIT3_LD_VAL_BIT);
        			p_can->tq_cnt ++;
        			break;
        		case 1:
		        	p_can->rx_flag = 0;
		        	p_can->rx_crc_ok = 0;
		        	p_can->tq_sync = 0;
		        	p_can->crc = 0;
		        	p_can->sync_rx = p_can->sync_tx;
					p_can->sm_rst ++;
        			p_can->tq_cnt ++;
        			break;
        		case 2:
        			bit_rx_parse_rst(&p_can->bit_rx_parse);
        			p_can->tq_cnt ++;
        			break;
        		case 3:
        			p_can->tq_cnt ++;
        			break;
        		case 4:
        			p_can->tq_cnt = 0;
        			p_can->sm = CAN_CTL_SM_RX_WAIT;	
        			break;
        		default:
        			p_can->tq_cnt = 0;
        			p_can->sm = CAN_CTL_SM_RX_WAIT;	
        			break;
        	}
        	break;
		case CAN_CTL_SM_TX_ARB:	// 
		{
		    switch(p_can->tq_cnt)
			{
                case 0:
			        if(bit_tx_parse(&p_can->p_tx_bit_parse, &p_can->tx_bit))
			        {
			        	TX_PIN_SET(p_can->tx_bit);
			        }
			        p_can->tq_cnt ++;
			        break;
			    case 7:
			        p_can->bit_stat = RX_PIN_STAT();
			        b_res = bit_rx_parse(&p_can->bit_rx_parse, p_can->bit_stat);
			        // 仲裁判断
			        if(p_can->tx_bit == p_can->bit_stat)
			        {
			            ;
			        }
			        else
			        {
			        	// 仲裁失败
			        	p_can->arb_fail ++;
			        	p_can->arb_fail_info = (p_can->p_tx_bit_parse.sm<<8)+(p_can->p_tx_bit_parse.bit_in_seg);
			        	p_can->sync_rx = p_can->sync_tx;
			            p_can->sm = CAN_CTL_SM_RX;
			        }
			        p_can->tq_cnt ++;
			        break;
                case 1:
                case 2:
			    case 3:
			    case 4:
			    case 5:
			    case 6:
			    case 8: 
			        p_can->tq_cnt ++;
			        break;			    
			    case 9:
			        if(p_can->sync_tx != p_can->sync_rx)
			        {
			            p_can->sync_rx = p_can->sync_tx;
			        }
			        p_can->tq_cnt =0;
			        // 检查仲裁场是否结束
			        if(p_can->p_tx_bit_parse.sm >= 5)
			        {
			            p_can->sm = CAN_CTL_SM_TX;
			        }
			        break;			   
			    default:
			        p_can->tx_err_sm ++;
			        p_can->sm = CAN_CTL_SM_TX_GAP;
			        break;
			}
            break;
        }
		case CAN_CTL_SM_TX:		// 
		    switch(p_can->tq_cnt)
			{
			    case 0:
			    	if(bit_tx_parse(&p_can->p_tx_bit_parse, &p_can->tx_bit))
			        {
			        	TX_PIN_SET(p_can->tx_bit);
			        }
		        case 1:
                case 2:
			    case 3:
			    case 4:
			    case 5:
			    case 6:
			    case 7:
			    case 8: 
			        p_can->tq_cnt ++;
			        break;		
		        case 9:
			        if(p_can->sync_tx != p_can->sync_rx)
			        {
			            p_can->sync_rx = p_can->sync_tx;
			        }
			        p_can->tq_cnt =0;
			        if(9 == p_can->p_tx_bit_parse.sm)
			        {
			           p_can->sm =  CAN_CTL_SM_TX_ACK;
			        }
			        break;
			    default:
			    	p_can->tx_err_sm ++;
			        p_can->sm = CAN_CTL_SM_TX_GAP;
			    	break;
			}
			break;
		case CAN_CTL_SM_TX_ACK:	// 
			switch(p_can->tq_cnt)
			{			    
				case 0:
					TX_PIN_SET(1);
			        if(bit_tx_parse(&p_can->p_tx_bit_parse, &p_can->tx_bit))
			        {
			        	;
			        }
			        p_can->tq_cnt ++;
			        break; 
			    case 1:
                case 2:
			    case 3:
			    case 4:
			    case 5:
			    case 6:
			    case 8: 
			        p_can->tq_cnt ++;
			        break;	
			    case 7:
			        p_can->bit_stat = RX_PIN_STAT();
			        // 检查应答
			        if(0==p_can->bit_stat)
			        {
			            p_can->tx_fifo_rd = (p_can->tx_fifo_rd+1)&TX_FIFO_LEN_MSK;
			        	p_can->tx_succ ++;
			        }
			        else
			        {
			        	p_can->ack_err ++;
			        }
               		p_can->tq_cnt ++;
			        break;	    
			    case 9:
			        if(p_can->sync_tx != p_can->sync_rx)
			        {
			            p_can->sync_rx = p_can->sync_tx;
			        }
			        p_can->tq_cnt =0;
			       
		            p_can->sm = CAN_CTL_SM_TX_GAP;
			        break;			   
			    default:
			        p_can->tx_err_sm ++;
			        p_can->sm = CAN_CTL_SM_TX_GAP;
			        break;
			}
			break;
		case CAN_CTL_SM_TX_GAP:	// 8
		    switch(p_can->tq_cnt)
			{
			    case 0:
			       	TX_PIN_SET(1);
			       	PIT_SET_LDVAL(PIT3_LD_VAL_BIT);
			       	p_can->tq_cnt ++;
			       	break;
                case 1:
                	bit_rx_parse_rst(&p_can->bit_rx_parse);
			       	bit_tx_parse_rst(&p_can->p_tx_bit_parse);
			       	p_can->tq_cnt ++;
			        break;
                case 2:
                case 3:
			    	p_can->tq_cnt ++;
			        break;
			    case 4:
			        if(p_can->sync_tx != p_can->sync_rx)
			        {
			            p_can->sync_rx = p_can->sync_tx;
			        }
			        p_can->tq_cnt =0;
			        p_can->sm = CAN_CTL_SM_RX_WAIT;
			        break;
			    default:
			    	p_can->tq_cnt = 0;
			    	p_can->sm = CAN_CTL_SM_RX_WAIT;
			    	break;
			}
			break;
		default:
		    p_can->sm = 0;
			break;
	}
	if(CAN_CTL_SM_IDLE != sm)
	{
		tm_s=tm_s-PIT_GET_CNT();//getTimeTicks();
		if(tm_s>p_can->timing_tm_max)
		{
			p_can->timing_tm_max_st = tm_sm;
			p_can->timing_tm_max = tm_s;
		}
		if(tm_s<p_can->timing_tm_min)
			p_can->timing_tm_min = tm_s;
	}
}
#pragma section code_in_ram end

