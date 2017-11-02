#ifndef BIT_PARSE_C_
#define BIT_PARSE_C_
#endif
#include "sf_Driver.h"
#include "bit_parse.h"

#pragma define_section code_in_ram ".app_code_ram" abs32 RWX
#pragma section code_in_ram begin

uint8 can_bit_stuff(TY_BIT_RX_PARSE *p_parse, uint8 bit_val);


uint16 can_crc(uint16 crc, uint8 bit)
{
	uint16 tmp;
	
	tmp = bit;
	tmp <<= 14;
	tmp = tmp^crc;
	tmp &= (1<<14);
	if(tmp)
		tmp = 0x4599;
	else
		tmp = 0x0000;
	
	crc <<= 1;
	
	crc = crc^tmp;
	crc &= 0x7fff;
	
	return crc;
}

/// 计算CAN报文的CRC
/// 1、SOF不参与计算
/// @param	p_frame		[i]接收到的CAN报文
/// @retval	计算出来的CRC结果
uint16 can_frame_crc(TY_CAN_FRM *p_frame)
{
	TY_CAN_FRM tmp = *p_frame;
	uint16 crc=0;
	uint8 bit;
	uint8 cnt, loop;
	
	for(cnt=0; cnt<11; cnt++)
	{
		if(tmp.id & 0x10000000)
		{
			bit = 1;
		}
		else
		{
			bit = 0;
		}
		tmp.id <<= 1;
		crc = can_crc(crc, bit);
	}

	for(cnt=0; cnt<1; cnt++)
	{
		if(tmp.flag & F_SRR)
		{
			bit = 1;
		}
		else
		{
			bit = 0;
		}
		crc = can_crc(crc, bit);
	}

	for(cnt=0; cnt<1; cnt++)
	{
		if(tmp.flag & F_IDE)
		{
			bit = 1;
		}
		else
		{
			bit = 0;
		}
		crc = can_crc(crc, bit);
	}

	for(cnt=0; cnt<18; cnt++)
	{
		if(tmp.id & 0x10000000)
		{
			bit = 1;
		}
		else
		{
			bit = 0;
		}
		tmp.id <<= 1;
		crc = can_crc(crc, bit);
	}

	for(cnt=0; cnt<1; cnt++)
	{
		if(tmp.flag & F_RTR)
		{
			bit = 1;
		}
		else
		{
			bit = 0;
		}
		crc = can_crc(crc, bit);
	}

	for(cnt=0; cnt<1; cnt++)
	{
		if(tmp.flag & F_R0)
		{
			bit = 1;
		}
		else
		{
			bit = 0;
		}
		crc = can_crc(crc, bit);
	}

	for(cnt=0; cnt<1; cnt++)
	{
		if(tmp.flag & F_R1)
		{
			bit = 1;
		}
		else
		{
			bit = 0;
		}
		crc = can_crc(crc, bit);
	}

	for(cnt=0; cnt<4; cnt++)
	{
		if(tmp.len & 0x08)
		{
			bit = 1;
		}
		else
		{
			bit = 0;
		}
		tmp.len <<= 1;
		crc = can_crc(crc, bit);
	}

	for(cnt=0; cnt<8; cnt++)
	{
		for(loop=0; loop<8; loop++)
		{
			if(tmp.dat[cnt] & 0x80)
			{
				bit = 1;
			}
			else
			{
				bit = 0;
			}
			tmp.dat[cnt] <<= 1;
			crc = can_crc(crc, bit);
		}
	}
	return crc;
}

void bit_rx_parse_init(TY_BIT_RX_PARSE *p_parse)
{
	memset(p_parse, 0, sizeof(TY_BIT_RX_PARSE));
}

void bit_rx_parse_rst(TY_BIT_RX_PARSE *p_parse)
{
	p_parse->valid_bit_st = BIT_IDLE;
	p_parse->frm_parse_st = 0;
}

/// 输入若干个bit，解析出can报文，现在实际只处理一位
/// 1、剔除填充位
/// 2、使用状态机对有效bit流进行处理
/// 3、通过返回值+p_parse结构体的信息可以得到当前解码的详细状态
/// @param	p_parse			[io]解析器句柄
/// @param	p_bit			[i]	bit流缓冲区
/// @param	bit_num			[i]	bit数
/// @param	p_can_frm		[o]	CAN帧缓冲区
/// @param	p_can_frm_num	[io]输入是CAN帧缓冲区的大小，输出是解码出来的CAN帧的个数
/// @retval	true			接收到CRC界定符（接收完毕）
/// @retval	false			其他情况
//#define BIT_RX_PARSE_TM
//#define BIT_RX_PARSE_TS
bool bit_rx_parse(TY_BIT_RX_PARSE *p_parse, uint8 bit_val)
{
#ifdef BIT_RX_PARSE_TM
	uint32 	stat = (p_parse->valid_bit_st<<8)+p_parse->frm_parse_st;
	uint32  tm_probe;
#endif
	bool	res=false;
	
#ifdef BIT_RX_PARSE_TM
	tm_probe = getTimeTicks();
#endif
	
	p_parse->valid_bit = 0;
	// remove the stuff bit
	switch(p_parse->valid_bit_st)
	{
		case BIT_IDLE:		
			if(bit_val)
			{
				p_parse->frm_parse_st = 0;
				p_parse->valid_bit_st = BIT_IDLE;
			}
			else
			{
				p_parse->valid_bit_st = BIT_L_1;
			}
			break;
		case BIT_H_1:	// have rx 1 bit 1
			if(bit_val)
				p_parse->valid_bit_st = BIT_H_2;
			else
				p_parse->valid_bit_st = BIT_L_1;
			break;
		case BIT_H_2:		
			if(bit_val)
				p_parse->valid_bit_st = BIT_H_3;
			else
				p_parse->valid_bit_st = BIT_L_1;
			break;
		case BIT_H_3:		
			if(bit_val)
				p_parse->valid_bit_st = BIT_H_4;
			else
				p_parse->valid_bit_st = BIT_L_1;
			break;
		case BIT_H_4:	// have rx 4 bit 1		
			if(bit_val)
				p_parse->valid_bit_st = BIT_H_5;
			else
				p_parse->valid_bit_st = BIT_L_1;
			break;
		case BIT_H_5:	// have rx 5 bit 1
			// rx 6 bit 1 means bus in idle
			// rx 5 bit 1 then rx bit 0 means bit 0 is a stuff bit 
			if(bit_val)
			{
				p_parse->error |= ERR_BIT1;
				p_parse->frm_parse_st = 0;
				p_parse->valid_bit_st = BIT_IDLE;
			}
			else
			{
				p_parse->valid_bit_st = BIT_L_1;
				p_parse->stf_bit = 1;
#ifdef BIT_RX_PARSE_TM				
				tm_probe = tm_probe-getTimeTicks();
		        if(p_parse->tm_max < tm_probe)
		        {
		        	p_parse->tm_max_stat = stat;
		        	p_parse->tm_max = tm_probe;
		        }
#endif		        
				return false;
			}
			break;
		case BIT_L_1:		
			if(!bit_val)
				p_parse->valid_bit_st = BIT_L_2;
			else
				p_parse->valid_bit_st = BIT_H_1;
			break;
		case BIT_L_2:		
			if(!bit_val)
				p_parse->valid_bit_st = BIT_L_3;
			else
				p_parse->valid_bit_st = BIT_H_1;
			break;
		case BIT_L_3:		
			if(!bit_val)
				p_parse->valid_bit_st = BIT_L_4;
			else
				p_parse->valid_bit_st = BIT_H_1;
			break;
		case BIT_L_4:		
			if(!bit_val)
				p_parse->valid_bit_st = BIT_L_5;
			else
				p_parse->valid_bit_st = BIT_H_1;
			break;
		case BIT_L_5:	
			// rx 6 bit 0 means bus in error
			// rx 5 bit 0 then rx bit 1 means bit 1 is a stuff bit 	
			if(!bit_val)
			{
				p_parse->error |= ERR_BIT0;
				p_parse->frm_parse_st = 0;
				p_parse->valid_bit_st = BIT_L_5;
			}
			else
			{
				p_parse->valid_bit_st = BIT_H_1;
				p_parse->stf_bit = 1;
#ifdef BIT_RX_PARSE_TM				
				tm_probe = tm_probe-getTimeTicks();
		        if(p_parse->tm_max < tm_probe)
		        {
		        	p_parse->tm_max_stat = stat;
		        	p_parse->tm_max = tm_probe;
		        }
#endif		        
				return false;
			}
			break;
		 default:
		 	p_parse->error |= ERR_SM_BIT;
		 	p_parse->frm_parse_st = 0;
			p_parse->valid_bit_st = BIT_IDLE;
			break;
	}
	p_parse->stf_bit = 0;
	p_parse->valid_bit = 1;

	switch(p_parse->frm_parse_st)
	{
	  case 0:
	    // SOF
	    p_parse->valid_bit = 0;
	    if(bit_val)
	    {
	      ;
	    }
	    else
	    {
	      p_parse->frm_parse_st = 1;
	    }
	    break;
	  case 1:
	    // EX-ID 28, BASE-ID 10
	    p_parse->can_frm.id = 0;
	    if(bit_val)
	    {
	      p_parse->can_frm.id |= 0x00000400;
	    }
	    else
	    {
	      p_parse->can_frm.id &= 0xfffffbff;
	    }
	    p_parse->frm_parse_st = 2;
	    break;
	  case 2:
	    // EX-ID 27, BASE-ID 9
	    if(bit_val)
	    {
	      p_parse->can_frm.id |= 0x00000200;
	    }
	    else
	    {
	      p_parse->can_frm.id &= 0xfffffdff;
	    }
	    p_parse->frm_parse_st = 3;
	    break;
	  case 3:
	    // EX-ID 26, BASE-ID 8
	    if(bit_val)
	    {
	      p_parse->can_frm.id |= 0x00000100;
	    }
	    else
	    {
	      p_parse->can_frm.id &= 0xfffffeff;
	    }
	    p_parse->frm_parse_st = 4;
	    break;
	  case 4:
	    // EX-ID 25, BASE-ID 7
	    if(bit_val)
	    {
	      p_parse->can_frm.id |= 0x00000080;
	    }
	    else
	    {
	      p_parse->can_frm.id &= 0xffffff7f;
	    }
	    p_parse->frm_parse_st = 5;
	    break;
	  case 5:
	    // EX-ID 24, BASE-ID 6
	    if(bit_val)
	    {
	      p_parse->can_frm.id |= 0x00000040;
	    }
	    else
	    {
	      p_parse->can_frm.id &= 0xffffffbf;
	    }
	    p_parse->frm_parse_st = 6;
	    break;
	  case 6:
	    // EX-ID 23, BASE-ID 5
	    if(bit_val)
	    {
	      p_parse->can_frm.id |= 0x00000020;
	    }
	    else
	    {
	      p_parse->can_frm.id &= 0xffffffdf;
	    }
	    p_parse->frm_parse_st = 7;
	    break;
	  case 7:
	    // EX-ID 22, BASE-ID 4
	    if(bit_val)
	    {
	      p_parse->can_frm.id |= 0x00000010;
	    }
	    else
	    {
	      p_parse->can_frm.id &= 0xffffffef;
	    }
	    p_parse->frm_parse_st = 8;
	    break;
	  case 8:
	    // EX-ID 21, BASE-ID 3
	    if(bit_val)
	    {
	      p_parse->can_frm.id |= 0x00000008;
	    }
	    else
	    {
	      p_parse->can_frm.id &= 0xfffffff7;
	    }
	    p_parse->frm_parse_st = 9;
	    break;
	  case 9:
	    // EX-ID 20, BASE-ID 2
	    if(bit_val)
	    {
	      p_parse->can_frm.id |= 0x00000004;
	    }
	    else
	    {
	      p_parse->can_frm.id &= 0xfffffffb;
	    }
	    p_parse->frm_parse_st = 10;
	    break;
	  case 10:
	    // EX-ID 19, BASE-ID 1
	    if(bit_val)
	    {
	      p_parse->can_frm.id |= 0x00000002;
	    }
	    else
	    {
	      p_parse->can_frm.id &= 0xfffffffd;
	    }
	    p_parse->frm_parse_st = 11;
	    break;
	  case 11:
	    // EX-ID 18, BASE-ID 0
	    if(bit_val)
	    {
	      p_parse->can_frm.id |= 0x00000001;
	    }
	    else
	    {
	      p_parse->can_frm.id &= 0xfffffffe;
	    }
	    p_parse->frm_parse_st = 12;
	    break;
	  case 12:
	    // SRR
	    if(bit_val)
	    {
	      p_parse->can_frm.flag |= F_SRR;
	    }
	    else
	    {
	      p_parse->can_frm.flag &= ~F_SRR;
	    }
	    p_parse->frm_parse_st = 13;
	    break;
	  case 13:
	    // IDE
	    if(bit_val)
	    {
	      p_parse->can_frm.flag |= F_IDE;
	      p_parse->can_frm.id <<= 18;
	      p_parse->frm_parse_st = 14;
	    }
	    else
	    {
	      p_parse->can_frm.flag &= ~F_IDE;
	      p_parse->frm_parse_st = 32;
	    }
	    break;
	  case 14:
	    // EX-ID 17
	    if(bit_val)
	    {
	      p_parse->can_frm.id |= 0x00020000;
	    }
	    else
	    {
	      p_parse->can_frm.id &= 0xfffdffff;
	    }
	    p_parse->frm_parse_st = 15;
	    break;
	  case 15:
	    // EX-ID 16
	    if(bit_val)
	    {
	      p_parse->can_frm.id |= 0x00010000;
	    }
	    else
	    {
	      p_parse->can_frm.id &= 0xfffeffff;
	    }
	    p_parse->frm_parse_st = 16;
	    break;
	  case 16:
	    // EX-ID 15
	    if(bit_val)
	    {
	      p_parse->can_frm.id |= 0x00008000;
	    }
	    else
	    {
	      p_parse->can_frm.id &= 0xffff7fff;
	    }
	    p_parse->frm_parse_st = 17;
	    break;
	  case 17:
	    // EX-ID 14
	    if(bit_val)
	    {
	      p_parse->can_frm.id |= 0x00004000;
	    }
	    else
	    {
	      p_parse->can_frm.id &= 0xffffbfff;
	    }
	    p_parse->frm_parse_st = 18;
	    break;
	  case 18:
	    // EX-ID 13
	    if(bit_val)
	    {
	      p_parse->can_frm.id |= 0x00002000;
	    }
	    else
	    {
	      p_parse->can_frm.id &= 0xffffdfff;
	    }
	    p_parse->frm_parse_st = 19;
	    break;
	  case 19:
	    // EX-ID 12
	    if(bit_val)
	    {
	      p_parse->can_frm.id |= 0x00001000;
	    }
	    else
	    {
	      p_parse->can_frm.id &= 0xffffefff;
	    }
	    p_parse->frm_parse_st = 20;
	    break;
	  case 20:
	    // EX-ID 11
	    if(bit_val)
	    {
	      p_parse->can_frm.id |= 0x00000800;
	    }
	    else
	    {
	      p_parse->can_frm.id &= 0xfffff7ff;
	    }
	    p_parse->frm_parse_st = 21;
	    break;
	  case 21:
	    // EX-ID 10
	    if(bit_val)
	    {
	      p_parse->can_frm.id |= 0x00000400;
	    }
	    else
	    {
	      p_parse->can_frm.id &= 0xfffffbff;
	    }
	    p_parse->frm_parse_st = 22;
	    break;
	  case 22:
	    // EX-ID 9
	    if(bit_val)
	    {
	      p_parse->can_frm.id |= 0x00000200;
	    }
	    else
	    {
	      p_parse->can_frm.id &= 0xfffffdff;
	    }
	    p_parse->frm_parse_st = 23;
	    break;
	  case 23:
	    // EX-ID 8
	    if(bit_val)
	    {
	      p_parse->can_frm.id |= 0x00000100;
	    }
	    else
	    {
	      p_parse->can_frm.id &= 0xfffffeff;
	    }
	    p_parse->frm_parse_st = 24;
	    break;
	  case 24:
	    // EX-ID 7
	    if(bit_val)
	    {
	      p_parse->can_frm.id |= 0x00000080;
	    }
	    else
	    {
	      p_parse->can_frm.id &= 0xffffff7f;
	    }
	    p_parse->frm_parse_st = 25;
	    break;
	  case 25:
	    // EX-ID 6
	    if(bit_val)
	    {
	      p_parse->can_frm.id |= 0x00000040;
	    }
	    else
	    {
	      p_parse->can_frm.id &= 0xffffffbf;
	    }
	    p_parse->frm_parse_st = 26;
	    break;
	  case 26:
	    // EX-ID 5
	    if(bit_val)
	    {
	      p_parse->can_frm.id |= 0x00000020;
	    }
	    else
	    {
	      p_parse->can_frm.id &= 0xffffffdf;
	    }
	    p_parse->frm_parse_st = 27;
	    break;
	  case 27:
	    // EX-ID 4
	    if(bit_val)
	    {
	      p_parse->can_frm.id |= 0x00000010;
	    }
	    else
	    {
	      p_parse->can_frm.id &= 0xffffffef;
	    }
	    p_parse->frm_parse_st = 28;
	    break;
	  case 28:
	    // EX-ID 3
	    if(bit_val)
	    {
	      p_parse->can_frm.id |= 0x00000008;
	    }
	    else
	    {
	      p_parse->can_frm.id &= 0xfffffff7;
	    }
	    p_parse->frm_parse_st = 29;
	    break;
	  case 29:
	    // EX-ID 2
	    if(bit_val)
	    {
	      p_parse->can_frm.id |= 0x00000004;
	    }
	    else
	    {
	      p_parse->can_frm.id &= 0xfffffffb;
	    }
	    p_parse->frm_parse_st = 30;
	    break;
	  case 30:
	    // EX-ID 1
	    if(bit_val)
	    {
	      p_parse->can_frm.id |= 0x00000002;
	    }
	    else
	    {
	      p_parse->can_frm.id &= 0xfffffffd;
	    }
	    p_parse->frm_parse_st = 31;
	    break;
	  case 31:
	    // EX-ID 0
	    if(bit_val)
	    {
	      p_parse->can_frm.id |= 0x00000001;
	    }
	    else
	    {
	      p_parse->can_frm.id &= 0xfffffffe;
	    }
	    p_parse->frm_parse_st = 32;
	    break;
	  case 32:
	      // F_RTR
	    if(bit_val)
	    {
	      p_parse->can_frm.flag |= F_RTR;
	    }
	    else
	    {
	      p_parse->can_frm.flag &= ~F_RTR;
	    }
	    p_parse->frm_parse_st = 33;
	    break;
	  case 33:
	      // F_R0
	    if(bit_val)
	    {
	      p_parse->can_frm.flag |= F_R0;
	    }
	    else
	    {
	      p_parse->can_frm.flag &= ~F_R0;
	    }
	    p_parse->frm_parse_st = 34;
	    break;
	  case 34:
	      // F_R1
	    if(bit_val)
	    {
	      p_parse->can_frm.flag |= F_R1;
	    }
	    else
	    {
	      p_parse->can_frm.flag &= ~F_R1;
	    }
	    p_parse->frm_parse_st = 35;
	    break;
	  case 35:
	    // LEN bit 3
	    p_parse->can_frm.len = 0;
	    if(bit_val)
	    {
	      p_parse->can_frm.len |= 0x08;
	    }
	    else
	    {
	      p_parse->can_frm.len &= 0xf7;
	    }
	    p_parse->frm_parse_st = 36;
	    break;
	  case 36:
	    // LEN bit 2
	    if(bit_val)
	    {
	      p_parse->can_frm.len |= 0x04;
	    }
	    else
	    {
	      p_parse->can_frm.len &= 0xfb;
	    }
	    p_parse->frm_parse_st = 37;
	    break;
	  case 37:
	    // LEN bit 1
	    if(bit_val)
	    {
	      p_parse->can_frm.len |= 0x02;
	    }
	    else
	    {
	      p_parse->can_frm.len &= 0xfd;
	    }
	    p_parse->frm_parse_st = 38;
	    break;
	  case 38:
	    // LEN bit 0
	    if(bit_val)
	    {
	      p_parse->can_frm.len |= 0x01;
	    }
	    else
	    {
	      p_parse->can_frm.len &= 0xfe;
	    }
	    if(0 == p_parse->can_frm.len)
	    {
	      p_parse->frm_parse_st = 103;
	    }
	    else if(8 >= p_parse->can_frm.len)
	    {
	      p_parse->frm_parse_st = 39;
	    }
	    else
	    {
	      p_parse->frm_parse_st = 0;
	    }
	    break;
	  case 39:
	    // DATA 0 bit 7
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[0] |= 0x80;
	    }
	    else
	    {
	      p_parse->can_frm.dat[0] &= 0x7f;
	    }
	     p_parse->frm_parse_st = 40;
	    break;
	  case 40:
	    // DATA 0 bit 6
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[0] |= 0x40;
	    }
	    else
	    {
	      p_parse->can_frm.dat[0] &= 0xbf;
	    }
	     p_parse->frm_parse_st = 41;
	    break;
	  case 41:
	    // DATA 0 bit 5
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[0] |= 0x20;
	    }
	    else
	    {
	      p_parse->can_frm.dat[0] &= 0xdf;
	    }
	     p_parse->frm_parse_st = 42;
	    break;
	  case 42:
	    // DATA 0 bit 4
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[0] |= 0x10;
	    }
	    else
	    {
	      p_parse->can_frm.dat[0] &= 0xef;
	    }
	     p_parse->frm_parse_st = 43;
	    break;
	  case 43:
	    // DATA 0 bit 3
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[0] |= 0x08;
	    }
	    else
	    {
	      p_parse->can_frm.dat[0] &= 0xf7;
	    }
	     p_parse->frm_parse_st = 44;
	    break;
	  case 44:
	    // DATA 0 bit 2
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[0] |= 0x04;
	    }
	    else
	    {
	      p_parse->can_frm.dat[0] &= 0xfb;
	    }
	     p_parse->frm_parse_st = 45;
	    break;
	  case 45:
	    // DATA 0 bit 1
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[0] |= 0x02;
	    }
	    else
	    {
	      p_parse->can_frm.dat[0] &= 0xfd;
	    }
	     p_parse->frm_parse_st = 46;
	    break;
	  case 46:
	    // DATA 0 bit 0
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[0] |= 0x01;
	    }
	    else
	    {
	      p_parse->can_frm.dat[0] &= 0xfe;
	    }
	    if(p_parse->can_frm.len > 1)
	    {
	       p_parse->frm_parse_st = 47;
	    }
	    else
	    {
	       p_parse->frm_parse_st = 103;
	    }
	    break;
	  case 47:
	    // DATA 1 bit 7
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[1] |= 0x80;
	    }
	    else
	    {
	      p_parse->can_frm.dat[1] &= 0x7f;
	    }
	     p_parse->frm_parse_st = 48;
	    break;
	  case 48:
	    // DATA 1 bit 6
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[1] |= 0x40;
	    }
	    else
	    {
	      p_parse->can_frm.dat[1] &= 0xbf;
	    }
	     p_parse->frm_parse_st = 49;
	    break;
	  case 49:
	    // DATA 1 bit 5
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[1] |= 0x20;
	    }
	    else
	    {
	      p_parse->can_frm.dat[1] &= 0xdf;
	    }
	     p_parse->frm_parse_st = 50;
	    break;
	  case 50:
	    // DATA 1 bit 4
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[1] |= 0x10;
	    }
	    else
	    {
	      p_parse->can_frm.dat[1] &= 0xef;
	    }
	     p_parse->frm_parse_st = 51;
	    break;
	  case 51:
	    // DATA 1 bit 3
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[1] |= 0x08;
	    }
	    else
	    {
	      p_parse->can_frm.dat[1] &= 0xf7;
	    }
	     p_parse->frm_parse_st = 52;
	    break;
	  case 52:
	    // DATA 1 bit 2
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[1] |= 0x04;
	    }
	    else
	    {
	      p_parse->can_frm.dat[1] &= 0xfb;
	    }
	     p_parse->frm_parse_st = 53;
	    break;
	  case 53:
	    // DATA 1 bit 1
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[1] |= 0x02;
	    }
	    else
	    {
	      p_parse->can_frm.dat[1] &= 0xfd;
	    }
	     p_parse->frm_parse_st = 54;
	    break;
	  case 54:
	    // DATA 1 bit 0
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[1] |= 0x01;
	    }
	    else
	    {
	      p_parse->can_frm.dat[1] &= 0xfe;
	    }
	    if(p_parse->can_frm.len > 2)
	    {
	       p_parse->frm_parse_st = 55;
	    }
	    else
	    {
	       p_parse->frm_parse_st = 103;
	    }
	    break;
	  case 55:
	    // DATA 2 bit 7
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[2] |= 0x80;
	    }
	    else
	    {
	      p_parse->can_frm.dat[2] &= 0x7f;
	    }
	     p_parse->frm_parse_st = 56;
	    break;
	  case 56:
	    // DATA 2 bit 6
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[2] |= 0x40;
	    }
	    else
	    {
	      p_parse->can_frm.dat[2] &= 0xbf;
	    }
	     p_parse->frm_parse_st = 57;
	    break;
	  case 57:
	    // DATA 2 bit 5
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[2] |= 0x20;
	    }
	    else
	    {
	      p_parse->can_frm.dat[2] &= 0xdf;
	    }
	     p_parse->frm_parse_st = 58;
	    break;
	  case 58:
	    // DATA 2 bit 4
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[2] |= 0x10;
	    }
	    else
	    {
	      p_parse->can_frm.dat[2] &= 0xef;
	    }
	     p_parse->frm_parse_st = 59;
	    break;
	  case 59:
	    // DATA 2 bit 3
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[2] |= 0x08;
	    }
	    else
	    {
	      p_parse->can_frm.dat[2] &= 0xf7;
	    }
	     p_parse->frm_parse_st = 60;
	    break;
	  case 60:
	    // DATA 2 bit 2
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[2] |= 0x04;
	    }
	    else
	    {
	      p_parse->can_frm.dat[2] &= 0xfb;
	    }
	     p_parse->frm_parse_st = 61;
	    break;
	  case 61:
	    // DATA 2 bit 1
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[2] |= 0x02;
	    }
	    else
	    {
	      p_parse->can_frm.dat[2] &= 0xfd;
	    }
	     p_parse->frm_parse_st = 62;
	    break;
	  case 62:
	    // DATA 2 bit 0
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[2] |= 0x01;
	    }
	    else
	    {
	      p_parse->can_frm.dat[2] &= 0xfe;
	    }
	    if(p_parse->can_frm.len > 3)
	    {
	       p_parse->frm_parse_st = 63;
	    }
	    else
	    {
	       p_parse->frm_parse_st = 103;
	    }
	    break;
	  case 63:
	    // DATA 3 bit 7
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[3] |= 0x80;
	    }
	    else
	    {
	      p_parse->can_frm.dat[3] &= 0x7f;
	    }
	     p_parse->frm_parse_st = 64;
	    break;
	  case 64:
	    // DATA 3 bit 6
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[3] |= 0x40;
	    }
	    else
	    {
	      p_parse->can_frm.dat[3] &= 0xbf;
	    }
	     p_parse->frm_parse_st = 65;
	    break;
	  case 65:
	    // DATA 3 bit 5
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[3] |= 0x20;
	    }
	    else
	    {
	      p_parse->can_frm.dat[3] &= 0xdf;
	    }
	     p_parse->frm_parse_st = 66;
	    break;
	  case 66:
	    // DATA 3 bit 4
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[3] |= 0x10;
	    }
	    else
	    {
	      p_parse->can_frm.dat[3] &= 0xef;
	    }
	     p_parse->frm_parse_st = 67;
	    break;
	  case 67:
	    // DATA 3 bit 3
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[3] |= 0x08;
	    }
	    else
	    {
	      p_parse->can_frm.dat[3] &= 0xf7;
	    }
	     p_parse->frm_parse_st = 68;
	    break;
	  case 68:
	    // DATA 3 bit 2
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[3] |= 0x04;
	    }
	    else
	    {
	      p_parse->can_frm.dat[3] &= 0xfb;
	    }
	     p_parse->frm_parse_st = 69;
	    break;
	  case 69:
	    // DATA 3 bit 1
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[3] |= 0x02;
	    }
	    else
	    {
	      p_parse->can_frm.dat[3] &= 0xfd;
	    }
	     p_parse->frm_parse_st = 70;
	    break;
	  case 70:
	    // DATA 3 bit 0
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[3] |= 0x01;
	    }
	    else
	    {
	      p_parse->can_frm.dat[3] &= 0xfe;
	    }
	    if(p_parse->can_frm.len > 4)
	    {
	       p_parse->frm_parse_st = 71;
	    }
	    else
	    {
	       p_parse->frm_parse_st = 103;
	    }
	    break;
	  case 71:
	    // DATA 4 bit 7
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[4] |= 0x80;
	    }
	    else
	    {
	      p_parse->can_frm.dat[4] &= 0x7f;
	    }
	     p_parse->frm_parse_st = 72;
	    break;
	  case 72:
	    // DATA 4 bit 6
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[4] |= 0x40;
	    }
	    else
	    {
	      p_parse->can_frm.dat[4] &= 0xbf;
	    }
	     p_parse->frm_parse_st = 73;
	    break;
	  case 73:
	    // DATA 4 bit 5
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[4] |= 0x20;
	    }
	    else
	    {
	      p_parse->can_frm.dat[4] &= 0xdf;
	    }
	     p_parse->frm_parse_st = 74;
	    break;
	  case 74:
	    // DATA 4 bit 4
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[4] |= 0x10;
	    }
	    else
	    {
	      p_parse->can_frm.dat[4] &= 0xef;
	    }
	     p_parse->frm_parse_st = 75;
	    break;
	  case 75:
	    // DATA 4 bit 3
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[4] |= 0x08;
	    }
	    else
	    {
	      p_parse->can_frm.dat[4] &= 0xf7;
	    }
	     p_parse->frm_parse_st = 76;
	    break;
	  case 76:
	    // DATA 4 bit 2
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[4] |= 0x04;
	    }
	    else
	    {
	      p_parse->can_frm.dat[4] &= 0xfb;
	    }
	     p_parse->frm_parse_st = 77;
	    break;
	  case 77:
	    // DATA 4 bit 1
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[4] |= 0x02;
	    }
	    else
	    {
	      p_parse->can_frm.dat[4] &= 0xfd;
	    }
	     p_parse->frm_parse_st = 78;
	    break;
	  case 78:
	    // DATA 4 bit 0
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[4] |= 0x01;
	    }
	    else
	    {
	      p_parse->can_frm.dat[4] &= 0xfe;
	    }
	    if(p_parse->can_frm.len > 5)
	    {
	       p_parse->frm_parse_st = 79;
	    }
	    else
	    {
	       p_parse->frm_parse_st = 103;
	    }
	    break;
	  case 79:
	    // DATA 5 bit 7
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[5] |= 0x80;
	    }
	    else
	    {
	      p_parse->can_frm.dat[5] &= 0x7f;
	    }
	     p_parse->frm_parse_st = 80;
	    break;
	  case 80:
	    // DATA 5 bit 6
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[5] |= 0x40;
	    }
	    else
	    {
	      p_parse->can_frm.dat[5] &= 0xbf;
	    }
	     p_parse->frm_parse_st = 81;
	    break;
	  case 81:
	    // DATA 5 bit 5
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[5] |= 0x20;
	    }
	    else
	    {
	      p_parse->can_frm.dat[5] &= 0xdf;
	    }
	     p_parse->frm_parse_st = 82;
	    break;
	  case 82:
	    // DATA 5 bit 4
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[5] |= 0x10;
	    }
	    else
	    {
	      p_parse->can_frm.dat[5] &= 0xef;
	    }
	     p_parse->frm_parse_st = 83;
	    break;
	  case 83:
	    // DATA 5 bit 3
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[5] |= 0x08;
	    }
	    else
	    {
	      p_parse->can_frm.dat[5] &= 0xf7;
	    }
	     p_parse->frm_parse_st = 84;
	    break;
	  case 84:
	    // DATA 5 bit 2
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[5] |= 0x04;
	    }
	    else
	    {
	      p_parse->can_frm.dat[5] &= 0xfb;
	    }
	     p_parse->frm_parse_st = 85;
	    break;
	  case 85:
	    // DATA 5 bit 1
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[5] |= 0x02;
	    }
	    else
	    {
	      p_parse->can_frm.dat[5] &= 0xfd;
	    }
	     p_parse->frm_parse_st = 86;
	    break;
	  case 86:
	    // DATA 5 bit 0
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[5] |= 0x01;
	    }
	    else
	    {
	      p_parse->can_frm.dat[5] &= 0xfe;
	    }
	    if(p_parse->can_frm.len > 6)
	    {
	       p_parse->frm_parse_st = 87;
	    }
	    else
	    {
	       p_parse->frm_parse_st = 103;
	    }
	    break;
	  case 87:
	    // DATA 6 bit 7
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[6] |= 0x80;
	    }
	    else
	    {
	      p_parse->can_frm.dat[6] &= 0x7f;
	    }
	     p_parse->frm_parse_st = 88;
	    break;
	  case 88:
	    // DATA 6 bit 6
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[6] |= 0x40;
	    }
	    else
	    {
	      p_parse->can_frm.dat[6] &= 0xbf;
	    }
	     p_parse->frm_parse_st = 89;
	    break;
	  case 89:
	    // DATA 6 bit 5
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[6] |= 0x20;
	    }
	    else
	    {
	      p_parse->can_frm.dat[6] &= 0xdf;
	    }
	     p_parse->frm_parse_st = 90;
	    break;
	  case 90:
	    // DATA 6 bit 4
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[6] |= 0x10;
	    }
	    else
	    {
	      p_parse->can_frm.dat[6] &= 0xef;
	    }
	     p_parse->frm_parse_st = 91;
	    break;
	  case 91:
	    // DATA 6 bit 3
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[6] |= 0x08;
	    }
	    else
	    {
	      p_parse->can_frm.dat[6] &= 0xf7;
	    }
	     p_parse->frm_parse_st = 92;
	    break;
	  case 92:
	    // DATA 6 bit 2
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[6] |= 0x04;
	    }
	    else
	    {
	      p_parse->can_frm.dat[6] &= 0xfb;
	    }
	     p_parse->frm_parse_st = 93;
	    break;
	  case 93:
	    // DATA 6 bit 1
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[6] |= 0x02;
	    }
	    else
	    {
	      p_parse->can_frm.dat[6] &= 0xfd;
	    }
	     p_parse->frm_parse_st = 94;
	    break;
	  case 94:
	    // DATA 6 bit 0
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[6] |= 0x01;
	    }
	    else
	    {
	      p_parse->can_frm.dat[6] &= 0xfe;
	    }
	    if(p_parse->can_frm.len > 7)
	    {
	       p_parse->frm_parse_st = 95;
	    }
	    else
	    {
	       p_parse->frm_parse_st = 103;
	    }
	    break;
	  case 95:
	    // DATA 7 bit 7
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[7] |= 0x80;
	    }
	    else
	    {
	      p_parse->can_frm.dat[7] &= 0x7f;
	    }
	     p_parse->frm_parse_st = 96;
	    break;
	  case 96:
	    // DATA 7 bit 6
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[7] |= 0x40;
	    }
	    else
	    {
	      p_parse->can_frm.dat[7] &= 0xbf;
	    }
	     p_parse->frm_parse_st = 97;
	    break;
	  case 97:
	    // DATA 7 bit 5
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[7] |= 0x20;
	    }
	    else
	    {
	      p_parse->can_frm.dat[7] &= 0xdf;
	    }
	     p_parse->frm_parse_st = 98;
	    break;
	  case 98:
	    // DATA 7 bit 4
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[7] |= 0x10;
	    }
	    else
	    {
	      p_parse->can_frm.dat[7] &= 0xef;
	    }
	     p_parse->frm_parse_st = 99;
	    break;
	  case 99:
	    // DATA 7 bit 3
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[7] |= 0x08;
	    }
	    else
	    {
	      p_parse->can_frm.dat[7] &= 0xf7;
	    }
	     p_parse->frm_parse_st = 100;
	    break;
	  case 100:
	    // DATA 7 bit 2
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[7] |= 0x04;
	    }
	    else
	    {
	      p_parse->can_frm.dat[7] &= 0xfb;
	    }
	     p_parse->frm_parse_st = 101;
	    break;
	  case 101:
	    // DATA 7 bit 1
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[7] |= 0x02;
	    }
	    else
	    {
	      p_parse->can_frm.dat[7] &= 0xfd;
	    }
	     p_parse->frm_parse_st = 102;
	    break;
	  case 102:
	    // DATA 7 bit 0
	    if(bit_val)
	    {
	      p_parse->can_frm.dat[7] |= 0x01;
	    }
	    else
	    {
	      p_parse->can_frm.dat[7] &= 0xfe;
	    }
	    if(p_parse->can_frm.len > 8)
	    {
	       p_parse->frm_parse_st = 103;
	    }
	    else
	    {
	       p_parse->frm_parse_st = 103;
	    }
	    break;	    
	  case 103:
	    // CRC bit 14
	    p_parse->can_frm.crc = 0;
	    p_parse->valid_bit = 0;
	    if(bit_val)
	    {
	      p_parse->can_frm.crc |= 0x4000;
	    }
	    else
	    {
	      p_parse->can_frm.crc &= 0xbfff;
	    }
	     p_parse->frm_parse_st = 104;
	    break;
	  case 104:
	    // CRC bit 13
	    p_parse->valid_bit = 0;
	    if(bit_val)
	    {
	      p_parse->can_frm.crc |= 0x2000;
	    }
	    else
	    {
	      p_parse->can_frm.crc &= 0xdfff;
	    }
	     p_parse->frm_parse_st = 105;
	    break;
	  case 105:
	    // CRC bit 12
	    p_parse->valid_bit = 0;
	    if(bit_val)
	    {
	      p_parse->can_frm.crc |= 0x1000;
	    }
	    else
	    {
	      p_parse->can_frm.crc &= 0xefff;
	    }
	     p_parse->frm_parse_st = 106;
	    break;
	  case 106:
	    // CRC bit 11
	    p_parse->valid_bit = 0;
	    if(bit_val)
	    {
	      p_parse->can_frm.crc |= 0x800;
	    }
	    else
	    {
	      p_parse->can_frm.crc &= 0xf7ff;
	    }
	     p_parse->frm_parse_st = 107;
	    break;
	  case 107:
	    // CRC bit 10
	    p_parse->valid_bit = 0;
	    if(bit_val)
	    {
	      p_parse->can_frm.crc |= 0x400;
	    }
	    else
	    {
	      p_parse->can_frm.crc &= 0xfbff;
	    }
	     p_parse->frm_parse_st = 108;
	    break;
	  case 108:
	    // CRC bit 9
	    p_parse->valid_bit = 0;
	    if(bit_val)
	    {
	      p_parse->can_frm.crc |= 0x200;
	    }
	    else
	    {
	      p_parse->can_frm.crc &= 0xfdff;
	    }
	     p_parse->frm_parse_st = 109;
	    break;
	  case 109:
	    // CRC bit 8
	    p_parse->valid_bit = 0;
	    if(bit_val)
	    {
	      p_parse->can_frm.crc |= 0x100;
	    }
	    else
	    {
	      p_parse->can_frm.crc &= 0xfeff;
	    }
	     p_parse->frm_parse_st = 110;
	    break;
	  case 110:
	    // CRC bit 7
	    p_parse->valid_bit = 0;
	    if(bit_val)
	    {
	      p_parse->can_frm.crc |= 0x80;
	    }
	    else
	    {
	      p_parse->can_frm.crc &= 0xff7f;
	    }
	     p_parse->frm_parse_st = 111;
	    break;
	  case 111:
	    // CRC bit 6
	    p_parse->valid_bit = 0;
	    if(bit_val)
	    {
	      p_parse->can_frm.crc |= 0x40;
	    }
	    else
	    {
	      p_parse->can_frm.crc &= 0xffbf;
	    }
	     p_parse->frm_parse_st = 112;
	    break;
	  case 112:
	    // CRC bit 5
	    p_parse->valid_bit = 0;
	    if(bit_val)
	    {
	      p_parse->can_frm.crc |= 0x20;
	    }
	    else
	    {
	      p_parse->can_frm.crc &= 0xffdf;
	    }
	     p_parse->frm_parse_st = 113;
	    break;
	  case 113:
	    // CRC bit 4
	    p_parse->valid_bit = 0;
	    if(bit_val)
	    {
	      p_parse->can_frm.crc |= 0x10;
	    }
	    else
	    {
	      p_parse->can_frm.crc &= 0xffef;
	    }
	     p_parse->frm_parse_st = 114;
	    break;
	  case 114:
	    // CRC bit 3
	    p_parse->valid_bit = 0;
	    if(bit_val)
	    {
	      p_parse->can_frm.crc |= 0x08;
	    }
	    else
	    {
	      p_parse->can_frm.crc &= 0xfff7;
	    }
	     p_parse->frm_parse_st = 115;
	    break;
	  case 115:
	    // CRC bit 2
	    p_parse->valid_bit = 0;
	    if(bit_val)
	    {
	      p_parse->can_frm.crc |= 0x04;
	    }
	    else
	    {
	      p_parse->can_frm.crc &= 0xfffb;
	    }
	     p_parse->frm_parse_st = 116;
	    break;
	  case 116:
	    // CRC bit 1
	    p_parse->valid_bit = 0;
	    if(bit_val)
	    {
	      p_parse->can_frm.crc |= 0x02;
	    }
	    else
	    {
	      p_parse->can_frm.crc &= 0xfffd;
	    }
	     p_parse->frm_parse_st = 117;
	    break;
	  case 117:
	    // CRC bit 0
	    p_parse->valid_bit = 0;
	    if(bit_val)
	    {
	      p_parse->can_frm.crc |= 0x01;
	    }
	    else
	    {
	      p_parse->can_frm.crc &= 0xfffe;
	    }
	     p_parse->frm_parse_st = 118;
	    break;
	  case 118:
	    // CRC GAP bit
	    p_parse->valid_bit = 0;
	    if(bit_val)
	    {
	      p_parse->can_frm.flag |= F_CRC_GAP;
	    }
	    else
	    {
	      p_parse->can_frm.flag &= ~F_CRC_GAP;
	    }
	    res = true;
	    p_parse->valid_bit_st = 0;
	    p_parse->frm_parse_st = 0;
	    break;
	  default:
	    p_parse->error |= ERR_SM_FRM;
	    p_parse->valid_bit_st = 0;
	    p_parse->frm_parse_st = 0;
	    break;
	}
#ifdef BIT_RX_PARSE_TM
	tm_probe = tm_probe-getTimeTicks();
    if(p_parse->tm_max < tm_probe)
    {
    	p_parse->tm_max_stat = stat;
    	p_parse->tm_max = tm_probe;
    }
#endif
	return res;
}


void bit_tx_parse_init(TY_BIT_TX_PARSE *p_parse)
{
	memset(p_parse, 0, sizeof(TY_BIT_RX_PARSE));
}

void bit_tx_parse_rst(TY_BIT_TX_PARSE *p_parse)
{
	p_parse->sm = 0;
}

/// 解析出要发送的数据
/// @param	p_tx_parse	[io]发送控制结构体
/// @param	p_tx_bit	[o] 待发送数据
/// @retval	bool		true-发送中，false-发送完毕
bool bit_tx_parse(TY_BIT_TX_PARSE *p_tx_parse, uint8 *p_tx_bit)
{
	bool b_res=true;
	
	
	if(0 != p_tx_parse->sm)
	{
		if(p_tx_parse->bit0_cnt >= 5)
		{
			*p_tx_bit = 1;
			p_tx_parse->bit0_cnt =0;
			p_tx_parse->bit1_cnt =1;
			return true;
		}
		else if(p_tx_parse->bit1_cnt >= 5)
		{
			*p_tx_bit = 0;
			p_tx_parse->bit0_cnt =1;
			p_tx_parse->bit1_cnt =0;
			return true;
		}
		else
		{
			;
		}
	}
	
	switch(p_tx_parse->sm)
	{
		case 0: 	
			*p_tx_bit = 0;
			p_tx_parse->bit0_cnt =1;
			p_tx_parse->bit1_cnt =0;
			p_tx_parse->bit_in_seg = 0;
			p_tx_parse->sm = 1;
			break;
		case 1:
			*p_tx_bit = (p_tx_parse->frame.id>>28)&0x01;
			p_tx_parse->frame.id <<= 1;
			p_tx_parse->bit_in_seg ++;
			if(p_tx_parse->bit_in_seg >= 11)
			{
				p_tx_parse->bit_in_seg = 0;
				p_tx_parse->sm = 2;
			}
			break;
		case 2:
			if(p_tx_parse->bit_in_seg)	// ide
			{
				if(p_tx_parse->frame.flag&F_IDE)
					*p_tx_bit = 1;
				else
					*p_tx_bit = 0;
			}
			else						// srr
			{
				if(p_tx_parse->frame.flag&F_SRR)
					*p_tx_bit = 1;
				else
					*p_tx_bit = 0;
			}
			p_tx_parse->bit_in_seg ++;
			if(p_tx_parse->bit_in_seg >= 2)
			{
				p_tx_parse->bit_in_seg = 0;
				p_tx_parse->sm = 3;
			}
			break;
		case 3:
			*p_tx_bit = (p_tx_parse->frame.id>>28)&0x01;
			p_tx_parse->frame.id <<= 1;
			p_tx_parse->bit_in_seg ++;
			if(p_tx_parse->bit_in_seg >= 18)
			{
				p_tx_parse->bit_in_seg = 0;
				p_tx_parse->sm = 4;
			}
			break;
		case 4:
			if(0 == p_tx_parse->bit_in_seg)	
			{
				if(p_tx_parse->frame.flag&F_RTR)
					*p_tx_bit = 1;
				else
					*p_tx_bit = 0;
			}
			else if(1 == p_tx_parse->bit_in_seg)
			{
				if(p_tx_parse->frame.flag&F_R0)
					*p_tx_bit = 1;
				else
					*p_tx_bit = 0;
			}	
			else						
			{
				if(p_tx_parse->frame.flag&F_R1)
					*p_tx_bit = 1;
				else
					*p_tx_bit = 0;
			}
			p_tx_parse->bit_in_seg ++;
			if(p_tx_parse->bit_in_seg >= 3)
			{
				p_tx_parse->bit_in_seg = 0;
				p_tx_parse->len = p_tx_parse->frame.len;
				p_tx_parse->sm = 5;
			}
			break;
		case 5:		// len
			*p_tx_bit = (p_tx_parse->frame.len>>3)&0x01;
			p_tx_parse->frame.len <<= 1;
			p_tx_parse->bit_in_seg ++;
			if(p_tx_parse->bit_in_seg >= 4)
			{
				p_tx_parse->bit_in_seg = 0;
				if(0==p_tx_parse->len)
				{
					p_tx_parse->sm = 7;
				}
				else
				{
					p_tx_parse->sm = 6;
				}
			}
			break;
		case 6:
			*p_tx_bit = (p_tx_parse->frame.dat[p_tx_parse->bit_in_seg/8]>>7)&0x01;
			p_tx_parse->frame.dat[p_tx_parse->bit_in_seg/8] <<= 1;
			p_tx_parse->bit_in_seg ++;
			if(p_tx_parse->bit_in_seg >= p_tx_parse->len*8)
			{
				p_tx_parse->bit_in_seg = 0;
				p_tx_parse->sm = 7;
			}
			break;
		case 7:
			*p_tx_bit = (p_tx_parse->frame.crc>>14)&0x01;
			p_tx_parse->frame.crc <<= 1;
			p_tx_parse->bit_in_seg ++;
			if(p_tx_parse->bit_in_seg >= 15)
			{
				p_tx_parse->bit_in_seg = 0;
				p_tx_parse->sm = 8;
			}
			break;
		case 8:
			*p_tx_bit = 1;
			p_tx_parse->sm = 9;
			break;
		case 9:						// ack
			*p_tx_bit = 1;
			p_tx_parse->bit_in_seg = 0;
			p_tx_parse->sm = 0;
			break;
		default:
			*p_tx_bit = 1;
			p_tx_parse->bit_in_seg = 0;
			p_tx_parse->sm = 0;
			b_res = false;
			break;
	}
	
	if(*p_tx_bit)
	{
		p_tx_parse->bit0_cnt =0;
		p_tx_parse->bit1_cnt ++;
	}
	else
	{
		p_tx_parse->bit0_cnt ++;
		p_tx_parse->bit1_cnt =0;
	}
	return b_res;
}
#pragma section code_in_ram end
