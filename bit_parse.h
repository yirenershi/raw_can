#ifndef BIT_TYPE_H_
#define BIT_TYPE_H_

#define CAN_CTL_GET_TIME()	getTimeTicks()

#define F_SRR		0x01	// 扩展帧才有（为了兼容标准帧），固定1
#define F_IDE		0x02	// 0-标准帧，1-扩展帧
#define F_RTR		0x04	// 1-远程帧，0-数据帧
#define F_R0		0x08	
#define F_R1		0x10
#define F_SOF		0x20
#define F_EOF		0x40
#define F_CRC_GAP	0x80

typedef enum
{
	BIT_STF=0,		// 填充位
	BIT_ERR_IDLE,	// 隐形超限
	BIT_LOCK,		// 显性超限
	BIT_OK,			// 正常
}ENUM_BIT_ST;

typedef enum
{
	BIT_IDLE=0,
	BIT_H_1,
	BIT_H_2,
	BIT_H_3,
	BIT_H_4,
	BIT_H_5,
	BIT_L_1,	// 6
	BIT_L_2,
	BIT_L_3,
	BIT_L_4,	// 9
	BIT_L_5,
}ENUM_BIT_SM;

typedef struct
{
	uint32 ts;
	uint32 id;
	uint8  len;
	uint8  dat[8];
	uint16 crc;
	uint8  flag;
	uint8  len_bit;
}TY_CAN_FRM;

typedef enum
{
	MODE_NU=0,
	MODE_STD,
	MODE_EXT
}ENUM_MODE;

typedef enum
{
	ERR_BIT0	=0x01,	// 收到超过5个bit0
	ERR_BIT1	=0x02,	// 收到超过5个bit1
	ERR_LEN		=0x04,	// 长度超范围
	ERR_SM_BIT  =0x08,	// 删除填充位状态机出错
	ERR_SM_FRM	=0x10,	// BIT流解码状态机出错
}ENUM_ERR;

typedef struct
{
	uint8		valid_bit_st;	// 删除填充位状态机
	uint8		frm_parse_st;	// bit流解码状态机
	uint8		bit_in_seg;		// 帧内段解码bit计数
	uint8		stf_bit;		// 填充bit
	uint8		valid_bit;		// 这个bit是否参与CRC计算，0-不参与
	uint8		error;			// 错误，见ENUM_ERR
	uint8		mode;			// CAN网工作模式，标准还是扩展模式
	TY_CAN_FRM 	can_frm;		// CAN报文存储区
	uint16		tm_max_stat;	// 最大耗时状态机
	uint32		tm_max;			// 最大耗时
}TY_BIT_RX_PARSE;

typedef struct
{
	uint32		bit_cnt;
	uint8		sm;				// bit流解码状态机
	uint8		bit_in_seg;		// 帧内段解码bit计数
	uint8		bit0_cnt;		// bit0连续计数
	uint8		bit1_cnt;		// bit1连续计数
	uint8		error;			// 错误
	uint8		mode;			// CAN网工作模式，标准还是扩展模式
	uint8		len;
	TY_CAN_FRM	frame;
	uint16		tm_max_stat;
	uint32		tm_max;
}TY_BIT_TX_PARSE;

uint16 can_frame_crc(TY_CAN_FRM *p_frame);

void bit_rx_parse_rst(TY_BIT_RX_PARSE *p_parse);

bool bit_rx_parse(TY_BIT_RX_PARSE *p_parse, uint8 bit_val);

bool tx_bit_parse(TY_BIT_TX_PARSE *p_tx_parse, uint8 *p_tx_bit);

#endif
