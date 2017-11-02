#ifndef CAN_CTL_H_
#define CAN_CTL_H_

#include "drvDef.h"
#include "bit_parse.h"

#ifndef CAN_CTL_DEBUG
//#define CAN_CTL_DEBUG
#endif

typedef enum
{
	CAN_CTL_SM_STOP=0,
	CAN_CTL_SM_IDLE,
	CAN_CTL_SM_RX_WAIT,
	
	CAN_CTL_SM_RX_GO,
	CAN_CTL_SM_RX,
	CAN_CTL_SM_RX_ACK,
	CAN_CTL_SM_RX_RST,
	
	CAN_CTL_SM_TX_ARB,
	CAN_CTL_SM_TX,
	CAN_CTL_SM_TX_ACK,
	CAN_CTL_SM_TX_GAP,
}ENUM_CAN_CTL_SM;

#define TX_FIFO_LEN		(128)
#define TX_FIFO_LEN_MSK	(TX_FIFO_LEN-1)
#define RX_FIFO_LEN		(128)
#define RX_FIFO_LEN_MAK	(RX_FIFO_LEN-1)
#define BIT_BUF_LEN		(128)
#define BIT_BUF_LEN_MSK	(BIT_BUF_LEN-1)
typedef struct
{
	uint8			sm;             	// 状态机
	
	uint32			timing_cnt;			// 接收，接收处理统计
	uint32			timing_tm_max_st;
	uint32			timing_tm_max;
	uint32			timing_tm_min;	
	
	uint8			sync_tx;        	// 接收，边沿信号信号量
	uint8			sync_rx;
	uint8			sync_err;
	uint32			sync_cnt;			// 接收，同步处理
	uint32			sync_tm_max;
	uint32			sync_tm_min;
	uint32			head_rsync;
	uint32			tail_rsync;
	uint32			tq9_rsync;
	
	uint8			bit_stat;			// 接收，当前值
	uint16			crc;
	TY_BIT_RX_PARSE	bit_rx_parse;		// 接收，bit流解码
	
	uint8			tq_cnt;        	 	// 当前bit中tq的计数
	uint8			tq_sync;        	// 当前bit中是否已经进行过再同步
	uint8           tq_sync_err;    	// 

#ifdef CAN_CTL_DEBUG	
	uint8			rx_bit_lock;		// 接收，调试时抓取bit用
	uint32			rx_bit_ts[BIT_BUF_LEN];
	uint8			rx_bit[BIT_BUF_LEN];		
	uint16			rx_bit_wr;
	uint16			rx_bit_rd;
	
	uint8			log_wr;
	uint8			log_rd;
	uint32			log[8][8];			// 调试，抓取异常数据	
#endif

	// rx
	uint8			rx_flag;
	uint8			rx_crc_ok;
	uint8			rx_fifo_ok;
	uint32			sm_rx_2_rx_ack;
	uint32			rx_err_crc;
	uint32			rx_err_bit_parse;
	uint32			rx_err_sm;
	uint32			sm_rst;
	uint32			sm_ack;
	
	
	TY_CAN_FRM		rx_fifo[RX_FIFO_LEN];	// 接收，fifo
	uint16			rx_fifo_wr;
	uint16			rx_fifo_rd;
	uint32			rx_ov_cnt;
	uint32			rx_cnt;
	
	// tx
	uint16			tx_fifo_wr;     		// 发送fifo
	uint16			tx_fifo_rd;
	TY_CAN_FRM		tx_fifo[TX_FIFO_LEN];
	
	uint8			tx_bit;
	TY_BIT_TX_PARSE p_tx_bit_parse;
	
	uint32			tx_cnt;
	uint32			arb_fail;
	uint32			arb_fail_info;
	uint32			ack_err;
	uint32			tx_err_sm;
	uint32			tx_succ;
}TY_CAN_CTL;

void can_ctl_init(TY_CAN_CTL *p_can, uint32 bit_rate);
void can_ctl_enable(TY_CAN_CTL *p_can);
void can_ctl_disable(TY_CAN_CTL *p_can);

bool can_ctl_wr(TY_CAN_CTL *p_can, uint32 id, uint8 len, uint8 *p_dat);
bool can_ctl_rd(TY_CAN_CTL *p_can, uint32 *p_ts, uint32 *p_id, uint8 *p_len, uint8 *p_dat);

// 读取采样电平					   
#define RX_PIN_STAT()       (0x01&(GPIO_PDD_GetPortDataInput(PTE_BASE_PTR)>>25))
#define TX_PIN_SET(stat)    do{if(stat){GPIO_PDD_SetPortDataOutputMask(PTE_BASE_PTR, PORT_PDD_PIN_24);}else{GPIO_PDD_ClearPortDataOutputMask(PTE_BASE_PTR, PORT_PDD_PIN_24);}}while(0)

#include "MK60DZ10.h"
#define PIT_GET_CNT()		(*(uint32*)((uint32)PIT_BASE_PTR+4*(256/4+2*4+1)))
// 设定定时器重装值
#define PIT_SET_LDVAL(val)	pit3_set_ldval(val)
#endif
