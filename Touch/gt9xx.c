/*******************************************************************************
  * Company: Wang Electronic Technology Co., Ltd.
  ******************************************************************************
  * 文件名称：gt9xx.c
  * 功能说明：红龙103 GT910-7寸电容驱动
  * 版    本：V1.0
	* 作    者：openmcu	
  * 日    期：2014-09-27
********************************************************************************
  * 文件名称：
  * 功能说明：
  * 版    本：
	* 更新作者:	
  * 日    期：
	* 更新内容：
********************************************************************************/
#include "stm32f10x.h"
#include "gt9xx.h"
#include "iic.h"
#include "stdio.h"
#include "string.h"
#include "gt910_firmware.h"

__IO uint16_t point_x,point_y;
uint8_t Touch_success=0;
#define GTP_REG_HAVE_KEY                0x804E
#define GTP_REG_MATRIX_DRVNUM           0x8069     
#define GTP_REG_MATRIX_SENNUM           0x806A
#define GTP_REG_CONFIG_DATA   0x8047

#define GTP_BAK_REF_SEND                0
#define GTP_BAK_REF_STORE               1
#define CFG_LOC_DRVA_NUM                (29-2)
#define CFG_LOC_DRVB_NUM                (30-2)
#define CFG_LOC_SENS_NUM                (31-2)
#define GTP_REG_BAK_REF                 0x99D0
#define GTP_REG_MAIN_CLK                0x8020

#define _bRW_MISCTL__SRAM_BANK       0x4048
#define _bRW_MISCTL__MEM_CD_EN       0x4049
#define _bRW_MISCTL__CACHE_EN        0x404B
#define _bRW_MISCTL__TMR0_EN         0x40B0
#define _rRW_MISCTL__SWRST_B0_       0x4180
#define _bWO_MISCTL__CPU_SWRST_PULSE 0x4184
#define _rRW_MISCTL__BOOTCTL_B0_     0x4190
#define _rRW_MISCTL__BOOT_OPT_B0_    0x4218
#define _rRW_MISCTL__BOOT_CTL_       0x5094



struct goodix_ts_data ts;
uint8_t i2c_opr_buf[FL_PACK_SIZE] = {0};
uint8_t chk_cmp_buf[FL_PACK_SIZE] = {0};

uint16_t version_info;
uint8_t cfg[240] = {0};


uint8_t CTP_CFG_GROUP[] = {\
0x00,0xE0,0x01,0x20,0x03,0x0A,0x05,0x00,0x01,0x0F,\
0x28,0x0F,0x50,0x32,0x03,0x05,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x89,0x29,0x0A,\
0x52,0x50,0x0C,0x08,0x00,0x00,0x00,0x00,0x03,0x1D,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x32,0x00,0x00,\
0x00,0x48,0x70,0x94,0xC5,0x02,0x07,0x00,0x00,0x04,\
0x87,0x4B,0x00,0x7D,0x52,0x00,0x74,0x59,0x00,0x6B,\
0x62,0x00,0x64,0x6B,0x00,0x64,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x08,0x0A,0x0C,0x0E,0x10,0x12,0x14,0x16,\
0x18,0x1A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x2A,0x29,0x28,0x24,0x22,0x20,0x1F,0x1E,\
0x1D,0x0E,0x0C,0x0A,0x08,0x06,0x05,0x04,0x02,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0xAA,0x01};
struct CNTOUCH_DATA tds;

static void start_delay(int cnt)
{
	volatile unsigned int dl;
	while(cnt--)
	{
		for(dl=0; dl<500; dl++);
	}
}


void gtp_reset_guitar(void)
{
#ifdef GT910_ADDR_BABBH
	
	//设定IIC的地址  
	CLR_RST();
	CLR_INT();
 	start_delay(100);    //15MS

	SET_RST();
 	start_delay(50);    //5MS
	
	
#else
	CLR_RST();
	start_delay(500);
	SET_INT();
	start_delay(1000);
	SET_RST();
#endif
}

ErrorStatus gt910_write_reg(uint16_t reg_addr, uint32_t cnt, uint8_t *value)
{
	ErrorStatus err;
	uint32_t i;

	err = SUCCESS;
	iic_start();
	if (iic_write_byte(GT910_IIC_WADDR, 1) == E_Ok)
	{
		if (iic_write_byte((uint8_t)(reg_addr >> 8), 1) == E_Ok)
		{
			if (iic_write_byte((uint8_t)(reg_addr), 1) == E_Ok)
			{
				for(i = 0; i < cnt; i++)
				{
					if (iic_write_byte(value[i], 1) != E_Ok)
					{
						err = ERROR;						
						break;
					}
				}
			}
		}
	}

	iic_stop();
	return err;
}

ErrorStatus gt910_read_reg(uint16_t reg_addr, uint32_t cnt, uint8_t *value)
{
	ErrorStatus err;
	uint32_t i;

	err = ERROR;
	iic_start();
	if (iic_write_byte(GT910_IIC_WADDR, 1) == E_Ok)
	{
		if (iic_write_byte((uint8_t)(reg_addr >> 8), 1) == E_Ok)
		{
			if (iic_write_byte((uint8_t)(reg_addr), 1) == E_Ok)
			{
				iic_stop();
				iic_start();
				if (iic_write_byte(GT910_IIC_RADDR, 1) == E_Ok)
				{
					for(i = 0 ; i < cnt; i++)
					{
						if (i == (cnt - 1))
						{
							value[i] = iic_read_byte(0);
						}
						else
						{
							value[i] = iic_read_byte(1);
						}

					}					
					iic_stop();
					err = SUCCESS;
				}
			}
		}
	}
	iic_stop();
	return (err);	
}

static void gtp_get_chip_type(void)
{
	uint8_t temp[16] = {0};
	gt910_read_reg(GTP_REG_CHIP_TYPE,16,temp);
	if (!memcmp(temp,"GOODIX_GT9",10))
	{
		ts.chip_type = CHIP_TYPE_GT9;
	}
	else
	{
		ts.chip_type = CHIP_TYPE_GT9F;
	}
}

static ErrorStatus gup_check_update_file_fl(st_fw_head* fw_head)
{
	ErrorStatus ret;
	int32_t i = 0;
	int32_t fw_checksum = 0;

	memcpy(fw_head, gtp_default_FW_fl, FW_HEAD_LENGTH);

	fw_head->vid = ((fw_head->vid & 0xFF00) >> 8) + ((fw_head->vid & 0x00FF) << 8);


	//check firmware legality
	fw_checksum = 0;
	for(i = FW_HEAD_LENGTH; i < (FW_HEAD_LENGTH+FW_SECTION_LENGTH*4+FW_DSP_LENGTH); i += 2)
	{
		fw_checksum += (gtp_default_FW_fl[i] << 8) + gtp_default_FW_fl[i+1];
	}
	ret = SUCCESS;
	if (fw_checksum & 0xFFFF)
	{
		ret = ERROR;
	}

	return ret;
}



ErrorStatus gup_hold_ss51_dsp(void)
{
	ErrorStatus ret;
	uint32_t retry = 0;
	uint8_t rd_buf[3];

	while(retry++ < 200)
	{
		// step4:Hold ss51 & dsp
		rd_buf[0] = 0x0C;
		
		ret = gt910_write_reg(_rRW_MISCTL__SWRST_B0_,1,rd_buf);
		if(ret == ERROR)
		{
			continue;
		}

		// step5:Confirm hold
		rd_buf[0] = 0x0c;//rd_buf[0] = 0x00;
		ret = gt910_read_reg(_rRW_MISCTL__SWRST_B0_,1,rd_buf);
		if (ret == ERROR)
		{
			continue;
		}
		if (0x0C == rd_buf[0])
		{
			break;
		}
	}
	if(retry >= 200)
	{
		return ERROR;
	}

	rd_buf[0] = 0x00;
	ret = gt910_write_reg(0x4010,1,rd_buf);
	if (ret == ERROR)
	{
		
		return ERROR;
	}

	rd_buf[0] = 0x00;
	ret = gt910_write_reg(_bRW_MISCTL__TMR0_EN,1,rd_buf);

	if (ret == ERROR)
	{
		return ERROR;
	}

	rd_buf[0] = 0x00;
	ret = gt910_write_reg(_bRW_MISCTL__CACHE_EN,1,rd_buf);

	if (ret == ERROR)
	{
		return ERROR;
	}

	rd_buf[0] = 0x02;
	ret = gt910_write_reg(_rRW_MISCTL__BOOTCTL_B0_,1,rd_buf);

	if (ret == ERROR)
	{
		return ERROR;
	}


	rd_buf[0] = 0x01;
	ret = gt910_write_reg(_bWO_MISCTL__CPU_SWRST_PULSE,1,rd_buf);
	if (ret == ERROR)
	{    
		return ERROR;
	}

	return SUCCESS;
}


ErrorStatus gup_enter_update_mode_fl(void)
{
	ErrorStatus ret;
	//s32 retry = 0;
	uint8_t rd_buf[3];

	//第一步：上电初始化
	gtp_reset_guitar();

	ret = gup_hold_ss51_dsp();
	if (ret == ERROR)
	{
		return ERROR;
	}

	rd_buf[0] = 0x00;
	ret = gt910_write_reg(_rRW_MISCTL__BOOT_CTL_,1,rd_buf);

	if (ret == ERROR)
	{
		return ERROR;
	}

	//set scramble
	//ret = gup_set_ic_msg(_rRW_MISCTL__BOOT_OPT_B0_, 0x00);
	rd_buf[0] = 0x00;
	ret = gt910_write_reg(_rRW_MISCTL__BOOT_OPT_B0_,1,rd_buf);

	if (ret == ERROR)
	{
		return ERROR;
	}

	//enable accessing code
	//ret = gup_set_ic_msg(_bRW_MISCTL__MEM_CD_EN, 0x01);
	rd_buf[0] = 0x01;
	ret = gt910_write_reg(_bRW_MISCTL__MEM_CD_EN,1,rd_buf);


	if (ret == ERROR)
	{
		return ERROR;
	}

	return SUCCESS;
}

ErrorStatus gup_write_fw(uint16_t addr, uint8_t *buf, int32_t len)
{

	ErrorStatus ret;
	uint32_t write_bytes = 0;
	uint32_t retry = 0;
	uint8_t *tx_buf = buf;

	while (len > 0)
	{
		if (len > FL_PACK_SIZE)
		{
			write_bytes = FL_PACK_SIZE;
		}
		else
		{
			write_bytes = len;
		}
		memcpy(i2c_opr_buf, tx_buf, write_bytes);
		for (retry = 0; retry < 5; ++retry)
		{
			ret = gt910_write_reg(addr,write_bytes,i2c_opr_buf);
			if (ret == SUCCESS)
			{
				break;
			}
		}
		if (retry >= 5)
		{
			return ERROR;
		}
		addr += write_bytes;
		len -= write_bytes;
		tx_buf += write_bytes;
	}

	return ret;

}

ErrorStatus gup_burn_fw_proc(uint16_t start_addr, int32_t start_index, int32_t burn_len)
{
	ErrorStatus ret;

	ret = gup_write_fw(start_addr,(uint8_t*)&gtp_default_FW_fl[FW_HEAD_LENGTH + start_index], burn_len);
	if (ret == ERROR)
	{
		return ERROR;
	}
	return SUCCESS;
}

static ErrorStatus gup_check_and_repair(uint16_t start_addr, int32_t start_index, int32_t chk_len)
{
	ErrorStatus ret;
	int32_t cmp_len = 0;
	uint16_t cmp_addr = start_addr;
	int32_t i = 0;
	int32_t chked_times = 0;
	uint8_t chk_fail = 0;
	//uint8_t cmp_addr_reg[2];

	while ((chk_len > 0) && (chked_times < 40))
	{

		if (chk_len >= 256)
		{
			cmp_len = 256;
		}
		else
		{
			cmp_len = chk_len;
		}
		
		ret = gt910_read_reg(cmp_addr,cmp_len,chk_cmp_buf);
		if (ret == ERROR )
		{
			chk_fail = 1;
			break;
		}
		for (i = 0; i < cmp_len; ++i)
		{
			if (chk_cmp_buf[i] != gtp_default_FW_fl[14 + start_index +i])
			{
				chk_fail = 1;
				
				gt910_write_reg(cmp_addr+i,cmp_len-i,&gtp_default_FW_fl[14 + start_index + i]);
			
				break;
			}
		}
		if (chk_fail == 1)
		{
			chk_fail = 0;
			chked_times++;
		}
		else
		{
			cmp_addr += cmp_len;
			start_index += cmp_len;
			chk_len -= cmp_len;
		}
	}
	if (chk_len > 0)
	{
		return ERROR;
	}
	return SUCCESS;
}




static ErrorStatus gup_download_fw_ss51( uint8_t dwn_mode)
{
	uint32_t section = 0;
	ErrorStatus ret;
	uint32_t start_index = 0;
	uint8_t  bank = 0;
	uint16_t burn_addr = 0xC000;

	  
	for (section = 1; section <= 4; section += 2)
	{
		switch (section)
		{
		case 1:
			bank = 0x00;
			burn_addr = (section - 1) * FW_SS51_SECTION_LEN + 0xC000;
			break;
		case 3:
			bank = 0x01;
			burn_addr = (section - 3) * FW_SS51_SECTION_LEN + 0xC000;
			break;
		}
		start_index = (section - 1) * FW_SS51_SECTION_LEN;

		
		ret = gt910_write_reg(_bRW_MISCTL__SRAM_BANK,1,&bank);
		if (GTP_FL_FW_BURN == dwn_mode)
		{
			ret = gup_burn_fw_proc(burn_addr, start_index, 2 * FW_SS51_SECTION_LEN);
			if (ret == ERROR)
			{
				return ERROR;
			}
			ret = gup_check_and_repair(burn_addr, start_index, 2 * FW_SS51_SECTION_LEN);
			if (ret == ERROR)
			{
				return ERROR;
			}
		}
		else if (GTP_FL_ESD_RECOVERY == dwn_mode)// esd recovery mode
		{		
			ret = gup_burn_fw_proc(burn_addr, start_index, 2 * FW_SS51_SECTION_LEN);
			if (ret == ERROR)
			{
				return ERROR;
			}			
		}
		else
		{
			ret = gup_check_and_repair(burn_addr, start_index, 2 * FW_SS51_SECTION_LEN);
			if (ret == ERROR)
			{
				return ERROR;
			}
		}
	}

	return SUCCESS;
}



static ErrorStatus gup_download_fw_dsp(uint8_t dwn_mode)
{
	ErrorStatus ret;
	uint8_t bank = 0x02;

	//step1:select bank2
	ret = gt910_write_reg(_bRW_MISCTL__SRAM_BANK,1,&bank);
	if (ret == ERROR)
	{
		return ERROR;
	}

	if (GTP_FL_FW_BURN == dwn_mode)
	{
		if (ret == ERROR)
		{
			return ERROR;
		}
		ret = gup_burn_fw_proc(0xC000, 2 * FW_DOWNLOAD_LENGTH, FW_DSP_LENGTH); // write the second ban
		if (ERROR == ret)
		{
			return ERROR;
		}
		
		ret = gup_check_and_repair( 0xC000, 2 * FW_DOWNLOAD_LENGTH, FW_DSP_LENGTH);
		if (ERROR == ret)
		{
			return ERROR;
		}
	}
	else if (GTP_FL_ESD_RECOVERY == dwn_mode)
	{
		{
			ret = gup_burn_fw_proc(0xC000, 2 * FW_DOWNLOAD_LENGTH, FW_DSP_LENGTH);
			if (ERROR == ret)
			{
				return ERROR;
			}
		}
	}
	else
	{
		ret = gup_check_and_repair(0xC000, 2 * FW_DOWNLOAD_LENGTH, FW_DSP_LENGTH);
		if (ERROR == ret)
		{
			return ERROR;
		}
	}
	return SUCCESS;
}





ErrorStatus gup_fw_download_proc( uint8_t dwn_mode)
{
	ErrorStatus ret;
	uint8_t  retry = 0;
	st_fw_head fw_head;
	

	ret = gup_check_update_file_fl(&fw_head);
	//    show_len = 10;

	if (ERROR == ret)
	{
		goto file_fail;
	}

	if (!memcmp(fw_head.pid, "950", 3))
	{
		ts.is_950 = 1;
	}
	else
	{
		ts.is_950 = 0;
	}


#if GTP_ESD_PROTECT
	if (NULL != dir)
	{
		gtp_esd_switch(SWITCH_OFF);
	}
#endif

	ret = gup_enter_update_mode_fl();   
	//show_len = 20;    
	if (ERROR == ret)    
	{ 
		goto download_fail;    
	}

	while (retry++ < 5)
	{
		ret = gup_download_fw_ss51(dwn_mode);
		
		if (ERROR == ret)
		{
			continue;
		}

		ret = gup_download_fw_dsp(dwn_mode);
		// show_len = 80;
		if (ERROR == ret)
		{
			continue;
		}
		break;
	}

	if (retry >= 5)
	{
		goto download_fail;
	}

	return SUCCESS;

download_fail:

	//GT9XX_Resume();
file_fail:
	//show_len = 200;

	//ts->enter_udpate = 0;
	return ERROR;
}
void gt910_int_init(void)
{
	

	//中断引脚配置
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;

	GPIO_InitTypeDef GPIO_InitStructure;
	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);
	
		//Configure PG7 pin: TP_INT pin 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 	//上拉输入
	GPIO_Init(GPIOG,&GPIO_InitStructure);
	/* Connect EXTI Line3 to PE3 pin */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOG ,GPIO_PinSource7);
	/* Enable GPIOs clock */
	
	 EXTI_InitStructure.EXTI_Line = EXTI_Line7;
   EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
   EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);
  EXTI_GenerateSWInterrupt(EXTI_Line7);


	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	   // 设置中断组 为2 
	/* Enable and set EXTI Line0 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void gt910_int_sync(uint32_t dl_ms)
{
	CLR_INT();
	start_delay(dl_ms * 100);
	gt910_int_init();
}

static ErrorStatus gtp_fw_startup()
{
	//uint8_t opr_buf_reg8041[2] = {0x80, 0x41};
	//uint8_t opr_buf_reg4180[2] = {0x41, 0x80};
	uint8_t opr_buf[4];
	ErrorStatus ret;
	
 	opr_buf[0] = 0xAA;
 	ret = gt910_write_reg(0x8040, 1,opr_buf);// i2c_write_bytes(client, 0x8041, opr_buf, 1);
	
	
	opr_buf[0] = 0xAA;
	ret = gt910_write_reg(0x8041, 1 ,opr_buf);
	
	if (ERROR == ret)
	{
		return ERROR;
	}

	//release SS51 & DSP
// 	opr_buf = 0x00;
// 	ret = GT9XX_I2C_Write(opr_buf_reg4180, &opr_buf, 1);
	opr_buf[0] = 0x00;	
	ret = gt910_write_reg(0x4180, 1 ,opr_buf);
	//ret = i2c_write_bytes(client, 0x4180, opr_buf, 1);
	if (ERROR == ret)
	{
		return ERROR;
	}

	//int sync
	gt910_int_sync(25);  

	//check fw run status

	ret = gt910_read_reg(0x8041,  1 ,opr_buf);

	if (ERROR == ret)
	{
		return ERROR;
	}
	if(0xAA == opr_buf[0])
	{
		return ERROR;
	}
	else
	{
		opr_buf[0] = 0xAA;
		gt910_write_reg(0x8040, 1 ,opr_buf);
		
		opr_buf[0] = 0xAA;
		gt910_write_reg(0x8041, 1 ,opr_buf);
		return SUCCESS;
	}
}



ErrorStatus gtp_gt9xxf_init()
{
	ErrorStatus ret;
	

	ret = gup_fw_download_proc( GTP_FL_FW_BURN);

	if (ERROR == ret)
	{
		return ERROR;
	}

	ret = gtp_fw_startup();
	if (ERROR == ret)
	{
		return ERROR;
	}

	return SUCCESS;
}
ErrorStatus gt910_read_version(uint16_t* version)
{
	ErrorStatus ret;
	u8 buf[8] = {0};

	ret = gt910_read_reg(GTP_REG_VERSION,6,buf);
	if (ret == ERROR)
	{

		return ret;
	}

	if (version)
	{
		*version = (buf[5] << 8) | buf[4];
	}
	
	return SUCCESS;
}


ErrorStatus gtp_init_panel(void)
{
	ErrorStatus ret;
	uint8_t check_sum = 0;
	uint8_t opr_buf[16] = {0};
	//uint8_t sensor_id = 0; 
	int32_t i;

	//uint8_t cfg_info_group1[] = CTP_CFG_GROUP1;

	start_delay(50*100);	//delay 50ms

	ret = gt910_read_reg(GTP_REG_SENSOR_ID,1,opr_buf);
	if (ret == ERROR)
	{
		return ERROR;
	}

	if (opr_buf[0] > 0x06)
	{
		return ERROR;
	}


	memset(cfg,0,240);

	memcpy(cfg,CTP_CFG_GROUP, 228);


	check_sum = 0;
	for (i = 0; i < 228; i++)
	{
		check_sum += cfg[i];
	}
	cfg[228] = (~check_sum) + 1;

	return SUCCESS;

}

static ErrorStatus gtp_send_cfg(void)
{
	ErrorStatus ret;


	int32_t retry = 0;
	
	for (retry = 0; retry < 5; retry++)
	{
		//ret = gtp_i2c_wr ite(client, config , 240GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH);
		ret = gt910_write_reg(GTP_REG_CONFIG_DATA,228,CTP_CFG_GROUP);//cfg
		if (ret == SUCCESS)
		{
			break;
		}
	}

	return ret;
}


#define BAK_REF_LENGTH	(18*(10-2)+2)*2

  uint8_t p_bak_ref[BAK_REF_LENGTH] = {0};
	
ErrorStatus gtp_bak_ref_proc(uint8_t mode)
{
	ErrorStatus ret;
	uint16_t i =0;
	uint32_t ref_seg_len = 0;
	
	
	

	ref_seg_len = BAK_REF_LENGTH;
	ts.bak_ref_len = BAK_REF_LENGTH;

		
	for (i=0; i<ts.bak_ref_len; i++)
	{
		p_bak_ref[i] = 0;

	}
 	p_bak_ref[ref_seg_len - 1] = 0x01; 
	
	ret = gt910_write_reg(GTP_REG_BAK_REF,ref_seg_len,p_bak_ref);//

	if (ret == ERROR)
	{
		return ERROR;
	}
	return SUCCESS;


}

static ErrorStatus gtp_esd_recovery(void)
{
	uint32_t retry = 0;
	ErrorStatus ret;
	

	gtp_reset_guitar();       // reset & select I2C addr
	for (retry = 0; retry < 5; ++retry)
	{
		ret = gup_fw_download_proc( GTP_FL_ESD_RECOVERY); 
		if (ERROR == ret)
		{
			continue;
		}
		ret = gtp_fw_startup();
		if (ERROR == ret)
		{
			continue;
		}
		break;
	}

	if (retry >= 5)
	{
		return ERROR;
	}
	return SUCCESS;
}

void gtp_recovery_reset(void)
{
#if GTP_ESD_PROTECT
	gtp_esd_switch(client, SWITCH_OFF);
#endif
	//GTP_DEBUG_FUNC();

	gtp_esd_recovery(); 

#if GTP_ESD_PROTECT
	gtp_esd_switch(client, SWITCH_ON);
#endif
}


static ErrorStatus gtp_main_clk_proc()
{


	//uint32_t ret = 0;
	uint32_t i = 0;
	uint32_t clk_chksum = 0;
	uint8_t p_main_clk[6] = {0};
	
	for (i = 0; i < 5; ++i)
	{
		p_main_clk[i] = 80;
		clk_chksum += p_main_clk[i];
	}
	p_main_clk[5] = 0 - clk_chksum;

	//ret = GT9XX_I2C_Write(p_main_clk_reg, p_main_clk, 6);
	if(gt910_write_reg(GTP_REG_MAIN_CLK,6,p_main_clk) == ERROR) 	
 	{
 		
 		return ERROR;
 	}
	return SUCCESS;

}



/******************************************************************************/
// Description: GT9XX_Read
// Dependence:  GETTING the data about touchpanel. the up-layer calls the 
//              function to get the data.
// Note:
/******************************************************************************/
#define GTP_RQST_CONFIG                 0x01
#define GTP_RQST_BAK_REF                0x02
#define GTP_RQST_RESET                  0x03
#define GTP_RQST_MAIN_CLOCK             0x04
#define GTP_RQST_RESPONDED              0x00
#define GTP_RQST_IDLE                   0xFF
extern __IO uint16_t point_x,point_y;
extern uint8_t Touch_success;

void gt910_isr(void)
{
	
 	uint8_t  point_data[1 + 8 * GTP_MAX_TOUCH + 1];
 	uint8_t  touch_num = 0;
 	uint8_t  finger = 0;

	ErrorStatus ret;

#if 1//GTP_COMPATIBLE_MODE
	//uint8 rqst_buf_reg[] = {0x80, 0x43};
	uint8_t rqst_buf[16];  // for GT9XXF
#endif



	ret = gt910_read_reg(GTP_READ_COOR_ADDR,10,point_data);

	if (ret == ERROR)
	{
		goto exit_work_func;
	}

	finger = point_data[0];    
#if 1//GTP_COMPATIBLE_MODE
	// GT9XXF
	if ((finger == 0x00) && (CHIP_TYPE_GT9F == ts.chip_type))     // request arrived
	{
		rqst_buf[0] = 0x00;
		ret = gt910_read_reg(0x8043,1,rqst_buf);
		if (ret == ERROR)
		{
			goto exit_work_func;
		} 

		switch (rqst_buf[0] & 0x0F)
		{
		case GTP_RQST_CONFIG:     //配置请求

			ret = gtp_send_cfg();
			if (ret == ERROR)
			{
				
			}
			else
			{
				rqst_buf[0] = GTP_RQST_RESPONDED;
				
				gt910_write_reg(0x8043,1,rqst_buf);
				
			}
			break;

		case GTP_RQST_BAK_REF:     //备份基准
	
			ret = gtp_bak_ref_proc(GTP_BAK_REF_SEND);  //bernard
			if (SUCCESS == ret)
			{
				rqst_buf[0] = GTP_RQST_RESPONDED;
				
				gt910_write_reg(0x8043,1,rqst_buf);
			}
			
			break;

		case GTP_RQST_RESET:             //ESD复位
			gtp_recovery_reset();                     //bernard
			break;

		case GTP_RQST_MAIN_CLOCK:            //主频配置
			
			ts.rqst_processing = 1;
			ret = gtp_main_clk_proc();             //bernard
			if (ERROR == ret)
			{
				
			}
			else
			{
				rqst_buf[0] = GTP_RQST_RESPONDED;
				//GT9XX_I2C_Write(rqst_buf_reg, &rqst_buf, 1);
				gt910_write_reg(0x8043,1,rqst_buf);

			}
			Touch_success = 1;//可以去掉
			break;

		case GTP_RQST_IDLE:
		default:
			break;
		}
	}
#endif


	if((finger & 0x80) == 0)
	{
		goto exit_work_func;
	}

	
	touch_num = finger & 0x0f;

	if (touch_num > GTP_MAX_TOUCH) //触摸点数大于5
	{
		goto exit_work_func;
	}

	if (touch_num >= 1)             //触摸点数大于1
	{
		ret = gt910_read_reg(GTP_READ_COOR_ADDR,8*(touch_num ),point_data);//&10
		switch(touch_num)
		{
			case 5:
			{
	      //lcd_draw_pixel(((point_data[3]<<8) | point_data[2]),((point_data[5]<<8) | point_data[4]),color_red);
      }
			case 4:
			{
// 			  lcd_draw_pixel(((point_data[5+24]<<8) | point_data[4+24]),480 - ((point_data[3+24]<<8) | point_data[2+24]),color_black);
// 	      lcd_draw_pixel(((point_data[5+24]<<8) | point_data[4+24])-1,480 - ((point_data[3+24]<<8) | point_data[2+24]),color_black);
// 	      lcd_draw_pixel(((point_data[5+24]<<8) | point_data[4+24])+1,480 - ((point_data[3+24]<<8) | point_data[2+24]),color_black);
// 	      lcd_draw_pixel(((point_data[5+24]<<8) | point_data[4+24]),480 - ((point_data[3+24]<<8) | point_data[2+24])+1,color_black);
// 	      lcd_draw_pixel(((point_data[5+24]<<8) | point_data[4+24]),480 - ((point_data[3+24]<<8) | point_data[2+24])-1,color_black);
			
			}
			case 3:
			{
// 	      lcd_draw_pixel(((point_data[5+16]<<8) | point_data[4+16]),480 - ((point_data[3+16]<<8) | point_data[2+16]),color_black);
// 	      lcd_draw_pixel(((point_data[5+16]<<8) | point_data[4+16])-1,480 - ((point_data[3+16]<<8) | point_data[2+16]),color_black);
// 	      lcd_draw_pixel(((point_data[5+16]<<8) | point_data[4+16])+1,480 - ((point_data[3+16]<<8) | point_data[2+16]),color_black);
// 	      lcd_draw_pixel(((point_data[5+16]<<8) | point_data[4+16]),480 - ((point_data[3+16]<<8) | point_data[2+16])+1,color_black);
// 	      lcd_draw_pixel(((point_data[5+16]<<8) | point_data[4+16]),480 - ((point_data[3+16]<<8) | point_data[2+16])-1,color_black);
			
      }
			case 2:
			{
// 	      lcd_draw_pixel(((point_data[5+8]<<8) | point_data[4+8]),480 - ((point_data[3+8]<<8) | point_data[2+8]),color_green);
// 	      lcd_draw_pixel(((point_data[5+8]<<8) | point_data[4+8])-1,480 - ((point_data[3+8]<<8) | point_data[2+8]),color_green);
// 	      lcd_draw_pixel(((point_data[5+8]<<8) | point_data[4+8])+1,480 - ((point_data[3+8]<<8) | point_data[2+8]),color_green);
// 	      lcd_draw_pixel(((point_data[5+8]<<8) | point_data[4+8]),480 - ((point_data[3+8]<<8) | point_data[2+8])+1,color_green);
// 	      lcd_draw_pixel(((point_data[5+8]<<8) | point_data[4+8]),480 - ((point_data[3+8]<<8) | point_data[2+8])-1,color_green);
				
      }
			case 1:
			{
// 	      lcd_draw_pixel(((point_data[5]<<8) | point_data[4]),480 - ((point_data[3]<<8) | point_data[2]),color_red);
// 	      lcd_draw_pixel(((point_data[5]<<8) | point_data[4])-1,480 - ((point_data[3]<<8) | point_data[2]),color_red);
// 	      lcd_draw_pixel(((point_data[5]<<8) | point_data[4])+1,480 - ((point_data[3]<<8) | point_data[2]),color_red);
// 	      lcd_draw_pixel(((point_data[5]<<8) | point_data[4]),480 - ((point_data[3]<<8) | point_data[2])+1,color_red);
// 	      lcd_draw_pixel(((point_data[5]<<8) | point_data[4]),480 - ((point_data[3]<<8) | point_data[2])-1,color_red);
				point_x = (point_data[5]<<8) | point_data[4];
				point_y = (point_data[3]<<8) | point_data[2];
				

				break;				
      }
			default:break;
    }	
	}// + 10



  else                       //如果没有有效触摸，就返回0xffff
		{
      point_x = 0xffff;
		  point_y = 0xffff;

    }
exit_work_func:
	//pre_touch = touch_num;
	//ret = GT9XX_I2C_Write(end_cmd_reg, &end_cmd_data, 1);
	memset(rqst_buf,0,16);
	ret = gt910_write_reg(GTP_READ_COOR_ADDR,1,rqst_buf);


	//return ret;

}





/******************************************************************************/
// Description: GT9XX_Initial
// Dependence: 
// Note:   GPIO_PROD_TP_INT_ID
/******************************************************************************/

ErrorStatus GT9XX_Initial(void)
{
	uint32_t reset_count;
	ErrorStatus ret;
	
	iic_init();

	reset_proc:	
  
	gtp_reset_guitar();   //第一步：上电初始化 设定地址
	

	gtp_get_chip_type();//读取IC类型

	if (CHIP_TYPE_GT9F == ts.chip_type)
	{

		ret = gtp_gt9xxf_init();
	}	
	

	if (gt910_read_version(&version_info) == SUCCESS)
	{		
	}


	if (gtp_init_panel() == SUCCESS)
	{
		
	}
	else
	{
		if (reset_count < 3)
		{
			reset_count++;
			goto reset_proc;
		}
	}

	return SUCCESS;
}

