/* ***********************************************************************/


#include "includes.h"

//FUNCTION POINTERS FOR INTERNAL LIBRARY USE
//SET THESE TO YOUR APPLICATION FUNCTIONS

static uint16_t w25qxx_spi_send_get(uint16_t dat)
{
	return usr_spi_write_read(SPI1_INDEX,dat);
}


//w25q spi �ӿڳ�ʼ��
void w25qxx_spi_init(void)
{
	usr_spi_init(SPI1_INDEX,0);   //����2�Ƿ�ʹ��remap������0��ʾ��ʹ��

	//ʱ��ʹ��
	rcu_periph_clock_enable(RCU_GPIOA);	
	gpio_bit_set(GPIOA, GPIO_PIN_4);   //cs�ǵ͵�ƽ��Ч���ߵ�ƽ��ѡ��	
	gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_4);	
	
}


static void w25qxx_spi_cs_low(void)
{
	gpio_bit_set(GPIOA, GPIO_PIN_4);
}


static void w25qxx_spi_cs_high(void)
{
	gpio_bit_reset(GPIOA, GPIO_PIN_4);
}

static uint16_t (*spi_send_get_function)(uint16_t) = w25qxx_spi_send_get;

static void (*spi_cs_low_function)(void) = w25qxx_spi_cs_low;
static void (*spi_cs_high_function)(void) = w25qxx_spi_cs_high;





void FLASH_W25QXX_Enable_Reset(void)
{
	//RESET W25QXX DEVICE
	
	//WAIT FOR THE FLASH TO BE FREE
	while(FLASH_W25QXX_Get_Busy_Status() != 0){};
	
	//REASSERT CS LINE
	(*spi_cs_low_function)();
		
	(*spi_send_get_function)(FLASH_W25QXX_COMMAND_ENABLE_RESET);
}

void FLASH_W25QXX_Do_Reset(void)
{
	//RESET W25QXX DEVICE
	
	//WAIT FOR THE FLASH TO BE FREE
	while(FLASH_W25QXX_Get_Busy_Status() != 0){};
	
	//REASSERT CS LINE
	(*spi_cs_low_function)();
		
	(*spi_send_get_function)(FLASH_W25QXX_COMMAND_RESET);
}

uint8_t FLASH_W25QXX_Get_Status_Register(uint8_t register_num)
{
	//RETURN THE VALUE OF THE SPECIFIED STATUS REGISTER
	
	if(register_num == 1)
	{
		(*spi_send_get_function)(FLASH_W25QXX_COMMAND_READ_STATUS_REG_1);
		//DEASSERT CS LINE
		(*spi_cs_high_function)();
		return (uint8_t)(*spi_send_get_function)(FLASH_W25QXX_COMMAND_NOP);
	}
	else if(register_num == 2)
	{
		(*spi_send_get_function)(FLASH_W25QXX_COMMAND_READ_STATUS_REG_2);
		//DEASSERT CS LINE
		(*spi_cs_high_function)();
		return (uint8_t)(*spi_send_get_function)(FLASH_W25QXX_COMMAND_NOP);
	}
	(*spi_cs_high_function)();
	return 0;
}

uint8_t FLASH_W25QXX_Get_Busy_Status(void)
{
	//RETURN THE BUSY STATUS OF THE DEVICE
	
	uint8_t val = FLASH_W25QXX_Get_Status_Register(1);
	return (val & 0x01);
}

void FLASH_W25QXX_Write_Enable(void)
{
	//SET THE WRITE ENABLE FLAG IN DEVICE
	
	//WAIT FOR THE FLASH TO BE FREE
	while(FLASH_W25QXX_Get_Busy_Status() != 0){};
	
	//REASSERT CS LINE
	(*spi_cs_low_function)();
		
	(*spi_send_get_function)(FLASH_W25QXX_COMMAND_WRITE_ENABLE);
}

void FLASH_W25QXX_Write_Disable(void)
{
	//SET THE WRITE DISABLE FLAG IN DEVICE
	
	//WAIT FOR THE FLASH TO BE FREE
	while(FLASH_W25QXX_Get_Busy_Status() != 0){};
	
	//REASSERT CS LINE
	(*spi_cs_low_function)();
		
	(*spi_send_get_function)(FLASH_W25QXX_COMMAND_WRITE_DISABLE);
}

uint16_t FLASH_W25QXX_Get_Manufacturer_Id(void)
{
	//RETURN MANUFACTURER ID(1 BYTE) & DEVICE ID(1 BYTE)
	
	//WAIT FOR THE FLASH TO BE FREE
	while(FLASH_W25QXX_Get_Busy_Status() != 0){};
	
	//REASSERT CS LINE
	(*spi_cs_low_function)();
		
	uint16_t temp;
	
	(*spi_send_get_function)(FLASH_W25QXX_COMMAND_MANUFACTURER_ID);
	(*spi_send_get_function)(FLASH_W25QXX_COMMAND_NOP);
	(*spi_send_get_function)(FLASH_W25QXX_COMMAND_NOP);
	(*spi_send_get_function)(FLASH_W25QXX_COMMAND_NOP);
	temp = (*spi_send_get_function)(FLASH_W25QXX_COMMAND_NOP);
	temp = temp << 8;
	temp |= (*spi_send_get_function)(FLASH_W25QXX_COMMAND_NOP);
	
	return temp;
}


void FLASH_W25QXX_Get_Unique_Id(uint8_t* ret_ptr)
{
	//RETURN 64 BIT DEVICE UNIQUE ID
	
	//WAIT FOR THE FLASH TO BE FREE
	while(FLASH_W25QXX_Get_Busy_Status() != 0){};
	
	//REASSERT CS LINE
	(*spi_cs_low_function)();
	
	(*spi_send_get_function)(FLASH_W25QXX_COMMAND_UNIQUE_ID);
	(*spi_send_get_function)(FLASH_W25QXX_COMMAND_NOP);
	(*spi_send_get_function)(FLASH_W25QXX_COMMAND_NOP);
	(*spi_send_get_function)(FLASH_W25QXX_COMMAND_NOP);
	(*spi_send_get_function)(FLASH_W25QXX_COMMAND_NOP);
	
	ret_ptr[0] = (uint8_t)(*spi_send_get_function)(FLASH_W25QXX_COMMAND_NOP);
	ret_ptr[1] = (uint8_t)(*spi_send_get_function)(FLASH_W25QXX_COMMAND_NOP);
	ret_ptr[2] = (uint8_t)(*spi_send_get_function)(FLASH_W25QXX_COMMAND_NOP);
	ret_ptr[3] = (uint8_t)(*spi_send_get_function)(FLASH_W25QXX_COMMAND_NOP);
	ret_ptr[4] = (uint8_t)(*spi_send_get_function)(FLASH_W25QXX_COMMAND_NOP);
	ret_ptr[5] = (uint8_t)(*spi_send_get_function)(FLASH_W25QXX_COMMAND_NOP);
	ret_ptr[6] = (uint8_t)(*spi_send_get_function)(FLASH_W25QXX_COMMAND_NOP);
	ret_ptr[7] = (uint8_t)(*spi_send_get_function)(FLASH_W25QXX_COMMAND_NOP);
}


void FLASH_W25QXX_Get_Jedec_Id(uint8_t* ret_ptr)
{
	//RETURN DEVICE JEDEC ID
	
	//WAIT FOR THE FLASH TO BE FREE
	while(FLASH_W25QXX_Get_Busy_Status() != 0){};
	
	//REASSERT CS LINE
	(*spi_cs_low_function)();
	
	(*spi_send_get_function)(FLASH_W25QXX_COMMAND_JEDEC_ID);
	ret_ptr[0] = (*spi_send_get_function)(FLASH_W25QXX_COMMAND_NOP);
	ret_ptr[1] = (*spi_send_get_function)(FLASH_W25QXX_COMMAND_NOP);
	ret_ptr[2] = (*spi_send_get_function)(FLASH_W25QXX_COMMAND_NOP);	
}


uint8_t FLASH_W25QXX_Read_Data_Byte(uint32_t add)
{
	//READ AND RETURN BYTE OF DATA READ FROM THE SPECIFIED ADDRESS
	//AUTO INCREAMENTS THE READ ADDRESS FOR THE NEXT READ OPERATION
	
	//WAIT FOR THE FLASH TO BE FREE
	while(FLASH_W25QXX_Get_Busy_Status() != 0){};
		
	//REASSERT CS LINE
	(*spi_cs_low_function)();
		
	uint8_t add_hi = (uint8_t)((add & 0x00FF0000) >> 16);
	uint8_t add_mid = (uint8_t)((add & 0x0000FF00) >> 8);
	uint8_t add_lo = (uint8_t)(add);
	(*spi_send_get_function)(FLASH_W25QXX_COMMAND_READ_DATA);
	(*spi_send_get_function)(add_hi);
	(*spi_send_get_function)(add_mid);
	(*spi_send_get_function)(add_lo);
	return (uint8_t)(*spi_send_get_function)(FLASH_W25QXX_COMMAND_NOP);
}


void FLASH_W25QXX_Read_Data_Block(uint32_t add, uint8_t* data_ptr, uint32_t read_len)
{
	//READ A BLOCK OF DATA FROM THE SPECIFIED ADDRESS AND STORE IN THE BLOCK POINTER
	
	//WAIT FOR THE FLASH TO BE FREE
	while(FLASH_W25QXX_Get_Busy_Status() != 0){};
		
	//REASSERT CS LINE
	(*spi_cs_low_function)();
	
	uint8_t add_hi = (uint8_t)((add & 0x00FF0000) >> 16);
	uint8_t add_mid = (uint8_t)((add & 0x0000FF00) >> 8);
	uint8_t add_lo = (uint8_t)(add);
	(*spi_send_get_function)(FLASH_W25QXX_COMMAND_READ_DATA);
	(*spi_send_get_function)(add_hi);
	(*spi_send_get_function)(add_mid);
	(*spi_send_get_function)(add_lo);
	
	for(uint32_t i=0; i<read_len; i++)
	{
		data_ptr[i] = (uint8_t)(*spi_send_get_function)(FLASH_W25QXX_COMMAND_NOP);
	}
}

void FLASH_W25QXX_Write_Status_Registers(uint8_t val_stat_reg_1, uint8_t val_stat_reg_2)
{
	//WRITE THE SPECIFIED VALUES TO STATUS REGISTERS 1 AND 2
	
	//WAIT FOR THE FLASH TO BE FREE
	while(FLASH_W25QXX_Get_Busy_Status() != 0){};
		
	//REASSERT CS LINE
	(*spi_cs_low_function)();
	
	(*spi_send_get_function)(FLASH_W25QXX_COMMAND_WRITE_STATUS_REGS);
	(*spi_send_get_function)(val_stat_reg_1);
	(*spi_send_get_function)(val_stat_reg_2);
}


void FLASH_W25QXX_Sector_Erase_4kb(uint32_t add_sector)
{
	//ERASE THE SPECIFIED SECTOR (4KB)
	
	//WAIT FOR THE FLASH TO BE FREE
	while(FLASH_W25QXX_Get_Busy_Status() != 0){};
		
	//REASSERT CS LINE
	(*spi_cs_low_function)();
	
	uint8_t add_hi = (uint8_t)((add_sector & 0x00FF0000) >> 16);
	uint8_t add_mid = (uint8_t)((add_sector & 0x0000FF00) >> 8);
	uint8_t add_lo = (uint8_t)(add_sector);
	
	(*spi_send_get_function)(FLASH_W25QXX_COMMAND_SECTOR_ERASE_4KB);
	(*spi_send_get_function)(add_hi);
	(*spi_send_get_function)(add_mid);
	(*spi_send_get_function)(add_lo);
}

void FLASH_W25QXX_Block_Erase_32Kb(uint32_t add_block)
{
	//ERASE THE SPECFIFIED BLOCK (32KB)
	
	//WAIT FOR THE FLASH TO BE FREE
	while(FLASH_W25QXX_Get_Busy_Status() != 0){};
		
	//REASSERT CS LINE
	(*spi_cs_low_function)();
	
	uint8_t add_hi = (uint8_t)((add_block & 0x00FF0000) >> 16);
	uint8_t add_mid = (uint8_t)((add_block & 0x0000FF00) >> 8);
	uint8_t add_lo = (uint8_t)(add_block);
	
	(*spi_send_get_function)(FLASH_W25QXX_COMMAND_BLOCK_ERASE_32KB);
	(*spi_send_get_function)(add_hi);
	(*spi_send_get_function)(add_mid);
	(*spi_send_get_function)(add_lo);
}

void FLASH_W25QXX_Block_Erase_64Kb(uint32_t add_block)
{
	//ERASE THE SPECIFIED BLOCK (64KB)
	
	//WAIT FOR THE FLASH TO BE FREE
	while(FLASH_W25QXX_Get_Busy_Status() != 0){};
		
	//REASSERT CS LINE
	(*spi_cs_low_function)();
	
	uint8_t add_hi = (uint8_t)((add_block & 0x00FF0000) >> 16);
	uint8_t add_mid = (uint8_t)((add_block & 0x0000FF00) >> 8);
	uint8_t add_lo = (uint8_t)(add_block);
	
	(*spi_send_get_function)(FLASH_W25QXX_COMMAND_BLOCK_ERASE_64KB);
	(*spi_send_get_function)(add_hi);
	(*spi_send_get_function)(add_mid);
	(*spi_send_get_function)(add_lo);
}

void FLASH_W25QXX_Chip_Erase(void)
{
	//ERASE THE COMPLETE CHIP
	
	//WAIT FOR THE FLASH TO BE FREE
	while(FLASH_W25QXX_Get_Busy_Status() != 0){};
		
	//REASSERT CS LINE
	(*spi_cs_low_function)();
	
	(*spi_send_get_function)(FLASH_W25QXX_COMMAND_CHIP_ERASE);
}

