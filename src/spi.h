
#ifndef __USR_SPI_H__
#define __USR_SPI_H__

typedef enum{
	SPI1_INDEX = 0    // SPI0 PA4 (CS),5,6,7   ·ÇÓ³ÉäÄ£Ê½
	,SPI2_INDEX = 1     //
	,SPI3_INDEX = 2    //
}spi_index_t;


void usr_spi_init(spi_index_t index,uint8_t isremap);

uint8_t usr_spi_write_read(spi_index_t index,uint8_t dat);


#endif
