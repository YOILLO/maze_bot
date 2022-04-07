#include "display.h"
//#include "pins.h"
/*
Просто инициализация нужных пинов
*/
Display::Display()
{
    GPIO_InitTypeDef port;
    SPI_InitTypeDef spi;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
 
    SPI_StructInit(&spi);
    spi.SPI_Direction = SPI_Direction_1Line_Tx;
    spi.SPI_Mode = SPI_Mode_Master;
    spi.SPI_DataSize = SPI_DataSize_8b;
    spi.SPI_CPOL = SPI_CPOL_Low;
    spi.SPI_CPHA = SPI_CPHA_1Edge;
    spi.SPI_NSS = SPI_NSS_Soft;
    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
    spi.SPI_FirstBit = SPI_FirstBit_MSB;
    spi.SPI_CRCPolynomial = 7;
    SPI_Init(SPI3, &spi);
    
    SPI_Cmd(SPI3,ENABLE);
    
    SPI_NSSInternalSoftwareConfig(SPI3, SPI_NSSInternalSoft_Set);
    
    GPIO_StructInit(&port);
    port.GPIO_OType = GPIO_OType_PP;
    port.GPIO_PuPd = GPIO_PuPd_NOPULL;
    port.GPIO_Mode = GPIO_Mode_AF;
    port.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_10;
    port.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &port);
    
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12,GPIO_AF_SPI3);
    //GPIO_PinAFConfig(GPIOC, GPIO_PinSource11,GPIO_AF_SPI3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10,GPIO_AF_SPI3);
    
    port.GPIO_Mode = GPIO_Mode_OUT;
    port.GPIO_Pin = OLED_RST_Pin;
    port.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(OLED_RST_GPIO_Port, &port);
    
    port.GPIO_Mode = GPIO_Mode_OUT;
    port.GPIO_Pin = OLED_DC_Pin;
    port.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(OLED_DC_GPIO_Port, &port);
    
    /*port.GPIO_Mode = GPIO_Mode_OUT;
    port.GPIO_Pin = OLED_CS_Pin;
    port.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(OLED_CS_GPIO_Port, &port);*/
}
