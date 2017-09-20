#include <bcm2835.h>
#include <stdio.h>
 
 
int main(int arc, char **argv) 
{
 
  char out_ch0[] = { 0b00000110, 0b00000000, 0b00000000 };
  char ch0_data[] = { 0x00, 0x00, 0x00 };
  char out_ch1[] = { 0b00000110, 0b01000000, 0b00000000 };
  char ch1_data[] = { 0x00, 0x00, 0x00 };
  char out_ch2[] = { 0b00000110, 0b10000000, 0b00000000 };
  char ch2_data[] = { 0x00, 0x00, 0x00 }; 
  
  if(!bcm2835_init()) return 1;
         
  bcm2835_spi_begin();
  bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
  bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);         
  bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_128); 
  bcm2835_spi_chipSelect(BCM2835_SPI_CS1);                 
  bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS1, LOW); 
     
  bcm2835_spi_transfernb(out_ch0, ch0_data, 3);
  printf("CH0:    %02X %02X %02X\n", ch0_data[0], ch0_data[1], ch0_data[2]);
  bcm2835_spi_transfernb(out_ch1, ch1_data, 3);
  printf("CH1:    %02X %02X %02X\n", ch1_data[0], ch1_data[1], ch1_data[2]);
  bcm2835_spi_transfernb(out_ch2, ch2_data, 3);
  printf("CH2:    %02X %02X %02X\n", ch2_data[0], ch2_data[1], ch2_data[2]);
     
  bcm2835_spi_end();
  bcm2835_close();
}