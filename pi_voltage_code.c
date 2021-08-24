#include "bcm2835.h"
#include <stdio.h>
#include "ADS114S08IRHBR.h"
#include <string.h>
#include<time.h>
#include "mine.h"
#include <unistd.h>
#include <math.h>

#define VREF 5.06
#define RES VREF/32767.00
#define pin 5 //drdy pi gpio 4
#define start 22
#define reset 26

int spi_init(void);
int ADC114S08_RegWrite(uint8_t regnum, uint8_t data , uint8_t length);
int ADC114S08_RegRead(uint8_t regnum, uint8_t data, uint8_t length );
void adc_start();
int ADC114S08_RegRead(uint8_t regnum, uint8_t data, uint8_t length )
{
        char tx_buff_read[length];
        char rx_buff_read[length];
        memset(tx_buff_read, 0, length*sizeof(tx_buff_read[0]));
        memset(rx_buff_read, 0, length*sizeof(rx_buff_read[0]));
        tx_buff_read[0] = 0x20 + (regnum & 0x1f);
        tx_buff_read[1] = length -3;
        bcm2835_spi_transfernb(tx_buff_read,rx_buff_read,3);
        printf("rxd byte = %d \n", rx_buff_read[length-1] & 0x0f);
        return rx_buff_read[length-1];
}
int ADC114S08_RegWrite(uint8_t regnum, uint8_t data , uint8_t length)
{
        char tx_buff[length];
        char rx_buff[length];
        uint8_t rxd_Data = 0;
        memset(tx_buff, 0, length*sizeof(tx_buff[0]));
        memset(rx_buff, 0, length*sizeof(rx_buff[0]));
        tx_buff[0] = 0x40 + (regnum & 0x1f);
        tx_buff[1] = 0;
        tx_buff[2] = data;
        bcm2835_spi_transfernb(tx_buff,rx_buff,3);
 rxd_Data = ADC114S08_RegRead(regnum , data , length);
        if(rxd_Data == data)
        {
                return 0;
        }
        else
                return -1;
}
int spi_init(){

  if (!bcm2835_init())return 1;

    usleep(1000);
    bcm2835_spi_begin();
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE1);
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_4096);
    bcm2835_spi_chipSelect(BCM2835_SPI_CS0);
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);
        return 1;
}
int main(void)
{
        uint8_t adc_ID = 0x4;//id = 100 or 0x04 first 3 bits
        uint8_t analog_data[3] = {0,0,0};
        uint8_t read_command[3] =  {0b00010010,0,0} ;
        int output = 0;
        float value = -1;
        usleep(22000);
        spi_init();
        bcm2835_gpio_fsel(pin,BCM2835_GPIO_FSEL_INPT);//gpio 5 is set to input
        bcm2835_gpio_fsel(reset,BCM2835_GPIO_FSEL_OUTP);
        bcm2835_gpio_fsel(start,BCM2835_GPIO_FSEL_OUTP);
        bcm2835_gpio_set(reset);
        bcm2835_gpio_clr(start);
        usleep(22000);
        bcm2835_spi_transfer(RESET_OPCODE_MASK);
        usleep(1000);

        if(adc_ID == ((0x4) & (ADC114S08_RegRead(ID_ADDR_MASK ,0 ,3))))
        {
                printf("Device found");
        }
       if(0 == ADC114S08_RegRead(STATUS_ADDR_MASK ,0 ,1))//0x01 
        {
                printf("adc ready");
        }
        adc_start();
        bcm2835_spi_transfer(START_OPCODE_MASK);
        usleep(10000);
        bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, HIGH);
        while(1)
        {
//        if(bcm2835_gpio_lev(pin) == LOW)
//        {
        bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);
        usleep(9000);
        bcm2835_spi_transfernb(read_command , analog_data,3);
        int msb = 0;
        msb = (int)analog_data[1] << 8 ;
        output = msb + analog_data[2];
        value = (RES) * (float)output * 1000;
//      value = (output) ;
//        printf (" analog o/p  ==  %d  %d  %d\n" , analog_data[0] , analog_data[1],analog_data[2]);$
        printf(" Analog output  == %f \n " , value);
        usleep(32000);
        bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, HIGH); 
//        }
        }
}

void adc_start(){

bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);

ADC114S08_RegWrite( STATUS_ADDR_MASK, 0x80,3 );// flpor write 1 to clear the flag

ADC114S08_RegWrite( INPMUX_ADDR_MASK, 0x01,3 );// +v ain0 and -ve ain1  differential mode

ADC114S08_RegWrite( PGA_ADDR_MASK, 0x00,3 );//14.tmod pga power down gain = 1 

ADC114S08_RegWrite( DATARATE_ADDR_MASK, 0x14,3 );// g_chp dis , internal clk=4.09Mhz , continous , l$

ADC114S08_RegWrite( REF_ADDR_MASK, 0x10,3 );// ref mon disable , +ve and -ve buffer disable,ref p0 a$

ADC114S08_RegWrite( IDACMAG_ADDR_MASK, 0x00,3 );// dasable pga , low side open write 00 , idac off

ADC114S08_RegWrite( IDACMUX_ADDR_MASK, 0xff,3 );// rdac both disconnected

ADC114S08_RegWrite( VBIAS_ADDR_MASK, 0x00,3 );// vbias off 

ADC114S08_RegWrite( SYS_ADDR_MASK, 0x10,3 );//disable sys monitor 8 sampls , spi & crc& status = dis$

ADC114S08_RegWrite( OFCAL0_ADDR_MASK, 0x00,3 );

ADC114S08_RegWrite( OFCAL1_ADDR_MASK, 0x00,3 );

ADC114S08_RegWrite( OFCAL2_ADDR_MASK, 0x00,3 );

ADC114S08_RegWrite( FSCAL0_ADDR_MASK, 0x00,3 );

ADC114S08_RegWrite( FSCAL1_ADDR_MASK, 0x00,3 );

ADC114S08_RegWrite( FSCAL2_ADDR_MASK, 0x40,3 );// default

ADC114S08_RegWrite( GPIODAT_ADDR_MASK, 0x00,3 );

ADC114S08_RegWrite( GPIOCON_ADDR_MASK, 0x00,3 );

}

