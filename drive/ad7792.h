

#include "nrf_delay.h"

#include "nrf.h"
#include <stdio.h>
#include "nrf_gpio.h"
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define CS 13
#define SCLOCK 14
#define DIN 11
#define DOUT 12 



/*
  * CS - to digital pin 13  (CS pin)
  * SCLOCK - to digital pin 14 (CLK pin)
  * DIN - to digital pin 11 (MOSI pin)
  * DOUT - to digital pin 12 (MISO pin)
*/

void WriteToReg(uint8_t ByteData);
void ReadFromReg(unsigned int nByte,uint8_t *DataRead);
void Delay(unsigned int Time)
{
    nrf_delay_us(Time);
}

void AD_reset()
{
    /* PRECONFIGURE...*/
    nrf_gpio_pin_set(SCLOCK);
    nrf_gpio_pin_clear(CS); //to keep DIN=1 for 32 sclock to reset the part
    nrf_gpio_pin_set(DIN);
    int ResetTime = 32;
    while (ResetTime--)
    {
        Delay(20);
        nrf_gpio_pin_clear(SCLOCK);
        Delay(20);
        nrf_gpio_pin_set(SCLOCK);
    }
    nrf_gpio_pin_set(CS);

//    NRF_LOG_INFO("Reset\r\n");
    Delay(1000);
    
}
void setup()
{
    // set the slaveSelectPin as an output:
    
    nrf_gpio_cfg_output(CS);
    nrf_gpio_cfg_output(SCLOCK);
    nrf_gpio_cfg_input(DOUT,NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_output(DIN);

    AD_reset();
    WriteToReg(0x10); //write to Communication register.The next step is writing to Configuration register.
    WriteToReg(0x10); //set the Configuration unipolar mode.Gain=1.
    WriteToReg(0x00); //Configuration internal reference selected.1.17v
}


void WriteToReg(uint8_t ByteData) // write ByteData to the register
{
    uint8_t temp;
    int i;
    nrf_gpio_pin_clear(CS);
    temp = 0x80;
    for (i = 0; i < 8; i++)
    {
        if ((temp & ByteData) == 0)
        {
            nrf_gpio_pin_clear(DIN);
        }
        else
        {
            nrf_gpio_pin_set(DIN);
        }
        nrf_gpio_pin_clear(SCLOCK);
        Delay(20);
        nrf_gpio_pin_set(SCLOCK);
        Delay(20);
        temp = temp >> 1;
    }
    nrf_gpio_pin_set(CS);

}

void ReadFromReg(unsigned int nByte,uint8_t *DataRead) // nByte is the number of bytes which need to be read
{
    int i, j;
    uint16_t temp;
    nrf_gpio_pin_set(DIN);
    nrf_gpio_pin_clear(CS);
    temp = 0x0000;

    for (i = 0; i < nByte; i++)
    {
        for (j = 0; j < 8; j++)
        {
            nrf_gpio_pin_clear(SCLOCK);
            if (nrf_gpio_pin_read(DOUT) == 0x0)

					  {
                temp = temp << 1;
            }
            else
            {
                temp = temp << 1;
                temp = temp + 0x01;
            }
            Delay(20);
            nrf_gpio_pin_set(SCLOCK);
            Delay(20);
        }

	        DataRead[i] = temp;			
//        DataRead[i] = temp;
        temp = 0x0000;
    }
		
    nrf_gpio_pin_set(CS);
}
