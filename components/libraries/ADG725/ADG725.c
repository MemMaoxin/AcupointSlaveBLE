#include <string.h>
#include "nrf_drv_common.h"
#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "ADG725.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_delay.h"

#define SPI_INSTANCE  0 /**< SPI instance index. */
#define SYNC 6
#define SPI_SS_PIN 18
#define SPI_MISO_PIN 28
#define SPI_MOSI_PIN 8
#define SPI_SCK_PIN 15



static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */

static uint8_t    spi_tx_buf[256];   /**< TX buffer. */
static uint8_t    spi_rx_buf[256];   /**< RX buffer. */

//spi 初始化
void ADG725_spi_init(void)
{
	nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = SPI_SS_PIN;  
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin  = SPI_SCK_PIN;
		spi_config.frequency = NRF_DRV_SPI_FREQ_4M;		//时钟设置为4MHz
    spi_config.mode = NRF_DRV_SPI_MODE_1;			//CPOL=0, CPHA=1,active high and sampling at the trailing edge
    
    nrf_gpio_cfg_output(SYNC);
    
		APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, NULL, NULL));
}

void set_mux(uint8_t reg)
{
    nrf_gpio_pin_write(SYNC,0);
    uint8_t len = 1;
    spi_tx_buf[0] = reg;
		APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, spi_tx_buf, len, spi_rx_buf, len));
    nrf_gpio_pin_write(SYNC,1);
}
