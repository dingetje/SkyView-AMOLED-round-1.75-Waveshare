#pragma once

/***********************config*************************/

#define SPI_FREQUENCY         26670000 //
#define TFT_SPI_MODE          SPI_MODE0
#define TFT_SPI_HOST          SPI2_HOST

#define EXAMPLE_LCD_H_RES     466 // sls project horizontal resolution
#define EXAMPLE_LCD_V_RES     466 // sls project vertical resolution
#define LVGL_LCD_BUF_SIZE     (EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES)

#define LCD_WIDTH             466 //physical display horizontal resolution
#define LCD_HEIGHT            466 //physical display vertical resolution
#define SEND_BUF_SIZE         (0x4000)//(LCD_WIDTH * LCD_HEIGHT + 8) / 10

// WaveShare AMOLED - 1.75 inches

#define TFT_QSPI_CS           12
#define TFT_QSPI_SCK          38
#define TFT_QSPI_D0           4
#define TFT_QSPI_D1           5
#define TFT_QSPI_D2           6
#define TFT_QSPI_D3           7
#define TFT_QSPI_RST          39
#define LCD_VCI_EN            13
#define TFT_TE                -1

// IIC
#define IIC_SDA 15
#define IIC_SCL 14
#define SENSOR_SDA IIC_SDA
#define SENSOR_SCL IIC_SCL

// TOUCH
#define TOUCH_INT 11
#define TOUCH_RST 40

// Battery Voltage ADC
#define BATTERY_VOLTAGE_ADC_DATA 4

// SD
#define SD_CS 41
#define SD_MOSI 1
#define SD_MISO 3
#define SD_SCLK 2

const int SDMMC_CLK = 2;
const int SDMMC_CMD = 1;
const int SDMMC_DATA = 3;
const int SDMMC_CS = 41;

// PCF8563
#define PCF8563_INT 9

// ES8311
#define I2S_MCK_IO 16
#define I2S_BCK_IO 9
#define I2S_DI_IO 10
#define I2S_WS_IO 45
#define I2S_DO_IO 8

//   i2s.setPins(BCLKPIN, WSPIN, DIPIN, DOPIN, MCLKPIN);
#define MCLKPIN             42
#define BCLKPIN              9
#define WSPIN               45
#define DOPIN                8
#define DIPIN               10
#define PAPIN               46
