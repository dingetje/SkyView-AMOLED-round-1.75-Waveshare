/*
 * @Description: None
 * @Author: LILYGO_L
 * @Date: 2023-06-05 13:01:59
 * @LastEditTime: 2025-01-23 15:07:46
 */
#pragma once

// #define DO0143FAT01 //DO0143FMST02//1.43 inches (SH8601 FT3168)
// #define H0175Y003AM //1.75 inches (CO5300 CST9217)
// #define DO0143FMST10 //1.43 inches (CO5300 FT3168)

#if defined(DO0143FAT01)
#define LCD_SDIO0 11
#define LCD_SDIO1 12
#define LCD_SDIO2 13
#define LCD_SDIO3 14
#define LCD_SCLK 10
#define LCD_CS 9
#define LCD_RST 21
#define LCD_WIDTH 466
#define LCD_HEIGHT 466

#define LCD_EN 42

// IIC
#define IIC_SDA 47
#define IIC_SCL 48

// TOUCH
#define TP_INT 8

// Battery Voltage ADC
#define BATTERY_VOLTAGE_ADC_DATA 4

// SD
#define SD_CS 38
#define SD_MOSI 39
#define SD_MISO 40
#define SD_SCLK 41

// PCF8563
#define PCF8563_INT 15
#endif // DO0143FAT01

#if defined(H0175Y003AM)
//AMOLED
#define LCD_SDIO0 11
#define LCD_SDIO1 13
#define LCD_SDIO2 14
#define LCD_SDIO3 15
#define LCD_SCLK 12
#define LCD_CS 10
#define LCD_RST 17
#define LCD_WIDTH 466
#define LCD_HEIGHT 466
#define LCD_EN 16

// IIC
#define IIC_SDA 7
#define IIC_SCL 6

// TOUCH
#define TP_INT 9

// Battery Voltage ADC
#define BATTERY_VOLTAGE_ADC_DATA 4

// SD
#define SD_CS 38
#define SD_MOSI 39
#define SD_MISO 40
#define SD_SCLK 41

// PCF8563
#define PCF8563_INT 9

#define SLEEP_WAKE_UP_INT GPIO_NUM_0
#define BUTTON_PIN 5

#endif // H0175Y003AM

#if defined WAVESHARE_AMOLED_1_75
//AMOLED
#define LCD_SDIO0 4
#define LCD_SDIO1 5
#define LCD_SDIO2 6
#define LCD_SDIO3 7
#define LCD_SCLK 38
#define LCD_CS 12
#define LCD_RST 39
#define LCD_WIDTH 466
#define LCD_HEIGHT 466
#define LCD_EN 13

// IIC
#define IIC_SDA 15
#define IIC_SCL 14

// TOUCH
#define TP_INT 11

// Battery Voltage ADC
#define BATTERY_VOLTAGE_ADC_DATA 4

// SD
#define SD_CS 41
#define SD_MOSI 1
#define SD_MISO 3
#define SD_SCLK 2

// PCF8563 TBD
#define PCF8563_INT 9

// TBD
#define SLEEP_WAKE_UP_INT GPIO_NUM_0
#define BUTTON_PIN 5

#endif