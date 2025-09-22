/*
 * Platform_ESP32.cpp
 * Copyright (C) 2019-2022 Linar Yusupov
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
// #define ESP32
#if defined(ESP32)
#include "esp32/clk.h"
#include <SPI.h>
#include <esp_err.h>
#include <esp_netif.h>
#include <esp_wifi.h>
#include <soc/rtc_cntl_reg.h>
#include <rom/spi_flash.h>
#include <soc/adc_channel.h>
#include <flashchips.h>

#include "SoCHelper.h"
// #include "EPDHelper.h"
#include "TFTHelper.h"
#include "EEPROMHelper.h"
#include "WiFiHelper.h"
#include "BluetoothHelper.h"
#include "TouchHelper.h"
#include "BuddyHelper.h"

#include "SkyView.h"

#include <battery.h>
#include "BatteryHelper.h"
// #include <SD.h>

#if defined(DB)
#include <SD_MMC.h>
#include <uCDB.hpp>
uCDB<fs::SDMMCFS,fs::File> ucdb(SD_MMC);
#endif

#include <esp_wifi.h>
#include <esp_bt.h>
#include "Arduino_DriveBus_Library.h"

#if defined(ES8311_AUDIO)
#include "SoundHelper.h"
static const int8_t i2s_num = 0;
#endif
#include <esp_mac.h>

#include "driver/rtc_io.h"
#include "DebugLog.h"

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  10       /* Time ESP32 will go to sleep (in seconds) */

WebServer server ( 80 );

std::shared_ptr<Arduino_IIC_DriveBus> IIC_Bus = std::make_shared<Arduino_HWIIC>(IIC_SDA, IIC_SCL, &Wire);

extern TFT_eSPI tft;
extern TFT_eSprite sprite;

#if defined(USE_EPAPER)
/*
 * TTGO-T5S. Pin definition

#define BUSY_PIN        4
#define CS_PIN          5
#define RST_PIN         16
#define DC_PIN          17
#define SCK_PIN         18
#define MOSI_PIN        23

P1-1                    21
P1-2                    22 (LED)

I2S MAX98357A           26
                        25
                        19

I2S MIC                 27
                        32
                        33

B0                      RST
B1                      38
B2                      37
B3                      39

SD                      2
                        13
                        14
                        15

P2                      0
                        12
                        13
                        RXD
                        TXD
                        34
                        35 (BAT)
 */
GxEPD2_BW<GxEPD2_270, GxEPD2_270::HEIGHT> epd_ttgo_t5s_W3(GxEPD2_270(/*CS=5*/ 5, /*DC=*/ 17, /*RST=*/ 16, /*BUSY=*/ 4));
GxEPD2_BW<GxEPD2_270_T91, GxEPD2_270_T91::HEIGHT> epd_ttgo_t5s_T91(GxEPD2_270_T91(/*CS=5*/ 5, /*DC=*/ 17, /*RST=*/ 16, /*BUSY=*/ 4));

/*
 * Waveshare E-Paper ESP32 Driver Board

#define SCK_PIN         13
#define MOSI_PIN        14
#define CS_PIN          15
#define BUSY_PIN        25
#define RST_PIN         26
#define DC_PIN          27

B1                      0
LED                     2

RX0, TX0                3,1

P                       0,2,4,5,12,13,14,15,16,17,18,19,21,22,23,25,26,27,32,33,34,35
 */
GxEPD2_BW<GxEPD2_270, GxEPD2_270::HEIGHT> epd_waveshare_W3(GxEPD2_270(/*CS=15*/ 15, /*DC=*/ 27, /*RST=*/ 26, /*BUSY=*/ 25));
GxEPD2_BW<GxEPD2_270_T91, GxEPD2_270_T91::HEIGHT> epd_waveshare_T91(GxEPD2_270_T91(/*CS=15*/ 15, /*DC=*/ 27, /*RST=*/ 26, /*BUSY=*/ 25));
#endif

static union {
  uint8_t efuse_mac[6];
  uint64_t chipmacid;
};

// RTC_DATA_ATTR int bootCount = 0;

static uint32_t ESP32_getFlashId()
{
  return g_rom_flashchip.device_id;
}

void ESP32_TFT_fini(const char *msg)
{
  if (xSemaphoreTake(spiMutex, portMAX_DELAY)) {
    sprite.fillSprite(TFT_BLACK);
    sprite.setTextColor(TFT_BLUEBUTTON, TFT_BLACK);
    sprite.setFreeFont(&Orbitron_Light_32);
    uint16_t wd = sprite.textWidth(msg);

    sprite.setCursor(LCD_WIDTH / 2 - wd /2, LCD_HEIGHT /2 + 20);
    sprite.printf(msg);
  
    lcd_brightness(255);
    lcd_PushColors(display_column_offset, 0, 466, 466, (uint16_t*)sprite.getPointer());
    xSemaphoreGive(spiMutex);
  } else {
    PRINTLN("Failed to acquire SPI semaphore!");
  }
}

void SetUpWakeupSource()
{
  ("Enable wake up on GPIO_NUM_0 (button press)...");
  rtc_gpio_pullup_en(GPIO_NUM_0);
  rtc_gpio_pulldown_dis(GPIO_NUM_0); 
  // make sure BOOT button is the only wake up source
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, LOW);
#if defined (XPOWERS_CHIP_AXP2101)
  prepare_AXP2101_deep_sleep();
#endif
}

void ESP32_fini()
{
  WiFi_fini();
  battery_fini();
  // SPI.end();
  PRINTLN("Putting device to deep sleep...");
  delay(1000);
  lcd_sleep();
  BuddyManager::clearBuddyList();
  EEPROM_store();
  delay(1000);
  SetUpWakeupSource();
  SPI.end();
  esp_deep_sleep_start();
}

/*
  Method to print the reason by which ESP32
  has been awaken from sleep
*/
void print_wakeup_reason() 
{
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0:     PRINTLN("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1:     PRINTLN("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER:    PRINTLN("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: PRINTLN("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP:      PRINTLN("Wakeup caused by ULP program"); break;
    default:                        PRINTLN("Wakeup was not caused by deep sleep: " + String(wakeup_reason)); break;
  }
}

static void ESP32_setup()
{
  PRINTLN("ESP32_setup");
  pinMode(GPIO_NUM_0, INPUT);

  print_wakeup_reason();

  //Check if the WAKE reason was BOOT button press
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) 
  {
    if (digitalRead(GPIO_NUM_0) == LOW) 
    {
      // Only proceed if BOOT button is still pressed (i.e., held)
      unsigned long start = millis();
      bool bTwoSeconds = false;
      while (digitalRead(GPIO_NUM_0) == LOW) 
      {
        if (millis() - start >= 2000) 
        {
          bTwoSeconds = true;
          break;
        }
      }
      // If released before timeout, go back to sleep
      if (!bTwoSeconds) 
      {
      // Wait for button to be released
        while (digitalRead(GPIO_NUM_0) == LOW) {
          delay(10);  // avoid tight loop
        }
        PRINTLN("Boot button released too soon, going back to sleep...");
        SetUpWakeupSource();
        esp_deep_sleep_start();
      }
    }
    else 
    {
      // Pin is HIGH, accidental wakeup
      delay(1000);  // give some time before going back to sleep
      PRINTLN("Boot button glitch? Going back to sleep...");
      SetUpWakeupSource();
      esp_deep_sleep_start();
    }
  }
  uint32_t cpu_freq_hz = esp_clk_cpu_freq();
  float cpu_freq_mhz = cpu_freq_hz / 1000000.0;
  PRINTLN("CPU Frequency: " + String(cpu_freq_mhz) + " Mhz");
  PRINTLN("=== ESP32-S3 Memory Info ===");

  // Internal RAM
  PRINTLN("Free heap (internal RAM): " + String(esp_get_free_heap_size()) + " bytes");
  PRINTLN("Largest free block: " + String(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL)) + " bytes");
  PRINTLN("Minimum free heap ever: " + String(esp_get_minimum_free_heap_size()) + " bytes");

  // PSRAM (external RAM)
  PRINTLN("Free PSRAM: " + String(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)) + " bytes");
  PRINTLN("Largest PSRAM block: " + String(heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM)) + " bytes");

  // start I2C
  if (!Wire.begin(IIC_SDA,IIC_SCL))
  {
    PRINTLN("Failed to init I2C bus!");
    while(true)
    {
      delay(10);
    }
  }

  // Flash size
  uint32_t flash_size = 0;
  if (esp_flash_get_size(nullptr, &flash_size) == ESP_OK) {
    PRINTLN("Flash size: " + String(flash_size) + " bytes");
  }
  else 
  {
    PRINTLN("[ERROR] Failed to get flash size!");
  }  
  esp_err_t ret = ESP_OK;
  uint8_t null_mac[6] = {0};

  ret = esp_efuse_mac_get_custom(efuse_mac);
  if (ret != ESP_OK) 
  {
    // ESP_LOGE(TAG, "Get base MAC address from BLK3 of EFUSE error (%s)", esp_err_to_name(ret));
    /* If get custom base MAC address error, the application developer can decide what to do:
     * abort or use the default base MAC address which is stored in BLK0 of EFUSE by doing
     * nothing.
     */

    // ESP_LOGI(TAG, "Use base MAC address which is stored in BLK0 of EFUSE");
    chipmacid = ESP.getEfuseMac();
  }
  else 
  {
    if (memcmp(efuse_mac, null_mac, 6) == 0) 
    {
      // ESP_LOGI(TAG, "Use base MAC address which is stored in BLK0 of EFUSE");
      chipmacid = ESP.getEfuseMac();
    }
  }

  /*
   *    Board         |   Module   |  Flash memory IC
   *  ----------------+------------+--------------------
   *  DoIt ESP32      | WROOM      | GIGADEVICE_GD25Q32
   *  TTGO T3  V2.0   | PICO-D4 IC | GIGADEVICE_GD25Q32
   *  TTGO T3  V2.1.6 | PICO-D4 IC | GIGADEVICE_GD25Q32
   *  TTGO T22 V06    |            | WINBOND_NEX_W25Q32_V
   *  TTGO T22 V08    |            | WINBOND_NEX_W25Q32_V
   *  TTGO T22 V11    |            | BOYA_BY25Q32AL
   *  TTGO T8  V1.8   | WROVER     | GIGADEVICE_GD25LQ32
   *  TTGO T5S V1.9   |            | WINBOND_NEX_W25Q32_V
   *  TTGO T5S V2.8   |            | BOYA_BY25Q32AL
   *  TTGO T5  4.7    | WROVER-E   | XMC_XM25QH128C
   *  TTGO T-Watch    |            | WINBOND_NEX_W25Q128_V
   */

  // Check if PSRAM is available (it should)
  if (psramFound()) 
  {
    PRINTLN("PSRAM available");
    hw_info.revision = HW_REV_H741_01;
    // switch(flash_id)
    // {
    // case MakeFlashId(GIGADEVICE_ID, GIGADEVICE_GD25LQ32):
    //   /* ESP32-WROVER module */
    //   hw_info.revision = HW_REV_T8_1_8;
    //   break;
    // case MakeFlashId(ST_ID, XMC_XM25QH128C):
    //   /* custom ESP32-WROVER-E module with 16 MB flash */
    //   hw_info.revision = HW_REV_T5_1;
    //   break;
    // default:
    //   hw_info.revision = HW_REV_UNKNOWN;
    //   break;
    // }
  }
  else
  {
    LOG_ERROR("PSRAM is NOT enabled or not available!");
    // switch(flash_id)
    // {
    // case MakeFlashId(GIGADEVICE_ID, GIGADEVICE_GD25Q32):
    //   hw_info.revision = HW_REV_DEVKIT;
    //   break;
    // case MakeFlashId(WINBOND_NEX_ID, WINBOND_NEX_W25Q32_V):
    //   hw_info.revision = HW_REV_T5S_1_9;
    //   break;
    // case MakeFlashId(BOYA_ID, BOYA_BY25Q32AL):
    //   hw_info.revision = HW_REV_T5S_2_8;
    //   break;
    // default:
    //   hw_info.revision = HW_REV_UNKNOWN;
    //   break;
    // }
  }
  PRINTLN(hw_info.revision);
}

static uint32_t ESP32_getChipId()
{
  return (uint32_t) efuse_mac[5]        | (efuse_mac[4] << 8) | \
                   (efuse_mac[3] << 16) | (efuse_mac[2] << 24);
}

static bool ESP32_EEPROM_begin(size_t size)
{
  return EEPROM.begin(size);
}

static const int8_t ESP32_dB_to_power_level[21] = 
{
  8,  /* 2    dB, #0 */
  8,  /* 2    dB, #1 */
  8,  /* 2    dB, #2 */
  8,  /* 2    dB, #3 */
  8,  /* 2    dB, #4 */
  20, /* 5    dB, #5 */
  20, /* 5    dB, #6 */
  28, /* 7    dB, #7 */
  28, /* 7    dB, #8 */
  34, /* 8.5  dB, #9 */
  34, /* 8.5  dB, #10 */
  44, /* 11   dB, #11 */
  44, /* 11   dB, #12 */
  52, /* 13   dB, #13 */
  52, /* 13   dB, #14 */
  60, /* 15   dB, #15 */
  60, /* 15   dB, #16 */
  68, /* 17   dB, #17 */
  74, /* 18.5 dB, #18 */
  76, /* 19   dB, #19 */
  78  /* 19.5 dB, #20 */
};

static void ESP32_WiFi_setOutputPower(int dB)
{
  if (dB > 20) 
  {
    dB = 20;
  }

  if (dB < 0) 
  {
    dB = 0;
  }

  ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(ESP32_dB_to_power_level[dB]));
}

static bool ESP32_WiFi_hostname(String aHostname)
{
  return WiFi.setHostname(aHostname.c_str());
}

static void ESP32_swSer_begin(unsigned long baud)
{
  SerialInput.begin(baud, SERIAL_8N1, SOC_GPIO_PIN_GNSS_RX, SOC_GPIO_PIN_GNSS_TX);
  SerialInput.setRxBufferSize(baud / 10); /* 1 second */
}

static void ESP32_swSer_enableRx(boolean arg)
{

}

static uint32_t ESP32_maxSketchSpace()
{
  return 0x3F0000;
}

static void ESP32_WiFiUDP_stopAll()
{
/* not implemented yet */
}

static void ESP32_Battery_setup()
{
#if defined(ESP32S3)
  // calibrate_voltage(ADC1_GPIO4_CHANNEL);
  #if defined(SY6970)
  SY6970_setup();
  #elif defined(XPOWERS_CHIP_AXP2101)
  AXP2101_setup();
  #endif
#else
  
  // calibrate_voltage(settings->adapter == ADAPTER_TTGO_T5S ?
  //                   ADC1_GPIO35_CHANNEL : ADC1_GPIO36_CHANNEL);
#endif
}

static float ESP32_Battery_voltage()
{
  // PRINTLN("Reading battery voltage from SY6970...");
  delay(100); // Give some time for the SY6970 to stabilize
  #if defined(SY6970)
  float voltage = read_SY6970_voltage() * 0.001;
  #elif defined(XPOWERS_CHIP_AXP2101)
  float voltage = read_AXP2101_voltage() * 0.001;
  #elif
  float voltage = 4.2;
  #endif

  Serial.printf("Battery voltage: %.2fV\n", voltage);
  return voltage;

}


#if defined(USE_EPAPER)
static portMUX_TYPE EPD_ident_mutex;

//static ep_model_id ESP32_EPD_ident()
static int ESP32_EPD_ident()
{
  // ep_model_id rval = EP_GDEW027W3; /* default */
  int rval = EP_GDEW027W3; /* default */
#if 0
  vPortCPUInitializeMutex(&EPD_ident_mutex);

  digitalWrite(SOC_GPIO_PIN_SS_T5S, HIGH);
  pinMode(SOC_GPIO_PIN_SS_T5S, OUTPUT);
  digitalWrite(SOC_EPD_PIN_DC_T5S, HIGH);
  pinMode(SOC_EPD_PIN_DC_T5S, OUTPUT);

  digitalWrite(SOC_EPD_PIN_RST_T5S, LOW);
  pinMode(SOC_EPD_PIN_RST_T5S, OUTPUT);
  delay(20);
  pinMode(SOC_EPD_PIN_RST_T5S, INPUT_PULLUP);
  delay(200);
  pinMode(SOC_EPD_PIN_BUSY_T5S, INPUT);

  swSPI.begin();

  taskENTER_CRITICAL(&EPD_ident_mutex);

  digitalWrite(SOC_EPD_PIN_DC_T5S,  LOW);
  digitalWrite(SOC_GPIO_PIN_SS_T5S, LOW);

  swSPI.transfer_out(0x71);

  pinMode(SOC_GPIO_PIN_MOSI_T5S, INPUT);
  digitalWrite(SOC_EPD_PIN_DC_T5S, HIGH);

  uint8_t status = swSPI.transfer_in();

  digitalWrite(SOC_GPIO_PIN_SCK_T5S, LOW);
  digitalWrite(SOC_EPD_PIN_DC_T5S,  LOW);
  digitalWrite(SOC_GPIO_PIN_SS_T5S,  HIGH);

  taskEXIT_CRITICAL(&EPD_ident_mutex);

  swSPI.end();

//#if 0
//  PRINT("REG 71H: ");
//  PRINTLN(status, HEX);
//#endif

//  if (status != 2) {
//    rval = EP_GDEY027T91; /* TBD */
//  }
#endif
  return rval;
}

#define EPD_STACK_SZ      (256*4)
static TaskHandle_t EPD_Task_Handle = NULL;

//static ep_model_id ESP32_display = EP_UNKNOWN;
static int ESP32_display = EP_UNKNOWN;

static void ESP32_EPD_setup()
{
  switch(settings->adapter)
  {
  case ADAPTER_WAVESHARE_ESP32:
    display = &epd_waveshare_W3;
//    display = &epd_waveshare_T91;
    SPI.begin(SOC_GPIO_PIN_SCK_WS,
              SOC_GPIO_PIN_MISO_WS,
              SOC_GPIO_PIN_MOSI_WS,
              SOC_GPIO_PIN_SS_WS);
    break;
#if defined(BUILD_SKYVIEW_HD)
  case ADAPTER_TTGO_T5_4_7:
    display = NULL;
    break;
#endif /* BUILD_SKYVIEW_HD */
  case ADAPTER_TTGO_T5S:
  default:
    if (ESP32_display == EP_UNKNOWN) {
      ESP32_display = ESP32_EPD_ident();
    }

    switch (ESP32_display)
    {
    // case EP_GDEY027T91:
    //   display = &epd_ttgo_t5s_T91;
    //   break;
    case EP_GDEW027W3:
    // default:
      display = &epd_ttgo_t5s_W3;
      break;
    }
    PRINTLN("SPI begin");
    SPI.begin(SOC_GPIO_PIN_SCK_T5S,
              SOC_GPIO_PIN_MISO_T5S,
              SOC_GPIO_PIN_MOSI_T5S,
              SOC_GPIO_PIN_SS_T5S);

    /* SD-SPI init */
    uSD_SPI.begin(SOC_SD_PIN_SCK_T5S,
                  SOC_SD_PIN_MISO_T5S,
                  SOC_SD_PIN_MOSI_T5S,
                  SOC_SD_PIN_SS_T5S);
    break;
  }

  xTaskCreateUniversal(EPD_Task, "EPD update", EPD_STACK_SZ, NULL, 1,
                       &EPD_Task_Handle, CONFIG_ARDUINO_RUNNING_CORE);
}

static void ESP32_EPD_fini()
{
  if( EPD_Task_Handle != NULL )
  {
    vTaskDelete( EPD_Task_Handle );
  }
}

static bool ESP32_EPD_is_ready()
{
//  return true;
  return (EPD_task_command == EPD_UPDATE_NONE);
}

static void ESP32_EPD_update(int val)
{
//  EPD_Update_Sync(val);
  EPD_task_command = val;
}
#endif /* USE_EPAPER */

static size_t ESP32_WiFi_Receive_UDP(uint8_t *buf, size_t max_size)
{
  return WiFi_Receive_UDP(buf, max_size);
}

static IPAddress ESP32_WiFi_get_broadcast()
{
  esp_netif_ip_info_t info;
  IPAddress broadcastIp;

  esp_netif_t* netif = nullptr;
  if (WiFi.getMode() == WIFI_STA) {
    netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  } else {
    netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
  }
  if (netif) {
    esp_netif_get_ip_info(netif, &info);
    broadcastIp = ~info.netmask.addr | info.ip.addr;
  }
  return broadcastIp;
}
static void ESP32_WiFi_Transmit_UDP(int port, byte *buf, size_t size)
{
  IPAddress ClientIP;
  WiFiMode_t mode = WiFi.getMode();

  switch (mode)
  {
  case WIFI_STA:
    ClientIP = ESP32_WiFi_get_broadcast();
    Uni_Udp.beginPacket(ClientIP, port);
    Uni_Udp.write(buf, size);
    Uni_Udp.endPacket();
    break;
  case WIFI_AP: {
    wifi_sta_list_t stations;
    ESP_ERROR_CHECK(esp_wifi_ap_get_sta_list(&stations));

    esp_netif_sta_list_t infoList;
//    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    ESP_ERROR_CHECK(esp_netif_get_sta_list((const wifi_sta_list_t*) &stations, &infoList));

    for (int i = 0; i < infoList.num; ++i) {
      ClientIP = IPAddress(infoList.sta[i].ip.addr);
      Uni_Udp.beginPacket(ClientIP, port);
      Uni_Udp.write(buf, size);
      Uni_Udp.endPacket();
    }
    break;
  }
  case WIFI_OFF:
  default:
    break;
  }
}

static int ESP32_WiFi_clients_count()
{
  WiFiMode_t mode = WiFi.getMode();

  switch (mode)
  {
  case WIFI_AP: {
    wifi_sta_list_t stations;
    ESP_ERROR_CHECK(esp_wifi_ap_get_sta_list(&stations));

    esp_netif_sta_list_t infoList;
//    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    ESP_ERROR_CHECK(esp_netif_get_sta_list(&stations, &infoList));

    return infoList.num;
  }
  case WIFI_STA:
  default:
    return -1; /* error */
  }
}

#if defined(DB)
static bool SD_is_ok = false;
static bool ADB_is_open = false;

static bool ESP32_DB_init()
{
  bool rval = false;
  SD_is_ok = ADB_is_open = false;

  if (settings->adapter != ADAPTER_TTGO_T5S) 
  {
    return rval;
  }
  if (settings->adb == DB_NONE)
  {
    return rval;
  }
 
  // SD_MMC already mounted at this point
  // but check if card is found
  if (SD_MMC.cardType() == CARD_NONE)
  {
    return rval;
  }
  SD_is_ok = true;

#if 0
  // not yet supported!
  if (settings->adb == DB_FLN) 
  {
    if (ucdb.open("/Aircrafts/fln.cdb") == CDB_OK) 
    {
      PRINT("FLN records: ");
      PRINTLN(ucdb.recordsNumber());
      rval = true;
    }
    else
    {
      PRINTLN(F("Failed to open FlarmNet DB"));
    }
  }
#endif

  if (settings->adb == DB_OGN) 
  {
    PRINTLN("Loading /Aircrafts/ogn.cdb...");
    if (ucdb.open("/Aircrafts/ogn.cdb") == CDB_OK) 
    {
      PRINT("OGN records: ");
      PRINTLN(ucdb.recordsNumber());
      rval = true;
    }
    else
    {
      PRINTLN(F("Failed to open OGN DB"));
    }
  }

#if 0
  // not yet supported!
  if (settings->adb == DB_ICAO) 
  {
    if (ucdb.open("/Aircrafts/icao.cdb") == CDB_OK) 
    {
      PRINT("ICAO records: ");
      PRINTLN(ucdb.recordsNumber());
      rval = true;
    }
    else
    {
      PRINTLN(F("Failed to open ICAO DB"));
    }
  }
#endif

  ADB_is_open = rval;
  return rval;
}

// check DB if provided id can be found
static int ESP32_DB_query(uint8_t type, uint32_t id, char *buf, size_t size,
                            char *buf2=NULL, size_t size2=0)
{
  // The 'type' argument is for selecting which DB (OGN, FLN, etc).
  // For now we ignore it.  Only ogn.cdb is supported.

  char key[8];
  char out[64];
  uint8_t tokens[4] = { 0 };   // was [3] - allow room for future inclusion of aircraft type
  cdbResult rt;
  int c, i = 0;
  int token_cnt = 1;           // token[0] always exists, preset to index 0
  int nothing;

  if (!SD_is_ok)
  {
    return -2;   // no SD card
  }

  if (!ADB_is_open || OGN_Records() == 0)
  {
    return -1;   // no or empty database
  }

  snprintf(key, sizeof(key),"%06X", id);
  rt = ucdb.findKey(key, strlen(key));
  if (rt == KEY_FOUND) 
  {
      while ((c = ucdb.readValue()) != -1 && i < (sizeof(out) - 1)) 
      {
        if (c == '|') 
        {
          if (token_cnt < sizeof(tokens)) 
          {
            token_cnt++;
            tokens[token_cnt-1] = i+1;     // start of NEXT token
          }
          c = '\0';    // null-terminate the previous token
        }
        out[i++] = (char) c;
      }
      out[i] = '\0';    // null-terminate the last token
      nothing = i;      // index of an empty string
      if (token_cnt < 2)  tokens[1] = nothing;
      if (token_cnt < 3)  tokens[2] = nothing;
      if (token_cnt < 4)  tokens[3] = nothing;

      // this code is specific to ogn.cdb
      // if we ever use fln.cdb need specific code for that

      int pref1, pref2, pref3;
      switch (settings->idpref)
      {
      case ID_TAIL:
        pref1 = tokens[2];   // CN
        pref2 = tokens[0];   // M&M
        pref3 = tokens[1];   // reg
        break;
      case ID_MAM:
        pref1 = tokens[0];
        pref2 = tokens[1];
        pref3 = tokens[2];
        break;
      case ID_REG:
        pref1 = tokens[1];
        pref2 = tokens[0];
        pref3 = tokens[2];
        break;
      default:
        pref1 = nothing;
        pref2 = nothing;
        pref3 = nothing;
        break;
      }
      // try and show BOTH first and second preference
      // data fields, e.g., contest number AND M&M:
#if 0
// single line
      if (buf2) buf2[0] = '\0';
      if (strlen(out + pref1)) {
        snprintf(buf, size, "%s:%s",
          out + pref1,
          (strlen(out + pref2) ? out + pref2 :
           strlen(out + pref3) ? out + pref3 : ""))
      } else if (strlen(out + pref2)) {
        snprintf(buf, size, "%s:%s",
          out + pref2,
          (strlen(out + pref3) ? out + pref3 : ""))
      } else if (strlen(out + pref3)) {
        snprintf(buf, size, "%s", out + pref3);
      } else {
        buf[0] = '\0';
        return 2;   // found, but empty record
      }
#else
      // will be two lines on the display
      if (out[pref1]) 
      {
        snprintf(buf, size, "%s", &out[pref1]);
        if (buf2)
        {
          snprintf(buf2, size2, "%s",
            (out[pref2] ? &out[pref2] :
             out[pref3] ? &out[pref3] : ""));
        }
      }
      else if (out[pref2]) 
      {
        snprintf(buf, size, "%s", &out[pref2]);
        if (buf2)
        {
          snprintf(buf2, size2, "%s",
            (out[pref3] ? &out[pref3] : ""));
        }
      }
      else if (out[pref3]) 
      {
        snprintf(buf, size, "%s", &out[pref3]);
        if (buf2)  buf2[0] = '\0';
      }
      else 
      {
        buf[0] = '\0';
        if (buf2)  buf2[0] = '\0';
        return 2;   // found, but empty record
      }
#endif
      return 1;  // found
  }
  return 0;   // not found
}

// shutdown DB
static void ESP32_DB_fini()
{
#if !defined(BUILD_SKYVIEW_HD)
  if (settings->adapter == ADAPTER_TTGO_T5S) 
  {
    if (ADB_is_open) 
    {
      ucdb.close();
      ADB_is_open = false;
    }
    // ok to shutdown SD_MMC here too
    SD_MMC.end();
    SD_is_ok = false;
  }
}
#endif /* BUILD_SKYVIEW_HD */
#endif

#if defined(AUDIO)
// Text To Speach is supported on this board
void ESP32_TTS(char *message)
{
    PRINTLN("TTS: '" + String(message) + "'");
    // don't bother if Voice is disabled in settings or incorrect board
    if (settings->voice == VOICE_OFF || settings->adapter != ADAPTER_TTGO_T5S)
    {
      return;
    }

    // TTS not possible if sound is not correctly initialized
    if (!IsSoundInitialized())
    {
      return;
    }

    char filename[MAX_FILENAME_LEN];
    // *not* the post-booting demo
    if (strcmp(message, "POST")) 
    {

#if defined(USE_EPAPER)
      while (!SoC->EPD_is_ready()) {yield();}
      EPD_Message("VOICE", "ALERT");
      SoC->EPD_update(EPD_UPDATE_FAST);
      while (!SoC->EPD_is_ready()) {yield();}
#endif /* USE_EPAPER */

      if (SD_MMC.cardType() == CARD_NONE) 
      {
        LOG_ERROR(F("TTS: no SD card"));
        return;
      }

      // tokenize the message and add each word in the play queue
      // note that the actual playing of the files is handled
      // in the SoundHelper loop.
      char *word = strtok (message, " ");
      while (word != NULL)
      {
          strcpy(filename, AUDIO_FILE_PREFIX);
          strcat(filename, settings->voice == VOICE_1 ? VOICE1_SUBDIR :
                          (settings->voice == VOICE_2 ? VOICE2_SUBDIR :
                          (settings->voice == VOICE_3 ? VOICE3_SUBDIR :
                           "" )));
          strcat(filename, word);
          strcat(filename, WAV_FILE_SUFFIX);
          // add sound file to sound queue for playing
          add_file(filename);
          // next word
          word = strtok(NULL," ");
      }
   }
   else
   {
       /* post-booting */
      if (SD_MMC.cardType() == CARD_NONE) 
      {
        /* no SD card, can't play WAV files */
        LOG_ERROR(F("POST: no SD card"));
        //if (hw_info.display == DISPLAY_EPD_2_7)
        {
          /* keep boot-time SkyView logo on the screen for a while */
          delay(6000);
        }
        return;
      }
      // only very short sound is working (not sure why), 
      // so adjusted startup.wav to produce a short startup beep
      add_file((const char*) "/Audio/startup.wav");
    }
}
#endif /* AUDIO */

#if defined(BUTTONS)

#include <AceButton.h>
using namespace ace_button;

AceButton button_mode(BUTTON_MODE_PIN);
// AceButton button_up  (SOC_BUTTON_UP_T5S);
// AceButton button_down(SOC_BUTTON_DOWN_T5S);

// The event handler for the button.
void handleEvent(AceButton* button, uint8_t eventType, uint8_t buttonState) 
{
// #if 0
  // Print out a message for all events.
  if (button == &button_mode)
  {
    PRINT(F("MODE "));
  // } else if (button == &button_up) {
  //   PRINT(F("UP   "));
  // } else if (button == &button_down) {
  //   PRINT(F("DOWN "));
  // }

  PRINT(F("handleEvent(): eventType: "));
  PRINT(eventType);
  PRINT(F("; buttonState: "));
  PRINTLN(buttonState);
// #endif

    switch (eventType) 
    {
      case AceButton::kEventPressed:
        break;
      case AceButton::kEventReleased:
        if (button == &button_mode) 
        {
          TFT_Mode(true);
        }
        break;
      case AceButton::kEventLongPressed:
        if (button == &button_mode) 
        {
          shutdown("NORMAL OFF");
          PRINTLN(F("This will never be printed."));
        }
        break;
    }
  }
}

int OGN_Records()
{
  int n = 0;
  if (settings->adb == DB_OGN && ADB_is_open)
  {
    n = ucdb.recordsNumber();
  }
  return n;
}

/* Callbacks for push button interrupt */
// void onModeButtonEvent() {
//   button_mode.check();
// }

// void onUpButtonEvent() {
//   // button_up.check();
// }

// void onDownButtonEvent() {
//   // button_down.check();
// }

static void ESP32_Button_setup()
{
  int mode_button_pin = BUTTON_MODE_PIN;
  // Button(s) uses internal pull up resistor.
  pinMode(mode_button_pin, INPUT);

  button_mode.init(mode_button_pin);

  // Configure the ButtonConfig with the event handler, and enable all higher
  // level events.
  ButtonConfig* ModeButtonConfig = button_mode.getButtonConfig();
  ModeButtonConfig->setEventHandler(handleEvent);
  ModeButtonConfig->setFeature(ButtonConfig::kFeatureClick);
  ModeButtonConfig->setFeature(ButtonConfig::kFeatureLongPress);
  ModeButtonConfig->setDebounceDelay(15);
  ModeButtonConfig->setClickDelay(100);
  ModeButtonConfig->setDoubleClickDelay(1000);
  ModeButtonConfig->setLongPressDelay(2000);

  // attachInterrupt(digitalPinToInterrupt(mode_button_pin), onModeButtonEvent, CHANGE );

}

static void ESP32_Button_loop()
{
  button_mode.check();
}

static void ESP32_Button_fini()
{

}
#endif //BUTTONS

static void ESP32_WDT_setup()
{
  enableLoopWDT();
}

static void ESP32_WDT_fini()
{
  disableLoopWDT();
}

const SoC_ops_t ESP32_ops = {
  SOC_ESP32,
  "ESP32S3",
  ESP32_setup,
  ESP32_fini,
  ESP32_getChipId,
  ESP32_getFlashId,
  ESP32_EEPROM_begin,
  ESP32_WiFi_setOutputPower,
  ESP32_WiFi_hostname,
  ESP32_swSer_begin,
  ESP32_swSer_enableRx,
  ESP32_maxSketchSpace,
  ESP32_WiFiUDP_stopAll,
  ESP32_Battery_setup,
  ESP32_Battery_voltage,
#if defined(USE_EPAPER)
  ESP32_EPD_setup,
  ESP32_EPD_fini,
  ESP32_EPD_is_ready,
  ESP32_EPD_update,
#endif
  ESP32_WiFi_Receive_UDP,
  ESP32_WiFi_Transmit_UDP,
  ESP32_WiFi_clients_count,
  #if defined(DB)
  ESP32_DB_init,
  ESP32_DB_query,
  ESP32_DB_fini,
  #endif
  #if defined(AUDIO)
  ESP32_TTS,
  #endif
  #if defined(BUTTONS)
  ESP32_Button_setup,
  ESP32_Button_loop,
  ESP32_Button_fini,
  #endif /* BUTTONS */
  ESP32_WDT_setup,
  ESP32_WDT_fini,
  &ESP32_Bluetooth_ops
};

#endif /* ESP32 */
